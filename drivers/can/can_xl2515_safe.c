/*
 * Copyright (c) 2018 Karsten Koenig
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Modified for fibril_zephyr with bounded fault recovery.
 */

#define DT_DRV_COMPAT fibril_xl2515_safe

#include <zephyr/device.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(can_xl2515_safe, CONFIG_CAN_LOG_LEVEL);

#include "can_xl2515_safe.h"

/* Timeout for changing mode */
#define XL2515_SAFE_MODE_CHANGE_TIMEOUT_USEC 1000
#define XL2515_SAFE_MODE_CHANGE_RETRIES 100
#define XL2515_SAFE_MODE_CHANGE_DELAY                                          \
  K_USEC(XL2515_SAFE_MODE_CHANGE_TIMEOUT_USEC / XL2515_SAFE_MODE_CHANGE_RETRIES)

static void xl2515_safe_request_recovery(const struct device *dev, int reason);

static int xl2515_safe_cmd_soft_reset(const struct device *dev) {
  const struct xl2515_safe_config *dev_cfg = dev->config;
  uint8_t cmd_buf[] = {XL2515_SAFE_OPCODE_RESET};
  const struct spi_buf tx_buf = {
      .buf = cmd_buf,
      .len = sizeof(cmd_buf),
  };
  const struct spi_buf_set tx = {
      .buffers = &tx_buf,
      .count = 1U,
  };

  return spi_write_dt(&dev_cfg->bus, &tx);
}

static int xl2515_safe_cmd_bit_modify(const struct device *dev,
                                      uint8_t reg_addr, uint8_t mask,
                                      uint8_t data) {
  const struct xl2515_safe_config *dev_cfg = dev->config;
  uint8_t cmd_buf[] = {XL2515_SAFE_OPCODE_BIT_MODIFY, reg_addr, mask, data};
  const struct spi_buf tx_buf = {
      .buf = cmd_buf,
      .len = sizeof(cmd_buf),
  };
  const struct spi_buf_set tx = {
      .buffers = &tx_buf,
      .count = 1U,
  };

  return spi_write_dt(&dev_cfg->bus, &tx);
}

static int xl2515_safe_cmd_write_reg(const struct device *dev, uint8_t reg_addr,
                                     uint8_t *buf_data, uint8_t buf_len) {
  const struct xl2515_safe_config *dev_cfg = dev->config;
  uint8_t cmd_buf[] = {XL2515_SAFE_OPCODE_WRITE, reg_addr};
  struct spi_buf tx_buf[] = {
      {
          .buf = cmd_buf,
          .len = sizeof(cmd_buf),
      },
      {
          .buf = buf_data,
          .len = buf_len,
      },
  };
  const struct spi_buf_set tx = {
      .buffers = tx_buf,
      .count = ARRAY_SIZE(tx_buf),
  };

  return spi_write_dt(&dev_cfg->bus, &tx);
}

/*
 * Load TX buffer instruction
 *
 * When loading a transmit buffer, reduces the overhead of a normal WRITE
 * command by placing the Address Pointer at one of six locations, as
 * selected by parameter abc.
 *
 *   0: TX Buffer 0, Start at TXB0SIDH (0x31)
 *   1: TX Buffer 0, Start at TXB0D0 (0x36)
 *   2: TX Buffer 1, Start at TXB1SIDH (0x41)
 *   3: TX Buffer 1, Start at TXB1D0 (0x46)
 *   4: TX Buffer 2, Start at TXB2SIDH (0x51)
 *   5: TX Buffer 2, Start at TXB2D0 (0x56)
 */
static int xl2515_safe_cmd_load_tx_buffer(const struct device *dev, uint8_t abc,
                                          uint8_t *buf_data, uint8_t buf_len) {
  const struct xl2515_safe_config *dev_cfg = dev->config;
  uint8_t cmd_buf[] = {XL2515_SAFE_OPCODE_LOAD_TX_BUFFER | abc};
  struct spi_buf tx_buf[] = {
      {
          .buf = cmd_buf,
          .len = sizeof(cmd_buf),
      },
      {
          .buf = buf_data,
          .len = buf_len,
      },
  };
  const struct spi_buf_set tx = {.buffers = tx_buf,
                                 .count = ARRAY_SIZE(tx_buf)};

  __ASSERT(abc <= 5, "abc <= 5");

  return spi_write_dt(&dev_cfg->bus, &tx);
}

/*
 * Request-to-Send Instruction
 *
 * Parameter nnn is the combination of bits at positions 0, 1 and 2 in the RTS
 * opcode that respectively initiate transmission for buffers TXB0, TXB1 and
 * TXB2.
 */
static int xl2515_safe_cmd_rts(const struct device *dev, uint8_t nnn) {
  const struct xl2515_safe_config *dev_cfg = dev->config;

  uint8_t cmd_buf[] = {XL2515_SAFE_OPCODE_RTS | nnn};
  struct spi_buf tx_buf[] = {
      {
          .buf = cmd_buf,
          .len = sizeof(cmd_buf),
      },
  };
  const struct spi_buf_set tx = {
      .buffers = tx_buf,
      .count = ARRAY_SIZE(tx_buf),
  };

  __ASSERT(nnn < BIT(XL2515_SAFE_TX_CNT), "nnn < BIT(XL2515_SAFE_TX_CNT)");

  return spi_write_dt(&dev_cfg->bus, &tx);
}

static int xl2515_safe_cmd_read_reg(const struct device *dev, uint8_t reg_addr,
                                    uint8_t *buf_data, uint8_t buf_len) {
  const struct xl2515_safe_config *dev_cfg = dev->config;
  uint8_t cmd_buf[] = {XL2515_SAFE_OPCODE_READ, reg_addr};
  struct spi_buf tx_buf[] = {
      {
          .buf = cmd_buf,
          .len = sizeof(cmd_buf),
      },
      {
          .buf = NULL,
          .len = buf_len,
      },
  };
  const struct spi_buf_set tx = {
      .buffers = tx_buf,
      .count = ARRAY_SIZE(tx_buf),
  };
  struct spi_buf rx_buf[] = {
      {
          .buf = NULL,
          .len = sizeof(cmd_buf),
      },
      {
          .buf = buf_data,
          .len = buf_len,
      },
  };
  const struct spi_buf_set rx = {
      .buffers = rx_buf,
      .count = ARRAY_SIZE(rx_buf),
  };

  return spi_transceive_dt(&dev_cfg->bus, &tx, &rx);
}

/*
 * Read RX Buffer instruction
 *
 * When reading a receive buffer, reduces the overhead of a normal READ
 * command by placing the Address Pointer at one of four locations selected by
 * parameter nm:
 *   0: Receive Buffer 0, Start at RXB0SIDH (0x61)
 *   1: Receive Buffer 0, Start at RXB0D0 (0x66)
 *   2: Receive Buffer 1, Start at RXB1SIDH (0x71)
 *   3: Receive Buffer 1, Start at RXB1D0 (0x76)
 */
static int xl2515_safe_cmd_read_rx_buffer(const struct device *dev, uint8_t nm,
                                          uint8_t *buf_data, uint8_t buf_len) {
  const struct xl2515_safe_config *dev_cfg = dev->config;
  uint8_t cmd_buf[] = {XL2515_SAFE_OPCODE_READ_RX_BUFFER | (nm << 1)};
  struct spi_buf tx_buf[] = {
      {
          .buf = cmd_buf,
          .len = sizeof(cmd_buf),
      },
      {
          .buf = NULL,
          .len = buf_len,
      },
  };
  const struct spi_buf_set tx = {
      .buffers = tx_buf,
      .count = ARRAY_SIZE(tx_buf),
  };
  struct spi_buf rx_buf[] = {
      {
          .buf = NULL,
          .len = sizeof(cmd_buf),
      },
      {
          .buf = buf_data,
          .len = buf_len,
      },
  };
  const struct spi_buf_set rx = {
      .buffers = rx_buf,
      .count = ARRAY_SIZE(rx_buf),
  };

  __ASSERT(nm <= 0x03, "nm <= 0x03");

  return spi_transceive_dt(&dev_cfg->bus, &tx, &rx);
}

static void
xl2515_safe_convert_canframe_to_xl2515_safeframe(const struct can_frame *source,
                                                 uint8_t *target) {
  uint8_t rtr;
  uint8_t dlc;

  if ((source->flags & CAN_FRAME_IDE) != 0) {
    target[XL2515_SAFE_FRAME_OFFSET_SIDH] = source->id >> 21;
    target[XL2515_SAFE_FRAME_OFFSET_SIDL] = (((source->id >> 18) & 0x07) << 5) |
                                            (BIT(3)) |
                                            ((source->id >> 16) & 0x03);
    target[XL2515_SAFE_FRAME_OFFSET_EID8] = source->id >> 8;
    target[XL2515_SAFE_FRAME_OFFSET_EID0] = source->id;
  } else {
    target[XL2515_SAFE_FRAME_OFFSET_SIDH] = source->id >> 3;
    target[XL2515_SAFE_FRAME_OFFSET_SIDL] = (source->id & 0x07) << 5;
  }

  rtr = (source->flags & CAN_FRAME_RTR) != 0 ? BIT(6) : 0;
  dlc = (source->dlc) & 0x0F;

  target[XL2515_SAFE_FRAME_OFFSET_DLC] = rtr | dlc;

  if (rtr == 0U) {
    for (uint8_t data_idx = 0U; data_idx < dlc; data_idx++) {
      target[XL2515_SAFE_FRAME_OFFSET_D0 + data_idx] = source->data[data_idx];
    }
  }
}

static void
xl2515_safe_convert_xl2515_safeframe_to_canframe(const uint8_t *source,
                                                 struct can_frame *target) {
  memset(target, 0, sizeof(*target));

  if (source[XL2515_SAFE_FRAME_OFFSET_SIDL] & BIT(3)) {
    target->flags |= CAN_FRAME_IDE;
    target->id = (source[XL2515_SAFE_FRAME_OFFSET_SIDH] << 21) |
                 ((source[XL2515_SAFE_FRAME_OFFSET_SIDL] >> 5) << 18) |
                 ((source[XL2515_SAFE_FRAME_OFFSET_SIDL] & 0x03) << 16) |
                 (source[XL2515_SAFE_FRAME_OFFSET_EID8] << 8) |
                 source[XL2515_SAFE_FRAME_OFFSET_EID0];
  } else {
    target->id = (source[XL2515_SAFE_FRAME_OFFSET_SIDH] << 3) |
                 (source[XL2515_SAFE_FRAME_OFFSET_SIDL] >> 5);
  }

  target->dlc = source[XL2515_SAFE_FRAME_OFFSET_DLC] & 0x0F;

  if ((source[XL2515_SAFE_FRAME_OFFSET_DLC] & BIT(6)) != 0) {
    target->flags |= CAN_FRAME_RTR;
  } else {
    for (uint8_t data_idx = 0U; data_idx < target->dlc; data_idx++) {
      target->data[data_idx] = source[XL2515_SAFE_FRAME_OFFSET_D0 + data_idx];
    }
  }
}

static int xl2515_safe_set_mode_int(const struct device *dev,
                                    uint8_t xl2515_safe_mode) {
  int retries = XL2515_SAFE_MODE_CHANGE_RETRIES;
  uint8_t canstat;
  int ret;

  ret = xl2515_safe_cmd_bit_modify(
      dev, XL2515_SAFE_ADDR_CANCTRL, XL2515_SAFE_CANCTRL_MODE_MASK,
      xl2515_safe_mode << XL2515_SAFE_CANCTRL_MODE_POS);
  if (ret != 0) {
    return ret;
  }

  ret = xl2515_safe_cmd_read_reg(dev, XL2515_SAFE_ADDR_CANSTAT, &canstat, 1);
  if (ret != 0) {
    return ret;
  }

  while (((canstat & XL2515_SAFE_CANSTAT_MODE_MASK) >>
          XL2515_SAFE_CANSTAT_MODE_POS) != xl2515_safe_mode) {
    if (--retries < 0) {
      LOG_ERR("Timeout trying to set XL2515 operation mode");
      return -EIO;
    }

    k_sleep(XL2515_SAFE_MODE_CHANGE_DELAY);
    ret = xl2515_safe_cmd_read_reg(dev, XL2515_SAFE_ADDR_CANSTAT, &canstat, 1);
    if (ret != 0) {
      return ret;
    }
  }

  return 0;
}

static void xl2515_safe_tx_done(const struct device *dev, uint8_t tx_idx,
                                int status) {
  struct xl2515_safe_data *dev_data = dev->data;
  can_tx_callback_t callback;
  void *callback_arg;

  k_mutex_lock(&dev_data->mutex, K_FOREVER);
  callback = dev_data->tx_cb[tx_idx].cb;
  callback_arg = dev_data->tx_cb[tx_idx].cb_arg;
  if (callback != NULL) {
    dev_data->tx_cb[tx_idx].cb = NULL;
    dev_data->tx_cb[tx_idx].cb_arg = NULL;
    dev_data->tx_busy_map &= ~BIT(tx_idx);
  }
  k_mutex_unlock(&dev_data->mutex);

  if (callback != NULL) {
    k_sem_give(&dev_data->tx_sem);
    callback(dev, status, callback_arg);
  }
}

static int xl2515_safe_get_core_clock(const struct device *dev,
                                      uint32_t *rate) {
  const struct xl2515_safe_config *dev_cfg = dev->config;

  *rate = dev_cfg->osc_freq / 2;
  return 0;
}

static int xl2515_safe_get_max_filters(const struct device *dev, bool ide) {
  ARG_UNUSED(ide);

  return CONFIG_CAN_XL2515_SAFE_MAX_FILTERS;
}

static int xl2515_safe_apply_timing_locked(const struct device *dev,
                                           const struct can_timing *timing) {
  const uint8_t brp = timing->prescaler - 1;
  const uint8_t sjw = (timing->sjw - 1) << 6;
  const uint8_t phseg1 = (timing->phase_seg1 - 1) << 3;
  const uint8_t prseg = timing->prop_seg - 1;
  const uint8_t phseg2 = timing->phase_seg2 - 1;
  const uint8_t caninte = XL2515_SAFE_INTE_RX0IE | XL2515_SAFE_INTE_RX1IE |
                          XL2515_SAFE_INTE_TX0IE | XL2515_SAFE_INTE_ERRIE;
  const uint8_t rx0_ctrl = BIT(6) | BIT(5) | BIT(2);
  const uint8_t rx1_ctrl = BIT(6) | BIT(5);
  uint8_t config_buf[] = {
      phseg2,
      BIT(7) | phseg1 | prseg,
      sjw | brp,
      caninte,
  };
  int ret;

  ret = xl2515_safe_cmd_write_reg(dev, XL2515_SAFE_ADDR_CNF3, config_buf,
                                  sizeof(config_buf));
  if (ret != 0) {
    return ret;
  }

  ret = xl2515_safe_cmd_bit_modify(dev, XL2515_SAFE_ADDR_RXB0CTRL, rx0_ctrl,
                                   rx0_ctrl);
  if (ret != 0) {
    return ret;
  }

  return xl2515_safe_cmd_bit_modify(dev, XL2515_SAFE_ADDR_RXB1CTRL, rx1_ctrl,
                                    rx1_ctrl);
}

static int xl2515_safe_set_timing(const struct device *dev,
                                  const struct can_timing *timing) {
  struct xl2515_safe_data *dev_data = dev->data;
  int ret;

  if (timing == NULL) {
    return -EINVAL;
  }

  if (dev_data->common.started) {
    return -EBUSY;
  }

  __ASSERT(timing->prescaler > 0, "Prescaler should be bigger than zero");

  k_mutex_lock(&dev_data->mutex, K_FOREVER);
  ret = xl2515_safe_apply_timing_locked(dev, timing);
  if (ret == 0) {
    dev_data->timing = *timing;
    dev_data->timing_valid = true;
  }
  k_mutex_unlock(&dev_data->mutex);

  if (ret != 0 && dev_data->int_thread_started) {
    xl2515_safe_request_recovery(dev, ret);
  }

  return ret;
}

static void xl2515_safe_notify_state(const struct device *dev,
                                     enum can_state state) {
  struct xl2515_safe_data *dev_data = dev->data;
  can_state_change_callback_t callback;
  void *callback_arg;
  const struct can_bus_err_cnt err_cnt = {0};

  k_mutex_lock(&dev_data->mutex, K_FOREVER);
  callback = dev_data->common.state_change_cb;
  callback_arg = dev_data->common.state_change_cb_user_data;
  dev_data->old_state = state;
  k_mutex_unlock(&dev_data->mutex);

  if (callback != NULL) {
    callback(dev, state, err_cnt, callback_arg);
  }
}

static void xl2515_safe_request_recovery(const struct device *dev, int reason) {
  struct xl2515_safe_data *dev_data = dev->data;
  bool first_fault = atomic_cas(&dev_data->faulted, 0, 1);

  k_mutex_lock(&dev_data->mutex, K_FOREVER);
  dev_data->resume_after_recovery |= dev_data->common.started;
  k_mutex_unlock(&dev_data->mutex);

  if (first_fault) {
    LOG_ERR("XL2515 fault (%d); CAN isolated pending recovery", reason);
  }

  for (size_t i = 0; i < XL2515_SAFE_TX_CNT; i++) {
    xl2515_safe_tx_done(dev, i, reason == -ENETUNREACH ? reason : -EIO);
  }

  if (dev_data->int_thread_started) {
    k_sem_give(&dev_data->int_sem);
  }
}

static int xl2515_safe_reinitialize(const struct device *dev) {
  const struct xl2515_safe_config *dev_cfg = dev->config;
  struct xl2515_safe_data *dev_data = dev->data;
  bool restart;
  int ret;

  ret = gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio, GPIO_INT_DISABLE);
  if (ret != 0) {
    return ret;
  }

  k_mutex_lock(&dev_data->mutex, K_FOREVER);
  restart = dev_data->resume_after_recovery;
  dev_data->common.started = false;

  ret = xl2515_safe_cmd_soft_reset(dev);
  if (ret == 0) {
    k_usleep(XL2515_SAFE_OSC_STARTUP_US);
    if (!dev_data->timing_valid) {
      ret = -EINVAL;
    } else {
      ret = xl2515_safe_apply_timing_locked(dev, &dev_data->timing);
    }
  }
  if (ret == 0 && restart) {
    ret = xl2515_safe_set_mode_int(dev, dev_data->xl2515_safe_mode);
  }
  if (ret == 0) {
    ret = gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio,
                                          GPIO_INT_EDGE_TO_ACTIVE);
  }
  if (ret == 0) {
    dev_data->common.started = restart;
    dev_data->resume_after_recovery = false;
  }
  k_mutex_unlock(&dev_data->mutex);

  if (ret != 0) {
    return ret;
  }

  dev_data->fault_notified = false;
  dev_data->recovery_attempts = 0;
  atomic_clear(&dev_data->faulted);
  LOG_INF("XL2515 recovered%s", restart ? " and restarted" : "");

  xl2515_safe_notify_state(dev, restart ? CAN_STATE_ERROR_ACTIVE
                                        : CAN_STATE_STOPPED);

  ret = gpio_pin_get_dt(&dev_cfg->int_gpio);
  if (ret < 0) {
    xl2515_safe_request_recovery(dev, ret);
    return ret;
  }
  if (ret > 0) {
    k_sem_give(&dev_data->int_sem);
  }

  return 0;
}

static int xl2515_safe_get_capabilities(const struct device *dev,
                                        can_mode_t *cap) {
  ARG_UNUSED(dev);

  *cap = CAN_MODE_NORMAL | CAN_MODE_LISTENONLY | CAN_MODE_LOOPBACK;

  return 0;
}

static int xl2515_safe_start(const struct device *dev) {
  const struct xl2515_safe_config *dev_cfg = dev->config;
  struct xl2515_safe_data *dev_data = dev->data;
  int ret;

  if (atomic_get(&dev_data->faulted)) {
    return -EIO;
  }
  if (dev_data->common.started) {
    return -EALREADY;
  }

  if (dev_cfg->common.phy != NULL) {
    ret = can_transceiver_enable(dev_cfg->common.phy, dev_data->common.mode);
    if (ret != 0) {
      return ret;
    }
  }

  CAN_STATS_RESET(dev);
  k_mutex_lock(&dev_data->mutex, K_FOREVER);
  ret = xl2515_safe_set_mode_int(dev, dev_data->xl2515_safe_mode);
  if (ret == 0) {
    dev_data->common.started = true;
    dev_data->resume_after_recovery = false;
  }
  k_mutex_unlock(&dev_data->mutex);

  if (ret != 0) {
    if (dev_cfg->common.phy != NULL) {
      (void)can_transceiver_disable(dev_cfg->common.phy);
    }
    xl2515_safe_request_recovery(dev, ret);
  }

  return ret;
}

static int xl2515_safe_stop(const struct device *dev) {
  const struct xl2515_safe_config *dev_cfg = dev->config;
  struct xl2515_safe_data *dev_data = dev->data;
  int ret;

  if (!dev_data->common.started && !atomic_get(&dev_data->faulted)) {
    return -EALREADY;
  }

  k_mutex_lock(&dev_data->mutex, K_FOREVER);
  dev_data->resume_after_recovery = false;
  ret = xl2515_safe_cmd_bit_modify(dev, XL2515_SAFE_ADDR_TXB0CTRL,
                                   XL2515_SAFE_TXBNCTRL_TXREQ_MASK, 0);
  if (ret == 0) {
    ret = xl2515_safe_set_mode_int(dev, XL2515_SAFE_MODE_CONFIGURATION);
  }
  dev_data->common.started = false;
  k_mutex_unlock(&dev_data->mutex);

  for (size_t i = 0; i < XL2515_SAFE_TX_CNT; i++) {
    xl2515_safe_tx_done(dev, i, -ENETDOWN);
  }

  if (dev_cfg->common.phy != NULL) {
    int phy_ret = can_transceiver_disable(dev_cfg->common.phy);

    if (ret == 0) {
      ret = phy_ret;
    }
  }

  if (ret != 0) {
    xl2515_safe_request_recovery(dev, ret);
  }

  return ret;
}

static int xl2515_safe_set_mode(const struct device *dev, can_mode_t mode) {
  struct xl2515_safe_data *dev_data = dev->data;

  if (dev_data->common.started) {
    return -EBUSY;
  }

  switch (mode) {
  case CAN_MODE_NORMAL:
    dev_data->xl2515_safe_mode = XL2515_SAFE_MODE_NORMAL;
    break;
  case CAN_MODE_LISTENONLY:
    dev_data->xl2515_safe_mode = XL2515_SAFE_MODE_SILENT;
    break;
  case CAN_MODE_LOOPBACK:
    dev_data->xl2515_safe_mode = XL2515_SAFE_MODE_LOOPBACK;
    break;
  default:
    LOG_ERR("Unsupported CAN Mode %u", mode);
    return -ENOTSUP;
  }

  dev_data->common.mode = mode;

  return 0;
}

static int xl2515_safe_send(const struct device *dev,
                            const struct can_frame *frame, k_timeout_t timeout,
                            can_tx_callback_t callback, void *user_data) {
  struct xl2515_safe_data *dev_data = dev->data;
  uint8_t tx_frame[XL2515_SAFE_FRAME_LEN];
  uint8_t tx_idx = 0U;
  uint8_t len;
  int ret;

  if (frame->dlc > CAN_MAX_DLC) {
    return -EINVAL;
  }
  if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0) {
    return -ENOTSUP;
  }
  if (atomic_get(&dev_data->faulted)) {
    return -EIO;
  }
  if (!dev_data->common.started) {
    return -ENETDOWN;
  }
  if (k_sem_take(&dev_data->tx_sem, timeout) != 0) {
    return -EAGAIN;
  }
  xl2515_safe_convert_canframe_to_xl2515_safeframe(frame, tx_frame);
  len = sizeof(tx_frame) - CAN_MAX_DLC + frame->dlc;

  k_mutex_lock(&dev_data->mutex, K_FOREVER);
  if (atomic_get(&dev_data->faulted)) {
    ret = -EIO;
    goto no_slot;
  }
  if (!dev_data->common.started) {
    ret = -ENETDOWN;
    goto no_slot;
  }

  for (; tx_idx < XL2515_SAFE_TX_CNT; tx_idx++) {
    if ((BIT(tx_idx) & dev_data->tx_busy_map) == 0U) {
      dev_data->tx_busy_map |= BIT(tx_idx);
      dev_data->tx_cb[tx_idx].cb = callback;
      dev_data->tx_cb[tx_idx].cb_arg = user_data;
      break;
    }
  }
  if (tx_idx == XL2515_SAFE_TX_CNT) {
    ret = -EIO;
    goto no_slot;
  }

  ret = xl2515_safe_cmd_load_tx_buffer(dev, 2U * tx_idx, tx_frame, len);
  if (ret == 0) {
    ret = xl2515_safe_cmd_rts(dev, BIT(tx_idx));
  }
  k_mutex_unlock(&dev_data->mutex);

  if (ret != 0) {
    xl2515_safe_request_recovery(dev, ret);
    return ret;
  }

  return 0;

no_slot:
  k_mutex_unlock(&dev_data->mutex);
  k_sem_give(&dev_data->tx_sem);
  return ret;
}

static int xl2515_safe_add_rx_filter(const struct device *dev,
                                     can_rx_callback_t rx_cb, void *cb_arg,
                                     const struct can_filter *filter) {
  struct xl2515_safe_data *dev_data = dev->data;
  int filter_id = 0;

  __ASSERT(rx_cb != NULL, "response_ptr can not be null");

  if ((filter->flags & ~(CAN_FILTER_IDE)) != 0) {
    LOG_ERR("unsupported CAN filter flags 0x%02x", filter->flags);
    return -ENOTSUP;
  }

  k_mutex_lock(&dev_data->mutex, K_FOREVER);

  /* find free filter */
  while (filter_id < CONFIG_CAN_XL2515_SAFE_MAX_FILTERS &&
         (BIT(filter_id) & dev_data->filter_usage) != 0U) {
    filter_id++;
  }

  /* setup filter */
  if (filter_id < CONFIG_CAN_XL2515_SAFE_MAX_FILTERS) {
    dev_data->filter_usage |= BIT(filter_id);

    dev_data->filter[filter_id] = *filter;
    dev_data->rx_cb[filter_id] = rx_cb;
    dev_data->cb_arg[filter_id] = cb_arg;

  } else {
    filter_id = -ENOSPC;
  }

  k_mutex_unlock(&dev_data->mutex);

  return filter_id;
}

static void xl2515_safe_remove_rx_filter(const struct device *dev,
                                         int filter_id) {
  struct xl2515_safe_data *dev_data = dev->data;

  if (filter_id < 0 || filter_id >= CONFIG_CAN_XL2515_SAFE_MAX_FILTERS) {
    LOG_ERR("filter ID %d out of bounds", filter_id);
    return;
  }

  k_mutex_lock(&dev_data->mutex, K_FOREVER);
  dev_data->filter_usage &= ~BIT(filter_id);
  k_mutex_unlock(&dev_data->mutex);
}

static void xl2515_safe_set_state_change_callback(
    const struct device *dev, can_state_change_callback_t cb, void *user_data) {
  struct xl2515_safe_data *dev_data = dev->data;

  dev_data->common.state_change_cb = cb;
  dev_data->common.state_change_cb_user_data = user_data;
}

static void xl2515_safe_rx_filter(const struct device *dev,
                                  struct can_frame *frame) {
  struct xl2515_safe_data *dev_data = dev->data;
  can_rx_callback_t callbacks[CONFIG_CAN_XL2515_SAFE_MAX_FILTERS];
  void *callback_args[CONFIG_CAN_XL2515_SAFE_MAX_FILTERS];
  size_t callback_count = 0;

#ifndef CONFIG_CAN_ACCEPT_RTR
  if ((frame->flags & CAN_FRAME_RTR) != 0U) {
    return;
  }
#endif

  k_mutex_lock(&dev_data->mutex, K_FOREVER);
  for (size_t filter_id = 0; filter_id < CONFIG_CAN_XL2515_SAFE_MAX_FILTERS;
       filter_id++) {
    if ((BIT(filter_id) & dev_data->filter_usage) != 0U &&
        can_frame_matches_filter(frame, &dev_data->filter[filter_id])) {
      callbacks[callback_count] = dev_data->rx_cb[filter_id];
      callback_args[callback_count] = dev_data->cb_arg[filter_id];
      callback_count++;
    }
  }
  k_mutex_unlock(&dev_data->mutex);

  for (size_t i = 0; i < callback_count; i++) {
    struct can_frame callback_frame = *frame;

    callbacks[i](dev, &callback_frame, callback_args[i]);
  }
}

static int xl2515_safe_rx(const struct device *dev, uint8_t rx_idx) {
  struct can_frame frame;
  uint8_t rx_frame[XL2515_SAFE_FRAME_LEN];
  int ret;

  __ASSERT(rx_idx < XL2515_SAFE_RX_CNT, "rx_idx < XL2515_SAFE_RX_CNT");

  ret = xl2515_safe_cmd_read_rx_buffer(dev, 2U * rx_idx, rx_frame,
                                       sizeof(rx_frame));
  if (ret != 0) {
    return ret;
  }
  if ((rx_frame[XL2515_SAFE_FRAME_OFFSET_DLC] & 0x0FU) > CAN_MAX_DLC) {
    return -EBADMSG;
  }

  xl2515_safe_convert_xl2515_safeframe_to_canframe(rx_frame, &frame);
  xl2515_safe_rx_filter(dev, &frame);
  return 0;
}

static int xl2515_safe_get_state(const struct device *dev,
                                 enum can_state *state,
                                 struct can_bus_err_cnt *err_cnt) {
  struct xl2515_safe_data *dev_data = dev->data;
  uint8_t eflg;
  uint8_t err_cnt_buf[2];
  int ret;

  if (atomic_get(&dev_data->faulted)) {
    if (state != NULL) {
      *state = CAN_STATE_STOPPED;
    }
    if (err_cnt != NULL) {
      (void)memset(err_cnt, 0, sizeof(*err_cnt));
    }
    return 0;
  }

  ret =
      xl2515_safe_cmd_read_reg(dev, XL2515_SAFE_ADDR_EFLG, &eflg, sizeof(eflg));
  if (ret < 0) {
    LOG_ERR("Failed to read error register [%d]", ret);
    if (dev_data->int_thread_started) {
      xl2515_safe_request_recovery(dev, ret);
    }
    return -EIO;
  }

  if (state != NULL) {
    if (!dev_data->common.started) {
      *state = CAN_STATE_STOPPED;
    } else if (eflg & XL2515_SAFE_EFLG_TXBO) {
      *state = CAN_STATE_BUS_OFF;
    } else if ((eflg & XL2515_SAFE_EFLG_RXEP) ||
               (eflg & XL2515_SAFE_EFLG_TXEP)) {
      *state = CAN_STATE_ERROR_PASSIVE;
    } else if (eflg & XL2515_SAFE_EFLG_EWARN) {
      *state = CAN_STATE_ERROR_WARNING;
    } else {
      *state = CAN_STATE_ERROR_ACTIVE;
    }
  }

  if (err_cnt != NULL) {
    ret = xl2515_safe_cmd_read_reg(dev, XL2515_SAFE_ADDR_TEC, err_cnt_buf,
                                   sizeof(err_cnt_buf));
    if (ret < 0) {
      LOG_ERR("Failed to read error counters [%d]", ret);
      if (dev_data->int_thread_started) {
        xl2515_safe_request_recovery(dev, ret);
      }
      return -EIO;
    }

    err_cnt->tx_err_cnt = err_cnt_buf[0];
    err_cnt->rx_err_cnt = err_cnt_buf[1];
  }

#ifdef CONFIG_CAN_STATS
  if ((eflg & (XL2515_SAFE_EFLG_RX0OVR | XL2515_SAFE_EFLG_RX1OVR)) != 0U) {
    CAN_STATS_RX_OVERRUN_INC(dev);

    ret = xl2515_safe_cmd_bit_modify(
        dev, XL2515_SAFE_ADDR_EFLG,
        eflg & (XL2515_SAFE_EFLG_RX0OVR | XL2515_SAFE_EFLG_RX1OVR), 0U);
    if (ret < 0) {
      LOG_ERR("Failed to clear RX overrun flags [%d]", ret);
      if (dev_data->int_thread_started) {
        xl2515_safe_request_recovery(dev, ret);
      }
      return -EIO;
    }
  }
#endif /* CONFIG_CAN_STATS */

  return 0;
}

static int xl2515_safe_handle_errors(const struct device *dev) {
  struct xl2515_safe_data *dev_data = dev->data;
  can_state_change_callback_t state_change_cb =
      dev_data->common.state_change_cb;
  void *state_change_cb_data = dev_data->common.state_change_cb_user_data;
  struct can_bus_err_cnt err_cnt = {0};
  enum can_state state;
  int ret;

  ret = xl2515_safe_get_state(dev, &state,
                              state_change_cb != NULL ? &err_cnt : NULL);
  if (ret != 0) {
    return ret;
  }

  if (state_change_cb != NULL && dev_data->old_state != state) {
    dev_data->old_state = state;
    state_change_cb(dev, state, err_cnt, state_change_cb_data);
  }

  return state == CAN_STATE_BUS_OFF ? -ENETUNREACH : 0;
}

static int xl2515_safe_handle_interrupts(const struct device *dev) {
  const struct xl2515_safe_config *dev_cfg = dev->config;
  uint8_t canintf;
  int ret;

  for (size_t handled = 0; handled < CONFIG_CAN_XL2515_SAFE_IRQ_DRAIN_LIMIT;
       handled++) {
    ret = xl2515_safe_cmd_read_reg(dev, XL2515_SAFE_ADDR_CANINTF, &canintf, 1);
    if (ret != 0) {
      return ret;
    }

    if (canintf == 0U) {
      ret = gpio_pin_get_dt(&dev_cfg->int_gpio);
      if (ret < 0) {
        return ret;
      }
      return ret == 0 ? 0 : -EIO;
    }

    if ((canintf & XL2515_SAFE_CANINTF_RX0IF) != 0U) {
      ret = xl2515_safe_rx(dev, 0);
      if (ret != 0) {
        return ret;
      }
      canintf &= ~XL2515_SAFE_CANINTF_RX0IF;
    }
    if ((canintf & XL2515_SAFE_CANINTF_RX1IF) != 0U) {
      ret = xl2515_safe_rx(dev, 1);
      if (ret != 0) {
        return ret;
      }
      canintf &= ~XL2515_SAFE_CANINTF_RX1IF;
    }
    if ((canintf & XL2515_SAFE_CANINTF_TX0IF) != 0U) {
      xl2515_safe_tx_done(dev, 0, 0);
    }

    if ((canintf & XL2515_SAFE_CANINTF_ERRIF) != 0U) {
      ret = xl2515_safe_handle_errors(dev);
      if (ret != 0 && ret != -ENETUNREACH) {
        return ret;
      }
    } else {
      ret = 0;
    }

    if (canintf != 0U) {
      int clear_ret = xl2515_safe_cmd_bit_modify(dev, XL2515_SAFE_ADDR_CANINTF,
                                                 canintf, (uint8_t)~canintf);

      if (clear_ret != 0) {
        return clear_ret;
      }
    }
    if (ret == -ENETUNREACH) {
      return ret;
    }

    ret = gpio_pin_get_dt(&dev_cfg->int_gpio);
    if (ret < 0) {
      return ret;
    }
    if (ret == 0) {
      return 0;
    }
  }

  return -EAGAIN;
}

static void xl2515_safe_int_thread(void *p1, void *p2, void *p3) {
  const struct device *dev = p1;
  struct xl2515_safe_data *dev_data = dev->data;
  int ret;

  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  while (true) {
    k_sem_take(&dev_data->int_sem, K_FOREVER);

    if (atomic_get(&dev_data->faulted)) {
      if (!dev_data->fault_notified) {
        dev_data->fault_notified = true;
        xl2515_safe_notify_state(dev, CAN_STATE_STOPPED);
      }

      k_sleep(K_MSEC(CONFIG_CAN_XL2515_SAFE_RECOVERY_DELAY_MS));
      ret = xl2515_safe_reinitialize(dev);
      if (ret != 0) {
        dev_data->recovery_attempts++;
        if (dev_data->recovery_attempts == 1U ||
            (dev_data->recovery_attempts % 16U) == 0U) {
          LOG_ERR("XL2515 recovery failed (%d), attempt %u", ret,
                  dev_data->recovery_attempts);
        }
        k_sem_give(&dev_data->int_sem);
      }
      continue;
    }

    ret = xl2515_safe_handle_interrupts(dev);
    if (ret != 0) {
      xl2515_safe_request_recovery(dev, ret);
    }
  }
}

static void xl2515_safe_int_gpio_callback(const struct device *dev,
                                          struct gpio_callback *cb,
                                          uint32_t pins) {
  struct xl2515_safe_data *dev_data =
      CONTAINER_OF(cb, struct xl2515_safe_data, int_gpio_cb);

  k_sem_give(&dev_data->int_sem);
}

static DEVICE_API(can, can_api_funcs) = {
    .get_capabilities = xl2515_safe_get_capabilities,
    .set_timing = xl2515_safe_set_timing,
    .start = xl2515_safe_start,
    .stop = xl2515_safe_stop,
    .set_mode = xl2515_safe_set_mode,
    .send = xl2515_safe_send,
    .add_rx_filter = xl2515_safe_add_rx_filter,
    .remove_rx_filter = xl2515_safe_remove_rx_filter,
    .get_state = xl2515_safe_get_state,
    .set_state_change_callback = xl2515_safe_set_state_change_callback,
    .get_core_clock = xl2515_safe_get_core_clock,
    .get_max_filters = xl2515_safe_get_max_filters,
    .timing_min =
        {
            .sjw = 0x1,
            .prop_seg = 0x01,
            .phase_seg1 = 0x01,
            .phase_seg2 = 0x02,
            .prescaler = 0x01,
        },
    .timing_max =
        {
            .sjw = 0x04,
            .prop_seg = 0x08,
            .phase_seg1 = 0x08,
            .phase_seg2 = 0x08,
            .prescaler = 0x40,
        },
};

static int xl2515_safe_init(const struct device *dev) {
  const struct xl2515_safe_config *dev_cfg = dev->config;
  struct xl2515_safe_data *dev_data = dev->data;
  struct can_timing timing = {0};
  k_tid_t tid;
  int int_active;
  int ret;

  k_sem_init(&dev_data->int_sem, 0, 1);
  k_mutex_init(&dev_data->mutex);
  k_sem_init(&dev_data->tx_sem, XL2515_SAFE_TX_CNT, XL2515_SAFE_TX_CNT);
  atomic_clear(&dev_data->faulted);
  dev_data->int_thread_started = false;
  dev_data->timing_valid = false;
  dev_data->resume_after_recovery = false;
  dev_data->fault_notified = false;
  dev_data->recovery_attempts = 0;
  (void)memset(dev_data->rx_cb, 0, sizeof(dev_data->rx_cb));
  (void)memset(dev_data->filter, 0, sizeof(dev_data->filter));
  dev_data->old_state = CAN_STATE_STOPPED;

  if (dev_cfg->common.phy != NULL && !device_is_ready(dev_cfg->common.phy)) {
    return -ENODEV;
  }
  if (!spi_is_ready_dt(&dev_cfg->bus)) {
    return -ENODEV;
  }
  if (!gpio_is_ready_dt(&dev_cfg->int_gpio)) {
    return -ENODEV;
  }

  ret = xl2515_safe_cmd_soft_reset(dev);
  if (ret != 0) {
    return ret;
  }
  k_usleep(XL2515_SAFE_OSC_STARTUP_US);

  ret = can_calc_timing(dev, &timing, dev_cfg->common.bitrate,
                        dev_cfg->common.sample_point);
  if (ret == -EINVAL) {
    return -EIO;
  }
  ret = xl2515_safe_set_timing(dev, &timing);
  if (ret != 0) {
    return ret;
  }
  ret = xl2515_safe_set_mode(dev, CAN_MODE_NORMAL);
  if (ret != 0) {
    return ret;
  }

  ret = gpio_pin_configure_dt(&dev_cfg->int_gpio, GPIO_INPUT);
  if (ret != 0) {
    return ret;
  }
  gpio_init_callback(&dev_data->int_gpio_cb, xl2515_safe_int_gpio_callback,
                     BIT(dev_cfg->int_gpio.pin));
  ret = gpio_add_callback(dev_cfg->int_gpio.port, &dev_data->int_gpio_cb);
  if (ret != 0) {
    return ret;
  }
  ret = gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio,
                                        GPIO_INT_EDGE_TO_ACTIVE);
  if (ret != 0) {
    return ret;
  }

  int_active = gpio_pin_get_dt(&dev_cfg->int_gpio);
  if (int_active < 0) {
    return int_active;
  }

  tid = k_thread_create(
      &dev_data->int_thread, dev_data->int_thread_stack,
      dev_cfg->int_thread_stack_size, xl2515_safe_int_thread, (void *)dev, NULL,
      NULL, K_PRIO_PREEMPT(dev_cfg->int_thread_priority), 0, K_NO_WAIT);
  (void)k_thread_name_set(tid, "xl2515");
  dev_data->int_thread_started = true;

  if (int_active > 0) {
    k_sem_give(&dev_data->int_sem);
  }

  return 0;
}

#define XL2515_SAFE_INIT(inst)                                                 \
  static K_KERNEL_STACK_DEFINE(xl2515_safe_int_thread_stack_##inst,            \
                               CONFIG_CAN_XL2515_SAFE_INT_THREAD_STACK_SIZE);  \
                                                                               \
  static struct xl2515_safe_data xl2515_safe_data_##inst = {                   \
      .int_thread_stack = xl2515_safe_int_thread_stack_##inst,                 \
      .tx_busy_map = 0U,                                                       \
      .filter_usage = 0U,                                                      \
  };                                                                           \
                                                                               \
  static const struct xl2515_safe_config xl2515_safe_config_##inst = {         \
      .common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, 0, 1000000),               \
      .bus = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8)),                      \
      .int_gpio = GPIO_DT_SPEC_INST_GET(inst, int_gpios),                      \
      .int_thread_stack_size = CONFIG_CAN_XL2515_SAFE_INT_THREAD_STACK_SIZE,   \
      .int_thread_priority = CONFIG_CAN_XL2515_SAFE_INT_THREAD_PRIO,           \
      .osc_freq = DT_INST_PROP(inst, osc_freq),                                \
  };                                                                           \
                                                                               \
  CAN_DEVICE_DT_INST_DEFINE(inst, xl2515_safe_init, NULL,                      \
                            &xl2515_safe_data_##inst,                          \
                            &xl2515_safe_config_##inst, POST_KERNEL,           \
                            CONFIG_CAN_INIT_PRIORITY, &can_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(XL2515_SAFE_INIT)
