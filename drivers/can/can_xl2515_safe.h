/*
 * Copyright (c) 2018 Karsten Koenig
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef _XL2515_SAFE_H_
#define _XL2515_SAFE_H_

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>

#define XL2515_SAFE_RX_CNT 2
/* Reduce the number of Tx buffers to 1 in order to avoid priority inversion. */
#define XL2515_SAFE_TX_CNT 1
#define XL2515_SAFE_FRAME_LEN 13

struct xl2515_safe_tx_cb {
  can_tx_callback_t cb;
  void *cb_arg;
};

struct xl2515_safe_data {
  struct can_driver_data common;

  /* interrupt data */
  struct gpio_callback int_gpio_cb;
  struct k_thread int_thread;
  k_thread_stack_t *int_thread_stack;
  struct k_sem int_sem;
  bool int_thread_started;

  /* tx data */
  struct k_sem tx_sem;
  struct xl2515_safe_tx_cb tx_cb[XL2515_SAFE_TX_CNT];
  uint8_t tx_busy_map;

  /* filter data */
  uint32_t filter_usage;
  can_rx_callback_t rx_cb[CONFIG_CAN_XL2515_SAFE_MAX_FILTERS];
  void *cb_arg[CONFIG_CAN_XL2515_SAFE_MAX_FILTERS];
  struct can_filter filter[CONFIG_CAN_XL2515_SAFE_MAX_FILTERS];

  /* general data */
  struct k_mutex mutex;
  enum can_state old_state;
  uint8_t xl2515_safe_mode;
  struct can_timing timing;
  atomic_t faulted;
  bool timing_valid;
  bool resume_after_recovery;
  bool fault_notified;
  uint32_t recovery_attempts;
};

struct xl2515_safe_config {
  const struct can_driver_config common;

  /* spi configuration */
  struct spi_dt_spec bus;

  /* interrupt configuration */
  struct gpio_dt_spec int_gpio;
  size_t int_thread_stack_size;
  int int_thread_priority;

  /* CAN timing */
  uint32_t osc_freq;
};

/*
 * Startup time of 128 OSC1 clock cycles at 1MHz (minimum clock in frequency)
 * see the MCP2515-compatible register interface, section 8.1 Oscillator
 * Start-up Timer
 */
#define XL2515_SAFE_OSC_STARTUP_US 128U

/* MCP2515-compatible opcodes */
#define XL2515_SAFE_OPCODE_WRITE 0x02
#define XL2515_SAFE_OPCODE_READ 0x03
#define XL2515_SAFE_OPCODE_BIT_MODIFY 0x05
#define XL2515_SAFE_OPCODE_LOAD_TX_BUFFER 0x40
#define XL2515_SAFE_OPCODE_RTS 0x80
#define XL2515_SAFE_OPCODE_READ_RX_BUFFER 0x90
#define XL2515_SAFE_OPCODE_READ_STATUS 0xA0
#define XL2515_SAFE_OPCODE_RESET 0xC0

/* MCP2515-compatible registers */
#define XL2515_SAFE_ADDR_CANSTAT 0x0E
#define XL2515_SAFE_ADDR_CANCTRL 0x0F
#define XL2515_SAFE_ADDR_TEC 0x1C
#define XL2515_SAFE_ADDR_REC 0x1D
#define XL2515_SAFE_ADDR_CNF3 0x28
#define XL2515_SAFE_ADDR_CNF2 0x29
#define XL2515_SAFE_ADDR_CNF1 0x2A
#define XL2515_SAFE_ADDR_CANINTE 0x2B
#define XL2515_SAFE_ADDR_CANINTF 0x2C
#define XL2515_SAFE_ADDR_EFLG 0x2D
#define XL2515_SAFE_ADDR_TXB0CTRL 0x30
#define XL2515_SAFE_ADDR_TXB1CTRL 0x40
#define XL2515_SAFE_ADDR_TXB2CTRL 0x50
#define XL2515_SAFE_ADDR_RXB0CTRL 0x60
#define XL2515_SAFE_ADDR_RXB1CTRL 0x70

#define XL2515_SAFE_ADDR_OFFSET_FRAME2FRAME 0x10
#define XL2515_SAFE_ADDR_OFFSET_CTRL2FRAME 0x01

/* XL2515_SAFE Operation Modes */
#define XL2515_SAFE_MODE_NORMAL 0x00
#define XL2515_SAFE_MODE_LOOPBACK 0x02
#define XL2515_SAFE_MODE_SILENT 0x03
#define XL2515_SAFE_MODE_CONFIGURATION 0x04

/* XL2515_SAFE_FRAME_OFFSET */
#define XL2515_SAFE_FRAME_OFFSET_SIDH 0
#define XL2515_SAFE_FRAME_OFFSET_SIDL 1
#define XL2515_SAFE_FRAME_OFFSET_EID8 2
#define XL2515_SAFE_FRAME_OFFSET_EID0 3
#define XL2515_SAFE_FRAME_OFFSET_DLC 4
#define XL2515_SAFE_FRAME_OFFSET_D0 5

/* XL2515_SAFE_CANINTF */
#define XL2515_SAFE_CANINTF_RX0IF BIT(0)
#define XL2515_SAFE_CANINTF_RX1IF BIT(1)
#define XL2515_SAFE_CANINTF_TX0IF BIT(2)
#define XL2515_SAFE_CANINTF_TX1IF BIT(3)
#define XL2515_SAFE_CANINTF_TX2IF BIT(4)
#define XL2515_SAFE_CANINTF_ERRIF BIT(5)
#define XL2515_SAFE_CANINTF_WAKIF BIT(6)
#define XL2515_SAFE_CANINTF_MERRF BIT(7)

#define XL2515_SAFE_INTE_RX0IE BIT(0)
#define XL2515_SAFE_INTE_RX1IE BIT(1)
#define XL2515_SAFE_INTE_TX0IE BIT(2)
#define XL2515_SAFE_INTE_TX1IE BIT(3)
#define XL2515_SAFE_INTE_TX2IE BIT(4)
#define XL2515_SAFE_INTE_ERRIE BIT(5)
#define XL2515_SAFE_INTE_WAKIE BIT(6)
#define XL2515_SAFE_INTE_MERRE BIT(7)

#define XL2515_SAFE_EFLG_EWARN BIT(0)
#define XL2515_SAFE_EFLG_RXWAR BIT(1)
#define XL2515_SAFE_EFLG_TXWAR BIT(2)
#define XL2515_SAFE_EFLG_RXEP BIT(3)
#define XL2515_SAFE_EFLG_TXEP BIT(4)
#define XL2515_SAFE_EFLG_TXBO BIT(5)
#define XL2515_SAFE_EFLG_RX0OVR BIT(6)
#define XL2515_SAFE_EFLG_RX1OVR BIT(7)

#define XL2515_SAFE_TXCTRL_TXREQ BIT(3)

#define XL2515_SAFE_CANSTAT_MODE_POS 5
#define XL2515_SAFE_CANSTAT_MODE_MASK (0x07 << XL2515_SAFE_CANSTAT_MODE_POS)
#define XL2515_SAFE_CANCTRL_MODE_POS 5
#define XL2515_SAFE_CANCTRL_MODE_MASK (0x07 << XL2515_SAFE_CANCTRL_MODE_POS)
#define XL2515_SAFE_TXBNCTRL_TXREQ_POS 3
#define XL2515_SAFE_TXBNCTRL_TXREQ_MASK (0x01 << XL2515_SAFE_TXBNCTRL_TXREQ_POS)

#endif /*_XL2515_SAFE_H_*/
