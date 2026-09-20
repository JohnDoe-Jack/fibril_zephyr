/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT fibril_xl2515_safe

#include <string.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/spi_emul.h>

#define XL2515_OPCODE_WRITE 0x02
#define XL2515_OPCODE_READ 0x03
#define XL2515_OPCODE_BIT_MODIFY 0x05
#define XL2515_OPCODE_LOAD_TX_BUFFER 0x40
#define XL2515_OPCODE_RTS 0x80
#define XL2515_OPCODE_RESET 0xC0
#define XL2515_ADDR_CANSTAT 0x0E
#define XL2515_ADDR_CANCTRL 0x0F
#define XL2515_MODE_MASK 0xE0

struct xl2515_emul_data {
  uint8_t reg[128];
};

static void xl2515_emul_reset(struct xl2515_emul_data *data) {
  memset(data->reg, 0, sizeof(data->reg));
  data->reg[XL2515_ADDR_CANSTAT] = XL2515_MODE_MASK;
  data->reg[XL2515_ADDR_CANCTRL] = XL2515_MODE_MASK;
}

static int xl2515_emul_io(const struct emul *target,
                          const struct spi_config *config,
                          const struct spi_buf_set *tx_bufs,
                          const struct spi_buf_set *rx_bufs) {
  struct xl2515_emul_data *data = target->data;
  const struct spi_buf *command;
  const uint8_t *cmd;

  ARG_UNUSED(config);

  if (tx_bufs == NULL || tx_bufs->count == 0U ||
      tx_bufs->buffers[0].buf == NULL || tx_bufs->buffers[0].len == 0U) {
    return -EIO;
  }

  command = &tx_bufs->buffers[0];
  cmd = command->buf;

  switch (cmd[0]) {
  case XL2515_OPCODE_RESET:
    if (command->len != 1U) {
      return -EIO;
    }
    xl2515_emul_reset(data);
    return 0;
  case XL2515_OPCODE_BIT_MODIFY:
    if (command->len != 4U) {
      return -EIO;
    }
    data->reg[cmd[1]] = (data->reg[cmd[1]] & ~cmd[2]) | (cmd[3] & cmd[2]);
    if (cmd[1] == XL2515_ADDR_CANCTRL) {
      data->reg[XL2515_ADDR_CANSTAT] =
          (data->reg[XL2515_ADDR_CANSTAT] & ~XL2515_MODE_MASK) |
          (data->reg[XL2515_ADDR_CANCTRL] & XL2515_MODE_MASK);
    }
    return 0;
  case XL2515_OPCODE_WRITE:
    if (command->len != 2U || tx_bufs->count != 2U ||
        tx_bufs->buffers[1].buf == NULL ||
        cmd[1] + tx_bufs->buffers[1].len > sizeof(data->reg)) {
      return -EIO;
    }
    memcpy(&data->reg[cmd[1]], tx_bufs->buffers[1].buf,
           tx_bufs->buffers[1].len);
    return 0;
  case XL2515_OPCODE_READ:
    if (command->len != 2U || rx_bufs == NULL || rx_bufs->count != 2U ||
        rx_bufs->buffers[1].buf == NULL ||
        cmd[1] + rx_bufs->buffers[1].len > sizeof(data->reg)) {
      return -EIO;
    }
    memcpy(rx_bufs->buffers[1].buf, &data->reg[cmd[1]],
           rx_bufs->buffers[1].len);
    return 0;
  default:
    if ((cmd[0] & 0xF8U) == XL2515_OPCODE_LOAD_TX_BUFFER ||
        (cmd[0] & 0xF8U) == XL2515_OPCODE_RTS) {
      return 0;
    }
    return -EIO;
  }
}

static int xl2515_emul_init(const struct emul *target,
                            const struct device *parent) {
  ARG_UNUSED(parent);
  xl2515_emul_reset(target->data);
  return 0;
}

static struct spi_emul_api xl2515_emul_api = {
    .io = xl2515_emul_io,
};

#define XL2515_EMUL_DEFINE(inst)                                               \
  static struct xl2515_emul_data xl2515_emul_data_##inst;                      \
  EMUL_DT_INST_DEFINE(inst, xl2515_emul_init, &xl2515_emul_data_##inst, NULL,  \
                      &xl2515_emul_api, NULL)

DT_INST_FOREACH_STATUS_OKAY(XL2515_EMUL_DEFINE)
