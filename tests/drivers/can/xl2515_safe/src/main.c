/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/spi_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#define CAN_NODE DT_ALIAS(can0)

static int failing_spi_io(const struct emul *target,
                          const struct spi_config *config,
                          const struct spi_buf_set *tx_bufs,
                          const struct spi_buf_set *rx_bufs) {
  ARG_UNUSED(target);
  ARG_UNUSED(config);
  ARG_UNUSED(tx_bufs);
  ARG_UNUSED(rx_bufs);
  return -EIO;
}

ZTEST(xl2515_safe, test_spi_fault_does_not_starve_and_recovers) {
  const struct device *can_dev = DEVICE_DT_GET(CAN_NODE);
  const struct gpio_dt_spec int_gpio = GPIO_DT_SPEC_GET(CAN_NODE, int_gpios);
  const struct emul *emul = EMUL_DT_GET(CAN_NODE);
  struct spi_emul_api failing_api = {.io = failing_spi_io};
  struct can_bus_err_cnt err_cnt;
  const struct can_frame frame = {.id = 0x123, .dlc = 0};
  enum can_state state = CAN_STATE_STOPPED;
  int64_t start_ms;
  int ret;

  zassert_true(device_is_ready(can_dev));
  zassert_ok(can_start(can_dev));

  emul->bus.spi->mock_api = &failing_api;
  zassert_ok(gpio_emul_input_set_dt(&int_gpio, 1));

  start_ms = k_uptime_get();
  k_sleep(K_MSEC(30));
  zassert_true(k_uptime_get() - start_ms >= 20,
               "CAN worker starved the test thread");

  zassert_ok(can_get_state(can_dev, &state, &err_cnt));
  zassert_equal(state, CAN_STATE_STOPPED, "fault was not isolated");
  zassert_equal(can_send(can_dev, &frame, K_NO_WAIT, NULL, NULL), -EIO,
                "synchronous send did not fail promptly");

  zassert_ok(gpio_emul_input_set_dt(&int_gpio, 0));
  emul->bus.spi->mock_api = NULL;

  for (int i = 0; i < 50; i++) {
    ret = can_get_state(can_dev, &state, &err_cnt);
    if (ret == 0 && state == CAN_STATE_ERROR_ACTIVE) {
      break;
    }
    k_sleep(K_MSEC(10));
  }

  zassert_equal(state, CAN_STATE_ERROR_ACTIVE, "CAN did not recover");
  zassert_ok(can_stop(can_dev));
}

ZTEST_SUITE(xl2515_safe, NULL, NULL, NULL, NULL, NULL);
