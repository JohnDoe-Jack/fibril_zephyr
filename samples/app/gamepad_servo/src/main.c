/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/servo.h>
#include <esp_now_gamepad_bridge/gamepad_bridge.h>

LOG_MODULE_REGISTER(gamepad_servo, LOG_LEVEL_INF);

#define SERVO_NODE DT_ALIAS(servo0)
#define GAMEPAD_UART_NODE DT_ALIAS(gamepad_uart)

#define CONTROL_PERIOD_MS 10U
#define PULSE_STEP_US 5U
#define STATUS_PERIOD_MS 250U

#define SERVO_MIN_US DT_PROP(SERVO_NODE, min_pulse_us)
#define SERVO_CENTER_US DT_PROP(SERVO_NODE, center_pulse_us)
#define SERVO_MAX_US DT_PROP(SERVO_NODE, max_pulse_us)
#define SERVO_PERIOD_US (DT_PWMS_PERIOD(SERVO_NODE) / 1000U)

BUILD_ASSERT(SERVO_PERIOD_US == 20000U,
             "duty reporting assumes a 20 ms servo frame");

static uint32_t clamp_add(uint32_t value, uint32_t step, uint32_t maximum) {
  return value > maximum - step ? maximum : value + step;
}

static uint32_t clamp_sub(uint32_t value, uint32_t step, uint32_t minimum) {
  return value < minimum + step ? minimum : value - step;
}

static void log_status(const struct gamepad_state *state, uint32_t pulse_us) {
  /* centi-percent avoids floating-point formatting on the serial log path. */
  uint32_t duty_centi_percent = pulse_us * 10000U / SERVO_PERIOD_US;

  LOG_INF("connected=%u seq=%u buttons=0x%04x pulse=%u us duty=%u.%02u%%",
          state->connected, state->seq, state->buttons, pulse_us,
          duty_centi_percent / 100U, duty_centi_percent % 100U);
}

int main(void) {
  const struct device *servo = DEVICE_DT_GET(SERVO_NODE);
  const struct device *uart = DEVICE_DT_GET(GAMEPAD_UART_NODE);
  uint32_t pulse_us = SERVO_CENTER_US;
  int64_t next_status_ms = 0;
  bool was_connected = false;
  int ret;

  if (!device_is_ready(servo)) {
    LOG_ERR("Servo is not ready");
    return -ENODEV;
  }

  ret = gamepad_bridge_init(uart);
  if (ret == 0) {
    ret = gamepad_bridge_start();
  }
  if (ret < 0) {
    LOG_ERR("Gamepad receiver startup failed (%d)", ret);
    return ret;
  }

  ret = servo_set_pulse(servo, pulse_us);
  if (ret < 0) {
    LOG_ERR("Initial servo command failed (%d)", ret);
    return ret;
  }
  LOG_INF("Ready: D-pad UP/DOWN changes the pulse by %u us every %u ms",
          PULSE_STEP_US, CONTROL_PERIOD_MS);

  while (true) {
    struct gamepad_state state;
    uint32_t next_pulse_us = pulse_us;
    int64_t now_ms;
    bool up;
    bool down;

    ret = gamepad_bridge_get_state(&state);
    if (ret < 0) {
      LOG_ERR("Gamepad state read failed (%d)", ret);
      break;
    }

    up = (state.buttons & GAMEPAD_BUTTON_DPAD_UP) != 0U;
    down = (state.buttons & GAMEPAD_BUTTON_DPAD_DOWN) != 0U;
    if (!state.connected) {
      next_pulse_us = SERVO_CENTER_US;
      if (was_connected) {
        LOG_WRN("Gamepad link lost; returning servo to center");
      }
    } else if (up && !down) {
      next_pulse_us = clamp_add(pulse_us, PULSE_STEP_US, SERVO_MAX_US);
    } else if (down && !up) {
      next_pulse_us = clamp_sub(pulse_us, PULSE_STEP_US, SERVO_MIN_US);
    }

    if (next_pulse_us != pulse_us) {
      ret = servo_set_pulse(servo, next_pulse_us);
      if (ret < 0) {
        LOG_ERR("Servo command failed (%d)", ret);
        break;
      }
      pulse_us = next_pulse_us;
    }

    was_connected = state.connected;
    now_ms = k_uptime_get();
    if (now_ms >= next_status_ms) {
      log_status(&state, pulse_us);
      next_status_ms = now_ms + STATUS_PERIOD_MS;
    }
    k_sleep(K_MSEC(CONTROL_PERIOD_MS));
  }

  if (servo_set_pulse(servo, SERVO_CENTER_US) < 0) {
    LOG_ERR("Could not return servo to center after failure");
  }
  return ret;
}
