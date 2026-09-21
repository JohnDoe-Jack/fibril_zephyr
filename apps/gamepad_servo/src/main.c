/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <drivers/servo.h>
#include <esp_now_gamepad_bridge/gamepad_bridge.h>

#include "can_servo_control.h"

LOG_MODULE_REGISTER(gamepad_servo, LOG_LEVEL_INF);

#define SERVO_NODE DT_ALIAS(servo0)
#define GAMEPAD_UART_NODE DT_ALIAS(gamepad_uart)
#define CAN_NODE DT_ALIAS(can0)
#define LED_NODE DT_ALIAS(led0)

#define CONTROL_PERIOD_MS 10U
#define LED_BLINK_HALF_PERIOD_MS 500U
#define PULSE_STEP_US 5U
#define LOG_STATUS_PERIOD_MS 250U
#define CAN_STATUS_PERIOD_MS 50U

#define SERVO_MIN_US DT_PROP(SERVO_NODE, min_pulse_us)
#define SERVO_CENTER_US DT_PROP(SERVO_NODE, center_pulse_us)
#define SERVO_MAX_US DT_PROP(SERVO_NODE, max_pulse_us)
#define SERVO_MIN_ANGLE_DEG ((float)(int32_t)DT_PROP(SERVO_NODE, min_angle_deg))
#define SERVO_MAX_ANGLE_DEG ((float)(int32_t)DT_PROP(SERVO_NODE, max_angle_deg))
#define SERVO_PERIOD_US (DT_PWMS_PERIOD(SERVO_NODE) / 1000U)

BUILD_ASSERT(SERVO_PERIOD_US == 20000U,
	     "duty reporting assumes a 20 ms servo frame");

CAN_MSGQ_DEFINE(can_rx_queue, 8);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
static atomic_t can_tx_failures;

static uint32_t clamp_add(uint32_t value, uint32_t step, uint32_t maximum)
{
	return value > maximum - step ? maximum : value + step;
}

static uint32_t clamp_sub(uint32_t value, uint32_t step, uint32_t minimum)
{
	return value < minimum + step ? minimum : value - step;
}

static void can_tx_callback(const struct device *dev, int error,
			    void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (error != 0) {
		atomic_inc(&can_tx_failures);
	}
}

static int start_can(const struct device *can)
{
	const struct can_filter command_filter = {
		.id = CAN_SERVO_COMMAND_ID,
		.mask = CAN_STD_ID_MASK,
		.flags = 0,
	};
	const struct can_filter broadcast_filter = {
		.id = CAN_SERVO_BROADCAST_ID,
		.mask = CAN_STD_ID_MASK,
		.flags = 0,
	};
	int command_filter_id;
	int broadcast_filter_id;
	int ret;

	if (!device_is_ready(can)) {
		return -ENODEV;
	}

	command_filter_id =
		can_add_rx_filter_msgq(can, &can_rx_queue, &command_filter);
	if (command_filter_id < 0) {
		return command_filter_id;
	}

	broadcast_filter_id =
		can_add_rx_filter_msgq(can, &can_rx_queue, &broadcast_filter);
	if (broadcast_filter_id < 0) {
		can_remove_rx_filter(can, command_filter_id);
		return broadcast_filter_id;
	}

	ret = can_start(can);
	if (ret < 0) {
		can_remove_rx_filter(can, broadcast_filter_id);
		can_remove_rx_filter(can, command_filter_id);
	}
	return ret;
}

static int process_can_frames(const struct device *servo,
			      struct can_servo_control_state *control)
{
	struct can_frame frame;

	while (k_msgq_get(&can_rx_queue, &frame, K_NO_WAIT) == 0) {
		enum can_servo_output_action action = can_servo_handle_frame(
			control, &frame, SERVO_MIN_ANGLE_DEG,
			SERVO_MAX_ANGLE_DEG);
		int ret;

		switch (action) {
		case CAN_SERVO_OUTPUT_ATTACH_CURRENT:
			ret = servo_set_angle(servo, control->current_angle_deg);
			if (ret < 0) {
				return ret;
			}
			break;
		case CAN_SERVO_OUTPUT_DISABLE:
			ret = servo_disable(servo);
			if (ret < 0) {
				return ret;
			}
			break;
		default:
			break;
		}
	}

	return 0;
}

static int send_can_status(const struct device *can,
			   const struct can_servo_control_state *control)
{
	struct can_frame frame;
	int ret;

	can_servo_encode_status(control, &frame);
	ret = can_send(can, &frame, K_NO_WAIT, can_tx_callback, NULL);
	if (ret < 0) {
		atomic_inc(&can_tx_failures);
	}
	return ret;
}

static void log_status(const struct gamepad_state *gamepad,
		       const struct can_servo_control_state *control)
{
	/* centi-percent avoids floating-point formatting on the serial log path. */
	uint32_t duty_centi_percent = control->pulse_us * 10000U / SERVO_PERIOD_US;
	int32_t target_tenths = (int32_t)(control->target_angle_deg * 10.0f);
	int32_t current_tenths = (int32_t)(control->current_angle_deg * 10.0f);

	LOG_INF("source=%s connected=%u seq=%u buttons=0x%04x pulse=%u us "
		"duty=%u.%02u%% target=%d ddeg current=%d ddeg state=%s "
		"can_tx_failures=%u",
		can_servo_source_name(control->source), gamepad->connected,
		gamepad->seq, gamepad->buttons, control->pulse_us,
		duty_centi_percent / 100U, duty_centi_percent % 100U,
		target_tenths, current_tenths,
		can_servo_status_name(can_servo_get_status_state(control)),
		(uint32_t)atomic_get(&can_tx_failures));
}

int main(void)
{
	const struct device *servo = DEVICE_DT_GET(SERVO_NODE);
	const struct device *uart = DEVICE_DT_GET(GAMEPAD_UART_NODE);
	const struct device *can = DEVICE_DT_GET(CAN_NODE);
	struct can_servo_control_state control;
	int64_t next_log_status_ms = 0;
	int64_t next_can_status_ms = 0;
	int64_t next_led_toggle_ms = 0;
	int64_t previous_control_ms;
	bool can_started;
	bool was_connected = false;
	bool led_on = true;
	int ret;

	if (!gpio_is_ready_dt(&led)) {
		LOG_ERR("LED GPIO is not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("LED GPIO configuration failed (%d)", ret);
		return ret;
	}

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

	ret = servo_set_pulse(servo, SERVO_CENTER_US);
	if (ret < 0) {
		LOG_ERR("Initial servo command failed (%d)", ret);
		return ret;
	}
	can_servo_state_init(&control, SERVO_CENTER_US, SERVO_MIN_US,
			     SERVO_MAX_US, SERVO_MIN_ANGLE_DEG,
			     SERVO_MAX_ANGLE_DEG);

	ret = start_can(can);
	can_started = ret == 0;

	previous_control_ms = k_uptime_get();
	if (can_started) {
		LOG_INF("Ready: UART D-pad and Classic CAN servo ID %u at 1 Mbps",
			CAN_SERVO_ID);
	} else {
		LOG_ERR("CAN startup failed (%d); UART/PWM control remains available",
			ret);
	}

	while (true) {
		struct gamepad_state gamepad;
		int64_t now_ms;
		int64_t elapsed_ms;
		uint32_t next_pulse_us = control.pulse_us;
		bool up;
		bool down;

		ret = gamepad_bridge_get_state(&gamepad);
		if (ret < 0) {
			LOG_ERR("Gamepad state read failed (%d)", ret);
			break;
		}

		now_ms = k_uptime_get();
		elapsed_ms = now_ms - previous_control_ms;
		previous_control_ms = now_ms;

		/* CAN is processed before UART, so an active D-pad command in this
		 * iteration is the deterministic last command and takes ownership.
		 */
		if (can_started) {
			ret = process_can_frames(servo, &control);
			if (ret < 0) {
				LOG_ERR("CAN servo command failed (%d)", ret);
				break;
			}
		}

		up = (gamepad.buttons & GAMEPAD_BUTTON_DPAD_UP) != 0U;
		down = (gamepad.buttons & GAMEPAD_BUTTON_DPAD_DOWN) != 0U;
		if (gamepad.connected && up != down) {
			if (up) {
				next_pulse_us = clamp_add(control.pulse_us,
						  PULSE_STEP_US, SERVO_MAX_US);
			} else {
				next_pulse_us = clamp_sub(control.pulse_us,
						  PULSE_STEP_US, SERVO_MIN_US);
			}
			if (next_pulse_us != control.pulse_us) {
				ret = servo_set_pulse(servo, next_pulse_us);
				if (ret < 0) {
					LOG_ERR("UART servo command failed (%d)", ret);
					break;
				}
				can_servo_take_uart_pulse(
					&control, next_pulse_us, SERVO_MIN_US,
					SERVO_MAX_US, SERVO_MIN_ANGLE_DEG,
					SERVO_MAX_ANGLE_DEG);
			}
		}

		if (!gamepad.connected && was_connected &&
		    control.source == CAN_SERVO_SOURCE_UART) {
			ret = servo_set_pulse(servo, SERVO_CENTER_US);
			if (ret < 0) {
				LOG_ERR("UART timeout center command failed (%d)", ret);
				break;
			}
			can_servo_take_uart_pulse(
				&control, SERVO_CENTER_US, SERVO_MIN_US,
				SERVO_MAX_US, SERVO_MIN_ANGLE_DEG,
				SERVO_MAX_ANGLE_DEG);
			LOG_WRN("Gamepad link lost; returning UART-owned servo to center");
		}

		if (can_servo_step(&control, (float)elapsed_ms / 1000.0f,
				   SERVO_MIN_US, SERVO_MAX_US,
				   SERVO_MIN_ANGLE_DEG,
				   SERVO_MAX_ANGLE_DEG)) {
			ret = servo_set_angle(servo, control.current_angle_deg);
			if (ret < 0) {
				LOG_ERR("CAN slew command failed (%d)", ret);
				break;
			}
		}

		if (can_started &&
		    (control.status_requested || now_ms >= next_can_status_ms)) {
			bool periodic = now_ms >= next_can_status_ms;

			if (send_can_status(can, &control) == 0) {
				control.status_requested = false;
			}
			if (periodic) {
				next_can_status_ms = now_ms + CAN_STATUS_PERIOD_MS;
			}
		}

		if (gamepad.connected) {
			if (next_led_toggle_ms == 0) {
				next_led_toggle_ms = now_ms + LED_BLINK_HALF_PERIOD_MS;
			} else if (now_ms >= next_led_toggle_ms) {
				led_on = !led_on;
				ret = gpio_pin_set_dt(&led, led_on ? 1 : 0);
				if (ret < 0) {
					LOG_ERR("LED update failed (%d)", ret);
					break;
				}
				next_led_toggle_ms =
					now_ms + LED_BLINK_HALF_PERIOD_MS;
			}
		} else {
			next_led_toggle_ms = 0;
			if (!led_on) {
				ret = gpio_pin_set_dt(&led, 1);
				if (ret < 0) {
					LOG_ERR("LED update failed (%d)", ret);
					break;
				}
				led_on = true;
			}
		}

		was_connected = gamepad.connected;
		if (now_ms >= next_log_status_ms) {
			log_status(&gamepad, &control);
			next_log_status_ms = now_ms + LOG_STATUS_PERIOD_MS;
		}
		k_sleep(K_MSEC(CONTROL_PERIOD_MS));
	}

	if (control.attached && servo_set_pulse(servo, SERVO_CENTER_US) < 0) {
		LOG_ERR("Could not return servo to center after failure");
	}
	if (gpio_pin_set_dt(&led, 1) < 0) {
		LOG_ERR("Could not turn LED on after failure");
	}
	return ret;
}
