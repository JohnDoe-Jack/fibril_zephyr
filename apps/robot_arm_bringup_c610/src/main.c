/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/motor/robomaster.h>
#include <esp_now_gamepad_bridge/gamepad_bridge.h>

LOG_MODULE_REGISTER(c610_bringup, LOG_LEVEL_INF);

#define CAN_NODE DT_NODELABEL(xl2515)
#define TRANSPORT_NODE DT_NODELABEL(robomaster)
#define RIGHT_NODE DT_NODELABEL(wrist_right)
#define LEFT_NODE DT_NODELABEL(wrist_left)
#define GAMEPAD_UART_NODE DT_ALIAS(gamepad_uart)

#define LOOP_MS 10
#define COMMAND_FREE_MS 3000
#define ZERO_PROVE_MS 1000
#define TEST_CURRENT_A 0.25F
#define DEADMAN_MIN 200U
#define AXIS_NEUTRAL 5

enum phase {
	PHASE_COMMAND_FREE,
	PHASE_WAIT_FEEDBACK,
	PHASE_ZERO_PROVE,
	PHASE_LOW_CURRENT,
};

static bool feedback_ready(const struct device *right, const struct device *left,
			   struct robomaster_feedback_ex *right_fb,
			   struct robomaster_feedback_ex *left_fb)
{
	return robomaster_get_feedback_ex(right, right_fb) == 0 &&
	       robomaster_get_feedback_ex(left, left_fb) == 0 && right_fb->raw.online &&
	       left_fb->raw.online && !right_fb->raw.stale && !left_fb->raw.stale &&
	       right_fb->continuous_valid && left_fb->continuous_valid;
}

static bool neutral(const struct gamepad_state *pad)
{
	return pad->buttons == 0U && abs(pad->lx) <= AXIS_NEUTRAL &&
	       abs(pad->ly) <= AXIS_NEUTRAL && abs(pad->rx) <= AXIS_NEUTRAL &&
	       abs(pad->ry) <= AXIS_NEUTRAL && pad->lt == 0U && pad->rt == 0U;
}

static void stop_pair(const struct device *transport, const struct device *right,
		      const struct device *left)
{
	(void)robomaster_c610_set_pair_current_a(right, left, 0.0F, 0.0F);
	(void)robomaster_request_stop(transport, BIT(0) | BIT(1));
}

int main(void)
{
	const struct device *can_dev = DEVICE_DT_GET(CAN_NODE);
	const struct device *transport = DEVICE_DT_GET(TRANSPORT_NODE);
	const struct device *right = DEVICE_DT_GET(RIGHT_NODE);
	const struct device *left = DEVICE_DT_GET(LEFT_NODE);
	const struct device *uart = DEVICE_DT_GET(GAMEPAD_UART_NODE);
	enum phase phase = PHASE_COMMAND_FREE;
	int64_t phase_since_ms = k_uptime_get();
	int64_t next_log_ms = 0;
	uint16_t previous_buttons = 0U;
	uint8_t previous_seq = 0U;
	uint8_t neutral_count = 0U;
	bool have_seq = false;
	bool transport_started = false;

	if (!device_is_ready(can_dev) || !device_is_ready(transport) || !device_is_ready(right) ||
	    !device_is_ready(left) || !device_is_ready(uart)) {
		LOG_ERR("required device is not ready");
		return -ENODEV;
	}
	int ret = can_start(can_dev);
	if (ret != 0 && ret != -EALREADY) {
		LOG_ERR("CAN start failed (%d)", ret);
		return ret;
	}
	ret = gamepad_bridge_init(uart);
	if (ret == 0) {
		ret = gamepad_bridge_start();
	}
	if (ret != 0) {
		LOG_ERR("gamepad start failed (%d)", ret);
		return ret;
	}
	LOG_INF("command-free smoke for %u ms; C610 transport is not started", COMMAND_FREE_MS);

	while (true) {
		struct gamepad_state pad = {0};
		struct robomaster_feedback_ex right_fb = {0};
		struct robomaster_feedback_ex left_fb = {0};
		const int64_t now_ms = k_uptime_get();
		const bool fb_ok = feedback_ready(right, left, &right_fb, &left_fb);
		const bool pad_ok = gamepad_bridge_get_state(&pad) == 0 && pad.connected;
		const bool fresh = pad_ok && (!have_seq || pad.seq != previous_seq);
		uint16_t pressed = 0U;

		if (fresh) {
			have_seq = true;
			previous_seq = pad.seq;
			pressed = pad.buttons & ~previous_buttons;
			previous_buttons = pad.buttons;
			if (neutral(&pad)) {
				if (neutral_count < 3U) {
					neutral_count++;
				}
			} else if (neutral_count < 3U) {
				neutral_count = 0U;
			}
		}

		if (!pad_ok || (pad.buttons & GAMEPAD_BUTTON_B) != 0U ||
		    (phase != PHASE_COMMAND_FREE && !fb_ok)) {
			if (transport_started) {
				stop_pair(transport, right, left);
			}
			phase = now_ms - phase_since_ms < COMMAND_FREE_MS ? PHASE_COMMAND_FREE
								       : PHASE_WAIT_FEEDBACK;
			neutral_count = 0U;
		} else if (phase == PHASE_COMMAND_FREE &&
			   now_ms - phase_since_ms >= COMMAND_FREE_MS) {
			phase = PHASE_WAIT_FEEDBACK;
			LOG_INF("smoke complete; wait for both feedback streams and neutral x3");
		} else if (phase == PHASE_WAIT_FEEDBACK && fb_ok && neutral_count >= 3U &&
			   (pressed & GAMEPAD_BUTTON_A) != 0U) {
			ret = robomaster_transport_start(transport);
			if (ret == 0 || ret == -EALREADY) {
				transport_started = true;
				(void)robomaster_c610_set_pair_current_a(right, left, 0.0F, 0.0F);
				phase = PHASE_ZERO_PROVE;
				phase_since_ms = now_ms;
				LOG_INF("zero-current proof started; keep controls neutral");
			}
		} else if (phase == PHASE_ZERO_PROVE) {
			(void)robomaster_c610_set_pair_current_a(right, left, 0.0F, 0.0F);
			if (now_ms - phase_since_ms >= ZERO_PROVE_MS &&
			    (pressed & GAMEPAD_BUTTON_Y) != 0U) {
				phase = PHASE_LOW_CURRENT;
				LOG_WRN("LOW-CURRENT mode: hold RT and D-pad; B stops");
			}
		} else if (phase == PHASE_LOW_CURRENT) {
			float right_a = 0.0F;
			float left_a = 0.0F;
			if (pad.rt >= DEADMAN_MIN) {
				if ((pad.buttons & GAMEPAD_BUTTON_DPAD_UP) != 0U) {
					right_a = TEST_CURRENT_A;
					left_a = TEST_CURRENT_A;
				} else if ((pad.buttons & GAMEPAD_BUTTON_DPAD_DOWN) != 0U) {
					right_a = -TEST_CURRENT_A;
					left_a = -TEST_CURRENT_A;
				} else if ((pad.buttons & GAMEPAD_BUTTON_DPAD_LEFT) != 0U) {
					right_a = -TEST_CURRENT_A;
					left_a = TEST_CURRENT_A;
				} else if ((pad.buttons & GAMEPAD_BUTTON_DPAD_RIGHT) != 0U) {
					right_a = TEST_CURRENT_A;
					left_a = -TEST_CURRENT_A;
				}
			}
			if (robomaster_c610_set_pair_current_a(right, left, right_a, left_a) != 0) {
				stop_pair(transport, right, left);
				phase = PHASE_WAIT_FEEDBACK;
			}
		}

		if (now_ms >= next_log_ms) {
			LOG_INF("phase=%d pad=%d neutral=%u fb=%d R[count=%lld rpm=%d] "
				"L[count=%lld rpm=%d]",
				phase, pad_ok, neutral_count, fb_ok, right_fb.continuous_count,
				right_fb.raw.velocity, left_fb.continuous_count, left_fb.raw.velocity);
			next_log_ms = now_ms + 250;
		}
		k_sleep(K_MSEC(LOOP_MS));
	}
}
