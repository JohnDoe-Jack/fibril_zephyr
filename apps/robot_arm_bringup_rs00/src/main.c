/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/motor/robstride.h>
#include <esp_now_gamepad_bridge/gamepad_bridge.h>

LOG_MODULE_REGISTER(rs00_bringup, LOG_LEVEL_INF);

#define CAN_NODE DT_NODELABEL(xl2515)
#define TRANSPORT_NODE DT_NODELABEL(robstride)
#define SHOULDER_NODE DT_NODELABEL(shoulder)
#define ELBOW_NODE DT_NODELABEL(elbow)
#define GAMEPAD_UART_NODE DT_ALIAS(gamepad_uart)

#define LOOP_MS 10
#define COMMAND_FREE_MS 3000
#define DEADMAN_MIN 200U
#define AXIS_NEUTRAL 5
#define NUDGE_RAD 0.02F

enum phase {
	PHASE_COMMAND_FREE,
	PHASE_WAIT_STOP_FEEDBACK,
	PHASE_PREPARED,
	PHASE_HOLD,
};

static bool neutral(const struct gamepad_state *pad)
{
	return pad->buttons == 0U && abs(pad->lx) <= AXIS_NEUTRAL &&
	       abs(pad->ly) <= AXIS_NEUTRAL && abs(pad->rx) <= AXIS_NEUTRAL &&
	       abs(pad->ry) <= AXIS_NEUTRAL && pad->lt == 0U && pad->rt == 0U;
}

static uint32_t epoch(const struct device *transport)
{
	struct robstride_transport_stats stats = {0};
	return robstride_transport_get_stats(transport, &stats) == 0 ? stats.operation_epoch : 0U;
}

static void stop_both(const struct device *transport)
{
	(void)robstride_request_stop(transport, epoch(transport) + 1U);
}

static bool feedback_ready(const struct device *shoulder, const struct device *elbow,
			   struct robstride_feedback *shoulder_fb,
			   struct robstride_feedback *elbow_fb)
{
	return robstride_get_feedback(shoulder, shoulder_fb) == 0 &&
	       robstride_get_feedback(elbow, elbow_fb) == 0 && shoulder_fb->valid &&
	       elbow_fb->valid && shoulder_fb->fresh && elbow_fb->fresh &&
	       shoulder_fb->position_continuous && elbow_fb->position_continuous &&
	       shoulder_fb->fault_bits == 0U && elbow_fb->fault_bits == 0U;
}

int main(void)
{
	const struct device *can_dev = DEVICE_DT_GET(CAN_NODE);
	const struct device *transport = DEVICE_DT_GET(TRANSPORT_NODE);
	const struct device *shoulder = DEVICE_DT_GET(SHOULDER_NODE);
	const struct device *elbow = DEVICE_DT_GET(ELBOW_NODE);
	const struct device *uart = DEVICE_DT_GET(GAMEPAD_UART_NODE);
	const struct robstride_csp_config config = {
		.can_timeout_ms = 100U,
		.speed_limit_rad_s = 1.0F,
		.current_limit_a = 1.0F,
	};
	enum phase phase = PHASE_COMMAND_FREE;
	int64_t boot_ms = k_uptime_get();
	int64_t next_log_ms = 0;
	float shoulder_seed = 0.0F;
	float elbow_seed = 0.0F;
	uint16_t previous_buttons = 0U;
	uint8_t previous_seq = 0U;
	uint8_t neutral_count = 0U;
	bool have_seq = false;

	if (!device_is_ready(can_dev) || !device_is_ready(transport) ||
	    !device_is_ready(shoulder) || !device_is_ready(elbow) || !device_is_ready(uart)) {
		LOG_ERR("required device is not ready");
		return -ENODEV;
	}
	int ret = can_start(can_dev);
	if (ret != 0 && ret != -EALREADY) {
		return ret;
	}
	if (robstride_transport_start(transport) != 0 || gamepad_bridge_init(uart) != 0 ||
	    gamepad_bridge_start() != 0) {
		LOG_ERR("transport startup failed");
		return -EIO;
	}
	LOG_INF("command-free smoke for %u ms", COMMAND_FREE_MS);

	while (true) {
		struct gamepad_state pad = {0};
		struct robstride_feedback shoulder_fb = {0};
		struct robstride_feedback elbow_fb = {0};
		const int64_t now_ms = k_uptime_get();
		const bool fb_ok = feedback_ready(shoulder, elbow, &shoulder_fb, &elbow_fb);
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
		    (phase == PHASE_HOLD && !fb_ok)) {
			stop_both(transport);
			phase = now_ms - boot_ms < COMMAND_FREE_MS ? PHASE_COMMAND_FREE
								 : PHASE_WAIT_STOP_FEEDBACK;
			neutral_count = 0U;
		} else if (phase == PHASE_COMMAND_FREE && now_ms - boot_ms >= COMMAND_FREE_MS) {
			phase = PHASE_WAIT_STOP_FEEDBACK;
			LOG_INF("smoke complete; neutral x3 then A sends stop/discovery");
		} else if (phase == PHASE_WAIT_STOP_FEEDBACK && neutral_count >= 3U &&
			   (pressed & GAMEPAD_BUTTON_A) != 0U) {
			stop_both(transport);
			LOG_INF("new-epoch stop sent; wait for fresh feedback, then press Y");
		} else if (phase == PHASE_WAIT_STOP_FEEDBACK && fb_ok &&
			   (pressed & GAMEPAD_BUTTON_Y) != 0U) {
			const uint32_t op_epoch = epoch(transport);
			const int shoulder_ret = robstride_prepare_csp(shoulder, &config, op_epoch);
			const int elbow_ret = robstride_prepare_csp(elbow, &config, op_epoch);
			if (shoulder_ret == 0 && elbow_ret == 0) {
				shoulder_seed = shoulder_fb.position_rad;
				elbow_seed = elbow_fb.position_rad;
				phase = PHASE_PREPARED;
				LOG_INF("both axes prepared without enable; press X to commit Hold");
			} else {
				LOG_ERR("prepare failed shoulder=%d elbow=%d", shoulder_ret, elbow_ret);
				stop_both(transport);
			}
		} else if (phase == PHASE_PREPARED && (pressed & GAMEPAD_BUTTON_X) != 0U) {
			const uint32_t op_epoch = epoch(transport);
			const int shoulder_ret = robstride_commit_enable(shoulder, op_epoch);
			const int elbow_ret = robstride_commit_enable(elbow, op_epoch);
			if (shoulder_ret == 0 && elbow_ret == 0 &&
			    robstride_submit_position(shoulder, shoulder_seed, op_epoch) == 0 &&
			    robstride_submit_position(elbow, elbow_seed, op_epoch) == 0) {
				phase = PHASE_HOLD;
				LOG_WRN("HOLD active; hold RT and D-pad for +/-0.02 rad nudge; B stops");
			} else {
				LOG_ERR("commit/initial Hold failed shoulder=%d elbow=%d", shoulder_ret,
					elbow_ret);
				stop_both(transport);
				phase = PHASE_WAIT_STOP_FEEDBACK;
			}
		} else if (phase == PHASE_HOLD) {
			float shoulder_target = shoulder_seed;
			float elbow_target = elbow_seed;
			if (pad.rt >= DEADMAN_MIN) {
				if ((pad.buttons & GAMEPAD_BUTTON_DPAD_UP) != 0U) {
					shoulder_target += NUDGE_RAD;
				} else if ((pad.buttons & GAMEPAD_BUTTON_DPAD_DOWN) != 0U) {
					shoulder_target -= NUDGE_RAD;
				} else if ((pad.buttons & GAMEPAD_BUTTON_DPAD_LEFT) != 0U) {
					elbow_target -= NUDGE_RAD;
				} else if ((pad.buttons & GAMEPAD_BUTTON_DPAD_RIGHT) != 0U) {
					elbow_target += NUDGE_RAD;
				}
			}
			const uint32_t op_epoch = epoch(transport);
			if (robstride_submit_position(shoulder, shoulder_target, op_epoch) != 0 ||
			    robstride_submit_position(elbow, elbow_target, op_epoch) != 0) {
				stop_both(transport);
				phase = PHASE_WAIT_STOP_FEEDBACK;
			}
		}

		if (now_ms >= next_log_ms) {
			LOG_INF("phase=%d epoch=%u neutral=%u fb=%d shoulder_mrad=%d age=%lld "
				"elbow_mrad=%d age=%lld",
				phase, epoch(transport), neutral_count, fb_ok,
				(int32_t)(shoulder_fb.position_rad * 1000.0F), shoulder_fb.age_ms,
				(int32_t)(elbow_fb.position_rad * 1000.0F), elbow_fb.age_ms);
			next_log_ms = now_ms + 250;
		}
		k_sleep(K_MSEC(LOOP_MS));
	}
}
