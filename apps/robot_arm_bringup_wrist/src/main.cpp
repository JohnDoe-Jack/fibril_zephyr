/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <cstdlib>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <differential_screw/wrist.hpp>
#include <drivers/motor/robomaster.h>
#include <esp_now_gamepad_bridge/gamepad_bridge.h>

LOG_MODULE_REGISTER(wrist_bringup, LOG_LEVEL_INF);

namespace
{
constexpr int loop_ms = 10;
constexpr int command_free_ms = 3000;
constexpr int zero_prove_ms = 1000;
constexpr uint8_t deadman_min = 200U;
constexpr int axis_neutral = 5;

const device *const can_dev = DEVICE_DT_GET(DT_NODELABEL(xl2515));
const device *const transport = DEVICE_DT_GET(DT_NODELABEL(robomaster));
const device *const right_motor = DEVICE_DT_GET(DT_NODELABEL(wrist_right));
const device *const left_motor = DEVICE_DT_GET(DT_NODELABEL(wrist_left));
const device *const gamepad_uart = DEVICE_DT_GET(DT_ALIAS(gamepad_uart));

enum class Phase { CommandFree, WaitFeedback, ZeroProve, Homed, Armed };

bool neutral(const gamepad_state &pad)
{
	return pad.buttons == 0U && std::abs(pad.lx) <= axis_neutral &&
	       std::abs(pad.ly) <= axis_neutral && std::abs(pad.rx) <= axis_neutral &&
	       std::abs(pad.ry) <= axis_neutral && pad.lt == 0U && pad.rt == 0U;
}

bool feedbackReady(robomaster_feedback_ex &right, robomaster_feedback_ex &left)
{
	return robomaster_get_feedback_ex(right_motor, &right) == 0 &&
	       robomaster_get_feedback_ex(left_motor, &left) == 0 && right.raw.online &&
	       left.raw.online && !right.raw.stale && !left.raw.stale && right.continuous_valid &&
	       left.continuous_valid;
}

dscrew::Config wristConfig()
{
	dscrew::Config config{};
	config.current_limit_a = 0.5F;
	config.lift_rate_mm_s = -5.0F;
	config.spin_step_rad = -5.0F * dscrew::pi / 180.0F;
	config.stall_hold_a = 0.20F;
	return config;
}

dscrew::NutAngles angles(const dscrew::Config &config, const robomaster_feedback_ex &right,
			 const robomaster_feedback_ex &left)
{
	dscrew::NutAngles value{};
	value.phi_r =
		dscrew::nutRadFromCounts(config, right.continuous_count, config.right_direction);
	value.phi_l = dscrew::nutRadFromCounts(config, left.continuous_count, config.left_direction);
	return value;
}

dscrew::NutRates rates(const dscrew::Config &config, const robomaster_feedback_ex &right,
		       const robomaster_feedback_ex &left)
{
	dscrew::NutRates value{};
	value.dphi_r = dscrew::nutRadPerSecFromRpm(config, right.raw.velocity,
						    config.right_direction);
	value.dphi_l =
		dscrew::nutRadPerSecFromRpm(config, left.raw.velocity, config.left_direction);
	return value;
}

void stopPair()
{
	(void)robomaster_c610_set_pair_current_a(right_motor, left_motor, 0.0F, 0.0F);
	(void)robomaster_request_stop(transport, BIT(0) | BIT(1));
}
} // namespace

int main()
{
	if (!device_is_ready(can_dev) || !device_is_ready(transport) ||
	    !device_is_ready(right_motor) || !device_is_ready(left_motor) ||
	    !device_is_ready(gamepad_uart)) {
		LOG_ERR("required device is not ready");
		return -ENODEV;
	}
	int ret = can_start(can_dev);
	if (ret != 0 && ret != -EALREADY) {
		LOG_ERR("CAN start failed (%d)", ret);
		return ret;
	}
	if (gamepad_bridge_init(gamepad_uart) != 0 || gamepad_bridge_start() != 0) {
		LOG_ERR("gamepad start failed");
		return -EIO;
	}

	const dscrew::Config config = wristConfig();
	dscrew::Wrist wrist(config);
	if (!wrist.valid()) {
		LOG_ERR("safe wrist configuration is invalid");
		return -EINVAL;
	}

	Phase phase = Phase::CommandFree;
	int64_t phase_since = k_uptime_get();
	int64_t next_log = 0;
	uint16_t previous_buttons = 0U;
	uint8_t previous_seq = 0U;
	uint8_t neutral_count = 0U;
	bool have_seq = false;
	bool transport_started = false;
	LOG_INF("command-free smoke for %d ms; C610 transport is not started", command_free_ms);

	while (true) {
		gamepad_state pad{};
		robomaster_feedback_ex right{};
		robomaster_feedback_ex left{};
		const int64_t now = k_uptime_get();
		const bool fb_ok = feedbackReady(right, left);
		const bool pad_ok = gamepad_bridge_get_state(&pad) == 0 && pad.connected;
		const bool fresh = pad_ok && (!have_seq || pad.seq != previous_seq);
		uint16_t pressed = 0U;
		if (fresh) {
			have_seq = true;
			previous_seq = pad.seq;
			pressed = pad.buttons & ~previous_buttons;
			previous_buttons = pad.buttons;
			if (neutral(pad)) {
				neutral_count = neutral_count < 3U ? neutral_count + 1U : neutral_count;
			} else if (neutral_count < 3U) {
				neutral_count = 0U;
			}
		}

		const bool abort = !pad_ok || (pad.buttons & GAMEPAD_BUTTON_B) != 0U ||
				   (phase != Phase::CommandFree && !fb_ok);
		if (abort) {
			if (transport_started) {
				stopPair();
			}
			wrist.forget();
			phase = now - phase_since < command_free_ms ? Phase::CommandFree
								 : Phase::WaitFeedback;
			neutral_count = 0U;
		} else if (phase == Phase::CommandFree && now - phase_since >= command_free_ms) {
			phase = Phase::WaitFeedback;
			LOG_INF("smoke complete; wait for feedback, neutral x3, then A");
		} else if (phase == Phase::WaitFeedback && fb_ok && neutral_count >= 3U &&
			   (pressed & GAMEPAD_BUTTON_A) != 0U) {
			ret = robomaster_transport_start(transport);
			if (ret == 0 || ret == -EALREADY) {
				transport_started = true;
				(void)robomaster_c610_set_pair_current_a(right_motor, left_motor, 0.0F,
								     0.0F);
				phase = Phase::ZeroProve;
				phase_since = now;
				LOG_INF("zero-current proof started");
			}
		} else if (phase == Phase::ZeroProve) {
			(void)robomaster_c610_set_pair_current_a(right_motor, left_motor, 0.0F, 0.0F);
			if (now - phase_since >= zero_prove_ms &&
			    (pressed & GAMEPAD_BUTTON_Y) != 0U && wrist.seed(angles(config, right, left)) &&
			    wrist.setYawOrigin()) {
				phase = Phase::Homed;
				LOG_INF("wrist reference captured; press X to arm low-output control");
			}
		} else if (phase == Phase::Homed) {
			(void)robomaster_c610_set_pair_current_a(right_motor, left_motor, 0.0F, 0.0F);
			if ((pressed & GAMEPAD_BUTTON_X) != 0U && wrist.seed(angles(config, right, left))) {
				wrist.holdYaw(0.0F);
				phase = Phase::Armed;
				LOG_WRN("ARMED at 0.5 A: hold RT and use D-pad; B stops");
			}
		} else if (phase == Phase::Armed) {
			if (pad.rt >= deadman_min) {
				float lift = 0.0F;
				if ((pad.buttons & GAMEPAD_BUTTON_DPAD_UP) != 0U) {
					lift = 5.0F;
				} else if ((pad.buttons & GAMEPAD_BUTTON_DPAD_DOWN) != 0U) {
					lift = -5.0F;
				}
				(void)wrist.jog(lift, 0.01F);
				if ((pressed & GAMEPAD_BUTTON_DPAD_LEFT) != 0U) {
					wrist.spinBy(-1);
				} else if ((pressed & GAMEPAD_BUTTON_DPAD_RIGHT) != 0U) {
					wrist.spinBy(1);
				}
			}
			wrist.holdYaw(0.0F);
			dscrew::Currents current{};
			if (!wrist.step(angles(config, right, left), rates(config, right, left), 0.01F,
					current) ||
			    robomaster_c610_set_pair_current_a(right_motor, left_motor, current.right_a,
								 current.left_a) != 0) {
				stopPair();
				wrist.forget();
				phase = Phase::WaitFeedback;
			}
		}

		if (now >= next_log) {
			const dscrew::WristPose measured = wrist.measured();
			LOG_INF("phase=%d pad=%d fb=%d R=%lld L=%lld z_mm_x10=%d yaw_mrad=%d",
				static_cast<int>(phase), pad_ok, fb_ok, right.continuous_count,
				left.continuous_count, static_cast<int>(measured.z_mm * 10.0F),
				static_cast<int>(measured.theta_rad * 1000.0F));
			next_log = now + 250;
		}
		k_sleep(K_MSEC(loop_ms));
	}
}
