/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <arm_control/controller.hpp>
#include <arm_control/profile.hpp>
#include <arm_control/site.hpp>
#include <differential_screw/wrist.hpp>
#include <drivers/motor/robomaster.h>
#include <drivers/motor/robstride.h>
#include <drivers/servo.h>
#include <esp_now_gamepad_bridge/gamepad_bridge.h>

#include "input_logic.hpp"

LOG_MODULE_REGISTER(robot_arm, LOG_LEVEL_INF);

namespace
{

constexpr uint32_t control_period_us = 2000U;
constexpr uint32_t wrist_divider = 5U;
constexpr float axis_scale = 1.0F / 127.0F;
#ifdef ROBOT_ARM_BRINGUP
constexpr int64_t command_free_ms = 3000;
constexpr int64_t zero_prove_ms = 1000;
constexpr uint8_t deadman_min = 200U;
#endif

const device *const can_dev = DEVICE_DT_GET(DT_NODELABEL(xl2515));
const device *const robomaster = DEVICE_DT_GET(DT_NODELABEL(robomaster));
const device *const wrist_right = DEVICE_DT_GET(DT_NODELABEL(wrist_right));
const device *const wrist_left = DEVICE_DT_GET(DT_NODELABEL(wrist_left));
const device *const robstride = DEVICE_DT_GET(DT_NODELABEL(robstride));
const device *const shoulder = DEVICE_DT_GET(DT_NODELABEL(shoulder));
const device *const elbow = DEVICE_DT_GET(DT_NODELABEL(elbow));
const device *const servo = DEVICE_DT_GET(DT_ALIAS(servo0));
const device *const gamepad_uart = DEVICE_DT_GET(DT_ALIAS(gamepad_uart));

class Rs00Joint final: public armctl::JointDrive
{
      public:
	Rs00Joint(const device *motor, const device *transport)
		: motor_(motor), transport_(transport)
	{
	}

	bool init() override
	{
		return device_is_ready(motor_);
	}
	bool enterPositionMode() override
	{
		return true;
	}
	bool commandPosition(float rad) override
	{
		return robstride_submit_position(motor_, rad, epoch()) == 0;
	}
	bool readPosition(float &out) const override
	{
		robstride_feedback feedback{};
		if (robstride_get_feedback(motor_, &feedback) != 0 || !feedback.valid ||
		    !feedback.position_continuous || feedback.age_ms > 100) {
			return false;
		}
		out = feedback.position_rad;
		return true;
	}
	bool readTorque(float &out) const override
	{
		robstride_feedback feedback{};
		if (robstride_get_feedback(motor_, &feedback) != 0 || !feedback.valid) {
			return false;
		}
		out = feedback.torque_nm;
		return true;
	}
	bool release() override
	{
		return robstride_request_stop(transport_, epoch() + 1U) == 0;
	}
	bool isAlive() const override
	{
		robstride_feedback feedback{};
		return robstride_get_feedback(motor_, &feedback) == 0 && feedback.valid &&
		       feedback.age_ms <= 100;
	}
	bool isEnergized() const override
	{
		robstride_feedback feedback{};
		return robstride_get_feedback(motor_, &feedback) == 0 && feedback.valid &&
		       feedback.mode_state == ROBSTRIDE_MODE_MOTOR && feedback.fault_bits == 0U;
	}
	bool clearFaults() override
	{
		return robstride_clear_fault(motor_, epoch()) == 0;
	}

      private:
	uint32_t epoch() const
	{
		robstride_transport_stats stats{};
		return robstride_transport_get_stats(transport_, &stats) == 0
			       ? stats.operation_epoch
			       : 0U;
	}

	const device *motor_;
	const device *transport_;
};

bool prepareAndEnable(armctl::Controller &controller)
{
	robstride_transport_stats stats{};
	if (robstride_transport_get_stats(robstride, &stats) != 0 || !controller.clearFault()) {
		return false;
	}
	robstride_csp_config config{};
	config.can_timeout_ms = armctl::can_timeout_ms;
#ifdef ROBOT_ARM_BRINGUP
	config.speed_limit_rad_s = 1.0F;
	config.current_limit_a = 1.0F;
#else
	config.speed_limit_rad_s = armctl::joint_speed_limit_rad_s;
	config.current_limit_a = armctl::joint_current_limit_a;
#endif
	const uint32_t epoch = stats.operation_epoch;
	if (robstride_prepare_csp(shoulder, &config, epoch) != 0 ||
	    robstride_prepare_csp(elbow, &config, epoch) != 0 ||
	    robstride_commit_enable(shoulder, epoch) != 0 ||
	    robstride_commit_enable(elbow, epoch) != 0 || !controller.seedFromDrives()) {
		controller.emergencyStop();
		(void)robstride_request_stop(robstride, epoch + 1U);
		return false;
	}
	controller.hold();
	return true;
}

bool ready(const device *dev)
{
	if (!device_is_ready(dev)) {
		LOG_ERR("device %s is not ready", dev->name);
		return false;
	}
	return true;
}

#ifdef ROBOT_ARM_BRINGUP
dscrew::Config bringupWristConfig()
{
	dscrew::Config config{};
	config.current_limit_a = 0.5F;
	config.lift_rate_mm_s = -5.0F;
	config.spin_step_rad = -5.0F * dscrew::pi / 180.0F;
	config.stall_hold_a = 0.20F;
	return config;
}
#endif

} // namespace

int main()
{
	if (!ready(can_dev) || !ready(robomaster) || !ready(wrist_right) || !ready(wrist_left) ||
	    !ready(robstride) || !ready(shoulder) || !ready(elbow) || !ready(servo) ||
	    !ready(gamepad_uart)) {
		return -ENODEV;
	}
	int ret = can_start(can_dev);
	if (ret != 0 && ret != -EALREADY) {
		LOG_ERR("CAN start failed (%d)", ret);
		return ret;
	}
	if (
#ifndef ROBOT_ARM_BRINGUP
	    robomaster_transport_start(robomaster) != 0 ||
#endif
	    robstride_transport_start(robstride) != 0 || gamepad_bridge_init(gamepad_uart) != 0 ||
	    gamepad_bridge_start() != 0) {
		LOG_ERR("transport startup failed");
		return -EIO;
	}

	Rs00Joint shoulder_joint(shoulder, robstride);
	Rs00Joint elbow_joint(elbow, robstride);
	armctl::Space space;
	if (!armctl::buildWorkspace(space)) {
		return -EINVAL;
	}
	armctl::JogLimits jog{};
#ifdef ROBOT_ARM_BRINGUP
	jog.tool_v_max = 20.0F;
	jog.tool_a_max = 50.0F;
	jog.joint_v_max = 0.10F;
	jog.joint_a_max = 0.30F;
	jog.lift_mm_s = 5.0F;
#else
	jog.tool_v_max = armctl::jog_v_max;
	jog.tool_a_max = armctl::jog_a_max;
	jog.joint_v_max = armctl::joint_jog_v_max;
	jog.joint_a_max = armctl::joint_jog_a_max;
	jog.lift_mm_s = 20.0F;
#endif
	armctl::Controller controller(shoulder_joint, elbow_joint, armctl::kinematicsConfig(),
				      armctl::transmissionConfig(), space, jog);
	controller.setTorqueLimit(armctl::over_torque_nm);
#ifndef ROBOT_ARM_BRINGUP
	controller.emergencyStop();
#endif

#ifdef ROBOT_ARM_BRINGUP
	dscrew::Wrist wrist(bringupWristConfig());
	const int64_t boot_ms = k_uptime_get();
	int64_t zero_since_ms = -1;
	bool smoke_complete = false;
#else
	dscrew::Wrist wrist;
#endif
	robot_arm::InputLogic input_logic;
	robot_arm::TipServo tip_servo;
	uint32_t tick = 0U;
	bool enabled = false;
#ifdef ROBOT_ARM_ACCEPTANCE
	uint16_t acceptance_previous_buttons = 0U;
	uint32_t injected_stops = 0U;
	uint32_t deadline_misses = 0U;
	uint32_t max_work_us = 0U;
	int64_t next_diagnostic_ms = 0;
#endif

#ifdef ROBOT_ARM_BRINGUP
	LOG_INF("command-free smoke for %lld ms; outputs have not been commanded", command_free_ms);
#else
	LOG_INF("ready, outputs stopped; neutral x3 then Y home and A enable");
#endif
	while (true) {
#ifdef ROBOT_ARM_ACCEPTANCE
		const uint32_t work_start_cycles = k_cycle_get_32();
#endif
#ifdef ROBOT_ARM_BRINGUP
		const int64_t now_ms = k_uptime_get();
		if (!smoke_complete) {
			if (now_ms - boot_ms < command_free_ms) {
				k_sleep(K_USEC(control_period_us));
				continue;
			}
			if (robomaster_transport_start(robomaster) != 0) {
				LOG_ERR("C610 transport start failed");
				return -EIO;
			}
			controller.emergencyStop();
			(void)robomaster_c610_set_pair_current_a(wrist_right, wrist_left, 0.0F,
								 0.0F);
			zero_since_ms = now_ms;
			smoke_complete = true;
			LOG_INF("smoke complete; proving zero outputs for %lld ms", zero_prove_ms);
		}
#endif
		gamepad_state pad{};
		if (gamepad_bridge_get_state(&pad) != 0) {
			controller.emergencyStop();
			(void)robomaster_c610_set_pair_current_a(wrist_right, wrist_left, 0.0F,
								 0.0F);
			enabled = false;
			k_sleep(K_USEC(control_period_us));
			continue;
		}
		const robot_arm::Actions actions = input_logic.update(
			{pad.connected, pad.seq, pad.buttons}, controller.isHalted());
		if (actions.stop) {
			controller.emergencyStop();
			(void)robomaster_c610_set_pair_current_a(wrist_right, wrist_left, 0.0F,
								 0.0F);
			enabled = false;
		}
#ifdef ROBOT_ARM_ACCEPTANCE
		const uint16_t acceptance_pressed = pad.buttons & ~acceptance_previous_buttons;
		acceptance_previous_buttons = pad.buttons;
		if ((pad.buttons & GAMEPAD_BUTTON_VIEW) != 0U &&
		    (acceptance_pressed & GAMEPAD_BUTTON_X) != 0U) {
			controller.emergencyStop();
			(void)robomaster_c610_set_pair_current_a(wrist_right, wrist_left, 0.0F,
								 0.0F);
			enabled = false;
			++injected_stops;
			LOG_WRN("injected safe stop %u; re-home before re-enable", injected_stops);
		}
		if ((pad.buttons & GAMEPAD_BUTTON_VIEW) != 0U &&
		    (acceptance_pressed & GAMEPAD_BUTTON_MENU) != 0U) {
			deadline_misses = 0U;
			max_work_us = 0U;
			LOG_INF("acceptance timing counters cleared");
		}
#endif

		robomaster_feedback_ex right_fb{};
		robomaster_feedback_ex left_fb{};
		const bool wrist_feedback =
			robomaster_get_feedback_ex(wrist_right, &right_fb) == 0 &&
			robomaster_get_feedback_ex(wrist_left, &left_fb) == 0 &&
			right_fb.continuous_valid && left_fb.continuous_valid &&
			right_fb.raw.online && left_fb.raw.online;
#ifdef ROBOT_ARM_BRINGUP
		if (enabled && !wrist_feedback) {
			controller.emergencyStop();
			(void)robomaster_c610_set_pair_current_a(wrist_right, wrist_left, 0.0F,
								 0.0F);
			enabled = false;
			LOG_ERR("wrist feedback lost; all motor outputs stopped");
		}
#endif
		dscrew::NutAngles nuts{};
		if (wrist_feedback) {
			nuts.phi_r =
				dscrew::nutRadFromCounts(wrist.config(), right_fb.continuous_count,
							 wrist.config().right_direction);
			nuts.phi_l =
				dscrew::nutRadFromCounts(wrist.config(), left_fb.continuous_count,
							 wrist.config().left_direction);
			if (!wrist.isSeeded()) {
				(void)wrist.seed(nuts);
			}
		}
		if (actions.home && wrist_feedback) {
			(void)controller.setTransmission(armctl::transmissionConfig());
			(void)wrist.setYawOrigin();
			LOG_INF("home accepted while stopped");
		}
		if (actions.enable
#ifdef ROBOT_ARM_BRINGUP
		    && now_ms - zero_since_ms >= zero_prove_ms
#endif
		) {
			enabled = prepareAndEnable(controller) && wrist_feedback;
			LOG_INF("enable %s", enabled ? "accepted" : "rejected");
		}

		if (actions.servo_steps != 0
#ifdef ROBOT_ARM_BRINGUP
		    && enabled && pad.lt >= deadman_min && pad.rt < deadman_min
#endif
		) {
			(void)tip_servo.commandSteps(actions.servo_steps);
			(void)servo_set_pulse(servo, tip_servo.pulseUs());
		}

		if (enabled && pad.connected && input_logic.mode() == robot_arm::Mode::Hold
#ifdef ROBOT_ARM_BRINGUP
		    && pad.rt >= deadman_min && pad.lt < deadman_min
#endif
		) {
			controller.setToolVelocity(
				{static_cast<float>(pad.lx) * axis_scale * jog.tool_v_max,
				 -static_cast<float>(pad.ly) * axis_scale * jog.tool_v_max});
			(void)controller.startToolJog();
		} else {
			controller.setToolVelocity({});
		}
		const armctl::Fault fault = controller.step(armctl::control_dt);
		if (fault != armctl::Fault::None) {
			enabled = false;
		}

		if (++tick % wrist_divider == 0U) {
			dscrew::Currents currents{};
			if (enabled && wrist_feedback
#ifdef ROBOT_ARM_BRINGUP
			    && pad.rt >= deadman_min && pad.lt < deadman_min
#endif
			) {
				const float lift =
					actions.wrist_lift_up     ? wrist.config().lift_rate_mm_s
					: actions.wrist_lift_down ? -wrist.config().lift_rate_mm_s
								  : 0.0F;
				(void)wrist.jog(lift, wrist.config().command_period_s);
				wrist.spinBy(actions.wrist_spin_steps);
				wrist.holdYaw(controller.joints().theta2);
				const dscrew::NutRates rates{
					dscrew::nutRadPerSecFromRpm(wrist.config(),
								    right_fb.raw.velocity,
								    wrist.config().right_direction),
					dscrew::nutRadPerSecFromRpm(wrist.config(),
								    left_fb.raw.velocity,
								    wrist.config().left_direction)};
				if (!wrist.step(nuts, rates, wrist.config().command_period_s,
						currents)) {
					enabled = false;
				}
			}
			if (robomaster_c610_set_pair_current_a(wrist_right, wrist_left,
							       currents.right_a,
							       currents.left_a) != 0) {
				enabled = false;
			}
		}
#ifdef ROBOT_ARM_ACCEPTANCE
		const uint32_t work_us = k_cyc_to_us_floor32(k_cycle_get_32() - work_start_cycles);
		max_work_us = work_us > max_work_us ? work_us : max_work_us;
		if (work_us > control_period_us) {
			++deadline_misses;
		}
		if (k_uptime_get() >= next_diagnostic_ms) {
			robomaster_transport_stats rm_stats{};
			robstride_transport_stats rs_stats{};
			robstride_feedback shoulder_fb{};
			robstride_feedback elbow_fb{};
			(void)robomaster_transport_get_stats(robomaster, &rm_stats);
			(void)robstride_transport_get_stats(robstride, &rs_stats);
			(void)robstride_get_feedback(shoulder, &shoulder_fb);
			(void)robstride_get_feedback(elbow, &elbow_fb);
			LOG_INF("accept enabled=%d miss=%u max_us=%u injected=%u "
				"rm_epoch=%u rm_err=%u rm_to=%u R_ce=%u L_ce=%u "
				"rs_epoch=%u rs_err=%u rs_rej=%u S_age=%lld E_age=%lld",
				enabled, deadline_misses, max_work_us, injected_stops,
				rm_stats.operation_epoch, rm_stats.tx_errors,
				rm_stats.command_timeouts, right_fb.continuity_epoch,
				left_fb.continuity_epoch, rs_stats.operation_epoch, rs_stats.tx_errors,
				rs_stats.rejected_commands, shoulder_fb.age_ms, elbow_fb.age_ms);
			next_diagnostic_ms = k_uptime_get() + 1000;
		}
#endif
		k_sleep(K_USEC(control_period_us));
	}
}
