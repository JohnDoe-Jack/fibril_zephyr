/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <cstdlib>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <arm_control/controller.hpp>
#include <arm_control/profile.hpp>
#include <arm_control/site.hpp>
#include <drivers/motor/robstride.h>
#include <esp_now_gamepad_bridge/gamepad_bridge.h>

LOG_MODULE_REGISTER(arm_bringup, LOG_LEVEL_INF);

namespace
{
constexpr int command_free_ms = 3000;
constexpr uint8_t deadman_min = 200U;
constexpr int axis_neutral = 5;
constexpr float axis_scale = 1.0F / 127.0F;

const device *const can_dev = DEVICE_DT_GET(DT_NODELABEL(xl2515));
const device *const transport = DEVICE_DT_GET(DT_NODELABEL(robstride));
const device *const shoulder_motor = DEVICE_DT_GET(DT_NODELABEL(shoulder));
const device *const elbow_motor = DEVICE_DT_GET(DT_NODELABEL(elbow));
const device *const gamepad_uart = DEVICE_DT_GET(DT_ALIAS(gamepad_uart));

enum class Phase { CommandFree, WaitStopFeedback, Homed, Armed };
enum class JogMode { Joint, Tool };

uint32_t epoch()
{
	robstride_transport_stats stats{};
	return robstride_transport_get_stats(transport, &stats) == 0 ? stats.operation_epoch : 0U;
}

class Rs00Joint final : public armctl::JointDrive
{
      public:
	explicit Rs00Joint(const device *motor) : motor_(motor) {}
	bool init() override { return device_is_ready(motor_); }
	bool enterPositionMode() override { return true; }
	bool commandPosition(float rad) override
	{
		return robstride_submit_position(motor_, rad, epoch()) == 0;
	}
	bool readPosition(float &out) const override
	{
		robstride_feedback feedback{};
		if (robstride_get_feedback(motor_, &feedback) != 0 || !feedback.valid ||
		    !feedback.fresh || !feedback.position_continuous || feedback.age_ms > 100) {
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
	bool release() override { return robstride_request_stop(transport, epoch() + 1U) == 0; }
	bool isAlive() const override
	{
		float ignored = 0.0F;
		return readPosition(ignored);
	}
	bool isEnergized() const override
	{
		robstride_feedback feedback{};
		return robstride_get_feedback(motor_, &feedback) == 0 && feedback.valid &&
		       feedback.mode_state == ROBSTRIDE_MODE_MOTOR && feedback.fault_bits == 0U;
	}
	bool clearFaults() override { return robstride_clear_fault(motor_, epoch()) == 0; }

      private:
	const device *motor_;
};

bool neutral(const gamepad_state &pad)
{
	return pad.buttons == 0U && std::abs(pad.lx) <= axis_neutral &&
	       std::abs(pad.ly) <= axis_neutral && std::abs(pad.rx) <= axis_neutral &&
	       std::abs(pad.ry) <= axis_neutral && pad.lt == 0U && pad.rt == 0U;
}

bool feedbackReady(robstride_feedback &shoulder, robstride_feedback &elbow)
{
	return robstride_get_feedback(shoulder_motor, &shoulder) == 0 &&
	       robstride_get_feedback(elbow_motor, &elbow) == 0 && shoulder.valid && elbow.valid &&
	       shoulder.fresh && elbow.fresh && shoulder.position_continuous &&
	       elbow.position_continuous && shoulder.fault_bits == 0U && elbow.fault_bits == 0U;
}

bool prepareAndEnable(armctl::Controller &controller)
{
	if (!controller.clearFault()) {
		return false;
	}
	robstride_csp_config config{};
	config.can_timeout_ms = 100U;
	config.speed_limit_rad_s = 1.0F;
	config.current_limit_a = 1.0F;
	const uint32_t op_epoch = epoch();
	if (robstride_prepare_csp(shoulder_motor, &config, op_epoch) != 0 ||
	    robstride_prepare_csp(elbow_motor, &config, op_epoch) != 0 ||
	    robstride_commit_enable(shoulder_motor, op_epoch) != 0 ||
	    robstride_commit_enable(elbow_motor, op_epoch) != 0 || !controller.seedFromDrives()) {
		controller.emergencyStop();
		return false;
	}
	controller.hold();
	return true;
}
} // namespace

int main()
{
	if (!device_is_ready(can_dev) || !device_is_ready(transport) ||
	    !device_is_ready(shoulder_motor) || !device_is_ready(elbow_motor) ||
	    !device_is_ready(gamepad_uart)) {
		LOG_ERR("required device is not ready");
		return -ENODEV;
	}
	int ret = can_start(can_dev);
	if (ret != 0 && ret != -EALREADY) {
		return ret;
	}
	if (robstride_transport_start(transport) != 0 || gamepad_bridge_init(gamepad_uart) != 0 ||
	    gamepad_bridge_start() != 0) {
		LOG_ERR("transport startup failed");
		return -EIO;
	}

	Rs00Joint shoulder(shoulder_motor);
	Rs00Joint elbow(elbow_motor);
	armctl::Space space;
	if (!armctl::buildWorkspace(space)) {
		return -EINVAL;
	}
	armctl::JogLimits limits{};
	limits.tool_v_max = 20.0F;
	limits.tool_a_max = 50.0F;
	limits.joint_v_max = 0.10F;
	limits.joint_a_max = 0.30F;
	limits.lift_mm_s = 5.0F;
	armctl::Controller controller(shoulder, elbow, armctl::kinematicsConfig(),
				      armctl::transmissionConfig(), space, limits);

	Phase phase = Phase::CommandFree;
	JogMode jog_mode = JogMode::Joint;
	const int64_t boot_ms = k_uptime_get();
	int64_t next_log = 0;
	uint16_t previous_buttons = 0U;
	uint8_t previous_seq = 0U;
	uint8_t neutral_count = 0U;
	bool have_seq = false;
	bool stop_requested = false;
	LOG_INF("command-free smoke for %d ms; no stop or position command is sent", command_free_ms);

	while (true) {
		gamepad_state pad{};
		robstride_feedback shoulder_fb{};
		robstride_feedback elbow_fb{};
		const int64_t now = k_uptime_get();
		const bool fb_ok = feedbackReady(shoulder_fb, elbow_fb);
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

		if (phase == Phase::CommandFree && now - boot_ms >= command_free_ms) {
			phase = Phase::WaitStopFeedback;
			LOG_INF("smoke complete; neutral x3 then A requests released state");
		}
		if (phase != Phase::CommandFree &&
		    (!pad_ok || (pad.buttons & GAMEPAD_BUTTON_B) != 0U ||
		     (phase == Phase::Armed && !fb_ok))) {
			controller.emergencyStop();
			stop_requested = true;
			phase = Phase::WaitStopFeedback;
			neutral_count = 0U;
		}
		if (phase == Phase::WaitStopFeedback && neutral_count >= 3U &&
		    (pressed & GAMEPAD_BUTTON_A) != 0U) {
			controller.emergencyStop();
			stop_requested = true;
			LOG_INF("new-epoch stop requested; wait for feedback then Y at physical home");
		}
		if (phase == Phase::WaitStopFeedback && stop_requested && fb_ok &&
		    controller.isReleased() && (pressed & GAMEPAD_BUTTON_Y) != 0U &&
		    controller.setTransmission(armctl::transmissionConfig())) {
			phase = Phase::Homed;
			LOG_INF("home accepted while released; press X for limited hold");
		}
		if (phase == Phase::Homed && (pressed & GAMEPAD_BUTTON_X) != 0U) {
			if (prepareAndEnable(controller)) {
				phase = Phase::Armed;
				jog_mode = JogMode::Joint;
				LOG_WRN("ARMED: joint jog selected; hold RT; Menu toggles tool jog");
			} else {
				controller.emergencyStop();
				phase = Phase::WaitStopFeedback;
			}
		}

		if (phase == Phase::Armed) {
			if ((pressed & GAMEPAD_BUTTON_MENU) != 0U) {
				jog_mode = jog_mode == JogMode::Joint ? JogMode::Tool : JogMode::Joint;
				controller.hold();
				LOG_INF("jog mode=%s", jog_mode == JogMode::Joint ? "joint" : "tool");
			}
			if (jog_mode == JogMode::Joint) {
				arm::JointVels velocity{};
				if (pad.rt >= deadman_min) {
					if ((pad.buttons & GAMEPAD_BUTTON_DPAD_UP) != 0U) {
						velocity.dq1 = limits.joint_v_max;
					} else if ((pad.buttons & GAMEPAD_BUTTON_DPAD_DOWN) != 0U) {
						velocity.dq1 = -limits.joint_v_max;
					}
					if ((pad.buttons & GAMEPAD_BUTTON_DPAD_RIGHT) != 0U) {
						velocity.dq2 = limits.joint_v_max;
					} else if ((pad.buttons & GAMEPAD_BUTTON_DPAD_LEFT) != 0U) {
						velocity.dq2 = -limits.joint_v_max;
					}
				}
				if (controller.mode() != armctl::Mode::JointJog) {
					(void)controller.startJointJog();
				}
				controller.setJointVelocity(velocity);
			} else {
				arm::Velocity velocity{};
				if (pad.rt >= deadman_min) {
					velocity.vx = static_cast<float>(pad.lx) * axis_scale * limits.tool_v_max;
					velocity.vy = -static_cast<float>(pad.ly) * axis_scale * limits.tool_v_max;
				}
				if (controller.mode() != armctl::Mode::ToolJog) {
					const armctl::Fault start = controller.startToolJog();
					if (start != armctl::Fault::None) {
						LOG_ERR("tool jog rejected fault=%d", static_cast<int>(start));
					}
				}
				controller.setToolVelocity(velocity);
			}
		}

		if (stop_requested || phase == Phase::Armed) {
			const armctl::Fault fault = controller.step(armctl::control_dt);
			if (phase == Phase::Armed && fault != armctl::Fault::None) {
				LOG_ERR("controller fault=%d; releasing", static_cast<int>(fault));
				controller.emergencyStop();
				phase = Phase::WaitStopFeedback;
			}
		}
		if (now >= next_log) {
			LOG_INF("phase=%d mode=%d epoch=%u released=%d fb=%d q1_mrad=%d q2_mrad=%d",
				static_cast<int>(phase), static_cast<int>(jog_mode), epoch(),
				controller.isReleased(), fb_ok,
				static_cast<int>(shoulder_fb.position_rad * 1000.0F),
				static_cast<int>(elbow_fb.position_rad * 1000.0F));
			next_log = now + 250;
		}
		k_sleep(K_USEC(armctl::control_period_us));
	}
}
