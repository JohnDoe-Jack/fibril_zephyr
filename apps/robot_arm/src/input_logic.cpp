/* SPDX-License-Identifier: Apache-2.0 */
#include "input_logic.hpp"

namespace robot_arm
{

namespace
{
constexpr uint16_t left = 1U << 0;
constexpr uint16_t up = 1U << 1;
constexpr uint16_t right = 1U << 2;
constexpr uint16_t down = 1U << 3;
constexpr uint16_t y = 1U << 5;
constexpr uint16_t b = 1U << 6;
constexpr uint16_t a = 1U << 7;
constexpr uint16_t lb = 1U << 8;
constexpr uint16_t rb = 1U << 9;
constexpr uint16_t menu = 1U << 13;
} // namespace

Actions InputLogic::update(Input input, bool stopped)
{
	Actions out{};
	if (!input.connected) {
		neutral_count_ = 0;
		have_sequence_ = false;
		previous_buttons_ = 0;
		out.stop = true;
		return out;
	}

	const bool fresh = !have_sequence_ || input.sequence != previous_sequence_;
	if (!fresh) {
		if ((input.buttons & b) != 0U) {
			out.stop = true;
		}
		return out;
	}
	have_sequence_ = true;
	previous_sequence_ = input.sequence;

	const uint16_t pressed = input.buttons & ~previous_buttons_;
	previous_buttons_ = input.buttons;
	if ((input.buttons & b) != 0U) {
		neutral_count_ = 0;
		out.stop = true;
		return out;
	}

	if ((input.buttons & (directional_mask | action_mask | menu)) == 0U) {
		if (neutral_count_ < 3U) {
			++neutral_count_;
		}
	} else if (!neutralReady()) {
		neutral_count_ = 0;
	}
	if (!neutralReady()) {
		return out;
	}

	if ((pressed & menu) != 0U) {
		mode_ = mode_ == Mode::Hold ? Mode::WristJog : Mode::Hold;
		out.mode_changed = true;
	}
	out.enable = (pressed & a) != 0U && stopped;
	out.home = (pressed & y) != 0U && stopped;
	if (mode_ == Mode::WristJog) {
		out.wrist_lift_up = (input.buttons & up) != 0U && (input.buttons & down) == 0U;
		out.wrist_lift_down = (input.buttons & down) != 0U && (input.buttons & up) == 0U;
		if ((pressed & left) != 0U) {
			out.wrist_spin_steps = -1;
		} else if ((pressed & right) != 0U) {
			out.wrist_spin_steps = 1;
		}
	}
	if ((pressed & lb) != 0U) {
		out.servo_steps = -1;
	} else if ((pressed & rb) != 0U) {
		out.servo_steps = 1;
	}
	return out;
}

} // namespace robot_arm
