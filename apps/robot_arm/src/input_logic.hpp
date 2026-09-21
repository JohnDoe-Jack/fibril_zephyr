/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <cstdint>

namespace robot_arm
{

enum class Mode : uint8_t {
	Hold,
	WristJog
};

struct Input {
	bool connected;
	uint8_t sequence;
	uint16_t buttons;
};

struct Actions {
	bool stop = false;
	bool enable = false;
	bool home = false;
	bool mode_changed = false;
	bool wrist_lift_up = false;
	bool wrist_lift_down = false;
	int wrist_spin_steps = 0;
	int servo_steps = 0;
};

class InputLogic
{
      public:
	Actions update(Input input, bool stopped);
	Mode mode() const
	{
		return mode_;
	}
	bool neutralReady() const
	{
		return neutral_count_ >= 3;
	}

      private:
	static constexpr uint16_t directional_mask = 0x000f;
	static constexpr uint16_t action_mask = 0x03f0;
	uint16_t previous_buttons_ = 0;
	uint8_t previous_sequence_ = 0;
	uint8_t neutral_count_ = 0;
	bool have_sequence_ = false;
	Mode mode_ = Mode::Hold;
};

} // namespace robot_arm
