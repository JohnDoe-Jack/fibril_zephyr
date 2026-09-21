/* SPDX-License-Identifier: Apache-2.0 */
#include "input_logic.hpp"
#include <differential_screw/wrist.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace
{
int checks;
void check(bool value, const char *message)
{
	++checks;
	if (!value) {
		std::cerr << "FAIL: " << message << '\n';
		std::exit(1);
	}
}
} // namespace

int main()
{
	robot_arm::InputLogic input;
	robot_arm::TipServo servo;
	dscrew::Wrist wrist;
	dscrew::Currents current{};

	check(!servo.active() && servo.pulseUs() == 1210U, "PWM is dormant at boot");
	(void)input.update({true, 1, 0}, true);
	(void)input.update({true, 2, 0}, true);
	(void)input.update({true, 3, 0}, true);
	check(input.neutralReady(), "three fresh neutral frames unlock commands");

	check(servo.commandSteps(1), "new RB action starts PWM");
	check(servo.active() && servo.pulseUs() == 1215U, "servo advances by 5 us");
	const uint32_t held = servo.pulseUs();
	check(input.update({false, 4, 0}, false).stop, "link loss requests motor stop");
	check(servo.pulseUs() == held, "link loss preserves servo HOLD_LAST");

	check(wrist.seed({0.0F, 0.0F}), "continuous wrist feedback seeds controller");
	wrist.holdYaw(0.0F);
	wrist.spinBy(1);
	wrist.holdYaw(0.0F);
	check(wrist.step({0.0F, 0.0F}, {}, 0.01F, current), "wrist produces command");
	check(std::fabs(current.right_a) <= 4.0F && std::fabs(current.left_a) <= 4.0F,
	      "mixed runtime respects wrist current limit");

	const auto stop =
		input.update({true, 5, static_cast<uint16_t>((1U << 6) | (1U << 9))}, false);
	check(stop.stop && stop.servo_steps == 0, "B level wins over simultaneous servo action");
	current = {};
	check(current.right_a == 0.0F && current.left_a == 0.0F,
	      "stop path emits zero wrist current");
	check(servo.pulseUs() == held, "motor stop does not center tip servo");

	for (int i = 0; i < 100; ++i) {
		(void)servo.commandSteps(1);
	}
	check(servo.pulseUs() == 1400U, "tip upper clamp");
	for (int i = 0; i < 200; ++i) {
		(void)servo.commandSteps(-1);
	}
	check(servo.pulseUs() == 1000U, "tip lower clamp");

	std::cout << "PASS: " << checks << " checks\n";
	return 0;
}
