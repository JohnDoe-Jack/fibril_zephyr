/* SPDX-License-Identifier: Apache-2.0 */
#include "input_logic.hpp"

#include <cstdlib>
#include <iostream>

namespace
{
int checks;
void check(bool condition, const char *message)
{
	++checks;
	if (!condition) {
		std::cerr << "FAIL: " << message << '\n';
		std::exit(1);
	}
}
} // namespace

int main()
{
	using robot_arm::InputLogic;
	InputLogic input;
	check(input.update({false, 0, 0}, true).stop, "disconnect stops");
	check(!input.update({true, 1, 1U << 7}, true).enable, "A blocked before neutral");
	check(!input.update({true, 2, 0}, true).enable, "neutral 1");
	check(!input.update({true, 3, 0}, true).enable, "neutral 2");
	check(!input.update({true, 4, 0}, true).enable, "neutral 3");
	check(input.neutralReady(), "neutral gate opens");
	check(input.update({true, 5, 1U << 7}, true).enable, "A edge enables while stopped");
	check(!input.update({true, 5, 1U << 7}, true).enable, "duplicate sequence has no edge");
	check(input.update({true, 6, (1U << 6) | (1U << 7)}, true).stop, "B level overrides A");

	InputLogic mode;
	(void)mode.update({true, 1, 0}, true);
	(void)mode.update({true, 2, 0}, true);
	(void)mode.update({true, 3, 0}, true);
	check(mode.update({true, 4, 1U << 13}, true).mode_changed, "Menu toggles mode");
	check(mode.mode() == robot_arm::Mode::WristJog, "wrist mode selected");
	(void)mode.update({true, 5, 0}, true);
	check(mode.update({true, 6, 1U << 1}, false).wrist_lift_up, "D-pad jogs wrist");
	check(mode.update({true, 7, 1U << 0}, false).wrist_spin_steps == -1, "left spins one step");
	(void)mode.update({true, 8, 0}, false);
	check(mode.update({true, 9, 1U << 8}, false).servo_steps == -1, "LB starts servo");
	check(mode.update({true, 10, 1U << 8}, false).servo_steps == 0, "held LB does not repeat");
	check(mode.update({true, 11, 0}, false).servo_steps == 0, "release is quiet");
	check(mode.update({true, 12, 1U << 9}, false).servo_steps == 1, "RB starts servo");
	check(!mode.update({true, 13, 1U << 5}, false).home, "home blocked while running");
	(void)mode.update({true, 14, 0}, true);
	check(mode.update({true, 15, 1U << 5}, true).home, "home allowed while stopped");

	std::cout << "PASS: " << checks << " checks\n";
	return 0;
}
