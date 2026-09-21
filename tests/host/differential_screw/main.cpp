/* SPDX-License-Identifier: Apache-2.0 */

#include <differential_screw/wrist.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

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

void near(float actual, float expected, float tolerance, const char *message)
{
	check(std::fabs(actual - expected) <= tolerance, message);
}

} // namespace

int main()
{
	using namespace dscrew;

	const Config config;
	check(config.valid(), "default configuration");
	const WristPose pose{0.7F, 10.0F};
	const NutAngles nuts = wristInverse(config, pose);
	near(nuts.phi_r, 0.7F - config.alpha_r_rad_mm * 10.0F, 1e-6F, "inverse right");
	near(nuts.phi_l, 0.7F + config.alpha_l_rad_mm * 10.0F, 1e-6F, "inverse left");
	const WristPose roundtrip = wristForward(config, nuts);
	near(roundtrip.theta_rad, pose.theta_rad, 1e-5F, "roundtrip theta");
	near(roundtrip.z_mm, pose.z_mm, 1e-4F, "roundtrip lift");

	near(nutRadFromCounts(config, 8192LL * 36LL, 1.0F), 2.0F * pi, 1e-5F, "one nut revolution");
	near(nutRadFromCounts(config, 8192LL * 36LL, -1.0F), -2.0F * pi, 1e-5F,
	     "direction applies to angle");
	near(nutRadPerSecFromRpm(config, 2160, 1.0F), 2.0F * pi, 1e-5F, "rpm conversion");
	near(nutRadPerSecFromRpm(config, 2160, -1.0F), -2.0F * pi, 1e-5F,
	     "direction applies to rate");

	Config invalid = config;
	invalid.total_ratio = 0.0F;
	check(!invalid.valid(), "zero ratio rejected");
	invalid = config;
	invalid.alpha_l_rad_mm *= 1.1F;
	check(!invalid.valid(), "asymmetric alpha rejected");
	invalid = config;
	invalid.current_limit_a = std::numeric_limits<float>::quiet_NaN();
	check(!invalid.valid(), "nonfinite config rejected");

	Wrist unseeded;
	Currents currents{9.0F, 9.0F};
	check(unseeded.step({0.0F, 0.0F}, {0.0F, 0.0F}, 0.01F, currents),
	      "unseeded measurement accepted");
	near(currents.right_a, 0.0F, 0.0F, "unseeded right zero");
	near(currents.left_a, 0.0F, 0.0F, "unseeded left zero");
	check(!unseeded.step({NAN, 0.0F}, {}, 0.01F, currents), "NaN measurement rejected");
	near(currents.right_a, 0.0F, 0.0F, "invalid input right zero");

	Wrist wrist;
	check(wrist.seed({1.2F, 1.2F}), "seed");
	near(wrist.target().theta_rad, 1.2F, 1e-6F, "seed captures present theta");
	near(wrist.target().z_mm, 0.0F, 1e-6F, "seed captures present lift");
	wrist.holdYaw(0.3F);
	const float first_goal = wrist.goalAngle();
	wrist.spinBy(4);
	wrist.holdYaw(0.3F);
	near(wrist.goalAngle() - first_goal, -2.0F * pi, 1e-4F, "four steps retain one turn");
	check(wrist.setYawOrigin(), "yaw origin after measurement");
	check(wrist.isYawHomed(), "yaw homed");
	check(!wrist.hasYawGoal(), "origin discards old yaw goal");
	wrist.forget();
	check(!wrist.isSeeded(), "forget seed");
	check(!wrist.hasYawGoal(), "forget yaw goal");

	Wrist leashed;
	check(leashed.seed({0.0F, 0.0F}), "leash seed");
	check(leashed.jog(-120.0F, 0.01F), "jog 1");
	check(leashed.jog(-120.0F, 0.01F), "jog 2");
	check(leashed.jog(-120.0F, 0.01F), "jog 3");
	near(leashed.target().z_mm, -3.2F, 1e-5F, "500 Hz side uses loose leash");
	check(leashed.step({0.0F, 0.0F}, {}, 0.01F, currents), "step leash");
	near(leashed.target().z_mm, -2.0F, 1e-5F, "100 Hz step uses strict leash");

	Wrist damping;
	check(damping.seed({0.0F, 0.0F}), "damping seed");
	check(damping.step({0.0F, 0.0F}, {1.0F, 1.0F}, 0.01F, currents), "common damping");
	near(currents.right_a, -1.0F, 1e-5F, "spin damping right");
	near(currents.left_a, -1.0F, 1e-5F, "spin damping left");
	check(damping.step({0.0F, 0.0F}, {-1.0F, 1.0F}, 0.01F, currents), "differential damping");
	near(currents.right_a, config.kd_lift_a_per_rad_s, 1e-5F, "lift damping right");
	near(currents.left_a, -config.kd_lift_a_per_rad_s, 1e-5F, "lift damping left");

	Wrist stalled;
	check(stalled.seed({0.0F, 0.0F}), "stall seed");
	stalled.holdYaw(0.0F);
	stalled.spinBy(1);
	stalled.holdYaw(0.0F);
	for (int i = 0; i < 51; ++i) {
		check(stalled.step({0.0F, 0.0F}, {}, 0.01F, currents), "stall step");
	}
	check(stalled.isStalled(), "stall latched after 0.5 s");
	near(std::fabs(currents.right_a), config.stall_hold_a, 1e-4F, "stall relief right");
	near(std::fabs(currents.left_a), config.stall_hold_a, 1e-4F, "stall relief left");
	for (int i = 0; i < 500; ++i) {
		check(stalled.step({0.0F, 0.0F}, {}, 0.01F, currents), "finite retry step");
	}
	near(std::fabs(currents.right_a), config.stall_hold_a, 1e-4F,
	     "finite retries settle at relief current");

	std::cout << "PASS: " << checks << " checks\n";
	return 0;
}
