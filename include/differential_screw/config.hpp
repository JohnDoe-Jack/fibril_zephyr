/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <cmath>

namespace dscrew
{

inline constexpr float pi = 3.14159265358979323846F;

struct Config {
	float lead_mm = 100.0F;
	float alpha_r_rad_mm = 2.0F * pi / 100.0F;
	float alpha_l_rad_mm = 2.0F * pi / 100.0F;
	float counts_per_rotor_rev = 8192.0F;
	float total_ratio = 36.0F;
	float right_direction = 1.0F;
	float left_direction = 1.0F;
	float current_limit_a = 4.0F;
	float lift_leash_mm = 2.0F;
	float spin_leash_rad = 0.1F;
	float kp_a_per_rad = 40.0F;
	float kd_lift_a_per_rad_s = 4.0F / 12.0F;
	float kd_spin_a_per_rad_s = 1.0F;
	float lift_rate_mm_s = -120.0F;
	float spin_step_rad = -pi * 0.5F;
	float hand_at_home_rad = 0.0F;
	float yaw_sign = 1.0F;
	float command_period_s = 0.010F;
	float stall_speed_rad_s = 0.15F;
	float full_current_slack = 1.0F / 64.0F;
	float stall_after_s = 0.5F;
	float stall_hold_a = 2.0F;
	float stall_relief_s = 1.0F;
	int stall_retries = 3;

	float jogLeashMm() const
	{
		return lift_leash_mm + std::fabs(lift_rate_mm_s) * command_period_s;
	}

	bool valid() const;
};

} // namespace dscrew
