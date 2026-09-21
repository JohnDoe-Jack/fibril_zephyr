/* SPDX-License-Identifier: Apache-2.0 */

#include <differential_screw/wrist.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace dscrew
{

namespace
{

bool finitePositive(float value)
{
	return std::isfinite(value) && value > 0.0F;
}

bool directionValid(float value)
{
	return value == 1.0F || value == -1.0F;
}

} // namespace

bool Config::valid() const
{
	const float alpha_tolerance = 8.0F * std::numeric_limits<float>::epsilon() *
				      std::max(std::fabs(alpha_r_rad_mm), 1.0F);
	return finitePositive(lead_mm) && finitePositive(alpha_r_rad_mm) &&
	       finitePositive(alpha_l_rad_mm) &&
	       std::fabs(alpha_r_rad_mm - alpha_l_rad_mm) <= alpha_tolerance &&
	       finitePositive(counts_per_rotor_rev) && finitePositive(total_ratio) &&
	       directionValid(right_direction) && directionValid(left_direction) &&
	       finitePositive(current_limit_a) && finitePositive(lift_leash_mm) &&
	       finitePositive(spin_leash_rad) && finitePositive(kp_a_per_rad) &&
	       finitePositive(kd_lift_a_per_rad_s) && finitePositive(kd_spin_a_per_rad_s) &&
	       std::isfinite(lift_rate_mm_s) && std::isfinite(spin_step_rad) &&
	       std::isfinite(hand_at_home_rad) && directionValid(yaw_sign) &&
	       finitePositive(command_period_s) && finitePositive(stall_speed_rad_s) &&
	       std::isfinite(full_current_slack) && full_current_slack > 0.0F &&
	       full_current_slack < 1.0F && finitePositive(stall_after_s) &&
	       finitePositive(stall_hold_a) && stall_hold_a < current_limit_a &&
	       finitePositive(stall_relief_s) && stall_retries > 0 && jogLeashMm() > lift_leash_mm;
}

NutAngles wristInverse(const Config &config, WristPose pose)
{
	return {pose.theta_rad - config.alpha_r_rad_mm * pose.z_mm,
		pose.theta_rad + config.alpha_l_rad_mm * pose.z_mm};
}

WristPose wristForward(const Config &config, NutAngles nuts)
{
	const float sum = config.alpha_r_rad_mm + config.alpha_l_rad_mm;
	if (!config.valid() || !std::isfinite(nuts.phi_r) || !std::isfinite(nuts.phi_l)) {
		const float nan = std::numeric_limits<float>::quiet_NaN();
		return {nan, nan};
	}
	return {(config.alpha_l_rad_mm * nuts.phi_r + config.alpha_r_rad_mm * nuts.phi_l) / sum,
		(nuts.phi_l - nuts.phi_r) / sum};
}

float nutRadFromCounts(const Config &config, int64_t counts, float direction)
{
	if (!config.valid() || !directionValid(direction)) {
		return std::numeric_limits<float>::quiet_NaN();
	}
	return direction * static_cast<float>(counts) * (2.0F * pi) /
	       (config.counts_per_rotor_rev * config.total_ratio);
}

float nutRadPerSecFromRpm(const Config &config, int16_t rotor_rpm, float direction)
{
	if (!config.valid() || !directionValid(direction)) {
		return std::numeric_limits<float>::quiet_NaN();
	}
	return direction * static_cast<float>(rotor_rpm) * (2.0F * pi / 60.0F) / config.total_ratio;
}

Wrist::Wrist(Config config) : config_(config), valid_(config.valid())
{
}

bool Wrist::finite(NutAngles value)
{
	return std::isfinite(value.phi_r) && std::isfinite(value.phi_l);
}

bool Wrist::finite(NutRates value)
{
	return std::isfinite(value.dphi_r) && std::isfinite(value.dphi_l);
}

bool Wrist::seed(NutAngles measured)
{
	if (!valid_ || !finite(measured)) {
		return false;
	}
	target_ = wristForward(config_, measured);
	measured_ = target_;
	theta_goal_ = target_.theta_rad;
	measured_valid_ = true;
	seeded_ = true;
	spin_watch_ = {};
	lift_watch_ = {};
	return true;
}

void Wrist::forget()
{
	seeded_ = false;
	yaw_goal_set_ = false;
	spin_watch_ = {};
	lift_watch_ = {};
}

bool Wrist::setYawOrigin()
{
	if (!valid_ || !measured_valid_) {
		return false;
	}
	yaw_offset_ = measured_.theta_rad - config_.yaw_sign * config_.hand_at_home_rad;
	yaw_homed_ = true;
	yaw_goal_set_ = false;
	return true;
}

void Wrist::rebaseYawOrigin()
{
	if (yaw_homed_) {
		yaw_offset_ -= measured_.theta_rad;
	}
}

float Wrist::handYaw(float theta2_rad) const
{
	return theta2_rad + config_.yaw_sign * (measured_.theta_rad - yaw_offset_);
}

bool Wrist::jog(float lift_mm_s, float dt_s)
{
	if (!seeded_ || !std::isfinite(lift_mm_s) || !finitePositive(dt_s)) {
		return false;
	}
	target_.z_mm += lift_mm_s * dt_s;
	target_.z_mm = leash(target_.z_mm, measured_.z_mm, config_.jogLeashMm());
	return true;
}

void Wrist::spinBy(int steps)
{
	if (seeded_ && steps != 0 && yaw_goal_set_) {
		hand_yaw_goal_ += static_cast<float>(steps) * config_.spin_step_rad;
	}
}

float Wrist::nearestAngle(float reference, float target)
{
	return reference + std::remainder(target - reference, 2.0F * pi);
}

void Wrist::holdYaw(float theta2_rad)
{
	if (!seeded_ || !std::isfinite(theta2_rad)) {
		return;
	}
	if (!yaw_goal_set_) {
		hand_yaw_goal_ = handYaw(theta2_rad);
		yaw_goal_set_ = true;
		const float want = config_.yaw_sign * (hand_yaw_goal_ - theta2_rad) + yaw_offset_;
		theta_goal_ = nearestAngle(measured_.theta_rad, want);
		yaw_want_prev_ = want;
		return;
	}
	const float want = config_.yaw_sign * (hand_yaw_goal_ - theta2_rad) + yaw_offset_;
	theta_goal_ += want - yaw_want_prev_;
	yaw_want_prev_ = want;
}

float Wrist::leash(float target, float measured, float limit)
{
	return std::clamp(target, measured - limit, measured + limit);
}

float Wrist::clamp(float value, float limit)
{
	return std::clamp(value, -limit, limit);
}

float Wrist::watchLimit(StallWatch &watch, float demand, float rate, float dt_s)
{
	const bool pushing =
		std::fabs(demand) >= config_.current_limit_a * (1.0F - config_.full_current_slack);
	const bool moving = std::fabs(rate) > config_.stall_speed_rad_s;
	if (!pushing || moving) {
		watch = {};
		return config_.current_limit_a;
	}
	watch.stalled_s += dt_s;
	watch.phase_s += dt_s;
	if (!watch.relieving) {
		if (watch.phase_s >= config_.stall_after_s) {
			watch.relieving = true;
			watch.phase_s = 0.0F;
			++watch.relieved;
		}
	} else if (watch.relieved < config_.stall_retries &&
		   watch.phase_s >= config_.stall_relief_s) {
		watch.relieving = false;
		watch.phase_s = 0.0F;
	}
	return watch.relieving ? config_.stall_hold_a : config_.current_limit_a;
}

bool Wrist::step(NutAngles measured, NutRates rates, float dt_s, Currents &out)
{
	out = {};
	if (!valid_ || !finite(measured) || !finite(rates) || !finitePositive(dt_s)) {
		return false;
	}
	measured_ = wristForward(config_, measured);
	measured_valid_ = true;
	if (!seeded_) {
		spin_watch_ = {};
		lift_watch_ = {};
		return true;
	}

	target_.theta_rad = leash(theta_goal_, measured_.theta_rad, config_.spin_leash_rad);
	target_.z_mm = leash(target_.z_mm, measured_.z_mm, config_.lift_leash_mm);
	const NutAngles want = wristInverse(config_, target_);
	const float dtheta = (rates.dphi_r + rates.dphi_l) * 0.5F;
	const float alpha_dz = (rates.dphi_l - rates.dphi_r) * 0.5F;
	const float damp_common = config_.kd_spin_a_per_rad_s * dtheta;
	const float damp_diff = config_.kd_lift_a_per_rad_s * alpha_dz;
	const float raw_r =
		config_.kp_a_per_rad * (want.phi_r - measured.phi_r) - (damp_common - damp_diff);
	const float raw_l =
		config_.kp_a_per_rad * (want.phi_l - measured.phi_l) - (damp_common + damp_diff);
	const float common = (raw_r + raw_l) * 0.5F;
	const float diff = (raw_l - raw_r) * 0.5F;
	const float send_common = clamp(common, watchLimit(spin_watch_, common, dtheta, dt_s));
	const float send_diff = clamp(diff, watchLimit(lift_watch_, diff, alpha_dz, dt_s));
	out.right_a = clamp(send_common - send_diff, config_.current_limit_a);
	out.left_a = clamp(send_common + send_diff, config_.current_limit_a);
	return std::isfinite(out.right_a) && std::isfinite(out.left_a);
}

float Wrist::stalledFor() const
{
	return std::max(spin_watch_.stalled_s, lift_watch_.stalled_s);
}

bool Wrist::isStalled() const
{
	return stalledFor() >= config_.stall_after_s;
}

} // namespace dscrew
