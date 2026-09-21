/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <cstdint>

#include <differential_screw/config.hpp>
#include <differential_screw/types.hpp>

namespace dscrew
{

NutAngles wristInverse(const Config &config, WristPose pose);
WristPose wristForward(const Config &config, NutAngles nuts);
float nutRadFromCounts(const Config &config, int64_t counts, float direction);
float nutRadPerSecFromRpm(const Config &config, int16_t rotor_rpm, float direction);

class Wrist
{
      public:
	explicit Wrist(Config config = {});

	bool valid() const
	{
		return valid_;
	}
	const Config &config() const
	{
		return config_;
	}
	bool seed(NutAngles measured);
	void forget();
	bool step(NutAngles measured, NutRates rates, float dt_s, Currents &out);
	bool jog(float lift_mm_s, float dt_s);
	void spinBy(int steps);
	void holdYaw(float theta2_rad);
	bool setYawOrigin();
	void rebaseYawOrigin();

	WristPose target() const
	{
		return target_;
	}
	WristPose measured() const
	{
		return measured_;
	}
	float handYaw(float theta2_rad) const;
	float handYawGoal() const
	{
		return hand_yaw_goal_;
	}
	float goalAngle() const
	{
		return theta_goal_;
	}
	bool hasYawGoal() const
	{
		return yaw_goal_set_;
	}
	bool isSeeded() const
	{
		return seeded_;
	}
	bool isYawHomed() const
	{
		return yaw_homed_;
	}
	bool isStalled() const;
	float stalledFor() const;

      private:
	struct StallWatch {
		float stalled_s = 0.0F;
		float phase_s = 0.0F;
		int relieved = 0;
		bool relieving = false;
	};

	float watchLimit(StallWatch &watch, float demand, float rate, float dt_s);
	static float leash(float target, float measured, float limit);
	static float clamp(float value, float limit);
	static float nearestAngle(float reference, float target);
	static bool finite(NutAngles value);
	static bool finite(NutRates value);

	Config config_;
	bool valid_ = false;
	WristPose target_{};
	WristPose measured_{};
	float theta_goal_ = 0.0F;
	float hand_yaw_goal_ = 0.0F;
	float yaw_want_prev_ = 0.0F;
	float yaw_offset_ = 0.0F;
	bool yaw_goal_set_ = false;
	bool measured_valid_ = false;
	bool yaw_homed_ = false;
	bool seeded_ = false;
	StallWatch spin_watch_{};
	StallWatch lift_watch_{};
};

} // namespace dscrew
