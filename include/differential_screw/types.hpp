/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

namespace dscrew
{

struct WristPose {
	float theta_rad = 0.0F;
	float z_mm = 0.0F;
};

struct NutAngles {
	float phi_r = 0.0F;
	float phi_l = 0.0F;
};

struct NutRates {
	float dphi_r = 0.0F;
	float dphi_l = 0.0F;
};

struct Currents {
	float right_a = 0.0F;
	float left_a = 0.0F;
};

} // namespace dscrew
