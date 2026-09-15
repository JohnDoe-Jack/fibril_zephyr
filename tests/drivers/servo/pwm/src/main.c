/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <float.h>
#include <math.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm/pwm_fake.h>
#include <zephyr/init.h>
#include <zephyr/ztest.h>
#include <drivers/servo.h>

DEFINE_FFF_GLOBALS;

static const struct device *const servo0 = DEVICE_DT_GET(DT_NODELABEL(servo0));
static const struct device *const servo1 = DEVICE_DT_GET(DT_NODELABEL(servo1));
static unsigned int startup_calls;
static bool startup_nonzero;

static int startup_pwm(const struct device *dev, uint32_t channel, uint32_t period,
		       uint32_t pulse, pwm_flags_t flags)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(period);
	ARG_UNUSED(flags);
	startup_calls++;
	startup_nonzero |= pulse != 0;
	return channel == 7 ? -EIO : 0;
}

static int install_startup_pwm(void)
{
	fake_pwm_set_cycles_fake.custom_fake = startup_pwm;
	return 0;
}
SYS_INIT(install_startup_pwm, PRE_KERNEL_1, 0);

static void expect_output(uint32_t channel, uint32_t period_us, uint32_t pulse_us)
{
	zassert_equal(fake_pwm_set_cycles_fake.arg0_val, DEVICE_DT_GET(DT_NODELABEL(test_pwm)));
	zassert_equal(fake_pwm_set_cycles_fake.arg1_val, channel);
	zassert_equal(fake_pwm_set_cycles_fake.arg2_val, period_us);
	zassert_equal(fake_pwm_set_cycles_fake.arg3_val, pulse_us);
}

ZTEST(pwm_servo, test_init_leaves_outputs_inactive_and_propagates_pwm_failure)
{
	zassert_true(device_is_ready(servo0));
	zassert_true(device_is_ready(servo1));
	zassert_equal(startup_calls, 3);
	zassert_false(startup_nonzero);
	const struct device *failed = DEVICE_DT_GET(DT_NODELABEL(init_failure));
	zassert_false(device_is_ready(failed));
	zassert_equal(failed->state->init_res, EIO);
}

ZTEST(pwm_servo, test_angles_map_endpoints_midpoint_and_fractional_values)
{
	const float angles[] = {0, 90, 180, 45, 45.5f};
	const uint32_t pulses[] = {500, 1500, 2500, 1000, 1006};
	for (size_t i = 0; i < ARRAY_SIZE(angles); i++) {
		zassert_ok(servo_set_angle(servo0, angles[i]));
		expect_output(0, 20000, pulses[i]);
	}
}

ZTEST(pwm_servo, test_out_of_range_angles_are_clamped)
{
	zassert_ok(servo_set_angle(servo0, -10));
	expect_output(0, 20000, 500);
	zassert_ok(servo_set_angle(servo0, 200));
	expect_output(0, 20000, 2500);
	zassert_ok(servo_set_angle(servo0, -FLT_MAX));
	expect_output(0, 20000, 500);
	zassert_ok(servo_set_angle(servo0, FLT_MAX));
	expect_output(0, 20000, 2500);
}

ZTEST(pwm_servo, test_pulse_commands_are_clamped_before_unit_conversion)
{
	const uint32_t inputs[] = {0, 499, 500, 1500, 2500, 2501, UINT32_MAX};
	const uint32_t outputs[] = {500, 500, 500, 1500, 2500, 2500, 2500};
	for (size_t i = 0; i < ARRAY_SIZE(inputs); i++) {
		zassert_ok(servo_set_pulse(servo0, inputs[i]));
		expect_output(0, 20000, outputs[i]);
	}
}

ZTEST(pwm_servo, test_instances_use_their_own_signed_angle_range_period_and_channel)
{
	zassert_ok(servo_set_angle(servo1, -90));
	expect_output(1, 10000, 1000);
	zassert_ok(servo_set_angle(servo1, 0));
	expect_output(1, 10000, 1600);
	zassert_ok(servo_set_angle(servo1, 90));
	expect_output(1, 10000, 2200);
	zassert_ok(servo_set_angle(servo0, 90));
	expect_output(0, 20000, 1500);
}

ZTEST(pwm_servo, test_disable_preserves_period_and_next_command_resumes_output)
{
	zassert_ok(servo_set_angle(servo0, 90));
	zassert_ok(servo_disable(servo0));
	expect_output(0, 20000, 0);
	zassert_ok(servo_disable(servo0));
	expect_output(0, 20000, 0);
	zassert_ok(servo_set_pulse(servo0, 500));
	expect_output(0, 20000, 500);
}

ZTEST(pwm_servo, test_pwm_errors_reach_all_callers)
{
	fake_pwm_set_cycles_fake.return_val = -ENOTSUP;
	zassert_equal(servo_set_angle(servo0, 90), -ENOTSUP);
	zassert_equal(servo_set_pulse(servo0, 1500), -ENOTSUP);
	zassert_equal(servo_disable(servo0), -ENOTSUP);
}

ZTEST(pwm_servo, test_nonfinite_angles_do_not_change_output)
{
	zassert_equal(servo_set_angle(servo0, NAN), -EINVAL);
	zassert_equal(servo_set_angle(servo0, INFINITY), -EINVAL);
	zassert_equal(servo_set_angle(servo0, -INFINITY), -EINVAL);
	zassert_equal(fake_pwm_set_cycles_fake.call_count, 0);
}

ZTEST(pwm_servo, test_invalid_configuration_fails_before_pwm_output)
{
	const struct device *const invalid[] = {
		DEVICE_DT_GET(DT_NODELABEL(zero_pulse)),
		DEVICE_DT_GET(DT_NODELABEL(equal_pulses)),
		DEVICE_DT_GET(DT_NODELABEL(reversed_pulses)),
		DEVICE_DT_GET(DT_NODELABEL(center_low)),
		DEVICE_DT_GET(DT_NODELABEL(center_high)),
		DEVICE_DT_GET(DT_NODELABEL(equal_angles)),
		DEVICE_DT_GET(DT_NODELABEL(reversed_angles)),
		DEVICE_DT_GET(DT_NODELABEL(zero_period)),
		DEVICE_DT_GET(DT_NODELABEL(pulse_over_period)),
		DEVICE_DT_GET(DT_NODELABEL(pulse_overflow)),
	};
	for (size_t i = 0; i < ARRAY_SIZE(invalid); i++) {
		zassert_false(device_is_ready(invalid[i]));
		zassert_equal(invalid[i]->state->init_res, EINVAL);
		zassert_equal(servo_set_angle(invalid[i], 90), -ENODEV);
		zassert_equal(servo_set_pulse(invalid[i], 1500), -ENODEV);
		zassert_equal(servo_disable(invalid[i]), -ENODEV);
	}
	zassert_equal(fake_pwm_set_cycles_fake.call_count, 0);
}

ZTEST(pwm_servo, test_unready_pwm_and_null_devices_are_rejected)
{
	const struct device *unready = DEVICE_DT_GET(DT_NODELABEL(unready_servo));
	zassert_equal(device_init(unready), -ENODEV);
	zassert_false(device_is_ready(unready));
	zassert_equal(unready->state->init_res, ENODEV);
	zassert_equal(servo_set_angle(unready, 90), -ENODEV);
	zassert_equal(servo_set_pulse(unready, 1500), -ENODEV);
	zassert_equal(servo_disable(unready), -ENODEV);
	zassert_equal(servo_set_angle(NULL, 90), -ENODEV);
	zassert_equal(servo_set_pulse(NULL, 1500), -ENODEV);
	zassert_equal(servo_disable(NULL), -ENODEV);
	zassert_equal(fake_pwm_set_cycles_fake.call_count, 0);
}

ZTEST_SUITE(pwm_servo, NULL, NULL, NULL, NULL, NULL);
