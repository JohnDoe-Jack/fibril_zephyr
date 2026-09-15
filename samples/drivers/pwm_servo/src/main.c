/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <drivers/servo.h>

LOG_MODULE_REGISTER(servo_sample, LOG_LEVEL_INF);

#define SERVO_NODE DT_ALIAS(servo0)

int main(void)
{
	const struct device *servo = DEVICE_DT_GET(SERVO_NODE);
	const uint32_t pulses[] = {
		DT_PROP(SERVO_NODE, min_pulse_us),
		DT_PROP(SERVO_NODE, center_pulse_us),
		DT_PROP(SERVO_NODE, max_pulse_us),
	};
	int ret;

	if (!device_is_ready(servo)) {
		LOG_ERR("Servo is not ready");
		return -ENODEV;
	}

	/* Allow time to attach measurement equipment before the first movement. */
	k_sleep(K_SECONDS(3));
	for (size_t i = 0; i < ARRAY_SIZE(pulses); i++) {
		LOG_INF("Pulse: %u us", pulses[i]);
		ret = servo_set_pulse(servo, pulses[i]);
		if (ret < 0) {
			goto stop;
		}
		k_sleep(K_SECONDS(2));
	}

	LOG_INF("Angle midpoint");
	ret = servo_set_angle(servo, (float)(((double)(int32_t)DT_PROP(SERVO_NODE, min_angle_deg) +
					    (double)(int32_t)DT_PROP(SERVO_NODE, max_angle_deg)) / 2.0));
	if (ret == 0) {
		k_sleep(K_SECONDS(2));
	}

stop:
	if (ret < 0) {
		LOG_ERR("Servo command failed (%d)", ret);
	}
	int stop_ret = servo_disable(servo);

	if (stop_ret < 0) {
		LOG_ERR("Servo stop failed (%d)", stop_ret);
	} else {
		LOG_INF("Servo output stopped");
	}
	return ret < 0 ? ret : stop_ret;
}
