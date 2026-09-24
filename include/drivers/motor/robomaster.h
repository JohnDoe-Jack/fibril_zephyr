/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_ROBOMASTER_H_
#define FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_ROBOMASTER_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#include <drivers/motor.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Extended RoboMaster feedback. Continuous position is in rotor encoder
 * counts. */
struct robomaster_feedback_ex {
	struct motor_feedback raw;
	int64_t continuous_count;
	uint32_t continuity_breaks;
	uint32_t continuity_epoch;
	int32_t rotor_speed_bound_rpm;
	int32_t continuity_gap_limit_ms;
	bool continuous_valid;
};

/** Transport health counters. */
struct robomaster_transport_stats {
	uint32_t tx_accepted;
	uint32_t tx_completed;
	uint32_t tx_errors;
	uint32_t command_timeouts;
	uint32_t operation_epoch;
};

int robomaster_get_feedback_ex(const struct device *motor, struct robomaster_feedback_ex *out);
int robomaster_reset_continuous_position(const struct device *motor);
int robomaster_c610_set_current_a(const struct device *motor, float current_a);
int robomaster_c610_set_pair_current_a(const struct device *right, const struct device *left,
				       float right_a, float left_a);
int robomaster_request_stop(const struct device *transport, uint8_t motor_mask);
int robomaster_transport_start(const struct device *transport);
int robomaster_transport_stop(const struct device *transport);
int robomaster_transport_get_stats(const struct device *transport,
				   struct robomaster_transport_stats *out);

#ifdef __cplusplus
}
#endif

#endif
