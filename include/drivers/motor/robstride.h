/* SPDX-License-Identifier: Apache-2.0 */
#ifndef FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_ROBSTRIDE_H_
#define FIBRIL_ZEPHYR_INCLUDE_DRIVERS_MOTOR_ROBSTRIDE_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

enum robstride_mode_state {
	ROBSTRIDE_MODE_RESET = 0,
	ROBSTRIDE_MODE_CALI = 1,
	ROBSTRIDE_MODE_MOTOR = 2,
	ROBSTRIDE_MODE_UNKNOWN = 3,
};

struct robstride_feedback {
	bool valid;
	bool fresh;
	int64_t timestamp_ms;
	int64_t age_ms;
	float position_rad;
	float velocity_rad_s;
	float torque_nm;
	float temperature_c;
	int32_t position_turns;
	bool position_continuous;
	uint32_t continuity_epoch;
	enum robstride_mode_state mode_state;
	uint8_t fault_bits;
	uint32_t fault_raw;
	uint32_t warning_raw;
};

struct robstride_csp_config {
	uint32_t can_timeout_ms;
	float speed_limit_rad_s;
	float current_limit_a;
};

struct robstride_transport_stats {
	uint32_t tx_accepted;
	uint32_t tx_completed;
	uint32_t tx_errors;
	uint32_t rejected_commands;
	uint32_t operation_epoch;
};

int robstride_get_feedback(const struct device *motor, struct robstride_feedback *out);
int robstride_submit_position(const struct device *motor, float position_rad, uint32_t epoch);
int robstride_request_stop(const struct device *controller, uint32_t epoch);
int robstride_clear_fault(const struct device *motor, uint32_t epoch);
int robstride_prepare_csp(const struct device *motor, const struct robstride_csp_config *config,
			  uint32_t epoch);
int robstride_commit_enable(const struct device *motor, uint32_t epoch);
int robstride_read_param_u8(const struct device *motor, uint16_t index, uint8_t *out,
			    k_timeout_t timeout);
int robstride_read_param_u32(const struct device *motor, uint16_t index, uint32_t *out,
			     k_timeout_t timeout);
int robstride_read_param_float(const struct device *motor, uint16_t index, float *out,
			       k_timeout_t timeout);
int robstride_write_param_u8(const struct device *motor, uint16_t index, uint8_t value,
			     uint32_t epoch);
int robstride_write_param_u32(const struct device *motor, uint16_t index, uint32_t value,
			      uint32_t epoch);
int robstride_write_param_float(const struct device *motor, uint16_t index, float value,
				uint32_t epoch);
int robstride_rebase_position_from_mech_pos(const struct device *motor, uint32_t epoch);
int robstride_transport_start(const struct device *controller);
int robstride_transport_stop(const struct device *controller);
int robstride_transport_get_stats(const struct device *controller,
				  struct robstride_transport_stats *out);

#ifdef __cplusplus
}
#endif

#endif
