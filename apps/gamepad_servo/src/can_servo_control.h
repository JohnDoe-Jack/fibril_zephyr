/* SPDX-License-Identifier: Apache-2.0 */
#ifndef GAMEPAD_SERVO_CAN_SERVO_CONTROL_H_
#define GAMEPAD_SERVO_CAN_SERVO_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/drivers/can.h>

#define CAN_SERVO_ID 1U
#define CAN_SERVO_COMMAND_ID (0x300U + CAN_SERVO_ID)
#define CAN_SERVO_BROADCAST_ID 0x300U
#define CAN_SERVO_STATUS_ID (0x380U + CAN_SERVO_ID)

enum can_servo_control_source {
	CAN_SERVO_SOURCE_NONE,
	CAN_SERVO_SOURCE_UART,
	CAN_SERVO_SOURCE_CAN,
};

enum can_servo_output_action {
	CAN_SERVO_OUTPUT_NONE,
	CAN_SERVO_OUTPUT_ATTACH_CURRENT,
	CAN_SERVO_OUTPUT_DISABLE,
};

enum can_servo_status_state {
	CAN_SERVO_STATUS_HOLDING = 0,
	CAN_SERVO_STATUS_MOVING = 1,
	CAN_SERVO_STATUS_DETACHED = 2,
};

struct can_servo_control_state {
	enum can_servo_control_source source;
	float current_angle_deg;
	float target_angle_deg;
	float speed_limit_dps;
	uint32_t pulse_us;
	bool attached;
	bool clamped;
	bool rejected;
	bool status_requested;
};

void can_servo_state_init(struct can_servo_control_state *state,
			  uint32_t initial_pulse_us, uint32_t min_pulse_us,
			  uint32_t max_pulse_us, float min_angle_deg,
			  float max_angle_deg);

enum can_servo_output_action
can_servo_handle_frame(struct can_servo_control_state *state,
		       const struct can_frame *frame, float min_angle_deg,
		       float max_angle_deg);

bool can_servo_step(struct can_servo_control_state *state, float dt_sec,
		    uint32_t min_pulse_us, uint32_t max_pulse_us,
		    float min_angle_deg, float max_angle_deg);

void can_servo_take_uart_pulse(struct can_servo_control_state *state,
			       uint32_t pulse_us, uint32_t min_pulse_us,
			       uint32_t max_pulse_us, float min_angle_deg,
			       float max_angle_deg);

float can_servo_pulse_to_angle(uint32_t pulse_us, uint32_t min_pulse_us,
			       uint32_t max_pulse_us, float min_angle_deg,
			       float max_angle_deg);

uint32_t can_servo_angle_to_pulse(float angle_deg, uint32_t min_pulse_us,
				  uint32_t max_pulse_us, float min_angle_deg,
				  float max_angle_deg);

enum can_servo_status_state
can_servo_get_status_state(const struct can_servo_control_state *state);

void can_servo_encode_status(const struct can_servo_control_state *state,
			     struct can_frame *frame);

const char *can_servo_source_name(enum can_servo_control_source source);
const char *can_servo_status_name(enum can_servo_status_state state);

#endif /* GAMEPAD_SERVO_CAN_SERVO_CONTROL_H_ */
