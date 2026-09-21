/* SPDX-License-Identifier: Apache-2.0 */
#include "can_servo_control.h"

#include <limits.h>
#include <string.h>

#include <zephyr/sys/util.h>

#define CAN_SERVO_CMD_SET_POSITION 0x00U
#define CAN_SERVO_CMD_HOLD 0x01U
#define CAN_SERVO_CMD_DETACH 0x02U
#define CAN_SERVO_CMD_REQUEST_STATUS 0x03U
#define CAN_SERVO_ANGLE_EPSILON_DEG 0.01f

static float abs_float(float value)
{
	return value < 0.0f ? -value : value;
}

static float clamp_angle(float angle_deg, float min_angle_deg,
			 float max_angle_deg)
{
	return CLAMP(angle_deg, min_angle_deg, max_angle_deg);
}

float can_servo_pulse_to_angle(uint32_t pulse_us, uint32_t min_pulse_us,
			       uint32_t max_pulse_us, float min_angle_deg,
			       float max_angle_deg)
{
	uint32_t pulse = CLAMP(pulse_us, min_pulse_us, max_pulse_us);
	double fraction = (double)(pulse - min_pulse_us) /
			  (double)(max_pulse_us - min_pulse_us);

	return (float)((double)min_angle_deg +
		       fraction * ((double)max_angle_deg - (double)min_angle_deg));
}

uint32_t can_servo_angle_to_pulse(float angle_deg, uint32_t min_pulse_us,
				  uint32_t max_pulse_us, float min_angle_deg,
				  float max_angle_deg)
{
	double angle = clamp_angle(angle_deg, min_angle_deg, max_angle_deg);
	double fraction = (angle - (double)min_angle_deg) /
			  ((double)max_angle_deg - (double)min_angle_deg);
	double pulse_us = min_pulse_us +
			  fraction * (max_pulse_us - min_pulse_us);

	return (uint32_t)(pulse_us + 0.5);
}

void can_servo_state_init(struct can_servo_control_state *state,
			  uint32_t initial_pulse_us, uint32_t min_pulse_us,
			  uint32_t max_pulse_us, float min_angle_deg,
			  float max_angle_deg)
{
	memset(state, 0, sizeof(*state));
	state->pulse_us = CLAMP(initial_pulse_us, min_pulse_us, max_pulse_us);
	state->current_angle_deg = can_servo_pulse_to_angle(
		state->pulse_us, min_pulse_us, max_pulse_us, min_angle_deg,
		max_angle_deg);
	state->target_angle_deg = state->current_angle_deg;
	state->attached = true;
}

static int32_t decode_i16_le(const uint8_t *data)
{
	uint32_t raw = (uint32_t)data[0] | ((uint32_t)data[1] << 8);

	return (raw & 0x8000U) != 0U ? (int32_t)raw - 0x10000 : (int32_t)raw;
}

static uint16_t decode_u16_le(const uint8_t *data)
{
	return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

enum can_servo_output_action
can_servo_handle_frame(struct can_servo_control_state *state,
		       const struct can_frame *frame, float min_angle_deg,
		       float max_angle_deg)
{
	bool broadcast;
	uint8_t command;

	if ((frame->flags & (CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0U) {
		return CAN_SERVO_OUTPUT_NONE;
	}

	broadcast = frame->id == CAN_SERVO_BROADCAST_ID;
	if (!broadcast && frame->id != CAN_SERVO_COMMAND_ID) {
		return CAN_SERVO_OUTPUT_NONE;
	}
	if (frame->dlc == 0U) {
		return CAN_SERVO_OUTPUT_NONE;
	}

	command = frame->data[0];
	if (broadcast && command != CAN_SERVO_CMD_DETACH) {
		return CAN_SERVO_OUTPUT_NONE;
	}

	switch (command) {
	case CAN_SERVO_CMD_SET_POSITION: {
		float requested_angle;
		float accepted_angle;

		if (frame->dlc < 5U) {
			state->rejected = true;
			return CAN_SERVO_OUTPUT_NONE;
		}

		requested_angle = (float)decode_i16_le(&frame->data[1]) * 0.1f;
		accepted_angle = clamp_angle(requested_angle, min_angle_deg,
					     max_angle_deg);
		state->target_angle_deg = accepted_angle;
		state->speed_limit_dps =
			(float)decode_u16_le(&frame->data[3]) * 0.1f;
		state->source = CAN_SERVO_SOURCE_CAN;
		state->clamped = requested_angle != accepted_angle;
		state->rejected = false;
		if (!state->attached) {
			state->attached = true;
			return CAN_SERVO_OUTPUT_ATTACH_CURRENT;
		}
		return CAN_SERVO_OUTPUT_NONE;
	}
	case CAN_SERVO_CMD_HOLD:
		state->target_angle_deg = state->current_angle_deg;
		state->speed_limit_dps = 0.0f;
		state->source = CAN_SERVO_SOURCE_CAN;
		state->rejected = false;
		return CAN_SERVO_OUTPUT_NONE;
	case CAN_SERVO_CMD_DETACH:
		state->attached = false;
		state->target_angle_deg = state->current_angle_deg;
		state->source = CAN_SERVO_SOURCE_CAN;
		state->rejected = false;
		return CAN_SERVO_OUTPUT_DISABLE;
	case CAN_SERVO_CMD_REQUEST_STATUS:
		state->status_requested = true;
		state->rejected = false;
		return CAN_SERVO_OUTPUT_NONE;
	default:
		state->rejected = true;
		return CAN_SERVO_OUTPUT_NONE;
	}
}

bool can_servo_step(struct can_servo_control_state *state, float dt_sec,
		    uint32_t min_pulse_us, uint32_t max_pulse_us,
		    float min_angle_deg, float max_angle_deg)
{
	float delta;
	float next_angle;

	if (state->source != CAN_SERVO_SOURCE_CAN || !state->attached) {
		return false;
	}

	delta = state->target_angle_deg - state->current_angle_deg;
	if (abs_float(delta) <= CAN_SERVO_ANGLE_EPSILON_DEG) {
		return false;
	}

	if (state->speed_limit_dps == 0.0f) {
		next_angle = state->target_angle_deg;
	} else {
		float max_delta = state->speed_limit_dps * MAX(dt_sec, 0.0f);

		if (max_delta <= 0.0f) {
			return false;
		}
		if (max_delta >= abs_float(delta)) {
			next_angle = state->target_angle_deg;
		} else {
			next_angle = state->current_angle_deg +
				     (delta > 0.0f ? max_delta : -max_delta);
		}
	}

	state->current_angle_deg = next_angle;
	state->pulse_us = can_servo_angle_to_pulse(
		next_angle, min_pulse_us, max_pulse_us, min_angle_deg,
		max_angle_deg);
	return true;
}

void can_servo_take_uart_pulse(struct can_servo_control_state *state,
			       uint32_t pulse_us, uint32_t min_pulse_us,
			       uint32_t max_pulse_us, float min_angle_deg,
			       float max_angle_deg)
{
	state->pulse_us = CLAMP(pulse_us, min_pulse_us, max_pulse_us);
	state->current_angle_deg = can_servo_pulse_to_angle(
		state->pulse_us, min_pulse_us, max_pulse_us, min_angle_deg,
		max_angle_deg);
	state->target_angle_deg = state->current_angle_deg;
	state->speed_limit_dps = 0.0f;
	state->source = CAN_SERVO_SOURCE_UART;
	state->attached = true;
}

enum can_servo_status_state
can_servo_get_status_state(const struct can_servo_control_state *state)
{
	if (!state->attached) {
		return CAN_SERVO_STATUS_DETACHED;
	}
	if (abs_float(state->target_angle_deg - state->current_angle_deg) >
	    CAN_SERVO_ANGLE_EPSILON_DEG) {
		return CAN_SERVO_STATUS_MOVING;
	}
	return CAN_SERVO_STATUS_HOLDING;
}

static int16_t angle_to_tenths(float angle_deg)
{
	float scaled = angle_deg * 10.0f;
	int32_t rounded;

	if (scaled >= INT16_MAX) {
		return INT16_MAX;
	}
	if (scaled <= INT16_MIN) {
		return INT16_MIN;
	}
	rounded = (int32_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
	return (int16_t)rounded;
}

static void encode_i16_le(uint8_t *data, int16_t value)
{
	uint16_t raw = (uint16_t)value;

	data[0] = (uint8_t)raw;
	data[1] = (uint8_t)(raw >> 8);
}

void can_servo_encode_status(const struct can_servo_control_state *state,
			     struct can_frame *frame)
{
	memset(frame, 0, sizeof(*frame));
	frame->id = CAN_SERVO_STATUS_ID;
	frame->dlc = 5U;
	frame->data[0] = (uint8_t)can_servo_get_status_state(state) |
			 (state->clamped ? 0x10U : 0U) |
			 (state->rejected ? 0x20U : 0U);
	encode_i16_le(&frame->data[1], angle_to_tenths(state->target_angle_deg));
	encode_i16_le(&frame->data[3], angle_to_tenths(state->current_angle_deg));
}

const char *can_servo_source_name(enum can_servo_control_source source)
{
	switch (source) {
	case CAN_SERVO_SOURCE_UART:
		return "uart";
	case CAN_SERVO_SOURCE_CAN:
		return "can";
	default:
		return "none";
	}
}

const char *can_servo_status_name(enum can_servo_status_state state)
{
	switch (state) {
	case CAN_SERVO_STATUS_MOVING:
		return "moving";
	case CAN_SERVO_STATUS_DETACHED:
		return "detached";
	default:
		return "holding";
	}
}
