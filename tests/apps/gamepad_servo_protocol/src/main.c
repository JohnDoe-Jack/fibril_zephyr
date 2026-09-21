/* SPDX-License-Identifier: Apache-2.0 */
#include <string.h>

#include <zephyr/ztest.h>

#include "can_servo_control.h"

#define MIN_PULSE_US 1000U
#define CENTER_PULSE_US 1210U
#define MAX_PULSE_US 1400U
#define MIN_ANGLE_DEG 0.0f
#define MAX_ANGLE_DEG 180.0f

static struct can_servo_control_state control;

static void assert_float_near(float actual, float expected)
{
	zassert_true(actual > expected - 0.001f && actual < expected + 0.001f,
		     "actual=%d expected=%d (millidegrees)",
		     (int)(actual * 1000.0f), (int)(expected * 1000.0f));
}

static struct can_frame command_frame(uint8_t command)
{
	struct can_frame frame = {
		.id = CAN_SERVO_COMMAND_ID,
		.dlc = 1,
		.data = {command},
	};

	return frame;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	can_servo_state_init(&control, CENTER_PULSE_US, MIN_PULSE_US,
			     MAX_PULSE_US, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
}

ZTEST(gamepad_servo_protocol, test_set_position_known_vector_and_slew)
{
	struct can_frame frame = {
		.id = CAN_SERVO_COMMAND_ID,
		.dlc = 5,
		.data = {0x00, 0x84, 0x03, 0xa0, 0x0f},
	};

	zassert_equal(can_servo_handle_frame(&control, &frame, MIN_ANGLE_DEG,
					       MAX_ANGLE_DEG),
		      CAN_SERVO_OUTPUT_NONE);
	assert_float_near(control.target_angle_deg, 90.0f);
	assert_float_near(control.speed_limit_dps, 400.0f);
	zassert_equal(control.source, CAN_SERVO_SOURCE_CAN);
	zassert_false(control.clamped);
	zassert_false(control.rejected);

	zassert_true(can_servo_step(&control, 0.01f, MIN_PULSE_US,
				    MAX_PULSE_US, MIN_ANGLE_DEG,
				    MAX_ANGLE_DEG));
	assert_float_near(control.current_angle_deg, 90.5f);
	zassert_equal(control.pulse_us, 1201U);
	zassert_equal(can_servo_get_status_state(&control),
		      CAN_SERVO_STATUS_MOVING);
}

ZTEST(gamepad_servo_protocol, test_negative_angle_is_signed)
{
	struct can_frame frame = {
		.id = CAN_SERVO_COMMAND_ID,
		.dlc = 5,
		/* -12.3 degrees, unlimited speed. */
		.data = {0x00, 0x85, 0xff, 0x00, 0x00},
	};

	can_servo_handle_frame(&control, &frame, -180.0f, 180.0f);
	assert_float_near(control.target_angle_deg, -12.3f);
	zassert_false(control.clamped);
}

ZTEST(gamepad_servo_protocol, test_zero_speed_moves_immediately)
{
	struct can_frame frame = {
		.id = CAN_SERVO_COMMAND_ID,
		.dlc = 5,
		.data = {0x00, 0x84, 0x03, 0x00, 0x00},
	};

	can_servo_handle_frame(&control, &frame, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
	zassert_true(can_servo_step(&control, 0.0f, MIN_PULSE_US,
				    MAX_PULSE_US, MIN_ANGLE_DEG,
				    MAX_ANGLE_DEG));
	assert_float_near(control.current_angle_deg, 90.0f);
	zassert_equal(control.pulse_us, 1200U);
	zassert_equal(can_servo_get_status_state(&control),
		      CAN_SERVO_STATUS_HOLDING);
}

ZTEST(gamepad_servo_protocol, test_malformed_and_unknown_do_not_move_state)
{
	struct can_frame malformed = {
		.id = CAN_SERVO_COMMAND_ID,
		.dlc = 4,
		.data = {0x00, 0x84, 0x03, 0xa0},
	};
	float initial_target = control.target_angle_deg;
	uint32_t initial_pulse = control.pulse_us;

	can_servo_handle_frame(&control, &malformed, MIN_ANGLE_DEG,
			       MAX_ANGLE_DEG);
	zassert_true(control.rejected);
	assert_float_near(control.target_angle_deg, initial_target);
	zassert_equal(control.pulse_us, initial_pulse);

	struct can_frame unknown = command_frame(0x7f);

	can_servo_handle_frame(&control, &unknown, MIN_ANGLE_DEG,
			       MAX_ANGLE_DEG);
	zassert_true(control.rejected);
	assert_float_near(control.target_angle_deg, initial_target);
	zassert_equal(control.pulse_us, initial_pulse);
}

ZTEST(gamepad_servo_protocol, test_out_of_range_target_is_clamped)
{
	struct can_frame frame = {
		.id = CAN_SERVO_COMMAND_ID,
		.dlc = 5,
		/* 200.0 degrees. */
		.data = {0x00, 0xd0, 0x07, 0x00, 0x00},
	};

	can_servo_handle_frame(&control, &frame, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
	assert_float_near(control.target_angle_deg, 180.0f);
	zassert_true(control.clamped);
	zassert_false(control.rejected);
}

ZTEST(gamepad_servo_protocol, test_detach_hold_and_reattach)
{
	struct can_frame detach = command_frame(0x02);
	struct can_frame hold = command_frame(0x01);
	struct can_frame set_position = {
		.id = CAN_SERVO_COMMAND_ID,
		.dlc = 5,
		.data = {0x00, 0x84, 0x03, 0x58, 0x02},
	};

	zassert_equal(can_servo_handle_frame(&control, &detach, MIN_ANGLE_DEG,
					       MAX_ANGLE_DEG),
		      CAN_SERVO_OUTPUT_DISABLE);
	zassert_false(control.attached);
	zassert_equal(can_servo_get_status_state(&control),
		      CAN_SERVO_STATUS_DETACHED);

	can_servo_handle_frame(&control, &hold, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
	zassert_false(control.attached);
	zassert_equal(control.source, CAN_SERVO_SOURCE_CAN);

	zassert_equal(can_servo_handle_frame(&control, &set_position,
					       MIN_ANGLE_DEG,
					       MAX_ANGLE_DEG),
		      CAN_SERVO_OUTPUT_ATTACH_CURRENT);
	zassert_true(control.attached);
	assert_float_near(control.target_angle_deg, 90.0f);
}

ZTEST(gamepad_servo_protocol, test_broadcast_accepts_only_detach)
{
	struct can_frame broadcast_set = {
		.id = CAN_SERVO_BROADCAST_ID,
		.dlc = 5,
		.data = {0x00, 0x84, 0x03, 0x00, 0x00},
	};
	struct can_frame broadcast_detach = {
		.id = CAN_SERVO_BROADCAST_ID,
		.dlc = 1,
		.data = {0x02},
	};
	float initial_target = control.target_angle_deg;

	zassert_equal(can_servo_handle_frame(&control, &broadcast_set,
					       MIN_ANGLE_DEG,
					       MAX_ANGLE_DEG),
		      CAN_SERVO_OUTPUT_NONE);
	assert_float_near(control.target_angle_deg, initial_target);
	zassert_equal(control.source, CAN_SERVO_SOURCE_NONE);
	zassert_equal(can_servo_handle_frame(&control, &broadcast_detach,
					       MIN_ANGLE_DEG,
					       MAX_ANGLE_DEG),
		      CAN_SERVO_OUTPUT_DISABLE);
	zassert_false(control.attached);
}

ZTEST(gamepad_servo_protocol, test_request_status_and_status_encoding)
{
	struct can_frame request = command_frame(0x03);
	struct can_frame status;

	control.target_angle_deg = 180.0f;
	control.current_angle_deg = 90.0f;
	control.clamped = true;
	control.rejected = true;
	can_servo_handle_frame(&control, &request, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
	zassert_true(control.status_requested);
	zassert_false(control.rejected);
	zassert_equal(control.source, CAN_SERVO_SOURCE_NONE);

	can_servo_encode_status(&control, &status);
	zassert_equal(status.id, CAN_SERVO_STATUS_ID);
	zassert_equal(status.flags, 0U);
	zassert_equal(status.dlc, 5U);
	zassert_equal(status.data[0], CAN_SERVO_STATUS_MOVING | 0x10U);
	zassert_equal(status.data[1], 0x08U);
	zassert_equal(status.data[2], 0x07U);
	zassert_equal(status.data[3], 0x84U);
	zassert_equal(status.data[4], 0x03U);
}

ZTEST(gamepad_servo_protocol, test_extended_and_rtr_frames_are_ignored)
{
	struct can_frame frame = command_frame(0x02);

	frame.flags = CAN_FRAME_IDE;
	can_servo_handle_frame(&control, &frame, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
	zassert_true(control.attached);
	frame.flags = CAN_FRAME_RTR;
	can_servo_handle_frame(&control, &frame, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
	zassert_true(control.attached);
}

ZTEST(gamepad_servo_protocol, test_uart_input_updates_status_state)
{
	can_servo_take_uart_pulse(&control, 1400U, MIN_PULSE_US,
				  MAX_PULSE_US, MIN_ANGLE_DEG,
				  MAX_ANGLE_DEG);
	zassert_equal(control.source, CAN_SERVO_SOURCE_UART);
	zassert_true(control.attached);
	assert_float_near(control.current_angle_deg, 180.0f);
	assert_float_near(control.target_angle_deg, 180.0f);
	zassert_equal(can_servo_get_status_state(&control),
		      CAN_SERVO_STATUS_HOLDING);
}

ZTEST_SUITE(gamepad_servo_protocol, NULL, NULL, before, NULL, NULL);
