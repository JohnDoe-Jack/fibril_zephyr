/* SPDX-License-Identifier: Apache-2.0 */

#include <math.h>
#include <string.h>

#include "robstride_protocol.h"

#include <drivers/motor/robstride.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/can_fake.h>
#include <zephyr/fff.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#define MASTER_ID 0xFDU

static const struct device *const can_dev = DEVICE_DT_GET(DT_NODELABEL(test_can));
static const struct device *const controller = DEVICE_DT_GET(DT_NODELABEL(robstride_test));
static const struct device *const motor1 = DEVICE_DT_GET(DT_NODELABEL(rs00_1));
static const struct device *const motor2 = DEVICE_DT_GET(DT_NODELABEL(rs00_2));

static can_rx_callback_t rx_callback;
static void *rx_user_data;
static struct can_filter captured_filter;
static struct can_frame sent[64];
static size_t sent_count;
static bool answer_reads;

DEFINE_FFF_GLOBALS;

static void inject_frame(struct can_frame *frame)
{
	zassert_not_null(rx_callback);
	rx_callback(can_dev, frame, rx_user_data);
}

static void answer_param_read(const struct can_frame *request)
{
	uint16_t index = sys_get_le16(&request->data[0]);
	uint8_t motor_id = robstride_id_target(request->id);
	struct can_frame response = {
		.id = robstride_make_id(ROBSTRIDE_COMM_PARAM_READ, motor_id, MASTER_ID),
		.dlc = can_bytes_to_dlc(8),
		.flags = CAN_FRAME_IDE,
	};
	sys_put_le16(index, &response.data[0]);
	if (index == ROBSTRIDE_PARAM_RUN_MODE) {
		response.data[4] = 5;
	} else if (index == ROBSTRIDE_PARAM_MECH_POS) {
		robstride_put_float_le(&response.data[4], 1.0f);
	} else if (index == ROBSTRIDE_PARAM_CAN_TIMEOUT) {
		sys_put_le32(2000, &response.data[4]);
	} else {
		return;
	}
	inject_frame(&response);
}

static int fake_add_filter(const struct device *dev, can_rx_callback_t callback, void *user_data,
			   const struct can_filter *filter)
{
	ARG_UNUSED(dev);
	rx_callback = callback;
	rx_user_data = user_data;
	captured_filter = *filter;
	return 1;
}

static int fake_send(const struct device *dev, const struct can_frame *frame, k_timeout_t timeout,
		     can_tx_callback_t callback, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(timeout);
	if (sent_count < ARRAY_SIZE(sent)) {
		sent[sent_count++] = *frame;
	}
	if (answer_reads && robstride_id_type(frame->id) == ROBSTRIDE_COMM_PARAM_READ) {
		answer_param_read(frame);
	}
	if (callback != NULL) {
		callback(can_dev, 0, user_data);
	}
	return 0;
}

static int fake_start(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int install_fakes(void)
{
	fake_can_add_rx_filter_fake.custom_fake = fake_add_filter;
	fake_can_send_fake.custom_fake = fake_send;
	fake_can_start_fake.custom_fake = fake_start;
	return 0;
}

SYS_INIT(install_fakes, PRE_KERNEL_1, 0);

static void *suite_setup(void)
{
	zassert_true(device_is_ready(controller));
	zassert_true(device_is_ready(motor1));
	zassert_true(device_is_ready(motor2));
	zassert_equal(captured_filter.id, MASTER_ID);
	zassert_equal(captured_filter.mask, 0xFF);
	zassert_true((captured_filter.flags & CAN_FILTER_IDE) != 0);
	zassert_ok(robstride_transport_start(controller));
	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	install_fakes();
	sent_count = 0;
	answer_reads = false;
}

ZTEST_SUITE(robstride, NULL, suite_setup, before, NULL, NULL);

static uint32_t current_epoch(void)
{
	struct robstride_transport_stats stats;
	zassert_ok(robstride_transport_get_stats(controller, &stats));
	return stats.operation_epoch;
}

ZTEST(robstride, test_wire_golden_parameter_frames)
{
	uint32_t epoch = current_epoch();
	zassert_ok(robstride_write_param_u8(motor1, ROBSTRIDE_PARAM_RUN_MODE, 5, epoch));
	zassert_equal(sent[0].id, 0x1200FD01U);
	zassert_mem_equal(sent[0].data, ((uint8_t[]){0x05, 0x70, 0, 0, 5, 0, 0, 0}), 8);
	zassert_equal(sent[0].flags, CAN_FRAME_IDE);

	zassert_ok(robstride_write_param_float(motor1, ROBSTRIDE_PARAM_LOC_REF, 1.0f, epoch));
	zassert_equal(sent[1].id, 0x1200FD01U);
	zassert_mem_equal(sent[1].data, ((uint8_t[]){0x16, 0x70, 0, 0, 0x00, 0x00, 0x80, 0x3F}), 8);

	zassert_ok(robstride_write_param_u32(motor1, ROBSTRIDE_PARAM_CAN_TIMEOUT, 2000, epoch));
	zassert_mem_equal(sent[2].data, ((uint8_t[]){0x28, 0x70, 0, 0, 0xD0, 0x07, 0, 0}), 8);
}

static void inject_feedback(uint8_t motor_id, uint16_t position, uint8_t mode, uint8_t faults,
			    uint16_t velocity, uint16_t torque, int16_t temperature_x10)
{
	uint16_t upper = (uint16_t)(((mode & 3U) << 6) | (faults & 0x3FU));
	uint16_t data16 = (uint16_t)((upper << 8) | motor_id);
	struct can_frame frame = {
		.id = robstride_make_id(ROBSTRIDE_COMM_FEEDBACK, data16, MASTER_ID),
		.dlc = can_bytes_to_dlc(8),
		.flags = CAN_FRAME_IDE,
	};
	sys_put_be16(position, &frame.data[0]);
	sys_put_be16(velocity, &frame.data[2]);
	sys_put_be16(torque, &frame.data[4]);
	sys_put_be16((uint16_t)temperature_x10, &frame.data[6]);
	inject_frame(&frame);
}

ZTEST(robstride, test_feedback_decode_filter_and_continuity_latch)
{
	struct robstride_feedback feedback;
	inject_feedback(1, 65530, ROBSTRIDE_MODE_MOTOR, BIT(2), 32767, 32767, 255);
	zassert_ok(robstride_get_feedback(motor1, &feedback));
	zassert_equal(feedback.mode_state, ROBSTRIDE_MODE_MOTOR);
	zassert_equal(feedback.fault_bits, BIT(2));
	zassert_within(feedback.temperature_c, 25.5f, 0.01f);
	zassert_true(feedback.position_continuous);

	inject_feedback(1, 5, ROBSTRIDE_MODE_MOTOR, 0, 32767, 32767, 250);
	zassert_ok(robstride_get_feedback(motor1, &feedback));
	zassert_equal(feedback.position_turns, 1);
	zassert_true(feedback.position_continuous);

	k_msleep(381);
	inject_feedback(1, 6, ROBSTRIDE_MODE_MOTOR, 0, 32767, 32767, 250);
	zassert_ok(robstride_get_feedback(motor1, &feedback));
	zassert_false(feedback.position_continuous);
	uint32_t continuity_epoch = feedback.continuity_epoch;
	inject_feedback(1, 7, ROBSTRIDE_MODE_MOTOR, 0, 32767, 32767, 250);
	zassert_ok(robstride_get_feedback(motor1, &feedback));
	zassert_false(feedback.position_continuous);
	zassert_equal(feedback.continuity_epoch, continuity_epoch);
	int64_t timestamp = feedback.timestamp_ms;

	struct can_frame invalid = {
		.id = robstride_make_id(ROBSTRIDE_COMM_FEEDBACK, 1, MASTER_ID),
		.dlc = can_bytes_to_dlc(8),
		.flags = 0,
	};
	inject_frame(&invalid);
	zassert_ok(robstride_get_feedback(motor1, &feedback));
	zassert_equal(feedback.timestamp_ms, timestamp);
}

ZTEST(robstride, test_typed_reads_correlate_axis_and_index)
{
	answer_reads = true;
	uint8_t mode = 0;
	uint32_t timeout = 0;
	float position = 0.0f;
	zassert_ok(robstride_read_param_u8(motor1, ROBSTRIDE_PARAM_RUN_MODE, &mode, K_MSEC(10)));
	zassert_equal(mode, 5);
	zassert_ok(robstride_read_param_u32(motor1, ROBSTRIDE_PARAM_CAN_TIMEOUT, &timeout,
					    K_MSEC(10)));
	zassert_equal(timeout, 2000);
	zassert_ok(robstride_read_param_float(motor1, ROBSTRIDE_PARAM_MECH_POS, &position,
					      K_MSEC(10)));
	zassert_within(position, 1.0f, 0.0001f);
}

ZTEST(robstride, test_prepare_commit_submit_and_stop_epoch)
{
	answer_reads = true;
	uint32_t epoch = current_epoch();
	struct robstride_csp_config config = {
		.can_timeout_ms = 100,
		.speed_limit_rad_s = 8.5f,
		.current_limit_a = 6.0f,
	};
	zassert_ok(robstride_prepare_csp(motor1, &config, epoch));
	zassert_ok(robstride_commit_enable(motor1, epoch));
	zassert_ok(robstride_submit_position(motor1, 1.25f, epoch));
	k_msleep(2);
	bool saw_position = false;
	for (size_t i = 0; i < sent_count; ++i) {
		if (robstride_id_type(sent[i].id) == ROBSTRIDE_COMM_PARAM_WRITE &&
		    sys_get_le16(&sent[i].data[0]) == ROBSTRIDE_PARAM_LOC_REF &&
		    fabsf(robstride_get_float_le(&sent[i].data[4]) - 1.25f) < 0.001f) {
			saw_position = true;
		}
	}
	zassert_true(saw_position);
	zassert_ok(robstride_request_stop(controller, epoch + 1U));
	zassert_equal(robstride_submit_position(motor1, 0.0f, epoch), -ESTALE);
	k_msleep(2);
	bool saw_stop = false;
	for (size_t i = 0; i < sent_count; ++i) {
		if (robstride_id_type(sent[i].id) == ROBSTRIDE_COMM_STOP) {
			saw_stop = true;
		}
	}
	zassert_true(saw_stop);
}

ZTEST(robstride, test_rejects_nonfinite_and_out_of_range_position)
{
	zassert_equal(robstride_submit_position(motor2, NAN, 0), -EINVAL);
	zassert_equal(robstride_submit_position(motor2, 12.58f, 0), -ERANGE);
	zassert_equal(robstride_submit_position(motor2, -12.58f, 0), -ERANGE);
}
