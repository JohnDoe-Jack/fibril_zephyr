/* SPDX-License-Identifier: Apache-2.0 */

#include <math.h>
#include <string.h>

#include "robstride_protocol.h"

#include <drivers/motor/robstride.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(motor_robstride, CONFIG_MOTOR_LOG_LEVEL);

#define ROBSTRIDE_MAX_ID                127U
#define ROBSTRIDE_POSITION_SPAN_RAD     (ROBSTRIDE_POSITION_MAX_RAD - ROBSTRIDE_POSITION_MIN_RAD)
#define ROBSTRIDE_POSITION_GAP_LIMIT_MS 380
#define ROBSTRIDE_TIMEOUT_TICKS_PER_MS  20U
#define ROBSTRIDE_INTER_FRAME_US        500

enum pending_type {
	PENDING_NONE,
	PENDING_U8,
	PENDING_U32,
	PENDING_FLOAT,
};

struct robstride_controller_data;

struct robstride_controller_config {
	const struct device *can_dev;
	uint8_t master_id;
	uint32_t feedback_timeout_ms;
	uint32_t command_timeout_ms;
	bool external_bus_management;
	k_thread_stack_t *tx_stack;
	size_t tx_stack_size;
};

struct robstride_tx_context {
	struct robstride_controller_data *controller;
	atomic_t pending;
};

struct robstride_controller_data {
	struct k_spinlock lock;
	const struct device *dev;
	const struct device *motors[ROBSTRIDE_MAX_ID + 1U];
	struct robstride_tx_context tx_contexts[ROBSTRIDE_MAX_ID + 1U];
	struct k_work tx_work;
	struct k_work_q tx_queue;
	bool started;
	bool stop_latched;
	uint32_t stop_epoch;
	struct robstride_transport_stats stats;
};

struct robstride_motor_config {
	const struct device *controller;
	uint8_t motor_id;
};

struct robstride_motor_data {
	struct k_spinlock feedback_lock;
	uint16_t raw_position;
	uint16_t raw_velocity;
	uint16_t raw_torque;
	int16_t temperature_x10;
	int64_t timestamp_ms;
	uint16_t last_raw_position;
	int64_t last_position_ms;
	int32_t position_turns;
	uint32_t continuity_epoch;
	bool feedback_valid;
	bool has_last_position;
	bool position_continuous;
	enum robstride_mode_state mode_state;
	uint8_t fault_bits;
	uint32_t fault_raw;
	uint32_t warning_raw;

	struct k_sem param_sem;
	struct k_mutex management_lock;
	struct k_spinlock param_lock;
	uint16_t pending_index;
	enum pending_type pending_type;
	uint8_t pending_value[4];
	bool pending_active;
	bool pending_canceled;

	float latest_position_rad;
	int64_t latest_command_ms;
	uint32_t command_epoch;
	bool command_pending;
	bool prepared;
	bool committed;
	uint32_t prepared_epoch;
	struct robstride_csp_config csp_config;
};

static bool frame_flags_valid(const struct can_frame *frame)
{
	return (frame->flags & CAN_FRAME_IDE) != 0U &&
	       (frame->flags & (CAN_FRAME_RTR | CAN_FRAME_FDF | CAN_FRAME_BRS)) == 0U &&
	       can_dlc_to_bytes(frame->dlc) == 8U;
}

static void tx_complete(const struct device *dev, int error, void *user_data)
{
	struct robstride_tx_context *context = user_data;
	struct robstride_controller_data *controller = context->controller;
	ARG_UNUSED(dev);
	k_spinlock_key_t key = k_spin_lock(&controller->lock);
	if (error == 0) {
		controller->stats.tx_completed++;
	} else {
		controller->stats.tx_errors++;
	}
	k_spin_unlock(&controller->lock, key);
	atomic_clear(&context->pending);
}

static int send_async(const struct device *motor_dev, struct can_frame *frame)
{
	const struct robstride_motor_config *motor_config = motor_dev->config;
	const struct robstride_controller_config *controller_config =
		motor_config->controller->config;
	struct robstride_controller_data *controller = motor_config->controller->data;
	struct robstride_tx_context *context = &controller->tx_contexts[motor_config->motor_id];
	if (!controller->started) {
		return -ENETDOWN;
	}
	if (!atomic_cas(&context->pending, 0, 1)) {
		return -EBUSY;
	}
	int ret = can_send(controller_config->can_dev, frame, K_NO_WAIT, tx_complete, context);
	k_spinlock_key_t key = k_spin_lock(&controller->lock);
	if (ret == 0) {
		controller->stats.tx_accepted++;
	} else {
		controller->stats.tx_errors++;
		atomic_clear(&context->pending);
	}
	k_spin_unlock(&controller->lock, key);
	return ret;
}

static int send_management(const struct device *motor_dev, enum robstride_comm_type type,
			   const uint8_t data[8])
{
	const struct robstride_motor_config *motor_config = motor_dev->config;
	const struct robstride_controller_config *controller_config =
		motor_config->controller->config;
	struct robstride_controller_data *controller = motor_config->controller->data;
	if (!controller->started) {
		return -ENETDOWN;
	}
	struct can_frame frame = {
		.id = robstride_make_id(type, controller_config->master_id, motor_config->motor_id),
		.dlc = can_bytes_to_dlc(8U),
		.flags = CAN_FRAME_IDE,
	};
	if (data != NULL) {
		memcpy(frame.data, data, sizeof(frame.data));
	}
	int ret = can_send(controller_config->can_dev, &frame, K_MSEC(20), NULL, NULL);
	k_spinlock_key_t key = k_spin_lock(&controller->lock);
	if (ret == 0) {
		controller->stats.tx_accepted++;
		controller->stats.tx_completed++;
	} else {
		controller->stats.tx_errors++;
	}
	k_spin_unlock(&controller->lock, key);
	return ret;
}

static int send_stop_to_motor(const struct device *motor_dev, bool clear_fault)
{
	uint8_t data[8] = {0};
	data[0] = clear_fault ? 1U : 0U;
	return send_management(motor_dev, ROBSTRIDE_COMM_STOP, data);
}

static void tx_work_handler(struct k_work *work)
{
	struct robstride_controller_data *controller =
		CONTAINER_OF(work, struct robstride_controller_data, tx_work);
	const struct robstride_controller_config *config = controller->dev->config;
	const int64_t now = k_uptime_get();

	for (size_t id = 1; id <= ROBSTRIDE_MAX_ID; ++id) {
		const struct device *motor_dev = controller->motors[id];
		if (motor_dev == NULL) {
			continue;
		}
		struct robstride_motor_data *motor = motor_dev->data;
		bool send_stop;
		bool send_position;
		float position = 0.0f;
		uint32_t epoch;
		k_spinlock_key_t key = k_spin_lock(&controller->lock);
		send_stop = controller->stop_latched;
		epoch = controller->stats.operation_epoch;
		send_position = motor->command_pending && motor->command_epoch == epoch &&
				!send_stop &&
				(config->command_timeout_ms == 0U ||
				 (now - motor->latest_command_ms) <= config->command_timeout_ms);
		if (motor->command_pending && !send_position && !send_stop) {
			motor->command_pending = false;
			controller->stats.rejected_commands++;
		}
		if (send_position) {
			position = motor->latest_position_rad;
			motor->command_pending = false;
		}
		k_spin_unlock(&controller->lock, key);

		if (send_stop) {
			(void)send_stop_to_motor(motor_dev, false);
			continue;
		}
		if (!send_position) {
			continue;
		}
		uint8_t data[8];
		robstride_put_param_header(data, ROBSTRIDE_PARAM_LOC_REF);
		robstride_put_float_le(&data[4], position);
		struct can_frame frame = {
			.id = robstride_make_id(ROBSTRIDE_COMM_PARAM_WRITE, config->master_id, id),
			.dlc = can_bytes_to_dlc(8U),
			.flags = CAN_FRAME_IDE,
		};
		memcpy(frame.data, data, sizeof(data));
		int ret = send_async(motor_dev, &frame);
		if (ret == -EBUSY) {
			key = k_spin_lock(&controller->lock);
			if (motor->command_epoch == epoch) {
				motor->command_pending = true;
			}
			k_spin_unlock(&controller->lock, key);
		}
	}
}

static const struct device *find_motor(struct robstride_controller_data *controller, uint8_t id)
{
	return id <= ROBSTRIDE_MAX_ID ? controller->motors[id] : NULL;
}

static void on_feedback(const struct device *motor_dev, const struct can_frame *frame)
{
	struct robstride_motor_data *motor = motor_dev->data;
	const int64_t now = k_uptime_get();
	const uint16_t position = sys_get_be16(&frame->data[0]);
	k_spinlock_key_t key = k_spin_lock(&motor->feedback_lock);
	if (!motor->has_last_position) {
		motor->position_continuous = true;
		motor->has_last_position = true;
	} else if (!motor->position_continuous || now < motor->last_position_ms ||
		   (now - motor->last_position_ms) > ROBSTRIDE_POSITION_GAP_LIMIT_MS) {
		if (motor->position_continuous) {
			motor->continuity_epoch++;
		}
		motor->position_continuous = false;
	} else {
		int32_t delta = (int32_t)position - (int32_t)motor->last_raw_position;
		if (delta > 32767) {
			motor->position_turns--;
		} else if (delta < -32767) {
			motor->position_turns++;
		}
	}
	motor->raw_position = position;
	motor->raw_velocity = sys_get_be16(&frame->data[2]);
	motor->raw_torque = sys_get_be16(&frame->data[4]);
	motor->temperature_x10 = (int16_t)sys_get_be16(&frame->data[6]);
	motor->last_raw_position = position;
	motor->last_position_ms = now;
	motor->timestamp_ms = now;
	motor->mode_state = (enum robstride_mode_state)((frame->id >> 22) & 0x03U);
	motor->fault_bits = (uint8_t)((frame->id >> 16) & 0x3FU);
	motor->feedback_valid = true;
	k_spin_unlock(&motor->feedback_lock, key);
}

static void on_param_response(const struct device *motor_dev, const struct can_frame *frame)
{
	struct robstride_motor_data *motor = motor_dev->data;
	uint16_t index = sys_get_le16(&frame->data[0]);
	k_spinlock_key_t key = k_spin_lock(&motor->param_lock);
	if (!motor->pending_active || motor->pending_index != index) {
		k_spin_unlock(&motor->param_lock, key);
		return;
	}
	memcpy(motor->pending_value, &frame->data[4], 4);
	motor->pending_active = false;
	k_spin_unlock(&motor->param_lock, key);
	k_sem_give(&motor->param_sem);
}

static void rx_callback(const struct device *can_dev, struct can_frame *frame, void *user_data)
{
	const struct device *controller_dev = user_data;
	const struct robstride_controller_config *config = controller_dev->config;
	struct robstride_controller_data *controller = controller_dev->data;
	ARG_UNUSED(can_dev);
	if (!frame_flags_valid(frame) || robstride_id_target(frame->id) != config->master_id) {
		return;
	}
	const uint8_t source = robstride_id_source(frame->id);
	const struct device *motor_dev = find_motor(controller, source);
	if (motor_dev == NULL) {
		return;
	}
	switch (robstride_id_type(frame->id)) {
	case ROBSTRIDE_COMM_FEEDBACK:
		on_feedback(motor_dev, frame);
		break;
	case ROBSTRIDE_COMM_PARAM_READ:
		on_param_response(motor_dev, frame);
		break;
	case ROBSTRIDE_COMM_FAULT: {
		struct robstride_motor_data *motor = motor_dev->data;
		k_spinlock_key_t key = k_spin_lock(&motor->feedback_lock);
		motor->fault_raw = sys_get_le32(&frame->data[0]);
		motor->warning_raw = sys_get_le32(&frame->data[4]);
		k_spin_unlock(&motor->feedback_lock, key);
		break;
	}
	default:
		break;
	}
}

static int controller_init(const struct device *dev)
{
	const struct robstride_controller_config *config = dev->config;
	struct robstride_controller_data *data = dev->data;
	if (!device_is_ready(config->can_dev)) {
		return -ENODEV;
	}
	data->dev = dev;
	k_work_init(&data->tx_work, tx_work_handler);
	k_work_queue_start(&data->tx_queue, config->tx_stack, config->tx_stack_size,
			   CONFIG_MOTOR_ROBSTRIDE_TX_THREAD_PRIORITY, NULL);
	const struct can_filter filter = {
		.id = config->master_id,
		.mask = 0xFFU,
		.flags = CAN_FILTER_IDE,
	};
	int ret = can_add_rx_filter(config->can_dev, rx_callback, (void *)dev, &filter);
	if (ret < 0) {
		return ret;
	}
	if (!config->external_bus_management) {
		ret = can_start(config->can_dev);
		if (ret != 0 && ret != -EALREADY) {
			return ret;
		}
		data->started = true;
	}
	return 0;
}

static int motor_init(const struct device *dev)
{
	const struct robstride_motor_config *config = dev->config;
	struct robstride_controller_data *controller = config->controller->data;
	struct robstride_motor_data *motor = dev->data;
	if (!device_is_ready(config->controller) || config->motor_id == 0U ||
	    config->motor_id > ROBSTRIDE_MAX_ID) {
		return -EINVAL;
	}
	k_sem_init(&motor->param_sem, 0, 1);
	k_mutex_init(&motor->management_lock);
	k_spinlock_key_t key = k_spin_lock(&controller->lock);
	if (controller->motors[config->motor_id] != NULL) {
		k_spin_unlock(&controller->lock, key);
		return -EALREADY;
	}
	controller->motors[config->motor_id] = dev;
	controller->tx_contexts[config->motor_id].controller = controller;
	k_spin_unlock(&controller->lock, key);
	return 0;
}

int robstride_get_feedback(const struct device *motor_dev, struct robstride_feedback *out)
{
	if (motor_dev == NULL || out == NULL) {
		return -EINVAL;
	}
	const struct robstride_motor_config *config = motor_dev->config;
	const struct robstride_controller_config *controller_config = config->controller->config;
	struct robstride_motor_data *motor = motor_dev->data;
	uint16_t raw_position;
	uint16_t raw_velocity;
	uint16_t raw_torque;
	int16_t temperature_x10;
	int32_t turns;
	k_spinlock_key_t key = k_spin_lock(&motor->feedback_lock);
	memset(out, 0, sizeof(*out));
	out->valid = motor->feedback_valid;
	out->timestamp_ms = motor->timestamp_ms;
	out->position_turns = motor->position_turns;
	out->position_continuous = motor->position_continuous;
	out->continuity_epoch = motor->continuity_epoch;
	out->mode_state = motor->mode_state;
	out->fault_bits = motor->fault_bits;
	out->fault_raw = motor->fault_raw;
	out->warning_raw = motor->warning_raw;
	raw_position = motor->raw_position;
	raw_velocity = motor->raw_velocity;
	raw_torque = motor->raw_torque;
	temperature_x10 = motor->temperature_x10;
	turns = motor->position_turns;
	k_spin_unlock(&motor->feedback_lock, key);
	if (!out->valid) {
		return -ENODATA;
	}
	out->age_ms = k_uptime_get() - out->timestamp_ms;
	out->fresh = out->age_ms <= controller_config->feedback_timeout_ms;
	out->position_rad = robstride_u16_to_float(raw_position, ROBSTRIDE_POSITION_MIN_RAD,
						   ROBSTRIDE_POSITION_MAX_RAD) +
			    (float)turns * ROBSTRIDE_POSITION_SPAN_RAD;
	out->velocity_rad_s = robstride_u16_to_float(raw_velocity, ROBSTRIDE_VELOCITY_MIN_RAD_S,
						     ROBSTRIDE_VELOCITY_MAX_RAD_S);
	out->torque_nm = robstride_u16_to_float(raw_torque, ROBSTRIDE_TORQUE_MIN_NM,
						ROBSTRIDE_TORQUE_MAX_NM);
	out->temperature_c = (float)temperature_x10 / 10.0f;
	return out->fresh ? 0 : -EAGAIN;
}

int robstride_submit_position(const struct device *motor_dev, float position_rad, uint32_t epoch)
{
	if (motor_dev == NULL || !isfinite(position_rad)) {
		return -EINVAL;
	}
	if (position_rad < ROBSTRIDE_POSITION_MIN_RAD ||
	    position_rad > ROBSTRIDE_POSITION_MAX_RAD) {
		return -ERANGE;
	}
	const struct robstride_motor_config *config = motor_dev->config;
	struct robstride_controller_data *controller = config->controller->data;
	struct robstride_motor_data *motor = motor_dev->data;
	k_spinlock_key_t key = k_spin_lock(&controller->lock);
	if (controller->stop_latched || epoch != controller->stats.operation_epoch ||
	    !motor->committed || motor->prepared_epoch != epoch) {
		controller->stats.rejected_commands++;
		k_spin_unlock(&controller->lock, key);
		return -ESTALE;
	}
	motor->latest_position_rad = position_rad;
	motor->latest_command_ms = k_uptime_get();
	motor->command_epoch = epoch;
	motor->command_pending = true;
	k_spin_unlock(&controller->lock, key);
	k_work_submit_to_queue(&controller->tx_queue, &controller->tx_work);
	return 0;
}

int robstride_request_stop(const struct device *controller_dev, uint32_t epoch)
{
	if (controller_dev == NULL) {
		return -EINVAL;
	}
	struct robstride_controller_data *controller = controller_dev->data;
	k_spinlock_key_t key = k_spin_lock(&controller->lock);
	controller->stop_latched = true;
	controller->stop_epoch = epoch;
	controller->stats.operation_epoch = epoch;
	for (size_t id = 1; id <= ROBSTRIDE_MAX_ID; ++id) {
		if (controller->motors[id] == NULL) {
			continue;
		}
		struct robstride_motor_data *motor = controller->motors[id]->data;
		motor->command_pending = false;
		motor->prepared = false;
		motor->committed = false;
		k_spinlock_key_t param_key = k_spin_lock(&motor->param_lock);
		if (motor->pending_active) {
			motor->pending_active = false;
			motor->pending_canceled = true;
			k_sem_give(&motor->param_sem);
		}
		k_spin_unlock(&motor->param_lock, param_key);
	}
	k_spin_unlock(&controller->lock, key);
	k_work_submit_to_queue(&controller->tx_queue, &controller->tx_work);
	return 0;
}

int robstride_clear_fault(const struct device *motor_dev, uint32_t epoch)
{
	if (motor_dev == NULL) {
		return -EINVAL;
	}
	const struct robstride_motor_config *config = motor_dev->config;
	struct robstride_controller_data *controller = config->controller->data;
	if (epoch != controller->stats.operation_epoch) {
		return -ESTALE;
	}
	return send_stop_to_motor(motor_dev, true);
}

static int write_param(const struct device *motor_dev, uint16_t index, const uint8_t value[4],
		       uint32_t epoch)
{
	const struct robstride_motor_config *config = motor_dev->config;
	struct robstride_controller_data *controller = config->controller->data;
	if (epoch != controller->stats.operation_epoch) {
		return -ESTALE;
	}
	uint8_t data[8];
	robstride_put_param_header(data, index);
	memcpy(&data[4], value, 4);
	return send_management(motor_dev, ROBSTRIDE_COMM_PARAM_WRITE, data);
}

int robstride_write_param_u8(const struct device *motor, uint16_t index, uint8_t value,
			     uint32_t epoch)
{
	uint8_t data[4] = {value, 0, 0, 0};
	return write_param(motor, index, data, epoch);
}

int robstride_write_param_u32(const struct device *motor, uint16_t index, uint32_t value,
			      uint32_t epoch)
{
	uint8_t data[4];
	sys_put_le32(value, data);
	return write_param(motor, index, data, epoch);
}

int robstride_write_param_float(const struct device *motor, uint16_t index, float value,
				uint32_t epoch)
{
	if (!isfinite(value)) {
		return -EINVAL;
	}
	uint8_t data[4];
	robstride_put_float_le(data, value);
	return write_param(motor, index, data, epoch);
}

static int read_param(const struct device *motor_dev, uint16_t index, enum pending_type type,
		      uint8_t out[4], k_timeout_t timeout)
{
	struct robstride_motor_data *motor = motor_dev->data;
	if (k_mutex_lock(&motor->management_lock, K_NO_WAIT) != 0) {
		return -EBUSY;
	}
	k_sem_reset(&motor->param_sem);
	k_spinlock_key_t key = k_spin_lock(&motor->param_lock);
	motor->pending_index = index;
	motor->pending_type = type;
	motor->pending_active = true;
	motor->pending_canceled = false;
	k_spin_unlock(&motor->param_lock, key);
	uint8_t data[8];
	robstride_put_param_header(data, index);
	int ret = send_management(motor_dev, ROBSTRIDE_COMM_PARAM_READ, data);
	if (ret != 0) {
		key = k_spin_lock(&motor->param_lock);
		motor->pending_active = false;
		k_spin_unlock(&motor->param_lock, key);
		k_mutex_unlock(&motor->management_lock);
		return ret;
	}
	ret = k_sem_take(&motor->param_sem, timeout);
	if (ret != 0) {
		key = k_spin_lock(&motor->param_lock);
		motor->pending_active = false;
		k_spin_unlock(&motor->param_lock, key);
		k_mutex_unlock(&motor->management_lock);
		return -ETIMEDOUT;
	}
	key = k_spin_lock(&motor->param_lock);
	bool canceled = motor->pending_canceled;
	if (!canceled) {
		memcpy(out, motor->pending_value, 4);
	}
	k_spin_unlock(&motor->param_lock, key);
	if (canceled) {
		k_mutex_unlock(&motor->management_lock);
		return -ECANCELED;
	}
	k_mutex_unlock(&motor->management_lock);
	return 0;
}

int robstride_read_param_u8(const struct device *motor, uint16_t index, uint8_t *out,
			    k_timeout_t timeout)
{
	if (motor == NULL || out == NULL) {
		return -EINVAL;
	}
	uint8_t raw[4];
	int ret = read_param(motor, index, PENDING_U8, raw, timeout);
	if (ret == 0) {
		*out = raw[0];
	}
	return ret;
}

int robstride_read_param_u32(const struct device *motor, uint16_t index, uint32_t *out,
			     k_timeout_t timeout)
{
	if (motor == NULL || out == NULL) {
		return -EINVAL;
	}
	uint8_t raw[4];
	int ret = read_param(motor, index, PENDING_U32, raw, timeout);
	if (ret == 0) {
		*out = sys_get_le32(raw);
	}
	return ret;
}

int robstride_read_param_float(const struct device *motor, uint16_t index, float *out,
			       k_timeout_t timeout)
{
	if (motor == NULL || out == NULL) {
		return -EINVAL;
	}
	uint8_t raw[4];
	int ret = read_param(motor, index, PENDING_FLOAT, raw, timeout);
	if (ret == 0) {
		float value = robstride_get_float_le(raw);
		if (!isfinite(value)) {
			return -EBADMSG;
		}
		*out = value;
	}
	return ret;
}

int robstride_prepare_csp(const struct device *motor_dev, const struct robstride_csp_config *config,
			  uint32_t epoch)
{
	if (motor_dev == NULL || config == NULL || config->can_timeout_ms == 0U ||
	    !isfinite(config->speed_limit_rad_s) || !isfinite(config->current_limit_a) ||
	    config->speed_limit_rad_s <= 0.0f || config->speed_limit_rad_s > 33.0f ||
	    config->current_limit_a <= 0.0f || config->current_limit_a > 16.0f ||
	    config->can_timeout_ms > UINT32_MAX / ROBSTRIDE_TIMEOUT_TICKS_PER_MS) {
		return -EINVAL;
	}
	const struct robstride_motor_config *motor_config = motor_dev->config;
	struct robstride_controller_data *controller = motor_config->controller->data;
	struct robstride_motor_data *motor = motor_dev->data;
	if (epoch != controller->stats.operation_epoch) {
		return -ESTALE;
	}
	int ret = send_stop_to_motor(motor_dev, false);
	if (ret != 0) {
		return ret;
	}
	k_usleep(ROBSTRIDE_INTER_FRAME_US);
	ret = robstride_write_param_u32(motor_dev, ROBSTRIDE_PARAM_CAN_TIMEOUT,
					config->can_timeout_ms * ROBSTRIDE_TIMEOUT_TICKS_PER_MS,
					epoch);
	if (ret != 0) {
		return ret;
	}
	ret = robstride_write_param_u8(motor_dev, ROBSTRIDE_PARAM_RUN_MODE, 5U, epoch);
	if (ret != 0) {
		return ret;
	}
	uint8_t mode;
	ret = robstride_read_param_u8(motor_dev, ROBSTRIDE_PARAM_RUN_MODE, &mode, K_MSEC(50));
	if (ret != 0 || mode != 5U) {
		return ret != 0 ? ret : -EBADMSG;
	}
	float mech_pos;
	ret = robstride_read_param_float(motor_dev, ROBSTRIDE_PARAM_MECH_POS, &mech_pos,
					 K_MSEC(50));
	if (ret != 0 || mech_pos < ROBSTRIDE_POSITION_MIN_RAD ||
	    mech_pos > ROBSTRIDE_POSITION_MAX_RAD) {
		return ret != 0 ? ret : -ERANGE;
	}
	ret = robstride_write_param_float(motor_dev, ROBSTRIDE_PARAM_LOC_REF, mech_pos, epoch);
	if (ret != 0) {
		return ret;
	}
	motor->csp_config = *config;
	motor->prepared = true;
	motor->committed = false;
	motor->prepared_epoch = epoch;
	return 0;
}

int robstride_commit_enable(const struct device *motor_dev, uint32_t epoch)
{
	if (motor_dev == NULL) {
		return -EINVAL;
	}
	const struct robstride_motor_config *config = motor_dev->config;
	struct robstride_controller_data *controller = config->controller->data;
	struct robstride_motor_data *motor = motor_dev->data;
	if (!motor->prepared || motor->prepared_epoch != epoch ||
	    controller->stats.operation_epoch != epoch) {
		return -ESTALE;
	}
	int ret = robstride_write_param_float(motor_dev, ROBSTRIDE_PARAM_LIMIT_SPD,
					      motor->csp_config.speed_limit_rad_s, epoch);
	if (ret == 0) {
		ret = robstride_write_param_float(motor_dev, ROBSTRIDE_PARAM_LIMIT_CUR,
						  motor->csp_config.current_limit_a, epoch);
	}
	if (ret == 0) {
		ret = send_management(motor_dev, ROBSTRIDE_COMM_ENABLE, NULL);
	}
	if (ret != 0) {
		(void)send_stop_to_motor(motor_dev, false);
		return ret;
	}
	motor->committed = true;
	controller->stop_latched = false;
	return 0;
}

int robstride_rebase_position_from_mech_pos(const struct device *motor_dev, uint32_t epoch)
{
	if (motor_dev == NULL) {
		return -EINVAL;
	}
	const struct robstride_motor_config *config = motor_dev->config;
	struct robstride_controller_data *controller = config->controller->data;
	struct robstride_motor_data *motor = motor_dev->data;
	if (epoch != controller->stats.operation_epoch) {
		return -ESTALE;
	}
	float mech_pos;
	int ret = robstride_read_param_float(motor_dev, ROBSTRIDE_PARAM_MECH_POS, &mech_pos,
					     K_MSEC(50));
	if (ret != 0) {
		return ret;
	}
	k_spinlock_key_t key = k_spin_lock(&motor->feedback_lock);
	if (!motor->feedback_valid) {
		k_spin_unlock(&motor->feedback_lock, key);
		return -ENODATA;
	}
	float raw_rad = robstride_u16_to_float(motor->raw_position, ROBSTRIDE_POSITION_MIN_RAD,
					       ROBSTRIDE_POSITION_MAX_RAD);
	int32_t turns = (int32_t)lroundf((mech_pos - raw_rad) / ROBSTRIDE_POSITION_SPAN_RAD);
	float error = fabsf((raw_rad + turns * ROBSTRIDE_POSITION_SPAN_RAD) - mech_pos);
	if (error > 0.1f) {
		k_spin_unlock(&motor->feedback_lock, key);
		return -EBADMSG;
	}
	motor->position_turns = turns;
	motor->position_continuous = true;
	motor->continuity_epoch++;
	k_spin_unlock(&motor->feedback_lock, key);
	return 0;
}

int robstride_transport_start(const struct device *controller_dev)
{
	if (controller_dev == NULL) {
		return -EINVAL;
	}
	struct robstride_controller_data *controller = controller_dev->data;
	controller->started = true;
	return 0;
}

int robstride_transport_stop(const struct device *controller_dev)
{
	if (controller_dev == NULL) {
		return -EINVAL;
	}
	struct robstride_controller_data *controller = controller_dev->data;
	(void)robstride_request_stop(controller_dev, controller->stats.operation_epoch + 1U);
	return 0;
}

int robstride_transport_get_stats(const struct device *controller_dev,
				  struct robstride_transport_stats *out)
{
	if (controller_dev == NULL || out == NULL) {
		return -EINVAL;
	}
	struct robstride_controller_data *controller = controller_dev->data;
	k_spinlock_key_t key = k_spin_lock(&controller->lock);
	*out = controller->stats;
	k_spin_unlock(&controller->lock, key);
	return 0;
}

#define DT_DRV_COMPAT robstride_controller
#define CONTROLLER_DEFINE(inst)                                                                    \
	K_THREAD_STACK_DEFINE(robstride_tx_stack_##inst, CONFIG_MOTOR_ROBSTRIDE_TX_STACK_SIZE);    \
	static struct robstride_controller_data controller_data_##inst;                            \
	static const struct robstride_controller_config controller_config_##inst = {               \
		.can_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can)),                              \
		.master_id = DT_INST_PROP_OR(inst, master_id, 0xFD),                               \
		.feedback_timeout_ms = DT_INST_PROP_OR(inst, feedback_timeout_ms, 100),            \
		.command_timeout_ms = DT_INST_PROP_OR(inst, command_timeout_ms, 100),              \
		.external_bus_management = DT_INST_PROP(inst, external_bus_management),            \
		.tx_stack = robstride_tx_stack_##inst,                                             \
		.tx_stack_size = K_THREAD_STACK_SIZEOF(robstride_tx_stack_##inst),                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, controller_init, NULL, &controller_data_##inst,                \
			      &controller_config_##inst, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY,  \
			      NULL);
DT_INST_FOREACH_STATUS_OKAY(CONTROLLER_DEFINE)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT robstride_rs00
#define MOTOR_DEFINE(inst)                                                                         \
	static struct robstride_motor_data robstride_motor_data_##inst;                            \
	static const struct robstride_motor_config robstride_motor_config_##inst = {               \
		.controller = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                 \
		.motor_id = DT_INST_REG_ADDR(inst),                                                \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, motor_init, NULL, &robstride_motor_data_##inst,                \
			      &robstride_motor_config_##inst, POST_KERNEL,                         \
			      CONFIG_MOTOR_INIT_PRIORITY, NULL);
DT_INST_FOREACH_STATUS_OKAY(MOTOR_DEFINE)
