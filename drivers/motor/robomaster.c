/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <string.h>

#include <drivers/motor.h>
#include <drivers/motor/robomaster.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(motor_dji_robomaster, CONFIG_MOTOR_LOG_LEVEL);

#define ROBOMASTER_MAX_MOTORS              8
#define ROBOMASTER_GROUP_SIZE              4
#define ROBOMASTER_GROUP_COUNT             2
#define ROBOMASTER_MAX_CANS                4
#define ROBOMASTER_RX_ID_BASE              0x200U
#define ROBOMASTER_TX_ID_GROUP0            0x200U
#define ROBOMASTER_TX_ID_GROUP1            0x1FFU
#define ROBOMASTER_RX_FILTER_MASK          0x7F0U
#define ROBOMASTER_ENCODER_WRAP            8192
#define ROBOMASTER_ENCODER_HALF_WRAP       (ROBOMASTER_ENCODER_WRAP / 2)
#define ROBOMASTER_DEFAULT_MAX_CURRENT     10000
#define ROBOMASTER_DEFAULT_ROTOR_BOUND_RPM 10000
#define ROBOMASTER_CANBUS_UNKNOWN          0xFF
#define ROBOMASTER_START_RETRY_MS          500
#define ROBOMASTER_C610_COUNTS_PER_AMP     1000.0f

enum robomaster_model {
	ROBOMASTER_MODEL_C610 = 0,
	ROBOMASTER_MODEL_C620 = 1
};

struct robomaster_transport_data;

struct robomaster_tx_context {
	struct robomaster_transport_data *transport;
	atomic_t pending;
};

struct robomaster_transport_config {
	const struct device *const *can_devs;
	size_t can_count;
	uint32_t feedback_timeout_ms;
	uint32_t tx_period_us;
	uint32_t command_timeout_ms;
	bool external_bus_management;
	k_thread_stack_t *tx_stack;
	size_t tx_stack_size;
};

struct robomaster_transport_data {
	struct k_spinlock lock;
	const struct device *dev;
	const struct device *motors[ROBOMASTER_MAX_MOTORS];
	struct k_work tx_work;
	struct k_timer tx_timer;
	struct k_work_q tx_queue;
	struct robomaster_tx_context tx_contexts[ROBOMASTER_MAX_CANS][ROBOMASTER_GROUP_COUNT];
	bool started[ROBOMASTER_MAX_CANS];
	int64_t last_start_retry;
	struct robomaster_transport_stats stats;
};

struct robomaster_motor_config {
	const struct device *transport;
	uint8_t motor_id;
	enum robomaster_model model;
	int16_t max_current;
	int32_t rotor_speed_bound_rpm;
};

struct robomaster_motor_data {
	struct k_spinlock feedback_lock;
	bool enabled;
	bool stop_latched;
	bool command_timed_out;
	uint8_t detected_can_bus;
	int16_t requested_current;
	int64_t last_command_ms;
	bool has_last_orientation;
	uint16_t last_orientation_raw;
	int64_t continuous_count;
	uint32_t continuity_breaks;
	uint32_t continuity_epoch;
	bool continuous_valid;
	struct motor_feedback feedback;
};

static int32_t gap_limit_ms(int32_t bound_rpm)
{
	return (30000 / bound_rpm) - 1;
}

static int32_t saturate_i64_to_i32(int64_t value)
{
	return value > INT32_MAX ? INT32_MAX : value < INT32_MIN ? INT32_MIN : (int32_t)value;
}

static int find_can_bus(const struct robomaster_transport_config *config,
			const struct device *can_dev)
{
	for (size_t i = 0; i < config->can_count; ++i) {
		if (config->can_devs[i] == can_dev) {
			return (int)i;
		}
	}
	return -ENODEV;
}

static void invalidate_continuity(struct robomaster_motor_data *motor)
{
	if (motor->continuous_valid) {
		motor->continuous_valid = false;
		motor->continuity_breaks++;
		motor->continuity_epoch++;
	}
}

static void tx_callback(const struct device *dev, int error, void *user_data)
{
	struct robomaster_tx_context *context = user_data;
	struct robomaster_transport_data *transport = context->transport;
	ARG_UNUSED(dev);
	k_spinlock_key_t key = k_spin_lock(&transport->lock);
	if (error == 0) {
		transport->stats.tx_completed++;
	} else {
		transport->stats.tx_errors++;
	}
	k_spin_unlock(&transport->lock, key);
	atomic_clear(&context->pending);
}

static void retry_start(const struct device *dev)
{
	const struct robomaster_transport_config *config = dev->config;
	struct robomaster_transport_data *data = dev->data;
	const int64_t now = k_uptime_get();
	if (config->external_bus_management ||
	    (now - data->last_start_retry) < ROBOMASTER_START_RETRY_MS) {
		return;
	}
	data->last_start_retry = now;
	for (size_t i = 0; i < config->can_count; ++i) {
		if (data->started[i]) {
			continue;
		}
		int ret = can_start(config->can_devs[i]);
		if (ret == 0 || ret == -EALREADY) {
			data->started[i] = true;
			LOG_INF("CAN bus %u started", (unsigned int)i);
		}
	}
}

static bool snapshot_group(struct robomaster_transport_data *transport, size_t can_bus,
			   size_t group, struct can_frame *frame)
{
	const int64_t now = k_uptime_get();
	const struct robomaster_transport_config *config = transport->dev->config;
	bool has_motor = false;
	memset(frame->data, 0, sizeof(frame->data));
	k_spinlock_key_t key = k_spin_lock(&transport->lock);
	for (size_t slot = 0; slot < ROBOMASTER_GROUP_SIZE; ++slot) {
		const struct device *motor_dev =
			transport->motors[group * ROBOMASTER_GROUP_SIZE + slot];
		if (motor_dev == NULL) {
			continue;
		}
		struct robomaster_motor_data *motor = motor_dev->data;
		if (motor->detected_can_bus != can_bus) {
			continue;
		}
		has_motor = true;
		if (motor->enabled && !motor->stop_latched && !motor->command_timed_out &&
		    config->command_timeout_ms > 0U &&
		    (now - motor->last_command_ms) > config->command_timeout_ms) {
			motor->command_timed_out = true;
			motor->enabled = false;
			motor->requested_current = 0;
			transport->stats.command_timeouts++;
		}
		int16_t command =
			(motor->enabled && !motor->stop_latched && !motor->command_timed_out)
				? motor->requested_current
				: 0;
		sys_put_be16((uint16_t)command, &frame->data[slot * sizeof(int16_t)]);
	}
	k_spin_unlock(&transport->lock, key);
	return has_motor;
}

static void tx_work_handler(struct k_work *work)
{
	struct robomaster_transport_data *transport =
		CONTAINER_OF(work, struct robomaster_transport_data, tx_work);
	const struct robomaster_transport_config *config = transport->dev->config;
	retry_start(transport->dev);
	for (size_t bus = 0; bus < config->can_count; ++bus) {
		if (!transport->started[bus]) {
			continue;
		}
		for (size_t group = 0; group < ROBOMASTER_GROUP_COUNT; ++group) {
			struct robomaster_tx_context *context = &transport->tx_contexts[bus][group];
			if (!atomic_cas(&context->pending, 0, 1)) {
				continue;
			}
			struct can_frame frame = {
				.id = group == 0U ? ROBOMASTER_TX_ID_GROUP0
						  : ROBOMASTER_TX_ID_GROUP1,
				.dlc = can_bytes_to_dlc(8U),
				.flags = 0U,
			};
			if (!snapshot_group(transport, bus, group, &frame)) {
				atomic_clear(&context->pending);
				continue;
			}
			int ret = can_send(config->can_devs[bus], &frame, K_NO_WAIT, tx_callback,
					   context);
			k_spinlock_key_t key = k_spin_lock(&transport->lock);
			if (ret == 0) {
				transport->stats.tx_accepted++;
			} else {
				transport->stats.tx_errors++;
				atomic_clear(&context->pending);
			}
			k_spin_unlock(&transport->lock, key);
		}
	}
}

static void tx_timer_handler(struct k_timer *timer)
{
	struct robomaster_transport_data *transport = k_timer_user_data_get(timer);
	k_work_submit_to_queue(&transport->tx_queue, &transport->tx_work);
}

static void rx_callback(const struct device *can_dev, struct can_frame *frame, void *user_data)
{
	if ((frame->flags & (CAN_FRAME_IDE | CAN_FRAME_RTR | CAN_FRAME_FDF | CAN_FRAME_BRS)) !=
		    0U ||
	    can_dlc_to_bytes(frame->dlc) != 8U || frame->id < ROBOMASTER_RX_ID_BASE + 1U ||
	    frame->id > ROBOMASTER_RX_ID_BASE + ROBOMASTER_MAX_MOTORS) {
		return;
	}
	const uint16_t orientation_raw = sys_get_be16(&frame->data[0]);
	if (orientation_raw >= ROBOMASTER_ENCODER_WRAP) {
		return;
	}
	const struct device *transport_dev = user_data;
	const struct robomaster_transport_config *config = transport_dev->config;
	struct robomaster_transport_data *transport = transport_dev->data;
	const uint8_t motor_id = (uint8_t)(frame->id - ROBOMASTER_RX_ID_BASE);
	const int can_bus = find_can_bus(config, can_dev);
	if (can_bus < 0 || transport->motors[motor_id - 1U] == NULL) {
		return;
	}
	const struct device *motor_dev = transport->motors[motor_id - 1U];
	const struct robomaster_motor_config *motor_config = motor_dev->config;
	struct robomaster_motor_data *motor = motor_dev->data;
	k_spinlock_key_t transport_key = k_spin_lock(&transport->lock);
	if (motor->detected_can_bus != ROBOMASTER_CANBUS_UNKNOWN &&
	    motor->detected_can_bus != can_bus) {
		k_spin_unlock(&transport->lock, transport_key);
		return;
	}
	motor->detected_can_bus = (uint8_t)can_bus;
	k_spin_unlock(&transport->lock, transport_key);

	const int64_t now = k_uptime_get();
	const int16_t velocity = (int16_t)sys_get_be16(&frame->data[2]);
	const int16_t current = (int16_t)sys_get_be16(&frame->data[4]);
	const int32_t limit_ms = gap_limit_ms(motor_config->rotor_speed_bound_rpm);
	k_spinlock_key_t key = k_spin_lock(&motor->feedback_lock);
	if (!motor->has_last_orientation) {
		motor->continuous_count = orientation_raw;
		motor->continuous_valid = true;
		motor->has_last_orientation = true;
	} else {
		const int64_t gap_ms = now - motor->feedback.timestamp_ms;
		const int32_t velocity_i32 = velocity;
		const int32_t abs_velocity = velocity_i32 < 0 ? -velocity_i32 : velocity_i32;
		if (gap_ms < 0 || gap_ms > limit_ms ||
		    abs_velocity > motor_config->rotor_speed_bound_rpm) {
			invalidate_continuity(motor);
		} else if (motor->continuous_valid) {
			int32_t delta =
				(int32_t)orientation_raw - (int32_t)motor->last_orientation_raw;
			if (delta > ROBOMASTER_ENCODER_HALF_WRAP) {
				delta -= ROBOMASTER_ENCODER_WRAP;
			} else if (delta < -ROBOMASTER_ENCODER_HALF_WRAP) {
				delta += ROBOMASTER_ENCODER_WRAP;
			}
			motor->continuous_count += delta;
		}
	}
	motor->last_orientation_raw = orientation_raw;
	motor->feedback.valid_mask = MOTOR_FEEDBACK_CURRENT | MOTOR_FEEDBACK_VELOCITY |
				     MOTOR_FEEDBACK_POSITION | MOTOR_FEEDBACK_ORIENTATION;
	if (motor_config->model == ROBOMASTER_MODEL_C620) {
		motor->feedback.valid_mask |= MOTOR_FEEDBACK_TEMPERATURE;
		motor->feedback.temperature = frame->data[6];
	} else {
		motor->feedback.temperature = 0;
	}
	motor->feedback.current = current;
	motor->feedback.velocity = velocity;
	motor->feedback.orientation = orientation_raw;
	motor->feedback.position = saturate_i64_to_i32(motor->continuous_count);
	motor->feedback.online = true;
	motor->feedback.stale = false;
	motor->feedback.timestamp_ms = now;
	k_spin_unlock(&motor->feedback_lock, key);
	k_work_submit_to_queue(&transport->tx_queue, &transport->tx_work);
}

static int transport_init(const struct device *dev)
{
	const struct robomaster_transport_config *config = dev->config;
	struct robomaster_transport_data *data = dev->data;
	const struct can_filter filter = {
		.id = ROBOMASTER_RX_ID_BASE,
		.mask = ROBOMASTER_RX_FILTER_MASK,
		.flags = 0U,
	};
	if (config->can_count > ROBOMASTER_MAX_CANS || config->tx_period_us == 0U) {
		return -EINVAL;
	}
	data->dev = dev;
	k_work_init(&data->tx_work, tx_work_handler);
	k_work_queue_start(&data->tx_queue, config->tx_stack, config->tx_stack_size,
			   CONFIG_MOTOR_DJI_ROBOMASTER_TX_THREAD_PRIORITY, NULL);
	k_timer_init(&data->tx_timer, tx_timer_handler, NULL);
	k_timer_user_data_set(&data->tx_timer, data);
	for (size_t bus = 0; bus < config->can_count; ++bus) {
		for (size_t group = 0; group < ROBOMASTER_GROUP_COUNT; ++group) {
			data->tx_contexts[bus][group].transport = data;
		}
		if (!device_is_ready(config->can_devs[bus])) {
			return -ENODEV;
		}
		int filter_id =
			can_add_rx_filter(config->can_devs[bus], rx_callback, (void *)dev, &filter);
		if (filter_id < 0) {
			return filter_id;
		}
		if (!config->external_bus_management) {
			int ret = can_start(config->can_devs[bus]);
			if (ret == 0 || ret == -EALREADY) {
				data->started[bus] = true;
			} else {
				LOG_WRN("CAN bus %u not startable yet (%d), retrying",
					(unsigned int)bus, ret);
			}
		}
	}
	data->last_start_retry = k_uptime_get();
	k_timer_start(&data->tx_timer, K_USEC(config->tx_period_us), K_USEC(config->tx_period_us));
	return 0;
}

static int register_motor(const struct device *transport_dev, const struct device *motor_dev,
			  uint8_t motor_id)
{
	struct robomaster_transport_data *transport = transport_dev->data;
	if (motor_id < 1U || motor_id > ROBOMASTER_MAX_MOTORS) {
		return -EINVAL;
	}
	k_spinlock_key_t key = k_spin_lock(&transport->lock);
	if (transport->motors[motor_id - 1U] != NULL) {
		k_spin_unlock(&transport->lock, key);
		return -EALREADY;
	}
	struct robomaster_motor_data *motor = motor_dev->data;
	motor->detected_can_bus = ROBOMASTER_CANBUS_UNKNOWN;
	transport->motors[motor_id - 1U] = motor_dev;
	k_spin_unlock(&transport->lock, key);
	return 0;
}

static int motor_enable_impl(const struct device *dev)
{
	const struct robomaster_motor_config *config = dev->config;
	struct robomaster_transport_data *transport = config->transport->data;
	struct robomaster_motor_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&transport->lock);
	data->enabled = true;
	data->stop_latched = false;
	data->command_timed_out = false;
	data->last_command_ms = k_uptime_get();
	k_spin_unlock(&transport->lock, key);
	return 0;
}

static int motor_disable_impl(const struct device *dev)
{
	const struct robomaster_motor_config *config = dev->config;
	struct robomaster_transport_data *transport = config->transport->data;
	struct robomaster_motor_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&transport->lock);
	data->enabled = false;
	k_spin_unlock(&transport->lock, key);
	return 0;
}

static int motor_set_output_impl(const struct device *dev, enum motor_output_mode mode,
				 int16_t output)
{
	const struct robomaster_motor_config *config = dev->config;
	struct robomaster_transport_data *transport = config->transport->data;
	struct robomaster_motor_data *data = dev->data;
	if (mode != MOTOR_OUTPUT_MODE_CURRENT && mode != MOTOR_OUTPUT_MODE_TORQUE) {
		return -ENOTSUP;
	}
	k_spinlock_key_t key = k_spin_lock(&transport->lock);
	data->requested_current = CLAMP(output, -config->max_current, config->max_current);
	data->last_command_ms = k_uptime_get();
	k_spin_unlock(&transport->lock, key);
	return 0;
}

static int motor_get_feedback_impl(const struct device *dev, void *feedback)
{
	const struct robomaster_motor_config *config = dev->config;
	const struct robomaster_transport_config *transport_config = config->transport->config;
	struct robomaster_motor_data *data = dev->data;
	struct motor_feedback *out = feedback;
	if (out == NULL) {
		return -EINVAL;
	}
	k_spinlock_key_t key = k_spin_lock(&data->feedback_lock);
	*out = data->feedback;
	int ret = 0;
	if (!data->feedback.online) {
		ret = -ENODATA;
	} else if (transport_config->feedback_timeout_ms > 0U &&
		   (k_uptime_get() - data->feedback.timestamp_ms) >
			   transport_config->feedback_timeout_ms) {
		out->stale = true;
		ret = -EAGAIN;
	}
	k_spin_unlock(&data->feedback_lock, key);
	return ret;
}

int robomaster_get_feedback_ex(const struct device *motor, struct robomaster_feedback_ex *out)
{
	if (motor == NULL || out == NULL) {
		return -EINVAL;
	}
	const struct robomaster_motor_config *config = motor->config;
	struct robomaster_motor_data *data = motor->data;
	const int32_t limit_ms = gap_limit_ms(config->rotor_speed_bound_rpm);
	k_spinlock_key_t key = k_spin_lock(&data->feedback_lock);
	out->raw = data->feedback;
	out->continuous_count = data->continuous_count;
	out->continuity_breaks = data->continuity_breaks;
	out->continuity_epoch = data->continuity_epoch;
	out->rotor_speed_bound_rpm = config->rotor_speed_bound_rpm;
	out->continuity_gap_limit_ms = limit_ms;
	out->continuous_valid = data->continuous_valid;
	int ret = 0;
	if (!data->feedback.online) {
		ret = -ENODATA;
	} else if ((k_uptime_get() - data->feedback.timestamp_ms) > limit_ms) {
		out->raw.stale = true;
		out->continuous_valid = false;
		ret = -EAGAIN;
	}
	k_spin_unlock(&data->feedback_lock, key);
	return ret;
}

int robomaster_reset_continuous_position(const struct device *motor)
{
	if (motor == NULL) {
		return -EINVAL;
	}
	const struct robomaster_motor_config *config = motor->config;
	struct robomaster_motor_data *data = motor->data;
	const int32_t limit_ms = gap_limit_ms(config->rotor_speed_bound_rpm);
	k_spinlock_key_t key = k_spin_lock(&data->feedback_lock);
	const int32_t velocity_i32 = data->feedback.velocity;
	const int32_t abs_velocity = velocity_i32 < 0 ? -velocity_i32 : velocity_i32;
	if (!data->feedback.online || (k_uptime_get() - data->feedback.timestamp_ms) > limit_ms ||
	    abs_velocity > config->rotor_speed_bound_rpm) {
		k_spin_unlock(&data->feedback_lock, key);
		return -EAGAIN;
	}
	data->continuous_count = 0;
	data->feedback.position = 0;
	data->continuous_valid = true;
	data->continuity_epoch++;
	k_spin_unlock(&data->feedback_lock, key);
	return 0;
}

static int current_to_raw(const struct robomaster_motor_config *config, float current_a,
			  int16_t *raw)
{
	if (config->model != ROBOMASTER_MODEL_C610) {
		return -ENOTSUP;
	}
	if (!isfinite(current_a)) {
		return -EINVAL;
	}
	float limited = CLAMP(current_a * ROBOMASTER_C610_COUNTS_PER_AMP,
			      -(float)config->max_current, (float)config->max_current);
	*raw = (int16_t)limited;
	return 0;
}

int robomaster_c610_set_current_a(const struct device *motor, float current_a)
{
	if (motor == NULL) {
		return -EINVAL;
	}
	int16_t raw;
	int ret = current_to_raw(motor->config, current_a, &raw);
	return ret == 0 ? motor_set_output_impl(motor, MOTOR_OUTPUT_MODE_CURRENT, raw) : ret;
}

int robomaster_c610_set_pair_current_a(const struct device *right, const struct device *left,
				       float right_a, float left_a)
{
	if (right == NULL || left == NULL || right == left) {
		return -EINVAL;
	}
	const struct robomaster_motor_config *right_config = right->config;
	const struct robomaster_motor_config *left_config = left->config;
	if (right_config->transport != left_config->transport) {
		return -EXDEV;
	}
	int16_t right_raw;
	int16_t left_raw;
	int ret = current_to_raw(right_config, right_a, &right_raw);
	if (ret != 0) {
		return ret;
	}
	ret = current_to_raw(left_config, left_a, &left_raw);
	if (ret != 0) {
		return ret;
	}
	struct robomaster_transport_data *transport = right_config->transport->data;
	struct robomaster_motor_data *right_data = right->data;
	struct robomaster_motor_data *left_data = left->data;
	const int64_t now = k_uptime_get();
	k_spinlock_key_t key = k_spin_lock(&transport->lock);
	right_data->requested_current = right_raw;
	left_data->requested_current = left_raw;
	right_data->last_command_ms = now;
	left_data->last_command_ms = now;
	k_spin_unlock(&transport->lock, key);
	return 0;
}

int robomaster_request_stop(const struct device *transport_dev, uint8_t motor_mask)
{
	if (transport_dev == NULL) {
		return -EINVAL;
	}
	struct robomaster_transport_data *transport = transport_dev->data;
	k_spinlock_key_t key = k_spin_lock(&transport->lock);
	transport->stats.operation_epoch++;
	for (size_t i = 0; i < ROBOMASTER_MAX_MOTORS; ++i) {
		if ((motor_mask & BIT(i)) == 0U || transport->motors[i] == NULL) {
			continue;
		}
		struct robomaster_motor_data *motor = transport->motors[i]->data;
		motor->requested_current = 0;
		motor->enabled = false;
		motor->stop_latched = true;
	}
	k_spin_unlock(&transport->lock, key);
	k_work_submit_to_queue(&transport->tx_queue, &transport->tx_work);
	return 0;
}

int robomaster_transport_start(const struct device *transport_dev)
{
	if (transport_dev == NULL) {
		return -EINVAL;
	}
	const struct robomaster_transport_config *config = transport_dev->config;
	struct robomaster_transport_data *transport = transport_dev->data;
	k_spinlock_key_t key = k_spin_lock(&transport->lock);
	if (config->external_bus_management) {
		for (size_t i = 0; i < config->can_count; ++i) {
			transport->started[i] = true;
		}
	}
	k_spin_unlock(&transport->lock, key);
	return 0;
}

int robomaster_transport_stop(const struct device *transport_dev)
{
	return robomaster_request_stop(transport_dev, UINT8_MAX);
}

int robomaster_transport_get_stats(const struct device *transport_dev,
				   struct robomaster_transport_stats *out)
{
	if (transport_dev == NULL || out == NULL) {
		return -EINVAL;
	}
	struct robomaster_transport_data *transport = transport_dev->data;
	k_spinlock_key_t key = k_spin_lock(&transport->lock);
	*out = transport->stats;
	k_spin_unlock(&transport->lock, key);
	return 0;
}

static int motor_init(const struct device *dev)
{
	const struct robomaster_motor_config *config = dev->config;
	if (!device_is_ready(config->transport) || config->motor_id < 1U ||
	    config->motor_id > ROBOMASTER_MAX_MOTORS || config->rotor_speed_bound_rpm <= 0 ||
	    gap_limit_ms(config->rotor_speed_bound_rpm) < 0) {
		return -EINVAL;
	}
	return register_motor(config->transport, dev, config->motor_id);
}

static const struct motor_driver_api robomaster_motor_api = {
	.enable = motor_enable_impl,
	.disable = motor_disable_impl,
	.set_output = motor_set_output_impl,
	.get_feedback = motor_get_feedback_impl,
};

#define DT_DRV_COMPAT           dji_robomaster
#define CAN_DEV_ELEM(idx, inst) DEVICE_DT_GET(DT_INST_PHANDLE_BY_IDX(inst, cans, idx))
#define TRANSPORT_DEFINE(inst)                                                                     \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, cans) <= ROBOMASTER_MAX_CANS,                          \
		     "Too many CAN devices configured for RoboMaster transport");                  \
	static const struct device *const can_devs_##inst[] = {                                    \
		LISTIFY(DT_INST_PROP_LEN(inst, cans), CAN_DEV_ELEM, (, ), inst)};                       \
	K_THREAD_STACK_DEFINE(tx_stack_##inst, CONFIG_MOTOR_DJI_ROBOMASTER_TX_STACK_SIZE);         \
	static struct robomaster_transport_data transport_data_##inst;                             \
	static const struct robomaster_transport_config transport_config_##inst = {                \
		.can_devs = can_devs_##inst,                                                       \
		.can_count = DT_INST_PROP_LEN(inst, cans),                                         \
		.feedback_timeout_ms = DT_INST_PROP_OR(inst, feedback_timeout_ms, 100),            \
		.tx_period_us = DT_INST_PROP_OR(inst, tx_period_us, 1000),                         \
		.command_timeout_ms = DT_INST_PROP_OR(inst, command_timeout_ms, 0),                \
		.external_bus_management = DT_INST_PROP(inst, external_bus_management),            \
		.tx_stack = tx_stack_##inst,                                                       \
		.tx_stack_size = K_THREAD_STACK_SIZEOF(tx_stack_##inst),                           \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, transport_init, NULL, &transport_data_##inst,                  \
			      &transport_config_##inst, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY,   \
			      NULL);
DT_INST_FOREACH_STATUS_OKAY(TRANSPORT_DEFINE)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT dji_robomaster_motor
#define MOTOR_DEFINE(inst)                                                                         \
	static struct robomaster_motor_data motor_data_##inst;                                     \
	static const struct robomaster_motor_config motor_config_##inst = {                        \
		.transport = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                  \
		.motor_id = DT_INST_REG_ADDR(inst),                                                \
		.model = DT_INST_ENUM_IDX(inst, model),                                            \
		.max_current = DT_INST_PROP_OR(inst, max_current, ROBOMASTER_DEFAULT_MAX_CURRENT), \
		.rotor_speed_bound_rpm = DT_INST_PROP_OR(inst, rotor_speed_bound_rpm,              \
							 ROBOMASTER_DEFAULT_ROTOR_BOUND_RPM),      \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, motor_init, NULL, &motor_data_##inst, &motor_config_##inst,    \
			      POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY, &robomaster_motor_api);
DT_INST_FOREACH_STATUS_OKAY(MOTOR_DEFINE)
