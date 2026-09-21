/* SPDX-License-Identifier: Apache-2.0 */
#ifndef FIBRIL_ZEPHYR_DRIVERS_MOTOR_ROBSTRIDE_PROTOCOL_H_
#define FIBRIL_ZEPHYR_DRIVERS_MOTOR_ROBSTRIDE_PROTOCOL_H_

#include <stdint.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>

enum robstride_comm_type {
	ROBSTRIDE_COMM_FEEDBACK = 2,
	ROBSTRIDE_COMM_ENABLE = 3,
	ROBSTRIDE_COMM_STOP = 4,
	ROBSTRIDE_COMM_PARAM_READ = 17,
	ROBSTRIDE_COMM_PARAM_WRITE = 18,
	ROBSTRIDE_COMM_FAULT = 21,
};

#define ROBSTRIDE_PARAM_RUN_MODE    0x7005U
#define ROBSTRIDE_PARAM_LOC_REF     0x7016U
#define ROBSTRIDE_PARAM_LIMIT_SPD   0x7017U
#define ROBSTRIDE_PARAM_LIMIT_CUR   0x7018U
#define ROBSTRIDE_PARAM_MECH_POS    0x7019U
#define ROBSTRIDE_PARAM_CAN_TIMEOUT 0x7028U

#define ROBSTRIDE_POSITION_MIN_RAD   (-12.57f)
#define ROBSTRIDE_POSITION_MAX_RAD   12.57f
#define ROBSTRIDE_VELOCITY_MIN_RAD_S (-33.0f)
#define ROBSTRIDE_VELOCITY_MAX_RAD_S 33.0f
#define ROBSTRIDE_TORQUE_MIN_NM      (-14.0f)
#define ROBSTRIDE_TORQUE_MAX_NM      14.0f

static inline uint32_t robstride_make_id(enum robstride_comm_type type, uint16_t data16,
					 uint8_t target)
{
	return (((uint32_t)type & 0x1FU) << 24) | ((uint32_t)data16 << 8) | target;
}

static inline uint8_t robstride_id_type(uint32_t id)
{
	return (uint8_t)((id >> 24) & 0x1FU);
}

static inline uint8_t robstride_id_source(uint32_t id)
{
	return (uint8_t)((id >> 8) & 0xFFU);
}

static inline uint8_t robstride_id_target(uint32_t id)
{
	return (uint8_t)(id & 0xFFU);
}

static inline float robstride_u16_to_float(uint16_t value, float min, float max)
{
	return (float)value * (max - min) / 65535.0f + min;
}

static inline void robstride_put_param_header(uint8_t data[8], uint16_t index)
{
	memset(data, 0, 8);
	sys_put_le16(index, data);
}

static inline void robstride_put_float_le(uint8_t out[4], float value)
{
	uint32_t bits;
	memcpy(&bits, &value, sizeof(bits));
	sys_put_le32(bits, out);
}

static inline float robstride_get_float_le(const uint8_t in[4])
{
	uint32_t bits = sys_get_le32(in);
	float value;
	memcpy(&value, &bits, sizeof(value));
	return value;
}

#endif
