/* SPDX-License-Identifier: Apache-2.0 */
#ifndef FIBRIL_ZEPHYR_INCLUDE_DRIVERS_SERVO_H_
#define FIBRIL_ZEPHYR_INCLUDE_DRIVERS_SERVO_H_

#include <errno.h>
#include <stdint.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Generic RC servo interface (supervisor-mode API).
 * @defgroup servo_interface Servo driver class
 * @ingroup drivers
 * @{
 */

/** @brief Servo driver operations. */
struct servo_driver_api
{
  /** Set the angle in degrees. */
  int (*set_angle)(const struct device * dev, float angle_deg);
  /** Set the pulse width in microseconds. */
  int (*set_pulse)(const struct device * dev, uint32_t pulse_us);
  /** Stop control pulses. */
  int (*disable)(const struct device * dev);
};

/**
 * @brief Set an angle, clamped to the configured range.
 *
 * The PWM driver linearly maps the configured angle endpoints to the pulse
 * endpoints, rounding to the nearest microsecond. A successful command resumes
 * output after servo_disable().
 *
 * @param dev Servo device instance.
 * @param angle_deg Finite angle in degrees.
 * @return 0 on success, -ENODEV if dev is NULL or not ready, -EINVAL for
 *         nonfinite angles, or the underlying driver's negative errno.
 */
static inline int servo_set_angle(const struct device * dev, float angle_deg)
{
  if (!device_is_ready(dev)) {
    return -ENODEV;
  }

  const struct servo_driver_api * api =
    (const struct servo_driver_api *)dev->api;
  return api->set_angle(dev, angle_deg);
}

/**
 * @brief Set a pulse width, clamped to the configured range.
 *
 * Zero is clamped to the minimum pulse; use servo_disable() to stop output.
 * A successful command resumes output after servo_disable().
 *
 * @param dev Servo device instance.
 * @param pulse_us Pulse width in microseconds.
 * @return 0 on success, -ENODEV if dev is NULL or not ready, or the
 *         underlying driver's negative errno.
 */
static inline int servo_set_pulse(const struct device * dev, uint32_t pulse_us)
{
  if (!device_is_ready(dev)) {
    return -ENODEV;
  }

  const struct servo_driver_api * api =
    (const struct servo_driver_api *)dev->api;
  return api->set_pulse(dev, pulse_us);
}

/**
 * @brief Stop control pulses by holding the PWM output inactive.
 *
 * The configured period and polarity are retained. This does not power off
 * the servo or guarantee release of mechanical torque.
 *
 * @param dev Servo device instance.
 * @return 0 on success, -ENODEV if dev is NULL or not ready, or the
 *         underlying driver's negative errno.
 */
static inline int servo_disable(const struct device * dev)
{
  if (!device_is_ready(dev)) {
    return -ENODEV;
  }

  const struct servo_driver_api * api =
    (const struct servo_driver_api *)dev->api;
  return api->disable(dev);
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif
