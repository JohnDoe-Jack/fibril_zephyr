/* SPDX-License-Identifier: Apache-2.0 */
#define DT_DRV_COMPAT fibril_pwm_servo

#include <math.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <drivers/servo.h>

LOG_MODULE_REGISTER(pwm_servo, CONFIG_SERVO_LOG_LEVEL);

struct pwm_servo_config
{
  struct pwm_dt_spec pwm;
  uint32_t min_pulse_us;
  uint32_t center_pulse_us;
  uint32_t max_pulse_us;
  int32_t min_angle_deg;
  int32_t max_angle_deg;
  size_t pwm_count;
};

static int pwm_servo_set_pulse(const struct device * dev, uint32_t pulse_us)
{
  const struct pwm_servo_config * config = dev->config;

  if (!pwm_is_ready_dt(&config->pwm)) {
    return -ENODEV;
  }

  pulse_us = CLAMP(pulse_us, config->min_pulse_us, config->max_pulse_us);
  return pwm_set_pulse_dt(&config->pwm, PWM_USEC(pulse_us));
}

static int pwm_servo_set_angle(const struct device * dev, float angle_deg)
{
  const struct pwm_servo_config * config = dev->config;

  if (!isfinite(angle_deg)) {
    return -EINVAL;
  }

  /* Use double before subtraction to preserve signed DT endpoints and avoid
   * overflowing either the angle span or a rounded uint32_t pulse value.
   */
  double angle = CLAMP((double)angle_deg, (double)config->min_angle_deg,
                      (double)config->max_angle_deg);
  double fraction = (angle - (double)config->min_angle_deg) /
                    ((double)config->max_angle_deg - (double)config->min_angle_deg);
  double pulse_us = config->min_pulse_us +
                    fraction * (config->max_pulse_us - config->min_pulse_us);

  return pwm_servo_set_pulse(dev, (uint32_t)(pulse_us + 0.5));
}

static int pwm_servo_disable(const struct device * dev)
{
  const struct pwm_servo_config * config = dev->config;

  if (!pwm_is_ready_dt(&config->pwm)) {
    return -ENODEV;
  }

  return pwm_set_pulse_dt(&config->pwm, 0);
}

static int pwm_servo_init(const struct device * dev)
{
  const struct pwm_servo_config * config = dev->config;

  if (config->pwm_count != 1 || config->min_pulse_us == 0 ||
      config->min_pulse_us >= config->max_pulse_us ||
      config->center_pulse_us < config->min_pulse_us ||
      config->center_pulse_us > config->max_pulse_us ||
      config->min_angle_deg >= config->max_angle_deg ||
      config->pwm.period == 0 ||
      config->max_pulse_us > UINT32_MAX / 1000U ||
      (uint64_t)config->max_pulse_us * 1000U > config->pwm.period) {
    LOG_ERR("%s: invalid servo configuration", dev->name);
    return -EINVAL;
  }

  int ret = pwm_servo_disable(dev);

  if (ret < 0) {
    LOG_ERR("%s: PWM initialization failed (%d)", dev->name, ret);
  }
  return ret;
}

static const struct servo_driver_api pwm_servo_api = {
  .set_angle = pwm_servo_set_angle,
  .set_pulse = pwm_servo_set_pulse,
  .disable = pwm_servo_disable,
};

#define PWM_SERVO_DEFINE(inst)                                               \
  static const struct pwm_servo_config pwm_servo_config_##inst = {             \
    .pwm = PWM_DT_SPEC_INST_GET(inst),                                        \
    .min_pulse_us = DT_INST_PROP(inst, min_pulse_us),                           \
    .center_pulse_us = DT_INST_PROP(inst, center_pulse_us),                     \
    .max_pulse_us = DT_INST_PROP(inst, max_pulse_us),                           \
    .min_angle_deg = (int32_t)DT_INST_PROP(inst, min_angle_deg),                \
    .max_angle_deg = (int32_t)DT_INST_PROP(inst, max_angle_deg),                \
    .pwm_count = DT_INST_PROP_LEN(inst, pwms),                                 \
  };                                                                         \
  DEVICE_DT_INST_DEFINE(inst, pwm_servo_init, NULL, NULL,                      \
                        &pwm_servo_config_##inst, POST_KERNEL,                \
                        CONFIG_SERVO_INIT_PRIORITY, &pwm_servo_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_SERVO_DEFINE)
