/*
 * Copyright (c) 2026 Freedom Veiculos Eletricos
 * SPDX-License-Identifier: UNLICENSED
 */

#include "motor_test_fake.h"

#include <errno.h>
#include <string.h>

#include <zephyr/sys/util.h>

static int fake_sensor_start_sample(const struct device *dev, enum motor_sensor_channel ch)
{
	const struct motor_test_fake *fake = dev->data;

	if (ch == MOTOR_SENSOR_CHAN_ANGLE) {
		return fake->angle_valid ? 0 : -ENODATA;
	}
	if (ch != MOTOR_SENSOR_CHAN_CURRENT) {
		return -EINVAL;
	}
	return 0;
}

static int fake_sensor_get_sample(const struct device *dev, enum motor_sensor_channel ch,
				  float *out, size_t out_len, size_t *got)
{
	const struct motor_test_fake *fake = dev->data;

	if (ch == MOTOR_SENSOR_CHAN_ANGLE) {
		if (!fake->angle_valid || (out_len < 1U)) {
			return -ENODATA;
		}
		out[0] = fake->angle_rad;
		*got = 1U;
		return 0;
	}

	if (ch != MOTOR_SENSOR_CHAN_CURRENT) {
		return -ENOTSUP;
	}
	if (out_len < fake->sensor_sample_count || fake->sensor_sample_count == 0U) {
		return -EINVAL;
	}
	memcpy(out, fake->sensor_samples, fake->sensor_sample_count * sizeof(float));
	*got = fake->sensor_sample_count;
	return 0;
}

static int fake_sensor_calibrate(const struct device *dev, enum motor_sensor_channel ch)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ch);
	return 0;
}

static bool fake_sensor_channel_supported(const struct device *dev, enum motor_sensor_channel ch)
{
	ARG_UNUSED(dev);

	return (ch == MOTOR_SENSOR_CHAN_CURRENT) || (ch == MOTOR_SENSOR_CHAN_ANGLE);
}

static int fake_sensor_set_measurement_done_callback(const struct device *dev,
						   motor_sensor_measurement_done_cb_t cb,
						   void *user_data)
{
	struct motor_test_fake *fake = dev->data;

	fake->sensor_measurement_done_cb = cb;
	fake->sensor_measurement_done_user_data = user_data;
	return 0;
}

int motor_test_fake_sensor_init(const struct device *dev)
{
	struct motor_test_fake *fake = dev->data;

	fake->sensor_sample_count = 1U;
	fake->sensor_samples[0] = 0.0f;
	return 0;
}

const struct motor_sensor_ops motor_test_fake_sensor_ops = {
	.start_sample = fake_sensor_start_sample,
	.get_sample = fake_sensor_get_sample,
	.calibrate = fake_sensor_calibrate,
	.channel_supported = fake_sensor_channel_supported,
	.set_measurement_done_callback = fake_sensor_set_measurement_done_callback,
};

static const struct motor_stage_config motor_test_fake_stage_default = {
	.topology = MOTOR_STAGE_FULL_BRIDGE,
	.n_phases = 1U,
	.pwm_period_ns = 50000U,
	.deadtime_rising_ns = 0U,
	.deadtime_falling_ns = 0U,
	.v_bus_nominal = 12.0f,
	.v_bus_ov_thresh = 30.0f,
	.v_bus_uv_thresh = 6.0f,
	.i_peak_limit = 20.0f,
};

int motor_test_fake_actuator_init(const struct device *dev)
{
	struct motor_test_fake *fake = dev->data;

	memcpy(&fake->actuator_stage, &motor_test_fake_stage_default, sizeof(fake->actuator_stage));
	return 0;
}

static int fake_actuator_enable(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int fake_actuator_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int fake_actuator_set_duty(const struct device *dev, const float *duty, uint8_t n)
{
	struct motor_test_fake *fake = dev->data;

	if ((duty == NULL) || (n == 0U) || (n > MOTOR_ACTUATOR_DUTY_MAX)) {
		return -EINVAL;
	}

	memcpy(fake->last_duty, duty, (size_t)n * sizeof(float));
	fake->last_n_duty = n;
	fake->has_last_duty = true;
	return 0;
}

static int fake_actuator_set_drive_mode(const struct device *dev, enum motor_drive_mode mode)
{
	struct motor_test_fake *fake = dev->data;

	fake->drive_mode = mode;
	return 0;
}

static int fake_actuator_set_callback(const struct device *dev, motor_actuator_callback_t fast_cb,
				      void *fast_user_data, motor_actuator_callback_t slow_cb,
				      void *slow_user_data)
{
	struct motor_test_fake *fake = dev->data;

	fake->actuator_cb = fast_cb;
	fake->actuator_user_data = fast_user_data;
	fake->slow_cb = slow_cb;
	fake->slow_user_data = slow_user_data;
	return 0;
}

static int fake_actuator_set_fault_callback(const struct device *dev, motor_fault_cb_t cb,
					    void *user_data)
{
	struct motor_test_fake *fake = dev->data;

	fake->fault_cb = cb;
	fake->fault_user_data = user_data;
	return 0;
}

static int fake_actuator_clear_fault(const struct device *dev)
{
	struct motor_test_fake *fake = dev->data;

	fake->fault_flags = 0U;
	return 0;
}

static int fake_actuator_get_fault(const struct device *dev, uint32_t *flags)
{
	struct motor_test_fake *fake = dev->data;

	if (flags != NULL) {
		*flags = fake->fault_flags;
	}
	return 0;
}

static int fake_actuator_self_test(const struct device *dev, uint32_t *flags)
{
	ARG_UNUSED(dev);

	if (flags != NULL) {
		*flags = 0U;
	}
	return 0;
}

static const struct motor_stage_config *fake_actuator_get_config(const struct device *dev)
{
	struct motor_test_fake *fake = dev->data;

	return &fake->actuator_stage;
}

static int fake_actuator_sto_arm(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -ENOTSUP;
}

static int fake_actuator_sto_release(const struct device *dev, uint32_t *flags)
{
	ARG_UNUSED(dev);

	if (flags != NULL) {
		*flags = 0U;
	}
	return -ENOTSUP;
}

void motor_test_fake_sync_fire(const struct device *dev)
{
	struct motor_test_fake *fake = dev->data;

	if (fake->actuator_cb != NULL) {
		fake->actuator_cb(dev, fake->actuator_user_data);
	}
}

void motor_test_fake_slow_fire(const struct device *dev)
{
	struct motor_test_fake *fake = dev->data;

	if (fake->slow_cb != NULL) {
		fake->slow_cb(dev, fake->slow_user_data);
	}
}

const struct motor_actuator_ops motor_test_fake_actuator_ops = {
	.enable = fake_actuator_enable,
	.disable = fake_actuator_disable,
	.set_duty = fake_actuator_set_duty,
	.set_drive_mode = fake_actuator_set_drive_mode,
	.set_callback = fake_actuator_set_callback,
	.set_fault_callback = fake_actuator_set_fault_callback,
	.clear_fault = fake_actuator_clear_fault,
	.get_fault = fake_actuator_get_fault,
	.self_test = fake_actuator_self_test,
	.get_config = fake_actuator_get_config,
	.sto_arm = fake_actuator_sto_arm,
	.sto_release = fake_actuator_sto_release,
};
