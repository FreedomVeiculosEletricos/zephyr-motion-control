/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_motor_encoder_observer

#include <errno.h>
#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/motor/motor_encoder_observer.h>
#include <zephyr/drivers/sensor.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD_TO_DEG (180.0f / (float)M_PI)
#define RAD_S_TO_RPM (60.0f / (2.0f * (float)M_PI))

struct motor_encoder_observer_data {
	/* Written from the control ISR, read by the slow thread. Single-word
	 * stores, so no lock: a torn pair only costs one stale sample.
	 */
	float omega_rad_s;
	float theta_rad;
	bool valid;

	/* Snapshot taken by sample_fetch, in Sensor API units. */
	float sample_deg;
	float sample_rpm;
};

void motor_encoder_observer_publish(const struct device *dev, float omega_rad_s, float theta_rad,
				    bool valid)
{
	struct motor_encoder_observer_data *data;

	if (dev == NULL) {
		return;
	}

	data = dev->data;
	data->omega_rad_s = omega_rad_s;
	data->theta_rad = theta_rad;
	data->valid = valid;
}

static int motor_encoder_observer_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct motor_encoder_observer_data *data = dev->data;

	if ((chan != SENSOR_CHAN_ALL) && (chan != SENSOR_CHAN_ROTATION) &&
	    (chan != SENSOR_CHAN_RPM)) {
		return -ENOTSUP;
	}

	if (!data->valid) {
		return -ENODATA;
	}

	data->sample_deg = data->theta_rad * RAD_TO_DEG;
	data->sample_rpm = data->omega_rad_s * RAD_S_TO_RPM;

	return 0;
}

static int motor_encoder_observer_channel_get(const struct device *dev, enum sensor_channel chan,
					      struct sensor_value *val)
{
	struct motor_encoder_observer_data *data = dev->data;

	if (val == NULL) {
		return -EINVAL;
	}

	switch (chan) {
	case SENSOR_CHAN_ROTATION:
		return sensor_value_from_double(val, (double)data->sample_deg);
	case SENSOR_CHAN_RPM:
		return sensor_value_from_double(val, (double)data->sample_rpm);
	default:
		return -ENOTSUP;
	}
}

static DEVICE_API(sensor, motor_encoder_observer_api) = {
	.sample_fetch = motor_encoder_observer_sample_fetch,
	.channel_get = motor_encoder_observer_channel_get,
};

static int motor_encoder_observer_init(const struct device *dev)
{
	struct motor_encoder_observer_data *data = dev->data;

	data->omega_rad_s = 0.0f;
	data->theta_rad = 0.0f;
	data->valid = false;
	data->sample_deg = 0.0f;
	data->sample_rpm = 0.0f;

	return 0;
}

#define MOTOR_ENCODER_OBSERVER_DEFINE(inst)                                                        \
	static struct motor_encoder_observer_data motor_encoder_observer_data_##inst;              \
	DEVICE_DT_INST_DEFINE(inst, motor_encoder_observer_init, NULL,                             \
			      &motor_encoder_observer_data_##inst, NULL, POST_KERNEL,               \
			      CONFIG_SENSOR_INIT_PRIORITY, &motor_encoder_observer_api);

DT_INST_FOREACH_STATUS_OKAY(MOTOR_ENCODER_OBSERVER_DEFINE)
