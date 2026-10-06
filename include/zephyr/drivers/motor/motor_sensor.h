/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MOTOR_MOTOR_SENSOR_H_
#define ZEPHYR_INCLUDE_DRIVERS_MOTOR_MOTOR_SENSOR_H_

#include <errno.h>
#include <stddef.h>
#include <stdbool.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup motor_sensor Motor sensor API
 * @ingroup motor_control
 * @{
 */

/** Max scalar floats returned for @ref MOTOR_SENSOR_CHAN_CURRENT (composite). */
#define MOTOR_SENSOR_CURRENT_MAX 6

/**
 * @brief Wrong context for an operation (e.g. @ref motor_sensor_start_sample from ISR
 *        on a Zephyr Sensor-backed channel). Use this errno for all such cases.
 */
#define MOTOR_SENSOR_ERR_CONTEXT (-EPERM)

/**
 * @brief Logical channels (extend only at the end for ABI stability).
 */
enum motor_sensor_channel {
	/** Composite vector: all DC-link / shunt currents (order matches DT `adc-channels`). */
	MOTOR_SENSOR_CHAN_CURRENT = 0,
	/** Electrical angle [rad] from optional `feedback-sensor` (Zephyr Sensor API). */
	MOTOR_SENSOR_CHAN_ANGLE = 1,
	/**
	 * Single signed bridge current [A], resolved by the driver from the
	 * composite vector.
	 *
	 * Low-side sensing on an H-bridge (a shunt, or the Rds(on) of the
	 * low-side FET) only ever sees a magnitude, so which leg carries the
	 * load current and what sign that implies both depend on the command
	 * being applied. Drivers that can pair each reading with the leg it
	 * belongs to collapse that into one signed number here, so algorithms do not
	 * have to know the bridge topology. Unsupported by drivers that cannot
	 * make the association.
	 */
	MOTOR_SENSOR_CHAN_CURRENT_BRIDGE = 2,
	MOTOR_SENSOR_CHAN__RESERVED_3 = 3,
};

typedef void (*motor_sensor_measurement_done_cb_t)(const struct device *dev, void *user_data);

struct motor_sensor_ops {
	int (*start_sample)(const struct device *dev, enum motor_sensor_channel ch);
	int (*get_sample)(const struct device *dev, enum motor_sensor_channel ch, float *out,
			  size_t out_len, size_t *got);
	int (*calibrate)(const struct device *dev, enum motor_sensor_channel ch);
	bool (*channel_supported)(const struct device *dev, enum motor_sensor_channel ch);
	int (*set_measurement_done_callback)(const struct device *dev,
					     motor_sensor_measurement_done_cb_t cb, void *user_data);
};

static inline int motor_sensor_start_sample(const struct device *dev, enum motor_sensor_channel ch)
{
	const struct motor_sensor_ops *ops = dev->api;

	return ops->start_sample(dev, ch);
}

static inline int motor_sensor_get_sample(const struct device *dev, enum motor_sensor_channel ch,
					  float *out, size_t out_len, size_t *got)
{
	const struct motor_sensor_ops *ops = dev->api;

	if (out == NULL || got == NULL) {
		return -EINVAL;
	}

	return ops->get_sample(dev, ch, out, out_len, got);
}

static inline int motor_sensor_calibrate(const struct device *dev, enum motor_sensor_channel ch)
{
	const struct motor_sensor_ops *ops = dev->api;

	return ops->calibrate(dev, ch);
}

static inline bool motor_sensor_channel_supported(const struct device *dev,
						  enum motor_sensor_channel ch)
{
	const struct motor_sensor_ops *ops = dev->api;

	return ops->channel_supported(dev, ch);
}

static inline int motor_sensor_set_measurement_done_callback(const struct device *dev,
							     motor_sensor_measurement_done_cb_t cb,
							     void *user_data)
{
	const struct motor_sensor_ops *ops = dev->api;

	return ops->set_measurement_done_callback(dev, cb, user_data);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MOTOR_MOTOR_SENSOR_H_ */
