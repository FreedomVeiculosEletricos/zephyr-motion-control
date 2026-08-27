/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MOTOR_MOTOR_ENCODER_OBSERVER_H_
#define ZEPHYR_INCLUDE_DRIVERS_MOTOR_MOTOR_ENCODER_OBSERVER_H_

#include <stdbool.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Virtual rotation sensor fed by a sensorless observer.
 * @defgroup motor_encoder_observer Motor observer encoder facade
 * @ingroup motor_control
 * @{
 *
 * Implements the Zephyr Sensor API on top of an estimate published from the
 * control ISR, so a sensorless setup is a drop-in replacement for a magnetic
 * encoder: point @c feedback-sensor at this node instead of the real one.
 *
 * @c sensor_sample_fetch() returns @c -ENODATA while the estimate is not
 * trustworthy, which the motor sensor propagates as an invalid angle.
 */

/**
 * @brief Publish a new estimate.
 *
 * Safe to call from an ISR: it only stores scalars.
 *
 * @param dev Virtual encoder device.
 * @param omega_rad_s Estimated speed [rad/s].
 * @param theta_rad Estimated angle, wrapped to [-pi, pi] [rad].
 * @param valid Whether the estimate may be trusted.
 */
void motor_encoder_observer_publish(const struct device *dev, float omega_rad_s, float theta_rad,
				    bool valid);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MOTOR_MOTOR_ENCODER_OBSERVER_H_ */
