/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_MOTOR_ACTUATOR_COMMON_H_
#define ZEPHYR_DRIVERS_MOTOR_ACTUATOR_COMMON_H_

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/motor/motor_actuator.h>

/**
 * @file
 * @brief Plumbing shared by motor power-stage drivers.
 *
 * The actuator vtable is identical across SoC backends in everything except
 * the actual hardware writes (set_vector / set_duty / enable / disable).
 * This header gathers the small dispatch and command-shape boilerplate so
 * each driver can plug only the SoC-specific entry points and keep the
 * shared validation in one place.
 */

typedef int (*motor_actuator_set_vector_fn)(const struct device *dev, float valpha, float vbeta);
typedef int (*motor_actuator_set_duty_fn)(const struct device *dev, const float *duty, uint8_t n);

/**
 * @brief Generic motor_actuator_ops.set_command dispatch.
 *
 * Validates the command shape and routes it to the driver-supplied set_vector
 * (αβ) or set_duty (DUTY_DIRECT) callback. VD/VQ commands are not supported
 * by single-axis H-bridges and return -ENOTSUP.
 */
static inline int motor_actuator_dispatch_cmd(const struct device *dev,
					      const struct motor_actuator_cmd *cmd,
					      motor_actuator_set_vector_fn set_vector,
					      motor_actuator_set_duty_fn set_duty)
{
	if (cmd == NULL) {
		return -EINVAL;
	}

	switch (cmd->kind) {
	case MOTOR_ACTUATOR_CMD_ALPHA_BETA:
		return set_vector(dev, cmd->u.ab.valpha, cmd->u.ab.vbeta);
	case MOTOR_ACTUATOR_CMD_VD_VQ:
		return -ENOTSUP;
	case MOTOR_ACTUATOR_CMD_DUTY_DIRECT:
		if ((cmd->u.duty.n == 0U) || (cmd->u.duty.n > MOTOR_ACTUATOR_CMD_DUTY_MAX)) {
			return -EINVAL;
		}
		return set_duty(dev, cmd->u.duty.duty, cmd->u.duty.n);
	default:
		return -EINVAL;
	}
}

#endif /* ZEPHYR_DRIVERS_MOTOR_ACTUATOR_COMMON_H_ */
