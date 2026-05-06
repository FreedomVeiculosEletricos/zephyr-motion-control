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
 */

static inline int motor_actuator_self_test_noop(const struct device *dev, uint32_t *flags)
{
	ARG_UNUSED(dev);

	if (flags != NULL) {
		*flags = 0U;
	}
	return 0;
}

static inline int motor_actuator_sto_arm_unsupported(const struct device *dev)
{
	ARG_UNUSED(dev);

	return -ENOTSUP;
}

static inline int motor_actuator_sto_release_unsupported(const struct device *dev, uint32_t *flags)
{
	ARG_UNUSED(dev);

	if (flags != NULL) {
		*flags = 0U;
	}
	return -ENOTSUP;
}

#endif /* ZEPHYR_DRIVERS_MOTOR_ACTUATOR_COMMON_H_ */
