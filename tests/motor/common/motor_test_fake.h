/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_TEST_FAKE_H_
#define MOTOR_TEST_FAKE_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/drivers/motor/motor_sensor.h>

/**
 * Per-instance state shared by the fake sensor and the fake actuator. The
 * caller-defined storage lives in the test translation unit and is wired to
 * both pseudo-devices via @ref MOTOR_TEST_FAKE_DEFINE.
 */
struct motor_test_fake {
	struct motor_sensor_output sense;

	motor_control_cb_t control_cb;
	void *control_user_data;
	motor_fault_cb_t fault_cb;
	void *fault_user_data;

	enum motor_drive_mode drive_mode;
	uint32_t fault_flags;

	struct motor_actuator_cmd last_cmd;
	bool has_last_cmd;
};

extern const struct motor_sensor_ops motor_test_fake_sensor_ops;
extern const struct motor_actuator_ops motor_test_fake_actuator_ops;

int motor_test_fake_actuator_init(const struct device *dev);

/**
 * Define a fake sensor + actuator pair sharing one @ref motor_test_fake state.
 * Devices are registered as ``<name>_sensor`` and ``<name>_actuator``.
 */
#define MOTOR_TEST_FAKE_DEFINE(_name)                                                              \
	static struct motor_test_fake _name;                                                       \
	DEVICE_DEFINE(_name##_sensor, #_name "_sensor", NULL, NULL, &(_name), NULL, POST_KERNEL,   \
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &motor_test_fake_sensor_ops);            \
	DEVICE_DEFINE(_name##_actuator, #_name "_actuator", motor_test_fake_actuator_init, NULL,   \
		      &(_name), NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,             \
		      &motor_test_fake_actuator_ops)

#endif /* MOTOR_TEST_FAKE_H_ */
