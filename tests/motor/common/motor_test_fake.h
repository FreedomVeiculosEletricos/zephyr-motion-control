/*
 * Copyright (c) 2026 Freedom Veiculos Eletricos
 * SPDX-License-Identifier: UNLICENSED
 */

#ifndef MOTOR_TEST_FAKE_H_
#define MOTOR_TEST_FAKE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/drivers/motor/motor_sensor.h>

struct motor_test_fake {
	struct motor_stage_config actuator_stage;

	float sensor_samples[MOTOR_SENSOR_CURRENT_MAX];
	size_t sensor_sample_count;
	float angle_rad;
	bool angle_valid;

	motor_sensor_measurement_done_cb_t sensor_measurement_done_cb;
	void *sensor_measurement_done_user_data;
	motor_actuator_callback_t actuator_cb;
	void *actuator_user_data;
	motor_actuator_callback_t slow_cb;
	void *slow_user_data;
	motor_fault_cb_t fault_cb;
	void *fault_user_data;

	enum motor_drive_mode drive_mode;
	uint32_t fault_flags;

	float last_duty[MOTOR_ACTUATOR_DUTY_MAX];
	uint8_t last_n_duty;
	bool has_last_duty;
};

extern const struct motor_sensor_ops motor_test_fake_sensor_ops;
extern const struct motor_actuator_ops motor_test_fake_actuator_ops;

int motor_test_fake_sensor_init(const struct device *dev);
int motor_test_fake_actuator_init(const struct device *dev);

void motor_test_fake_sync_fire(const struct device *dev);
void motor_test_fake_slow_fire(const struct device *dev);

#define MOTOR_TEST_FAKE_DEFINE(_name)                                                              \
	static struct motor_test_fake _name;                                                       \
	DEVICE_DEFINE(_name##_sensor, #_name "_sensor", motor_test_fake_sensor_init, NULL,          \
		      &(_name), NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,             \
		      &motor_test_fake_sensor_ops);                                                \
	DEVICE_DEFINE(_name##_actuator, #_name "_actuator", motor_test_fake_actuator_init, NULL,   \
		      &(_name), NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,             \
		      &motor_test_fake_actuator_ops)

#endif /* MOTOR_TEST_FAKE_H_ */
