/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/drivers/motor/motor_sensor.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/subsys/motor/motor.h>

void motor_register_callbacks(motor_t motor, motor_state_cb_t state_cb,
			      motor_fault_notify_cb_t fault_cb, void *user_data)
{
	if (motor == NULL) {
		return;
	}

	motor->app_state_cb = state_cb;
	motor->app_fault_cb = fault_cb;
	motor->app_cb_data = user_data;
}

int motor_self_test(motor_t motor, uint32_t *faults)
{
	if (motor == NULL) {
		return -EINVAL;
	}

	return motor_ctrl_self_test(motor, faults);
}

int motor_enable(motor_t motor)
{
	if (motor == NULL) {
		return -EINVAL;
	}

	return motor_ctrl_enable(motor);
}

int motor_disable(motor_t motor)
{
	if (motor == NULL) {
		return -EINVAL;
	}

	return motor_ctrl_disable(motor);
}

void motor_estop(motor_t motor)
{
	if (motor == NULL) {
		return;
	}

	motor_ctrl_estop(motor);
}

int motor_set_current(motor_t motor, float i_a)
{
	if (motor == NULL) {
		return -EINVAL;
	}

	if (motor->state != MOTOR_STATE_RUN) {
		return -EINVAL;
	}

	k_mutex_lock(&motor->lock, K_FOREVER);
	motor->setpoints.i_ref_a = i_a;
	k_mutex_unlock(&motor->lock);

	return 0;
}

int motor_set_drive_mode(motor_t motor, enum motor_drive_mode mode)
{
	if (motor == NULL) {
		return -EINVAL;
	}

	return motor_actuator_set_drive_mode(motor->actuator, mode);
}

void motor_get_status(motor_t motor, enum motor_state *state, uint32_t *faults,
		      struct motor_sensor_output *sense)
{
	if (motor == NULL) {
		return;
	}

	if (sense != NULL) {
		(void)motor_sensor_get(motor->sensor, sense);
	}

	motor_ctrl_get_status(motor, state, faults);
}

int motor_clear_fault(motor_t motor)
{
	if (motor == NULL) {
		return -EINVAL;
	}

	return motor_ctrl_clear_fault(motor);
}
