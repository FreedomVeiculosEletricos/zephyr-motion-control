/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>

#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/subsys/motor/motor.h>

motor_t motor_init(struct motor_ctrl *ctrl, const struct device *sensor,
		   const struct device *actuator, const struct motor_algo_ops *algo, void *algo_data,
		   const struct motor_ctrl_params *params)
{
	int err;

	if (params == NULL) {
		return NULL;
	}

	err = motor_ctrl_init(ctrl, sensor, actuator, algo, algo_data, params);
	if (err != 0) {
		return NULL;
	}

	return ctrl;
}

void motor_register_callbacks(motor_t motor, motor_state_cb_t state_cb,
			      motor_fault_notify_cb_t fault_cb, void *user_data)
{
	if (motor == NULL) {
		return;
	}

	motor->app_state_cb = (motor_ctrl_app_state_cb)state_cb;
	motor->app_fault_cb = (motor_ctrl_app_fault_cb)fault_cb;
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

int motor_set_torque(motor_t motor, float torque_nm)
{
	if (motor == NULL) {
		return -EINVAL;
	}

	if (motor->state != MOTOR_STATE_RUN) {
		return -EINVAL;
	}

	if (motor->params.kt_nm_per_a <= 1e-9f) {
		return -EINVAL;
	}

	k_mutex_lock(&motor->lock, K_FOREVER);
	motor->mode = MOTOR_MODE_CURRENT;
	motor->setpoints.i_torque_a = torque_nm / motor->params.kt_nm_per_a;
	k_mutex_unlock(&motor->lock);

	return 0;
}

int motor_set_speed(motor_t motor, float speed_rpm)
{
	ARG_UNUSED(motor);
	ARG_UNUSED(speed_rpm);

	return -ENOTSUP;
}

int motor_set_position(motor_t motor, float angle_deg)
{
	ARG_UNUSED(motor);
	ARG_UNUSED(angle_deg);

	return -ENOTSUP;
}

int motor_set_drive_mode(motor_t motor, enum motor_drive_mode mode)
{
	if (motor == NULL) {
		return -EINVAL;
	}

	return motor_actuator_set_drive_mode(motor->actuator, mode);
}

int motor_set_params(motor_t motor, const struct motor_ctrl_params *params)
{
	if (motor == NULL) {
		return -EINVAL;
	}

	return motor_ctrl_set_params(motor, params);
}

void motor_get_status(motor_t motor, enum motor_state *state, uint32_t *faults,
		      struct motor_sensor_output *sense)
{
	if (motor == NULL) {
		return;
	}

	if (sense != NULL) {
		uint8_t idx = (uint8_t)atomic_get(&motor->sense_buf_idx);

		*sense = motor->sense_buf[idx];
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

int motor_sto_arm(motor_t motor)
{
	ARG_UNUSED(motor);

	return -ENOTSUP;
}

int motor_sto_release(motor_t motor, uint32_t *faults)
{
	ARG_UNUSED(motor);
	if (faults != NULL) {
		*faults = 0U;
	}

	return -ENOTSUP;
}

int motor_params_save(motor_t motor)
{
	ARG_UNUSED(motor);

	return -ENOTSUP;
}

int motor_params_load(motor_t motor)
{
	ARG_UNUSED(motor);

	return -ENOTSUP;
}
