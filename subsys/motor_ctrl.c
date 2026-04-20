/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/drivers/motor/motor_sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

static void motor_ctrl_current_isr(const struct device *actuator, void *user_data);

#if IS_ENABLED(CONFIG_MOTOR_CTRL_OUTER_LOOPS)
static void motor_outer_thread_fn(void *p1, void *p2, void *p3)
{
	struct motor_ctrl *ctrl = p1;
	uint32_t tick = 0U;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		k_sleep(K_MSEC(1));

		if (ctrl->state != MOTOR_STATE_RUN) {
			tick = 0U;
			continue;
		}

		k_mutex_lock(&ctrl->lock, K_FOREVER);
		{
			uint8_t idx = (uint8_t)atomic_get(&ctrl->sense_buf_idx);
			struct motor_sensor_output sense = ctrl->sense_buf[idx];

			if (ctrl->algo->outer_step_0 != NULL) {
				ctrl->algo->outer_step_0(ctrl->algo_data, &sense, &ctrl->setpoints);
			}

			if ((ctrl->algo->outer_step_1 != NULL) && ((tick % 10U) == 0U)) {
				ctrl->algo->outer_step_1(ctrl->algo_data, &sense, &ctrl->setpoints);
			}
		}
		k_mutex_unlock(&ctrl->lock);

		tick++;
	}
}
#endif

int motor_ctrl_init(struct motor_ctrl *ctrl, const struct device *sensor,
		    const struct device *actuator, const struct motor_algo_ops *algo, void *algo_data,
		    const struct motor_ctrl_params *params)
{
	int err;

	if ((ctrl == NULL) || (sensor == NULL) || (actuator == NULL) || (algo == NULL) ||
	    (algo_data == NULL) || (algo->inner_step == NULL) || (params == NULL)) {
		return -EINVAL;
	}

	memset(ctrl, 0, sizeof(*ctrl));

	memcpy(&ctrl->params, params, sizeof(ctrl->params));

	ctrl->sensor = sensor;
	ctrl->actuator = actuator;
	ctrl->algo = algo;
	ctrl->algo_data = algo_data;
	ctrl->state = MOTOR_STATE_IDLE;
	ctrl->mode = MOTOR_MODE_CURRENT;
	ctrl->inner_rate_hz =
		(uint32_t)(1.0f / ctrl->params.timing.control_loop_dt_s);
	k_mutex_init(&ctrl->lock);

	err = motor_sensor_init(sensor, NULL);
	if (err != 0) {
		return err;
	}

	err = algo->init(algo_data, &ctrl->params);
	if (err != 0) {
		return err;
	}

	err = motor_actuator_set_control_callback(actuator, motor_ctrl_current_isr, ctrl);
	if (err != 0) {
		return err;
	}

#if IS_ENABLED(CONFIG_MOTOR_CTRL_OUTER_LOOPS)
	ctrl->outer_thread_started = false;
#endif

	return 0;
}

static void motor_ctrl_current_isr(const struct device *actuator, void *user_data)
{
	struct motor_ctrl *ctrl = user_data;
	struct motor_sensor_output sense;
	struct motor_actuator_cmd cmd;

	if (ctrl->state != MOTOR_STATE_RUN) {
		return;
	}

	(void)motor_sensor_update(ctrl->sensor);
	(void)motor_sensor_get(ctrl->sensor, &sense);

	uint8_t new_idx = (uint8_t)((uint32_t)atomic_get(&ctrl->sense_buf_idx) ^ 1U);

	ctrl->sense_buf[new_idx] = sense;
	atomic_set(&ctrl->sense_buf_idx, (atomic_val_t)new_idx);

	ctrl->algo->inner_step(ctrl->algo_data, &sense, &ctrl->setpoints, &cmd);
	(void)motor_actuator_set_command(actuator, &cmd);
	atomic_inc(&ctrl->watchdog_cnt);
}

int motor_ctrl_self_test(struct motor_ctrl *ctrl, uint32_t *flags)
{
	uint32_t act_faults = 0U;
	int err;

	if (ctrl == NULL) {
		return -EINVAL;
	}

	err = motor_sensor_calibrate(ctrl->sensor);
	if (err != 0) {
		return err;
	}

	err = motor_actuator_self_test(ctrl->actuator, &act_faults);
	if (flags != NULL) {
		*flags = act_faults;
	}

	return err;
}

int motor_ctrl_enable(struct motor_ctrl *ctrl)
{
	int err;

	if (ctrl == NULL) {
		return -EINVAL;
	}

	if (ctrl->state == MOTOR_STATE_FAULT) {
		return -EFAULT;
	}

	k_mutex_lock(&ctrl->lock, K_FOREVER);
	ctrl->setpoints.i_torque_a = 0.0f;
	ctrl->setpoints.i_flux_a = 0.0f;
	err = motor_actuator_enable(ctrl->actuator);
	if (err == 0) {
		ctrl->state = MOTOR_STATE_RUN;
#if IS_ENABLED(CONFIG_MOTOR_CTRL_OUTER_LOOPS)
		if (!ctrl->outer_thread_started) {
			k_thread_create(&ctrl->outer_thread, ctrl->outer_stack_mem,
					K_KERNEL_STACK_SIZEOF(ctrl->outer_stack_mem), motor_outer_thread_fn,
					ctrl, NULL, NULL, CONFIG_MOTOR_CTRL_OUTER_THREAD_PRIO, 0, K_NO_WAIT);
			ctrl->outer_thread_started = true;
		}
#endif
		if (ctrl->app_state_cb != NULL) {
			ctrl->app_state_cb(ctrl, MOTOR_STATE_RUN, ctrl->app_cb_data);
		}
	}
	k_mutex_unlock(&ctrl->lock);

	return err;
}

int motor_ctrl_disable(struct motor_ctrl *ctrl)
{
	if (ctrl == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&ctrl->lock, K_FOREVER);
	(void)motor_actuator_disable(ctrl->actuator);
	ctrl->algo->reset(ctrl->algo_data);
	ctrl->state = MOTOR_STATE_IDLE;
	if (ctrl->app_state_cb != NULL) {
		ctrl->app_state_cb(ctrl, MOTOR_STATE_IDLE, ctrl->app_cb_data);
	}
	k_mutex_unlock(&ctrl->lock);

	return 0;
}

void motor_ctrl_estop(struct motor_ctrl *ctrl)
{
	if (ctrl == NULL) {
		return;
	}

	(void)motor_actuator_disable(ctrl->actuator);
	ctrl->algo->reset(ctrl->algo_data);
	ctrl->state = MOTOR_STATE_IDLE;
}

int motor_ctrl_clear_fault(struct motor_ctrl *ctrl)
{
	if (ctrl == NULL) {
		return -EINVAL;
	}

	ctrl->fault_flags = 0U;
	ctrl->state = MOTOR_STATE_IDLE;
	return 0;
}

int motor_ctrl_set_params(struct motor_ctrl *ctrl, const struct motor_ctrl_params *params)
{
	if ((ctrl == NULL) || (params == NULL)) {
		return -EINVAL;
	}

	k_mutex_lock(&ctrl->lock, K_FOREVER);
	memcpy(&ctrl->params, params, sizeof(*params));
	ctrl->algo->set_params(ctrl->algo_data, params);
	ctrl->inner_rate_hz =
		(uint32_t)(1.0f / ctrl->params.timing.control_loop_dt_s);
	k_mutex_unlock(&ctrl->lock);

	return 0;
}

void motor_ctrl_get_status(const struct motor_ctrl *ctrl, enum motor_state *state, uint32_t *faults)
{
	if (ctrl == NULL) {
		return;
	}

	if (state != NULL) {
		*state = ctrl->state;
	}

	if (faults != NULL) {
		*faults = ctrl->fault_flags;
	}
}
