/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/subsys/motor/motor_ctrl_priv.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/subsys/motor/motor_pipeline.h>
#include <zephyr/drivers/motor/motor_sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

/*
 * motor_ctrl is the scheduling shell. Dispatch goes through
 * @ref motor_pipeline_run_stage only.
 */

static void motor_ctrl_current_isr(const struct device *actuator, void *user_data);

static void motor_ctrl_run_inner(struct motor_ctrl *ctrl, const struct motor_sensor_output *sense,
				 struct motor_actuator_cmd *cmd)
{
	struct motor_block_in in = {
		.sense = sense,
		.sp = &ctrl->setpoints,
		.algo = ctrl->pipeline_ctx,
	};
	struct motor_block_out out = {
		.sp = &ctrl->setpoints,
		.cmd = cmd,
	};

	motor_pipeline_run_stage(ctrl->pipeline, ctrl->pipeline_ctx, MOTOR_STAGE_INNER_ISR,
				 ctrl->stage_tick[MOTOR_STAGE_INNER_ISR], &in, &out);
}

#if IS_ENABLED(CONFIG_MOTOR_CTRL_OUTER_LOOPS)
static void motor_ctrl_run_outer(struct motor_ctrl *ctrl, enum motor_pipeline_stage stage,
				 const struct motor_sensor_output *sense)
{
	struct motor_block_in in = {
		.sense = sense,
		.sp = &ctrl->setpoints,
		.algo = ctrl->pipeline_ctx,
	};
	struct motor_block_out out = {
		.sp = &ctrl->setpoints,
		.cmd = NULL,
	};

	motor_pipeline_run_stage(ctrl->pipeline, ctrl->pipeline_ctx, stage,
				 ctrl->stage_tick[stage], &in, &out);
}

static void motor_outer_thread_fn(void *p1, void *p2, void *p3)
{
	struct motor_ctrl *ctrl = p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		(void)k_sem_take(&ctrl->outer_wake, K_FOREVER);

		if (ctrl->state != MOTOR_STATE_RUN) {
			ctrl->stage_tick[MOTOR_STAGE_OUTER_FAST] = 0U;
			ctrl->stage_tick[MOTOR_STAGE_OUTER_SLOW] = 0U;
			continue;
		}

		k_mutex_lock(&ctrl->lock, K_FOREVER);
		{
			struct motor_sensor_output sense;
			k_spinlock_key_t key = k_spin_lock(&ctrl->outer_snap_lock);

			sense = ctrl->outer_snap;
			k_spin_unlock(&ctrl->outer_snap_lock, key);

			motor_ctrl_run_outer(ctrl, MOTOR_STAGE_OUTER_FAST, &sense);
			motor_ctrl_run_outer(ctrl, MOTOR_STAGE_OUTER_SLOW, &sense);
		}
		k_mutex_unlock(&ctrl->lock);

		ctrl->stage_tick[MOTOR_STAGE_OUTER_FAST]++;
		ctrl->stage_tick[MOTOR_STAGE_OUTER_SLOW]++;
	}
}
#endif

int motor_ctrl_init(struct motor_ctrl *ctrl, const struct device *sensor,
		    const struct device *actuator, struct motor_pipeline *pipeline, void *pipeline_ctx,
		    uint32_t inner_rate_hz)
{
	int err;

	if ((ctrl == NULL) || (sensor == NULL) || (actuator == NULL) || (pipeline == NULL) ||
	    (pipeline_ctx == NULL) || (inner_rate_hz == 0U) || (pipeline->n_blocks == 0U)) {
		return -EINVAL;
	}

	memset(ctrl, 0, sizeof(*ctrl));

	ctrl->sensor = sensor;
	ctrl->actuator = actuator;
	ctrl->pipeline = pipeline;
	ctrl->pipeline_ctx = pipeline_ctx;
	ctrl->state = MOTOR_STATE_IDLE;
	ctrl->mode = MOTOR_MODE_CURRENT;
	ctrl->inner_rate_hz = inner_rate_hz;
	k_mutex_init(&ctrl->lock);

#if IS_ENABLED(CONFIG_MOTOR_CTRL_OUTER_LOOPS)
	{
		uint32_t t = (uint32_t)CONFIG_MOTOR_CTRL_OUTER_TARGET_HZ;

		ctrl->outer_decim = (inner_rate_hz + t - 1U) / t;
		if (ctrl->outer_decim == 0U) {
			ctrl->outer_decim = 1U;
		}
		ctrl->outer_isr_phase = 0U;
		k_sem_init(&ctrl->outer_wake, 0, K_SEM_MAX_LIMIT);
	}
#endif

	err = motor_pipeline_init(pipeline, pipeline_ctx);
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

#if IS_ENABLED(CONFIG_MOTOR_CTRL_OUTER_LOOPS)
	{
		k_spinlock_key_t key = k_spin_lock(&ctrl->outer_snap_lock);

		ctrl->outer_snap = sense;
		k_spin_unlock(&ctrl->outer_snap_lock, key);

		ctrl->outer_isr_phase++;
		if (ctrl->outer_isr_phase >= ctrl->outer_decim) {
			ctrl->outer_isr_phase = 0U;
			k_sem_give(&ctrl->outer_wake);
		}
	}
#endif

	motor_ctrl_run_inner(ctrl, &sense, &cmd);
	(void)motor_actuator_set_command(actuator, &cmd);
	ctrl->stage_tick[MOTOR_STAGE_INNER_ISR]++;
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
		ctrl->outer_isr_phase = 0U;
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
	motor_pipeline_reset(ctrl->pipeline, ctrl->pipeline_ctx);
	ctrl->state = MOTOR_STATE_IDLE;
	if (ctrl->app_state_cb != NULL) {
		ctrl->app_state_cb(ctrl, MOTOR_STATE_IDLE, ctrl->app_cb_data);
	}
	k_mutex_unlock(&ctrl->lock);

#if IS_ENABLED(CONFIG_MOTOR_CTRL_OUTER_LOOPS)
	k_sem_give(&ctrl->outer_wake);
#endif

	return 0;
}

void motor_ctrl_estop(struct motor_ctrl *ctrl)
{
	if (ctrl == NULL) {
		return;
	}

	(void)motor_actuator_disable(ctrl->actuator);
	motor_pipeline_reset(ctrl->pipeline, ctrl->pipeline_ctx);
	ctrl->state = MOTOR_STATE_IDLE;

#if IS_ENABLED(CONFIG_MOTOR_CTRL_OUTER_LOOPS)
	k_sem_give(&ctrl->outer_wake);
#endif
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
