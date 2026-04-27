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

static void motor_ctrl_current_isr(const struct device *actuator, void *user_data);

static void motor_ctrl_run_inner(struct motor_ctrl *ctrl, const struct motor_sensor_output *sense,
				 struct motor_actuator_cmd *cmd)
{
	struct motor_block_in in = {
		.sense = sense,
		.sp = &ctrl->setpoints,
	};
	struct motor_block_out out = {
		.cmd = cmd,
	};

	motor_pipeline_run_stage(ctrl->pipeline, ctrl->pipeline_ctx, MOTOR_STAGE_INNER_ISR,
				 ctrl->inner_stage_tick, &in, &out);
}

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
	ctrl->inner_rate_hz = inner_rate_hz;
	k_mutex_init(&ctrl->lock);

	err = motor_pipeline_init(pipeline, pipeline_ctx);
	if (err != 0) {
		return err;
	}

	err = motor_actuator_set_control_callback(actuator, motor_ctrl_current_isr, ctrl);
	if (err != 0) {
		return err;
	}

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

	motor_ctrl_run_inner(ctrl, &sense, &cmd);
	(void)motor_actuator_set_command(actuator, &cmd);
	ctrl->inner_stage_tick++;
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
	ctrl->setpoints.i_ref_a = 0.0f;
	err = motor_actuator_enable(ctrl->actuator);
	if (err == 0) {
		ctrl->state = MOTOR_STATE_RUN;
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
}

int motor_ctrl_clear_fault(struct motor_ctrl *ctrl)
{
	if (ctrl == NULL) {
		return -EINVAL;
	}

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
		*faults = 0U;
	}
}
