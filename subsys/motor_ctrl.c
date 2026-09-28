/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/motor/motor_actuator.h>
#include "motor_ctrl_priv.h"

#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/subsys/motor/motor_pipeline.h>
#include <zephyr/drivers/motor/motor_sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

static void motor_ctrl_hot_pwm_isr(const struct device *actuator, void *user_data);
static void motor_ctrl_slow_timer_isr(const struct device *actuator, void *user_data);
static void motor_ctrl_slow_thread(void *p1, void *p2, void *p3);

static void motor_ctrl_run_inner(struct motor_ctrl *ctrl, const struct motor_sense_bundle *sense,
				 struct motor_block_out *out)
{
	struct motor_block_in in = {
		.sense = sense,
		.has_current_ref = ctrl->cascaded_current_ref_valid,
		.current_ref_a = ctrl->cascaded_current_ref_a,
		.has_applied_u = ctrl->last_applied_u_valid,
		.applied_u = ctrl->last_applied_u,
	};

	motor_pipeline_run_stage(ctrl->pipeline, ctrl->pipeline_ctx, MOTOR_STAGE_INNER_ISR,
				 ctrl->inner_stage_tick, &in, out);
}

int motor_ctrl_init(struct motor_ctrl *ctrl, const struct device *sensor,
		    const struct device *actuator, struct motor_pipeline *pipeline, void *pipeline_ctx)
{
	const struct motor_stage_config *stage;
	uint32_t inner_rate_hz;
	bool slow_thread_started;
	int err;

	if ((ctrl == NULL) || (sensor == NULL) || (actuator == NULL) || (pipeline == NULL) ||
	    (pipeline_ctx == NULL) || (pipeline->n_blocks == 0U)) {
		return -EINVAL;
	}

	stage = motor_actuator_get_config(actuator);
	if ((stage == NULL) || (stage->pwm_period_ns == 0U)) {
		return -EINVAL;
	}

	if ((NSEC_PER_SEC % stage->pwm_period_ns) != 0U) {
		return -EINVAL;
	}

	inner_rate_hz = (uint32_t)(NSEC_PER_SEC / stage->pwm_period_ns);
	if (inner_rate_hz == 0U) {
		return -EINVAL;
	}

	slow_thread_started = ctrl->slow_thread_started;
	if (!slow_thread_started) {
		memset(ctrl, 0, sizeof(*ctrl));
		k_mutex_init(&ctrl->lock);
		k_sem_init(&ctrl->slow_tick_sem, 0, 1);
	} else {
		memset(&ctrl->last_sense, 0, sizeof(ctrl->last_sense));
		ctrl->inner_stage_tick = 0U;
		ctrl->slow_stage_tick = 0U;
		k_sem_reset(&ctrl->slow_tick_sem);
	}

	ctrl->sensor = sensor;
	ctrl->actuator = actuator;
	ctrl->pipeline = pipeline;
	ctrl->pipeline_ctx = pipeline_ctx;
	ctrl->state = MOTOR_STATE_IDLE;
	ctrl->inner_rate_hz = inner_rate_hz;
	ctrl->cascaded_current_ref_a = 0.0f;
	ctrl->cascaded_current_ref_valid = false;
	ctrl->last_applied_u = 0.0f;
	ctrl->last_applied_u_valid = false;

	err = motor_pipeline_init(pipeline, pipeline_ctx);
	if (err != 0) {
		return err;
	}

	motor_pipeline_reset(pipeline, pipeline_ctx);

	if (!slow_thread_started) {
		k_thread_create(&ctrl->slow_thread, ctrl->slow_thread_stack,
				K_KERNEL_STACK_SIZEOF(ctrl->slow_thread_stack), motor_ctrl_slow_thread,
				ctrl, NULL, NULL, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
		ctrl->slow_thread_started = true;
	}

	err = motor_actuator_set_callback(actuator, motor_ctrl_hot_pwm_isr, ctrl,
					  motor_ctrl_slow_timer_isr, ctrl);
	if (err != 0) {
		return err;
	}

	return 0;
}

static void motor_ctrl_hot_pwm_isr(const struct device *actuator, void *user_data)
{
	struct motor_ctrl *ctrl = user_data;
	struct motor_sense_bundle sense = {0};
	struct motor_block_out out = {0};
	size_t got = 0U;
	int err;

	if (ctrl->state != MOTOR_STATE_RUN) {
		return;
	}

	err = motor_sensor_get_sample(ctrl->sensor, MOTOR_SENSOR_CHAN_CURRENT, sense.current_amps,
				      ARRAY_SIZE(sense.current_amps), &got);
	if (err != 0) {
		(void)motor_sensor_start_sample(ctrl->sensor, MOTOR_SENSOR_CHAN_CURRENT);
		return;
	}

	sense.current_count = got;
	if (motor_sensor_channel_supported(ctrl->sensor, MOTOR_SENSOR_CHAN_ANGLE)) {
		err = motor_sensor_get_sample(ctrl->sensor, MOTOR_SENSOR_CHAN_ANGLE, &sense.angle_rad,
					      1U, &got);
		sense.angle_valid = (err == 0) && (got == 1U);
	}

	ctrl->last_sense = sense;
	motor_ctrl_run_inner(ctrl, &sense, &out);
	if (out.n_duty != 0U) {
		(void)motor_actuator_set_duty(actuator, out.duty, out.n_duty);
	}
	if (out.has_applied_u) {
		ctrl->last_applied_u = out.applied_u;
		ctrl->last_applied_u_valid = true;
	}
	(void)motor_sensor_start_sample(ctrl->sensor, MOTOR_SENSOR_CHAN_CURRENT);
	ctrl->inner_stage_tick++;
}

static void motor_ctrl_slow_timer_isr(const struct device *actuator, void *user_data)
{
	struct motor_ctrl *ctrl = user_data;

	ARG_UNUSED(actuator);

	k_sem_give(&ctrl->slow_tick_sem);
}

static void motor_ctrl_slow_thread(void *p1, void *p2, void *p3)
{
	struct motor_ctrl *ctrl = p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		struct motor_sense_bundle sense;
		struct motor_block_in in;
		struct motor_block_out out = {0};
		size_t got = 0U;
		int err;

		k_sem_take(&ctrl->slow_tick_sem, K_FOREVER);
		if (ctrl->state != MOTOR_STATE_RUN) {
			continue;
		}

		sense = ctrl->last_sense;
		if (motor_sensor_channel_supported(ctrl->sensor, MOTOR_SENSOR_CHAN_ANGLE)) {
			err = motor_sensor_start_sample(ctrl->sensor, MOTOR_SENSOR_CHAN_ANGLE);
			if (err == 0) {
				err = motor_sensor_get_sample(ctrl->sensor, MOTOR_SENSOR_CHAN_ANGLE,
							      &sense.angle_rad, 1U, &got);
				sense.angle_valid = (err == 0) && (got == 1U);
			} else {
				sense.angle_valid = false;
			}
			ctrl->last_sense.angle_rad = sense.angle_rad;
			ctrl->last_sense.angle_valid = sense.angle_valid;
		}

		in.sense = &sense;
		in.has_current_ref = false;
		in.current_ref_a = 0.0f;

		motor_pipeline_run_stage(ctrl->pipeline, ctrl->pipeline_ctx, MOTOR_STAGE_OUTER,
					 ctrl->slow_stage_tick, &in, &out);

		if (out.has_current_ref) {
			ctrl->cascaded_current_ref_a = out.current_ref_a;
			ctrl->cascaded_current_ref_valid = true;
		}

		ctrl->slow_stage_tick++;
	}
}

int motor_ctrl_self_test(struct motor_ctrl *ctrl, uint32_t *flags)
{
	uint32_t act_faults = 0U;
	int err;

	if (ctrl == NULL) {
		return -EINVAL;
	}

	err = motor_sensor_calibrate(ctrl->sensor, MOTOR_SENSOR_CHAN_CURRENT);
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
	motor_pipeline_reset(ctrl->pipeline, ctrl->pipeline_ctx);
	ctrl->last_applied_u = 0.0f;
	ctrl->last_applied_u_valid = false;
	err = motor_actuator_enable(ctrl->actuator);
	if (err == 0) {
		(void)motor_sensor_start_sample(ctrl->sensor, MOTOR_SENSOR_CHAN_CURRENT);
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
	k_sem_reset(&ctrl->slow_tick_sem);
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
	k_sem_reset(&ctrl->slow_tick_sem);
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
