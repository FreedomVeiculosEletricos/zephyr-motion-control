/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DC brushed motor: scalar PI on armature current (tracks @ref
 * motor_ctrl_setpoints.i_torque_a vs measured Ia). Outputs normalised Valpha
 * for the H-bridge backend; Vbeta is unused.
 */

#include <math.h>
#include <string.h>

#include <zephyr/subsys/motor/motor_algo_dc_torque.h>
#include <zephyr/sys/util.h>

static void dc_torque_sync_from_ctrl(struct motor_algo_dc_torque_data *st,
				     const struct motor_ctrl_params *params)
{
	st->limits = params->limits;
	st->timing = params->timing;
	st->kt_nm_per_a = params->kt_nm_per_a;
	st->pole_pairs = params->pole_pairs;
}

static int dc_torque_init(void *algo_data, const struct motor_ctrl_params *params)
{
	struct motor_algo_dc_torque_data *st = algo_data;

	if (params == NULL) {
		return -EINVAL;
	}

	if ((st->current_loop.kp == 0.0f) && (st->current_loop.ki == 0.0f)) {
		return -EINVAL;
	}

	dc_torque_sync_from_ctrl(st, params);
	st->i_integral = 0.0f;
	return 0;
}

static void dc_torque_inner_step(void *algo_data, const struct motor_sensor_output *sense,
				 const struct motor_ctrl_setpoints *sp, struct motor_actuator_cmd *cmd)
{
	struct motor_algo_dc_torque_data *st = algo_data;
	float ia_meas = sense->hot.i_phase[0];
	float err = sp->i_torque_a - ia_meas;
	float Ts = st->timing.control_loop_dt_s;
	float kp = st->current_loop.kp;
	float ki = st->current_loop.ki;
	float u;

	if (Ts <= 0.0f) {
		Ts = 1.0f / 20000.0f;
	}

	st->i_integral += ki * err * Ts;
	u = kp * err + st->i_integral;

	if (u > st->current_loop.out_max) {
		st->i_integral -= ki * err * Ts;
		u = st->current_loop.out_max;
	} else if (u < st->current_loop.out_min) {
		st->i_integral -= ki * err * Ts;
		u = st->current_loop.out_min;
	}

	cmd->kind = MOTOR_ACTUATOR_CMD_ALPHA_BETA;
	cmd->u.ab.valpha = u;
	cmd->u.ab.vbeta = 0.0f;
}

static void dc_torque_set_params(void *algo_data, const struct motor_ctrl_params *params)
{
	struct motor_algo_dc_torque_data *st = algo_data;

	if (params != NULL) {
		dc_torque_sync_from_ctrl(st, params);
	}
}

static void dc_torque_reset(void *algo_data)
{
	struct motor_algo_dc_torque_data *st = algo_data;

	st->i_integral = 0.0f;
}

const struct motor_algo_ops motor_algo_dc_torque = {
	.init = dc_torque_init,
	.inner_step = dc_torque_inner_step,
	.outer_step_0 = NULL,
	.outer_step_1 = NULL,
	.set_params = dc_torque_set_params,
	.reset = dc_torque_reset,
};
