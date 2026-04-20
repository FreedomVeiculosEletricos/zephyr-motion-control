/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DC brushed motor: scalar PI on armature current (tracks @ref
 * motor_ctrl_setpoints.i_torque_a vs measured Ia). Outputs normalised Valpha
 * for the H-bridge backend; Vbeta is unused.
 */

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

static int dc_torque_init(void *algo, const struct motor_ctrl_params *params)
{
	struct motor_algo_dc_torque_data *st = algo;

	if (params == NULL) {
		return -EINVAL;
	}

	if (params->timing.control_loop_dt_s <= 0.0f) {
		return -EINVAL;
	}

	if ((st->current_loop.kp == 0.0f) && (st->current_loop.ki == 0.0f)) {
		return -EINVAL;
	}

	dc_torque_sync_from_ctrl(st, params);
	st->i_integral = 0.0f;
	return 0;
}

static void dc_torque_inner_step(void *algo, const struct motor_block_in *in,
				 struct motor_block_out *out)
{
	struct motor_algo_dc_torque_data *st = algo;
	float ia_meas = in->sense->hot.i_phase[0];
	float err = in->sp->i_torque_a - ia_meas;
	float Ts = st->timing.control_loop_dt_s;
	float kp = st->current_loop.kp;
	float ki = st->current_loop.ki;
	float u;

	st->i_integral += ki * err * Ts;
	u = kp * err + st->i_integral;

	if (u > st->current_loop.out_max) {
		st->i_integral -= ki * err * Ts;
		u = st->current_loop.out_max;
	} else if (u < st->current_loop.out_min) {
		st->i_integral -= ki * err * Ts;
		u = st->current_loop.out_min;
	}

	out->cmd->kind = MOTOR_ACTUATOR_CMD_ALPHA_BETA;
	out->cmd->u.ab.valpha = u;
	out->cmd->u.ab.vbeta = 0.0f;
}

static void dc_torque_set_params(void *algo, const struct motor_ctrl_params *params)
{
	struct motor_algo_dc_torque_data *st = algo;

	if (params != NULL) {
		dc_torque_sync_from_ctrl(st, params);
	}
}

static void dc_torque_reset(void *algo)
{
	struct motor_algo_dc_torque_data *st = algo;

	st->i_integral = 0.0f;
}

const struct motor_algo_ops motor_algo_dc_torque = {
	.init = dc_torque_init,
	.inner_step = dc_torque_inner_step,
	.outer_step_0 = NULL,
	.outer_step_1 = NULL,
	.set_params = dc_torque_set_params,
	.reset = dc_torque_reset,
	.outer_0_div = 0U,
	.outer_1_div = 0U,
};
