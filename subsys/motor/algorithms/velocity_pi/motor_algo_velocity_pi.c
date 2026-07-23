/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>

#include "motor_algo_velocity_pi_priv.h"

#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/subsys/motor/motor_pipeline.h>
#include <zephyr/sys/util.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static struct motor_algo_velocity_pi_data *velocity_data_from_block(struct motor_block *self)
{
	return CONTAINER_OF(self, struct motor_algo_velocity_pi_data, base);
}

static float angle_wrap_pi(float angle)
{
	while (angle > (float)M_PI) {
		angle -= 2.0f * (float)M_PI;
	}
	while (angle < -(float)M_PI) {
		angle += 2.0f * (float)M_PI;
	}

	return angle;
}

static bool velocity_params_valid(const struct motor_velocity_pi_params *params)
{
	if (params == NULL) {
		return false;
	}

	if ((params->pll_kp < 0.0f) || (params->pll_ki < 0.0f)) {
		return false;
	}

	return true;
}

void motor_velocity_pi_block_entry(struct motor_block *self, const struct motor_block_in *in,
				   struct motor_block_out *out)
{
	struct motor_algo_velocity_pi_data *st = velocity_data_from_block(self);
	float theta_meas;
	float e_pll;
	float e_vel;
	float i_ref;
	float dt = st->timing.control_loop_dt_s;

	out->n_duty = 0U;
	out->has_current_ref = false;

	if ((in->sense == NULL) || !in->sense->angle_valid) {
		out->has_current_ref = true;
		out->current_ref_a = st->state.i_ref_a;
		return;
	}

	theta_meas = in->sense->angle_rad;

	if (!st->pll_primed) {
		st->state.theta_hat_rad = theta_meas;
		st->state.omega_hat_rad_s = 0.0f;
		st->pll_integral = 0.0f;
		st->pll_primed = true;
	}

	e_pll = angle_wrap_pi(theta_meas - st->state.theta_hat_rad);
	st->pll_integral += st->pll_ki_dt * e_pll;
	st->state.omega_hat_rad_s = st->params.pll_kp * e_pll + st->pll_integral;
	st->state.theta_hat_rad =
		angle_wrap_pi(st->state.theta_hat_rad + st->state.omega_hat_rad_s * dt);

	e_vel = st->state.v_ref_rad_s - st->state.omega_hat_rad_s;
	st->i_integral += st->ki_dt * e_vel;
	i_ref = st->params.kp * e_vel + st->i_integral;

	if (i_ref > st->limits.i_max_a) {
		st->i_integral -= st->ki_dt * e_vel;
		i_ref = st->limits.i_max_a;
	} else if (i_ref < -st->limits.i_max_a) {
		st->i_integral -= st->ki_dt * e_vel;
		i_ref = -st->limits.i_max_a;
	}

	st->state.v_meas_rad_s = st->state.omega_hat_rad_s;
	st->state.error_rad_s = e_vel;
	st->state.i_ref_a = i_ref;

	out->has_current_ref = true;
	out->current_ref_a = i_ref;
}

void motor_velocity_pi_block_set_params(struct motor_block *self)
{
	struct motor_algo_velocity_pi_data *st = velocity_data_from_block(self);
	float dt = st->timing.control_loop_dt_s;

	st->ki_dt = st->params.ki * dt;
	st->pll_ki_dt = st->params.pll_ki * dt;
}

void motor_velocity_pi_block_reset(struct motor_block *self)
{
	struct motor_algo_velocity_pi_data *st = velocity_data_from_block(self);

	st->i_integral = 0.0f;
	st->pll_integral = 0.0f;
	st->pll_primed = false;
	st->state = (struct motor_velocity_pi_state){0};
}

static struct motor_algo_velocity_pi_data *velocity_data_from_motor(motor_t motor)
{
	if ((motor == NULL) || (motor->pipeline == NULL)) {
		return NULL;
	}

	for (uint8_t i = 0; i < motor->pipeline->n_blocks; i++) {
		struct motor_block *b = motor->pipeline->blocks[i];

		if ((b != NULL) && (b->entry == motor_velocity_pi_block_entry)) {
			return CONTAINER_OF(b, struct motor_algo_velocity_pi_data, base);
		}
	}

	return NULL;
}

int motor_algo_velocity_pi_set_velocity(motor_t motor, float rad_s)
{
	struct motor_algo_velocity_pi_data *vel;

	if (motor == NULL) {
		return -EINVAL;
	}

	if (motor->state != MOTOR_STATE_RUN) {
		return -EINVAL;
	}

	vel = velocity_data_from_motor(motor);
	if (vel == NULL) {
		return -ENOTSUP;
	}

	vel->state.v_ref_rad_s = rad_s;

	return 0;
}

int motor_algo_velocity_pi_get_state(motor_t motor, struct motor_velocity_pi_state *out)
{
	struct motor_algo_velocity_pi_data *vel = velocity_data_from_motor(motor);

	if (out == NULL) {
		return -EINVAL;
	}

	if (vel == NULL) {
		return -ENOTSUP;
	}

	*out = vel->state;

	return 0;
}

int motor_algo_velocity_pi_set_params(motor_t motor, const struct motor_velocity_pi_params *params,
				      bool reset_integrator)
{
	struct motor_algo_velocity_pi_data *vel = velocity_data_from_motor(motor);

	if (!velocity_params_valid(params)) {
		return -EINVAL;
	}

	if (vel == NULL) {
		return -ENOTSUP;
	}

	vel->params = *params;
	motor_velocity_pi_block_set_params(&vel->base);
	if (reset_integrator) {
		vel->i_integral = 0.0f;
		vel->pll_integral = 0.0f;
		vel->pll_primed = false;
	}

	return 0;
}

int motor_algo_velocity_pi_get_params(motor_t motor, struct motor_velocity_pi_params *out)
{
	struct motor_algo_velocity_pi_data *vel = velocity_data_from_motor(motor);

	if (out == NULL) {
		return -EINVAL;
	}

	if (vel == NULL) {
		return -ENOTSUP;
	}

	*out = vel->params;

	return 0;
}

int motor_algo_velocity_pi_set_limits(motor_t motor, const struct motor_velocity_pi_limits *limits)
{
	struct motor_algo_velocity_pi_data *vel = velocity_data_from_motor(motor);

	if ((limits == NULL) || (limits->i_max_a <= 0.0f)) {
		return -EINVAL;
	}

	if (vel == NULL) {
		return -ENOTSUP;
	}

	vel->limits = *limits;

	return 0;
}

int motor_algo_velocity_pi_get_limits(motor_t motor, struct motor_velocity_pi_limits *out)
{
	struct motor_algo_velocity_pi_data *vel = velocity_data_from_motor(motor);

	if (out == NULL) {
		return -EINVAL;
	}

	if (vel == NULL) {
		return -ENOTSUP;
	}

	*out = vel->limits;

	return 0;
}
