/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include "motor_algo_adrc_current_priv.h"

#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/subsys/motor/motor_pipeline.h>
#include <zephyr/sys/util.h>

static struct motor_algo_adrc_current_data *adrc_data_from_block(struct motor_block *self)
{
	return CONTAINER_OF(self, struct motor_algo_adrc_current_data, base);
}

static bool adrc_params_valid(const struct motor_adrc_current_params *params)
{
	if (params == NULL) {
		return false;
	}

	if (params->b0 <= 0.0f) {
		return false;
	}

	if (params->out_min >= params->out_max) {
		return false;
	}

	return true;
}

static void adrc_map_duty_sign_magnitude(float u, struct motor_block_out *out, float *duty_state)
{
	float mag = (u >= 0.0f) ? u : -u;

	if (mag > 1.0f) {
		mag = 1.0f;
	}

	out->n_duty = 2U;
	if (u >= 0.0f) {
		out->duty[0] = mag;
		out->duty[1] = 0.0f;
	} else {
		out->duty[0] = 0.0f;
		out->duty[1] = mag;
	}

	/* Report the command that survived the magnitude clamp, which is what
	 * actually reaches the bridge.
	 */
	out->applied_u = (u >= 0.0f) ? mag : -mag;
	out->has_applied_u = true;

	if (duty_state != NULL) {
		*duty_state = mag;
	}
}

void motor_adrc_current_block_entry(struct motor_block *self, const struct motor_block_in *in,
				    struct motor_block_out *out)
{
	struct motor_algo_adrc_current_data *st = adrc_data_from_block(self);
	float ia_meas = 0.0f;
	float e_obs;
	float err;
	float u;
	float d;
	float i_ref = in->has_current_ref ? in->current_ref_a : st->state.i_ref_a;

	if ((in->sense != NULL) && (in->sense->current_count > 0U)) {
		ia_meas = in->sense->current_amps[0];
	}

	if (!st->sign_magnitude) {
		if (i_ref > st->limits.i_max_a) {
			i_ref = st->limits.i_max_a;
		} else if (i_ref < -st->limits.i_max_a) {
			i_ref = -st->limits.i_max_a;
		}
	}

	if (st->sign_magnitude && (st->params.beta1 == 0.0f) && (st->params.beta2 == 0.0f)) {
		u = i_ref;
		st->state.i_ref_a = i_ref;
		st->state.i_meas_a = ia_meas;
		st->state.error_a = i_ref - ia_meas;
		st->state.u = u;
		adrc_map_duty_sign_magnitude(u, out, &d);
		st->state.duty = d;
		out->has_current_ref = false;
		return;
	}

	e_obs = ia_meas - st->state.z1;
	st->state.z1 += st->timing.control_loop_dt_s * (st->state.z2 + st->params.b0 * st->state.u) +
			st->beta1_dt * e_obs;
	st->state.z2 += st->beta2_dt * e_obs;

	err = i_ref - st->state.z1;
	u = (st->params.kp * err - st->state.z2) * st->inv_b0;
	u = CLAMP(u, st->params.out_min, st->params.out_max);

	st->state.i_ref_a = i_ref;
	st->state.i_meas_a = ia_meas;
	st->state.error_a = err;
	st->state.u = u;

	if (st->sign_magnitude) {
		adrc_map_duty_sign_magnitude(u, out, &d);
		st->state.duty = d;
		out->has_current_ref = false;
		return;
	}

	d = CLAMP((u + 1.0f) * 0.5f, 0.0f, 1.0f);

	st->state.duty = d;

	out->n_duty = 2U;
	out->duty[0] = d;
	out->duty[1] = 1.0f - d;
	out->has_current_ref = false;
	out->applied_u = u;
	out->has_applied_u = true;
}

void motor_adrc_current_block_set_params(struct motor_block *self)
{
	struct motor_algo_adrc_current_data *st = adrc_data_from_block(self);
	float dt = st->timing.control_loop_dt_s;

	if (st->params.b0 <= 0.0f) {
		st->inv_b0 = 0.0f;
	} else {
		st->inv_b0 = 1.0f / st->params.b0;
	}

	st->beta1_dt = st->params.beta1 * dt;
	st->beta2_dt = st->params.beta2 * dt;
}

void motor_adrc_current_block_reset(struct motor_block *self)
{
	struct motor_algo_adrc_current_data *st = adrc_data_from_block(self);

	st->state = (struct motor_adrc_current_state){0};
}

static struct motor_algo_adrc_current_data *adrc_data_from_motor(motor_t motor)
{
	if ((motor == NULL) || (motor->pipeline == NULL)) {
		return NULL;
	}

	for (uint8_t i = 0; i < motor->pipeline->n_blocks; i++) {
		struct motor_block *b = motor->pipeline->blocks[i];

		if ((b != NULL) && (b->entry == motor_adrc_current_block_entry)) {
			return CONTAINER_OF(b, struct motor_algo_adrc_current_data, base);
		}
	}

	return NULL;
}

int motor_algo_adrc_current_set_current(motor_t motor, float i_a)
{
	struct motor_algo_adrc_current_data *adrc;

	if (motor == NULL) {
		return -EINVAL;
	}

	if (motor->state != MOTOR_STATE_RUN) {
		return -EINVAL;
	}

	adrc = adrc_data_from_motor(motor);
	if (adrc == NULL) {
		return -ENOTSUP;
	}

	adrc->state.i_ref_a = i_a;

	return 0;
}

int motor_algo_adrc_current_get_state(motor_t motor, struct motor_adrc_current_state *out)
{
	struct motor_algo_adrc_current_data *adrc;

	if ((motor == NULL) || (out == NULL)) {
		return -EINVAL;
	}

	adrc = adrc_data_from_motor(motor);
	if (adrc == NULL) {
		return -ENOTSUP;
	}

	*out = adrc->state;

	return 0;
}

int motor_algo_adrc_current_set_params(motor_t motor, const struct motor_adrc_current_params *params,
				       bool reset_observer)
{
	struct motor_algo_adrc_current_data *adrc = adrc_data_from_motor(motor);

	if (!adrc_params_valid(params)) {
		return -EINVAL;
	}

	if (adrc == NULL) {
		return -ENOTSUP;
	}

	adrc->params = *params;
	motor_adrc_current_block_set_params(&adrc->base);
	if (reset_observer) {
		adrc->state.z1 = 0.0f;
		adrc->state.z2 = 0.0f;
		adrc->state.u = 0.0f;
	}

	return 0;
}

int motor_algo_adrc_current_get_params(motor_t motor, struct motor_adrc_current_params *out)
{
	struct motor_algo_adrc_current_data *adrc = adrc_data_from_motor(motor);

	if (out == NULL) {
		return -EINVAL;
	}

	if (adrc == NULL) {
		return -ENOTSUP;
	}

	*out = adrc->params;

	return 0;
}

int motor_algo_adrc_current_set_limits(motor_t motor, const struct motor_adrc_current_limits *limits)
{
	struct motor_algo_adrc_current_data *adrc = adrc_data_from_motor(motor);

	if ((limits == NULL) || (limits->i_max_a <= 0.0f)) {
		return -EINVAL;
	}

	if (adrc == NULL) {
		return -ENOTSUP;
	}

	adrc->limits = *limits;

	return 0;
}

int motor_algo_adrc_current_get_limits(motor_t motor, struct motor_adrc_current_limits *out)
{
	struct motor_algo_adrc_current_data *adrc = adrc_data_from_motor(motor);

	if (out == NULL) {
		return -EINVAL;
	}

	if (adrc == NULL) {
		return -ENOTSUP;
	}

	*out = adrc->limits;

	return 0;
}
