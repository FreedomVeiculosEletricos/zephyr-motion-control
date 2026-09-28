/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>

#include "motor_algo_velocity_observer_priv.h"

#include <zephyr/drivers/motor/motor_encoder_observer.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/subsys/motor/motor_pipeline.h>
#include <zephyr/sys/util.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static struct motor_algo_velocity_observer_data *observer_data_from_block(struct motor_block *self)
{
	return CONTAINER_OF(self, struct motor_algo_velocity_observer_data, base);
}

static bool observer_params_valid(const struct motor_velocity_observer_params *params, float dt)
{
	if (params == NULL) {
		return false;
	}

	if ((params->l_h <= 0.0f) || (params->ke_v_per_rad_s <= 0.0f) ||
	    (params->vbus_v <= 0.0f) || (params->bandwidth_rad_s <= 0.0f)) {
		return false;
	}

	if ((params->r_ohm < 0.0f) || (params->pll_kp < 0.0f) || (params->pll_ki < 0.0f)) {
		return false;
	}

	/* The tracking loop feeds its own output back one tick late, so the
	 * product of its discrete poles is -kp: at kp >= 1 one of them sits
	 * outside the unit circle no matter how small ki is.
	 */
	if (params->pll_kp >= 1.0f) {
		return false;
	}

	if ((params->pll_ki * dt) >= (2.0f * (1.0f - params->pll_kp))) {
		return false;
	}

	return true;
}

static bool observer_limits_valid(const struct motor_velocity_observer_limits *limits)
{
	if (limits == NULL) {
		return false;
	}

	if ((limits->valid_threshold_rad_s <= 0.0f) || (limits->valid_release_rad_s <= 0.0f)) {
		return false;
	}

	/* Without this ordering the band would chatter instead of latching. */
	if (limits->valid_release_rad_s > limits->valid_threshold_rad_s) {
		return false;
	}

	return true;
}

static void observer_publish(const struct motor_algo_velocity_observer_data *st)
{
#if IS_ENABLED(CONFIG_MOTOR_ENCODER_OBSERVER)
	if (st->sink != NULL) {
		motor_encoder_observer_publish(st->sink, st->state.omega_hat_rad_s,
					       st->state.theta_hat_rad, st->state.valid);
	}
#else
	ARG_UNUSED(st);
#endif
}

static void observer_update_validity(struct motor_algo_velocity_observer_data *st)
{
	float mag = fabsf(st->state.omega_hat_rad_s);

	if (st->settle_count < st->limits.settle_ticks) {
		st->settle_count++;
		st->state.valid = false;
		return;
	}

	if (st->state.valid) {
		if (mag < st->limits.valid_release_rad_s) {
			st->state.valid = false;
		}
	} else if (mag >= st->limits.valid_threshold_rad_s) {
		st->state.valid = true;
	}
}

void motor_velocity_observer_block_entry(struct motor_block *self, const struct motor_block_in *in,
					 struct motor_block_out *out)
{
	struct motor_algo_velocity_observer_data *st = observer_data_from_block(self);
	float i_meas = 0.0f;
	float u = 0.0f;
	float i_err;
	float omega_raw;
	float omega_err;

	/* Pure estimator: it drives nothing and forwards no reference. */
	out->n_duty = 0U;
	out->has_current_ref = false;
	out->has_applied_u = false;

	if ((in->sense != NULL) && (in->sense->current_count > 0U)) {
		i_meas = in->sense->current_amps[0];
	}

	if (in->has_applied_u) {
		u = in->applied_u;
	}

	i_err = i_meas - st->state.i_hat_a;
	st->state.i_hat_a += st->dt_over_l * (u * st->params.vbus_v - st->state.emf_hat_v) -
			     st->r_dt_over_l * st->state.i_hat_a + st->l1_dt * i_err;
	st->state.emf_hat_v += st->l2_dt * i_err;

	omega_raw = st->state.emf_hat_v * st->inv_ke;
	omega_err = omega_raw - st->state.omega_hat_rad_s;
	st->pll_integral += st->pll_ki_dt * omega_err;

	st->state.omega_raw_rad_s = omega_raw;
	st->state.omega_hat_rad_s = st->params.pll_kp * omega_err + st->pll_integral;
	st->state.theta_hat_rad =
		remainderf(st->state.theta_hat_rad +
				   st->state.omega_hat_rad_s * st->timing.control_loop_dt_s,
			   2.0f * (float)M_PI);

	observer_update_validity(st);
	observer_publish(st);
}

void motor_velocity_observer_block_set_params(struct motor_block *self)
{
	struct motor_algo_velocity_observer_data *st = observer_data_from_block(self);
	float dt = st->timing.control_loop_dt_s;
	float w = st->params.bandwidth_rad_s;

	st->inv_l = (st->params.l_h > 0.0f) ? (1.0f / st->params.l_h) : 0.0f;
	st->inv_ke = (st->params.ke_v_per_rad_s > 0.0f) ? (1.0f / st->params.ke_v_per_rad_s) : 0.0f;
	st->dt_over_l = dt * st->inv_l;
	st->r_dt_over_l = st->params.r_ohm * st->dt_over_l;

	/* Both estimator poles at -w: the R/L term already present in the plant
	 * is subtracted out so the placement holds regardless of the motor.
	 */
	st->l1_dt = (2.0f * w - st->params.r_ohm * st->inv_l) * dt;
	st->l2_dt = -(w * w * st->params.l_h) * dt;

	st->pll_ki_dt = st->params.pll_ki * dt;
}

void motor_velocity_observer_block_reset(struct motor_block *self)
{
	struct motor_algo_velocity_observer_data *st = observer_data_from_block(self);

	st->pll_integral = 0.0f;
	st->settle_count = 0U;
	st->state = (struct motor_velocity_observer_state){0};

	/* Invalidate the facade too, so a stale estimate cannot survive a stop. */
	observer_publish(st);
}

static struct motor_algo_velocity_observer_data *observer_data_from_motor(motor_t motor)
{
	if ((motor == NULL) || (motor->pipeline == NULL)) {
		return NULL;
	}

	for (uint8_t i = 0; i < motor->pipeline->n_blocks; i++) {
		struct motor_block *b = motor->pipeline->blocks[i];

		if ((b != NULL) && (b->entry == motor_velocity_observer_block_entry)) {
			return CONTAINER_OF(b, struct motor_algo_velocity_observer_data, base);
		}
	}

	return NULL;
}

int motor_algo_velocity_observer_get_state(motor_t motor, struct motor_velocity_observer_state *out)
{
	struct motor_algo_velocity_observer_data *obs = observer_data_from_motor(motor);

	if (out == NULL) {
		return -EINVAL;
	}

	if (obs == NULL) {
		return -ENOTSUP;
	}

	*out = obs->state;

	return 0;
}

int motor_algo_velocity_observer_set_params(motor_t motor,
					    const struct motor_velocity_observer_params *params,
					    bool reset_state)
{
	struct motor_algo_velocity_observer_data *obs = observer_data_from_motor(motor);

	if (obs == NULL) {
		return -ENOTSUP;
	}

	if (!observer_params_valid(params, obs->timing.control_loop_dt_s)) {
		return -EINVAL;
	}

	obs->params = *params;
	motor_velocity_observer_block_set_params(&obs->base);
	if (reset_state) {
		motor_velocity_observer_block_reset(&obs->base);
	}

	return 0;
}

int motor_algo_velocity_observer_get_params(motor_t motor,
					    struct motor_velocity_observer_params *out)
{
	struct motor_algo_velocity_observer_data *obs = observer_data_from_motor(motor);

	if (out == NULL) {
		return -EINVAL;
	}

	if (obs == NULL) {
		return -ENOTSUP;
	}

	*out = obs->params;

	return 0;
}

int motor_algo_velocity_observer_set_limits(motor_t motor,
					    const struct motor_velocity_observer_limits *limits)
{
	struct motor_algo_velocity_observer_data *obs = observer_data_from_motor(motor);

	if (!observer_limits_valid(limits)) {
		return -EINVAL;
	}

	if (obs == NULL) {
		return -ENOTSUP;
	}

	obs->limits = *limits;

	return 0;
}

int motor_algo_velocity_observer_get_limits(motor_t motor,
					    struct motor_velocity_observer_limits *out)
{
	struct motor_algo_velocity_observer_data *obs = observer_data_from_motor(motor);

	if (out == NULL) {
		return -EINVAL;
	}

	if (obs == NULL) {
		return -ENOTSUP;
	}

	*out = obs->limits;

	return 0;
}
