/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include "motor_algo_dc_current_priv.h"

#include <zephyr/kernel.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/subsys/motor/motor_pipeline.h>
#include <zephyr/sys/util.h>

static struct motor_algo_dc_current_data *dc_data_from_block(struct motor_block *self)
{
	return CONTAINER_OF(self, struct motor_algo_dc_current_data, base);
}

void motor_dc_current_block_entry(struct motor_block *self, const struct motor_block_in *in,
				  struct motor_block_out *out)
{
	struct motor_algo_dc_current_data *st = dc_data_from_block(self);
	float ia_meas = 0.0f;
	float err;
	float u;
	float d;
	float i_ref = in->has_current_ref ? in->current_ref_a : st->state.i_ref_a;

	if ((in->sense != NULL) && (in->sense->current_count > 0U)) {
		ia_meas = in->sense->current_amps[0];
	}

	if (i_ref > st->limits.i_max_a) {
		i_ref = st->limits.i_max_a;
	} else if (i_ref < -st->limits.i_max_a) {
		i_ref = -st->limits.i_max_a;
	}

	err = i_ref - ia_meas;
	st->i_integral += st->ki_dt * err;
	u = st->pi.kp * err + st->i_integral;

	if (u > st->pi.out_max) {
		st->i_integral -= st->ki_dt * err;
		u = st->pi.out_max;
	} else if (u < st->pi.out_min) {
		st->i_integral -= st->ki_dt * err;
		u = st->pi.out_min;
	}

	/* Map signed PI output u in [out_min, out_max] (typically [-1, +1]) to
	 * unsigned duty d in [0, 1] with d=0.5 at u=0. duty[1] is the
	 * complementary leg for full-bridge bipolar PWM; half-bridge backends
	 * ignore duty[1] per set_duty contract.
	 */
	d = (u + 1.0f) * 0.5f;

	st->state.i_ref_a = i_ref;
	st->state.i_meas_a = ia_meas;
	st->state.error_a = err;
	st->state.duty = d;

	out->n_duty = 2U;
	out->duty[0] = d;
	out->duty[1] = 1.0f - d;
	out->has_current_ref = false;
}

void motor_dc_current_block_set_params(struct motor_block *self)
{
	struct motor_algo_dc_current_data *st = dc_data_from_block(self);

	st->ki_dt = st->pi.ki * st->timing.control_loop_dt_s;
}

void motor_dc_current_block_reset(struct motor_block *self)
{
	struct motor_algo_dc_current_data *st = dc_data_from_block(self);

	st->i_integral = 0.0f;
	st->state = (struct motor_dc_current_state){0};
}

static struct motor_algo_dc_current_data *dc_data_from_motor(motor_t motor)
{
	if ((motor == NULL) || (motor->pipeline == NULL)) {
		return NULL;
	}

	for (uint8_t i = 0; i < motor->pipeline->n_blocks; i++) {
		struct motor_block *b = motor->pipeline->blocks[i];

		if ((b != NULL) && (b->entry == motor_dc_current_block_entry)) {
			return CONTAINER_OF(b, struct motor_algo_dc_current_data, base);
		}
	}

	return NULL;
}

int motor_algo_dc_current_set_current(motor_t motor, float i_a)
{
	struct motor_algo_dc_current_data *dc;

	if (motor == NULL) {
		return -EINVAL;
	}

	if (motor->state != MOTOR_STATE_RUN) {
		return -EINVAL;
	}

	dc = dc_data_from_motor(motor);
	if (dc == NULL) {
		return -ENOTSUP;
	}

	dc->state.i_ref_a = i_a;

	return 0;
}

int motor_algo_dc_current_get_state(motor_t motor, struct motor_dc_current_state *out)
{
	struct motor_algo_dc_current_data *dc;

	if (out == NULL) {
		return -EINVAL;
	}

	if (motor == NULL) {
		return -EINVAL;
	}

	dc = dc_data_from_motor(motor);
	if (dc == NULL) {
		return -ENOTSUP;
	}

	*out = dc->state;

	return 0;
}

int motor_algo_dc_current_set_pi_gains(motor_t motor, const struct motor_dc_current_pi *pi,
				       bool reset_integral)
{
	struct motor_algo_dc_current_data *dc = dc_data_from_motor(motor);

	if (pi == NULL) {
		return -EINVAL;
	}

	if (dc == NULL) {
		return -ENOTSUP;
	}

	dc->pi = *pi;
	motor_dc_current_block_set_params(&dc->base);
	if (reset_integral) {
		dc->i_integral = 0.0f;
	}

	return 0;
}

int motor_algo_dc_current_get_pi_gains(motor_t motor, struct motor_dc_current_pi *out)
{
	struct motor_algo_dc_current_data *dc = dc_data_from_motor(motor);

	if (out == NULL) {
		return -EINVAL;
	}

	if (dc == NULL) {
		return -ENOTSUP;
	}

	*out = dc->pi;

	return 0;
}

int motor_algo_dc_current_set_limits(motor_t motor, const struct motor_dc_current_limits *limits)
{
	struct motor_algo_dc_current_data *dc = dc_data_from_motor(motor);

	if (limits == NULL) {
		return -EINVAL;
	}

	if (dc == NULL) {
		return -ENOTSUP;
	}

	dc->limits = *limits;

	return 0;
}

int motor_algo_dc_current_get_limits(motor_t motor, struct motor_dc_current_limits *out)
{
	struct motor_algo_dc_current_data *dc = dc_data_from_motor(motor);

	if (out == NULL) {
		return -EINVAL;
	}

	if (dc == NULL) {
		return -ENOTSUP;
	}

	*out = dc->limits;

	return 0;
}
