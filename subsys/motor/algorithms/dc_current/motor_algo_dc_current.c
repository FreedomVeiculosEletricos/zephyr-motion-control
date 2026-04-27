/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/subsys/motor/algorithms/dc_current/motor_algo_dc_current.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/sys/util.h>

static struct motor_algo_dc_current_data *dc_data_from_block(struct motor_block *self)
{
	return CONTAINER_OF(self, struct motor_algo_dc_current_data, base);
}

void motor_dc_current_block_entry(struct motor_block *self, const struct motor_block_in *in,
				  struct motor_block_out *out)
{
	struct motor_algo_dc_current_data *st = dc_data_from_block(self);
	float ia_meas = in->sense->hot.i_phase[0];
	float err = in->sp->i_ref_a - ia_meas;
	float Ts;
	float kp;
	float ki;
	float u;
	k_spinlock_key_t key;

	key = k_spin_lock(&st->current_pi_lock);
	Ts = st->timing.control_loop_dt_s;
	kp = st->pi.kp;
	ki = st->pi.ki;
	st->i_integral += ki * err * Ts;
	u = kp * err + st->i_integral;

	if (u > st->pi.out_max) {
		st->i_integral -= ki * err * Ts;
		u = st->pi.out_max;
	} else if (u < st->pi.out_min) {
		st->i_integral -= ki * err * Ts;
		u = st->pi.out_min;
	}
	k_spin_unlock(&st->current_pi_lock, key);

	out->cmd->kind = MOTOR_ACTUATOR_CMD_DUTY_DIRECT;
	out->cmd->u.duty.n = 1U;
	out->cmd->u.duty.duty[0] = u;
}

void motor_dc_current_block_set_params(struct motor_block *self)
{
	ARG_UNUSED(self);
}

void motor_dc_current_block_reset(struct motor_block *self)
{
	struct motor_algo_dc_current_data *st = dc_data_from_block(self);
	k_spinlock_key_t key;

	key = k_spin_lock(&st->current_pi_lock);
	st->i_integral = 0.0f;
	k_spin_unlock(&st->current_pi_lock, key);
}

static struct motor_algo_dc_current_data *dc_data_from_motor(motor_t motor)
{
	struct motor_algo_dc_current_data *dc;

	if (motor == NULL) {
		return NULL;
	}

	dc = motor->pipeline_ctx;
	if ((dc == NULL) || (dc->base.entry != motor_dc_current_block_entry)) {
		return NULL;
	}

	return dc;
}

int motor_algo_dc_current_set_pi_gains(motor_t motor, const struct motor_dc_current_pi *pi,
				       bool reset_integral)
{
	struct motor_algo_dc_current_data *dc = dc_data_from_motor(motor);
	k_spinlock_key_t key;

	if (pi == NULL) {
		return -EINVAL;
	}

	if (dc == NULL) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&dc->current_pi_lock);
	dc->pi = *pi;
	if (reset_integral) {
		dc->i_integral = 0.0f;
	}
	k_spin_unlock(&dc->current_pi_lock, key);

	return 0;
}

int motor_algo_dc_current_get_pi_gains(motor_t motor, struct motor_dc_current_pi *out)
{
	struct motor_algo_dc_current_data *dc = dc_data_from_motor(motor);
	k_spinlock_key_t key;

	if (out == NULL) {
		return -EINVAL;
	}

	if (dc == NULL) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&dc->current_pi_lock);
	*out = dc->pi;
	k_spin_unlock(&dc->current_pi_lock, key);

	return 0;
}

int motor_algo_dc_current_set_limits(motor_t motor, const struct motor_dc_current_limits *limits)
{
	struct motor_algo_dc_current_data *dc = dc_data_from_motor(motor);
	k_spinlock_key_t key;

	if (limits == NULL) {
		return -EINVAL;
	}

	if (dc == NULL) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&dc->current_pi_lock);
	dc->limits = *limits;
	k_spin_unlock(&dc->current_pi_lock, key);

	return 0;
}

int motor_algo_dc_current_get_limits(motor_t motor, struct motor_dc_current_limits *out)
{
	struct motor_algo_dc_current_data *dc = dc_data_from_motor(motor);
	k_spinlock_key_t key;

	if (out == NULL) {
		return -EINVAL;
	}

	if (dc == NULL) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&dc->current_pi_lock);
	*out = dc->limits;
	k_spin_unlock(&dc->current_pi_lock, key);

	return 0;
}
