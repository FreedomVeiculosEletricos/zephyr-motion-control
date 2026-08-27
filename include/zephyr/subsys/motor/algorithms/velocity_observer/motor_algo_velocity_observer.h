/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_VELOCITY_OBSERVER_MOTOR_ALGO_VELOCITY_OBSERVER_H_
#define ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_VELOCITY_OBSERVER_MOTOR_ALGO_VELOCITY_OBSERVER_H_

#include <stdint.h>

#include <zephyr/subsys/motor/motor_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup motor_algo_velocity_observer Sensorless velocity observer
 * @ingroup motor_control
 * @{
 *
 * Estimates brushed DC rotor speed from the measured current, with no position
 * sensor.  It runs on the inner (PWM ISR) stage and produces no actuator
 * command.
 *
 * The electrical equation @c L*di/dt = u*Vbus - R*i - Ke*w lets the back-EMF be
 * recovered as a disturbance state, avoiding a numeric derivative of the
 * current.  A second-order tracking loop then turns the estimated back-EMF into
 * a smooth speed.
 *
 * Only R, L and Ke are needed: the mechanical model (J, Kt, B) is deliberately
 * left out, so a bench only has to characterize the electrical side.
 *
 * The estimate collapses into noise as the speed approaches zero, because the
 * back-EMF vanishes while the offset and the temperature drift of R do not.
 * @ref motor_velocity_observer_state.valid reports when the estimate may be
 * trusted, using a hysteresis band.
 */

struct motor_velocity_observer_params {
	/** Armature resistance [ohm]. */
	float r_ohm;
	/** Armature inductance [H]. */
	float l_h;
	/** Back-EMF constant [V/(rad/s)]. */
	float ke_v_per_rad_s;
	/** Bus voltage backing the normalized command [V]. */
	float vbus_v;
	/** Observer bandwidth [rad/s]: places both estimator poles at -w_o. */
	float bandwidth_rad_s;
	/** Tracking loop proportional gain. */
	float pll_kp;
	/** Tracking loop integral gain. */
	float pll_ki;
};

struct motor_velocity_observer_limits {
	/** Speed above which the estimate becomes valid [rad/s]. */
	float valid_threshold_rad_s;
	/** Speed below which it becomes invalid again [rad/s]. */
	float valid_release_rad_s;
	/** Inner ticks to wait after a reset before validity may be asserted. */
	uint32_t settle_ticks;
};

struct motor_velocity_observer_state {
	/** Observed current [A]. */
	float i_hat_a;
	/** Observed back-EMF [V]. */
	float emf_hat_v;
	/** Speed straight out of the back-EMF, before the tracking loop [rad/s]. */
	float omega_raw_rad_s;
	/** Tracked speed [rad/s]. */
	float omega_hat_rad_s;
	/** Integrated angle, wrapped to [-pi, pi] [rad]. */
	float theta_hat_rad;
	/** Whether the estimate is currently trustworthy. */
	bool valid;
};

int motor_algo_velocity_observer_get_state(motor_t motor,
					   struct motor_velocity_observer_state *out);

int motor_algo_velocity_observer_set_params(motor_t motor,
					    const struct motor_velocity_observer_params *params,
					    bool reset_state);

int motor_algo_velocity_observer_get_params(motor_t motor,
					    struct motor_velocity_observer_params *out);

int motor_algo_velocity_observer_set_limits(motor_t motor,
					    const struct motor_velocity_observer_limits *limits);

int motor_algo_velocity_observer_get_limits(motor_t motor,
					    struct motor_velocity_observer_limits *out);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_VELOCITY_OBSERVER_MOTOR_ALGO_VELOCITY_OBSERVER_H_ */
