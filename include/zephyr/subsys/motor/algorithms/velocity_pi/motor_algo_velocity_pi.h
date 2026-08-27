/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_VELOCITY_PI_MOTOR_ALGO_VELOCITY_PI_H_
#define ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_VELOCITY_PI_MOTOR_ALGO_VELOCITY_PI_H_

#include <zephyr/subsys/motor/motor_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup motor_algo_velocity_pi Velocity PI algorithm
 * @ingroup motor_control
 * @{
 *
 * Outer velocity loop with a tracking PLL on measured angle and a PI that
 * outputs a current reference for the next (inner) pipeline block.
 */

struct motor_velocity_pi_params {
	float kp;
	float ki;
	float pll_kp;
	float pll_ki;
};

struct motor_velocity_pi_limits {
	float i_max_a;
	/**
	 * Current pushed in the commanded direction while the angle is invalid,
	 * in amps.  Zero keeps the last reference instead, which is the right
	 * behaviour with a real encoder but stalls a sensorless start, where the
	 * estimate only becomes valid once the rotor is already turning.
	 */
	float open_loop_startup_a;
};

struct motor_velocity_pi_state {
	float v_ref_rad_s;
	float v_meas_rad_s;
	float error_rad_s;
	float i_ref_a;
	float theta_hat_rad;
	float omega_hat_rad_s;
};

int motor_algo_velocity_pi_set_velocity(motor_t motor, float rad_s);

int motor_algo_velocity_pi_get_state(motor_t motor, struct motor_velocity_pi_state *out);

int motor_algo_velocity_pi_set_params(motor_t motor, const struct motor_velocity_pi_params *params,
				      bool reset_integrator);

int motor_algo_velocity_pi_get_params(motor_t motor, struct motor_velocity_pi_params *out);

int motor_algo_velocity_pi_set_limits(motor_t motor, const struct motor_velocity_pi_limits *limits);

int motor_algo_velocity_pi_get_limits(motor_t motor, struct motor_velocity_pi_limits *out);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_VELOCITY_PI_MOTOR_ALGO_VELOCITY_PI_H_ */
