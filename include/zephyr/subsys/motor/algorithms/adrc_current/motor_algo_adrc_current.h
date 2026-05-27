/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_ADRC_CURRENT_MOTOR_ALGO_ADRC_CURRENT_H_
#define ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_ADRC_CURRENT_MOTOR_ALGO_ADRC_CURRENT_H_

#include <zephyr/subsys/motor/motor_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup motor_algo_adrc_current ADRC current algorithm
 * @ingroup motor_control
 * @{
 *
 * Linear ADRC current loop with a 2-state LESO (z1, z2). Devicetree binds it to
 * a @c zephyr,motor-algorithm-adrc-current node; applications command and inspect
 * it through this header.
 */

struct motor_adrc_current_params {
	float kp;
	float b0;
	float beta1;
	float beta2;
	float out_min;
	float out_max;
};

struct motor_adrc_current_limits {
	float i_max_a;
};

struct motor_adrc_current_state {
	float i_ref_a;
	float i_meas_a;
	float error_a;
	float duty;
	float z1;
	float z2;
	float u;
};

int motor_algo_adrc_current_set_current(motor_t motor, float i_a);

int motor_algo_adrc_current_get_state(motor_t motor, struct motor_adrc_current_state *out);

int motor_algo_adrc_current_set_params(motor_t motor, const struct motor_adrc_current_params *params,
				       bool reset_observer);

int motor_algo_adrc_current_get_params(motor_t motor, struct motor_adrc_current_params *out);

int motor_algo_adrc_current_set_limits(motor_t motor, const struct motor_adrc_current_limits *limits);

int motor_algo_adrc_current_get_limits(motor_t motor, struct motor_adrc_current_limits *out);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_ADRC_CURRENT_MOTOR_ALGO_ADRC_CURRENT_H_ */
