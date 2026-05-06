/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_DC_CURRENT_MOTOR_ALGO_DC_CURRENT_H_
#define ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_DC_CURRENT_MOTOR_ALGO_DC_CURRENT_H_

#include <zephyr/subsys/motor/motor_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup motor_algo_dc_current DC current algorithm
 * @ingroup motor_control
 * @{
 *
 * Scalar current PI algorithm. Devicetree binds it to a motor controller through
 * a @c zephyr,motor-algorithm-dc-current node; applications command and inspect
 * it through the functions in this header.
 */

struct motor_dc_current_pi {
	float kp;
	float ki;
	float out_min;
	float out_max;
};

struct motor_dc_current_limits {
	float i_max_a;
	float vbus_derating_start;
	float temp_derating_start;
	float temp_fault;
};

struct motor_dc_current_state {
	float i_ref_a;
	float i_meas_a;
	float error_a;
	float duty;
};

int motor_algo_dc_current_set_current(motor_t motor, float i_a);

int motor_algo_dc_current_get_state(motor_t motor, struct motor_dc_current_state *out);

int motor_algo_dc_current_set_pi_gains(motor_t motor, const struct motor_dc_current_pi *pi,
				       bool reset_integral);

int motor_algo_dc_current_get_pi_gains(motor_t motor, struct motor_dc_current_pi *out);

int motor_algo_dc_current_set_limits(motor_t motor, const struct motor_dc_current_limits *limits);

int motor_algo_dc_current_get_limits(motor_t motor, struct motor_dc_current_limits *out);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_DC_CURRENT_MOTOR_ALGO_DC_CURRENT_H_ */
