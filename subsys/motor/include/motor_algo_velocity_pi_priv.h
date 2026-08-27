/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_MOTOR_ALGO_VELOCITY_PI_PRIV_H_
#define ZEPHYR_SUBSYS_MOTOR_ALGO_VELOCITY_PI_PRIV_H_

#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/subsys/motor/algorithms/velocity_pi/motor_algo_velocity_pi.h>
#include <zephyr/subsys/motor/motor_block.h>

struct motor_velocity_pi_timing {
	float control_loop_dt_s;
};

struct motor_algo_velocity_pi_data {
	struct motor_block base;
	struct motor_velocity_pi_state state;
	struct motor_velocity_pi_params params;
	struct motor_velocity_pi_limits limits;
	struct motor_velocity_pi_timing timing;
	float i_integral;
	float pll_integral;
	float ki_dt;
	float pll_ki_dt;
	bool pll_primed;
};

void motor_velocity_pi_block_entry(struct motor_block *self, const struct motor_block_in *in,
				   struct motor_block_out *out);
void motor_velocity_pi_block_reset(struct motor_block *self);
void motor_velocity_pi_block_set_params(struct motor_block *self);

#define MOTOR_ALGO_VELOCITY_PI_BASE_INITIALIZER                                                    \
	.base =                                                                                    \
		{                                                                                  \
			.name = "velocity_pi",                                                   \
			.stage = MOTOR_STAGE_OUTER,                                              \
			.period_div = 1,                                                         \
			.entry = motor_velocity_pi_block_entry,                                  \
			.reset = motor_velocity_pi_block_reset,                                  \
			.set_params = motor_velocity_pi_block_set_params,                        \
		},

#define MOTOR_VELOCITY_PI_DATA_INITIALIZER(controller_node_id, algo_node_id)                        \
	{                                                                                          \
		MOTOR_ALGO_VELOCITY_PI_BASE_INITIALIZER                                            \
		.state = {0},                                                                   \
		.params =                                                                        \
			{                                                                        \
				.kp = (float)DT_PROP(algo_node_id, velocity_pi_kp_milli) /       \
				      1000.0f,                                                   \
				.ki = (float)DT_PROP(algo_node_id, velocity_pi_ki_milli) /       \
				      1000.0f,                                                   \
				.pll_kp = (float)DT_PROP(algo_node_id, velocity_pll_kp_milli) /   \
					  1000.0f,                                               \
				.pll_ki = (float)DT_PROP(algo_node_id, velocity_pll_ki_milli) /   \
					  1000.0f,                                               \
			},                                                                       \
		.limits =                                                                         \
			{                                                                        \
				.i_max_a = (float)DT_PROP(algo_node_id, i_max_ma) / 1000.0f,   \
				.open_loop_startup_a =                                           \
					(float)DT_PROP(algo_node_id,                             \
						       open_loop_startup_current_ma) /           \
					1000.0f,                                                 \
			},                                                                       \
		.timing =                                                                        \
			{                                                                        \
				.control_loop_dt_s =                                             \
					(float)DT_PROP(DT_PHANDLE(controller_node_id, actuator), \
						       slow_sample_div) /                        \
					(float)DT_PROP(DT_PHANDLE(controller_node_id, actuator), \
						       pwm_frequency),                           \
			},                                                                       \
		.i_integral = 0.0f,                                                               \
		.pll_integral = 0.0f,                                                             \
		.ki_dt = 0.0f,                                                                    \
		.pll_ki_dt = 0.0f,                                                                \
		.pll_primed = false,                                                              \
	}

#endif /* ZEPHYR_SUBSYS_MOTOR_ALGO_VELOCITY_PI_PRIV_H_ */
