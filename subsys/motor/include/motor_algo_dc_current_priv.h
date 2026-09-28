/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_MOTOR_ALGO_DC_CURRENT_PRIV_H_
#define ZEPHYR_SUBSYS_MOTOR_ALGO_DC_CURRENT_PRIV_H_

#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/subsys/motor/algorithms/dc_current/motor_algo_dc_current.h>
#include <zephyr/subsys/motor/motor_block.h>

struct motor_dc_current_timing {
	float control_loop_dt_s;
};

struct motor_algo_dc_current_data {
	struct motor_block base;
	float i_integral;
	struct motor_dc_current_state state;
	struct motor_dc_current_pi pi;
	struct motor_dc_current_limits limits;
	struct motor_dc_current_timing timing;
	/* Precomputed in block_set_params: ki * control_loop_dt_s. */
	float ki_dt;
};

void motor_dc_current_block_entry(struct motor_block *self, const struct motor_block_in *in,
				  struct motor_block_out *out);
void motor_dc_current_block_reset(struct motor_block *self);
void motor_dc_current_block_set_params(struct motor_block *self);

#define MOTOR_ALGO_DC_CURRENT_BASE_INITIALIZER                                                     \
	.base =                                                                                    \
		{                                                                                  \
			.name = "dc_current",                                                    \
			.stage = MOTOR_STAGE_INNER_ISR,                                          \
			.period_div = 1,                                                         \
			.entry = motor_dc_current_block_entry,                                   \
			.reset = motor_dc_current_block_reset,                                   \
			.set_params = motor_dc_current_block_set_params,                         \
		},

#define MOTOR_DC_CURRENT_DATA_INITIALIZER(controller_node_id, algo_node_id)                         \
	{                                                                                          \
		MOTOR_ALGO_DC_CURRENT_BASE_INITIALIZER                                             \
		.i_integral = 0.0f,                                                            \
		.state = {0},                                                                  \
		.pi = {                                                                        \
			.kp = (float)DT_PROP(algo_node_id, dc_current_kp_milli) / 1000.0f,     \
			.ki = (float)DT_PROP(algo_node_id, dc_current_ki_milli) / 1000.0f,     \
			.out_min = (float)(int32_t)DT_PROP(algo_node_id,                       \
							   dc_current_out_min_milli) /         \
				   1000.0f,                                                    \
			.out_max = (float)(int32_t)DT_PROP(algo_node_id,                       \
							   dc_current_out_max_milli) /         \
				   1000.0f,                                                    \
		},                                                                               \
		.limits =                                                                         \
			{                                                                        \
				.i_max_a = (float)DT_PROP(algo_node_id, i_max_ma) / 1000.0f,   \
				.vbus_derating_start = 0.0f,                                     \
				.temp_derating_start = 0.0f,                                     \
				.temp_fault = 0.0f,                                              \
			},                                                                       \
		.timing =                                                                        \
			{                                                                        \
				.control_loop_dt_s =                                             \
					1.0f / (float)DT_PROP(DT_PHANDLE(controller_node_id,    \
									 actuator),             \
							      pwm_frequency),                    \
			},                                                                       \
		.ki_dt = 0.0f,                                                                   \
	}

#endif /* ZEPHYR_SUBSYS_MOTOR_ALGO_DC_CURRENT_PRIV_H_ */
