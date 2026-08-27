/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_MOTOR_ALGO_ADRC_CURRENT_PRIV_H_
#define ZEPHYR_SUBSYS_MOTOR_ALGO_ADRC_CURRENT_PRIV_H_

#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/subsys/motor/algorithms/adrc_current/motor_algo_adrc_current.h>
#include <zephyr/subsys/motor/motor_block.h>

struct motor_adrc_current_timing {
	float control_loop_dt_s;
};

struct motor_algo_adrc_current_data {
	struct motor_block base;
	struct motor_adrc_current_state state;
	struct motor_adrc_current_params params;
	struct motor_adrc_current_limits limits;
	struct motor_adrc_current_timing timing;
	float inv_b0;
	float beta1_dt;
	float beta2_dt;
	bool sign_magnitude;
};

void motor_adrc_current_block_entry(struct motor_block *self, const struct motor_block_in *in,
				    struct motor_block_out *out);
void motor_adrc_current_block_reset(struct motor_block *self);
void motor_adrc_current_block_set_params(struct motor_block *self);

#define MOTOR_ALGO_ADRC_CURRENT_BASE_INITIALIZER                                                   \
	.base =                                                                                    \
		{                                                                                  \
			.name = "adrc_current",                                                  \
			.stage = MOTOR_STAGE_INNER_ISR,                                          \
			.period_div = 1,                                                         \
			.entry = motor_adrc_current_block_entry,                                 \
			.reset = motor_adrc_current_block_reset,                                 \
			.set_params = motor_adrc_current_block_set_params,                       \
		},

#define MOTOR_ADRC_CURRENT_DATA_INITIALIZER(controller_node_id, algo_node_id)                       \
	{                                                                                          \
		MOTOR_ALGO_ADRC_CURRENT_BASE_INITIALIZER                                           \
		.state = {0},                                                                   \
		.params =                                                                        \
			{                                                                        \
				.kp = (float)DT_PROP(algo_node_id, adrc_current_kp_milli) /      \
				      1000.0f,                                                   \
				.b0 = (float)DT_PROP(algo_node_id, adrc_current_b0_milli) /      \
				      1000.0f,                                                   \
				.beta1 = (float)DT_PROP(algo_node_id, adrc_current_beta1_milli) /\
					 1000.0f,                                                \
				.beta2 = (float)DT_PROP(algo_node_id, adrc_current_beta2_milli) /\
					 1000.0f,                                                \
				.out_min = (float)(int32_t)DT_PROP(algo_node_id,                 \
								   adrc_current_out_min_milli) / \
					   1000.0f,                                              \
				.out_max = (float)(int32_t)DT_PROP(algo_node_id,                 \
								   adrc_current_out_max_milli) / \
					   1000.0f,                                              \
			},                                                                       \
		.limits =                                                                         \
			{                                                                        \
				.i_max_a = (float)DT_PROP(algo_node_id, i_max_ma) / 1000.0f,   \
			},                                                                       \
		.timing =                                                                        \
			{                                                                        \
				.control_loop_dt_s =                                             \
					1.0f / (float)DT_PROP(DT_PHANDLE(controller_node_id,    \
									 actuator),             \
							      pwm_frequency),                    \
			},                                                                       \
		.inv_b0 = 0.0f,                                                                   \
		.beta1_dt = 0.0f,                                                                 \
		.beta2_dt = 0.0f,                                                                 \
		.sign_magnitude = DT_PROP_OR(algo_node_id, sign_magnitude, 0),                   \
	}

#endif /* ZEPHYR_SUBSYS_MOTOR_ALGO_ADRC_CURRENT_PRIV_H_ */
