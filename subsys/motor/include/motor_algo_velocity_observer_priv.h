/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_MOTOR_ALGO_VELOCITY_OBSERVER_PRIV_H_
#define ZEPHYR_SUBSYS_MOTOR_ALGO_VELOCITY_OBSERVER_PRIV_H_

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/subsys/motor/algorithms/velocity_observer/motor_algo_velocity_observer.h>
#include <zephyr/subsys/motor/motor_block.h>

struct motor_velocity_observer_timing {
	float control_loop_dt_s;
};

struct motor_algo_velocity_observer_data {
	struct motor_block base;
	struct motor_velocity_observer_state state;
	struct motor_velocity_observer_params params;
	struct motor_velocity_observer_limits limits;
	struct motor_velocity_observer_timing timing;
	/* Optional virtual encoder fed with the estimate; NULL disables it. */
	const struct device *sink;
	float inv_l;
	float inv_ke;
	float r_dt_over_l;
	float dt_over_l;
	float l1_dt;
	float l2_dt;
	float pll_ki_dt;
	float pll_integral;
	uint32_t settle_count;
};

void motor_velocity_observer_block_entry(struct motor_block *self, const struct motor_block_in *in,
					 struct motor_block_out *out);
void motor_velocity_observer_block_reset(struct motor_block *self);
void motor_velocity_observer_block_set_params(struct motor_block *self);

#define MOTOR_ALGO_VELOCITY_OBSERVER_BASE_INITIALIZER                                              \
	.base =                                                                                    \
		{                                                                                  \
			.name = "velocity_observer",                                               \
			.stage = MOTOR_STAGE_INNER_ISR,                                            \
			.period_div = 1,                                                           \
			.entry = motor_velocity_observer_block_entry,                              \
			.reset = motor_velocity_observer_block_reset,                              \
			.set_params = motor_velocity_observer_block_set_params,                    \
		},

#define MOTOR_VELOCITY_OBSERVER_SINK(algo_node_id)                                                 \
	COND_CODE_1(DT_NODE_HAS_PROP(algo_node_id, sensor_sink),                                   \
		    (DEVICE_DT_GET(DT_PHANDLE(algo_node_id, sensor_sink))), (NULL))

#define MOTOR_VELOCITY_OBSERVER_DATA_INITIALIZER(controller_node_id, algo_node_id)                 \
	{                                                                                          \
		MOTOR_ALGO_VELOCITY_OBSERVER_BASE_INITIALIZER                                      \
		.state = {0},                                                                      \
		.params =                                                                          \
			{                                                                          \
				.r_ohm = (float)DT_PROP(algo_node_id, motor_resistance_mohm) /      \
					 1000.0f,                                                  \
				.l_h = (float)DT_PROP(algo_node_id, motor_inductance_uh) /          \
				       1000000.0f,                                                 \
				.ke_v_per_rad_s =                                                  \
					(float)DT_PROP(algo_node_id, motor_ke_uv_per_rad_s) /       \
					1000000.0f,                                                \
				.vbus_v = (float)DT_PROP(algo_node_id, vbus_nominal_mv) / 1000.0f,  \
				.bandwidth_rad_s =                                                 \
					(float)DT_PROP(algo_node_id, observer_bandwidth_rad_s),     \
				.pll_kp = (float)DT_PROP(algo_node_id, pll_kp_milli) / 1000.0f,     \
				.pll_ki = (float)DT_PROP(algo_node_id, pll_ki_milli) / 1000.0f,     \
			},                                                                         \
		.limits =                                                                          \
			{                                                                          \
				.valid_threshold_rad_s =                                           \
					(float)DT_PROP(algo_node_id, valid_threshold_mrad_s) /      \
					1000.0f,                                                   \
				.valid_release_rad_s =                                             \
					(float)DT_PROP(algo_node_id, valid_release_mrad_s) /        \
					1000.0f,                                                   \
				.settle_ticks = DT_PROP(algo_node_id, settle_ticks),                \
			},                                                                         \
		.timing =                                                                          \
			{                                                                          \
				.control_loop_dt_s =                                               \
					1.0f / (float)DT_PROP(DT_PHANDLE(controller_node_id,        \
									 actuator),                \
							      pwm_frequency),                      \
			},                                                                         \
		.sink = MOTOR_VELOCITY_OBSERVER_SINK(algo_node_id),                                \
		.inv_l = 0.0f,                                                                     \
		.inv_ke = 0.0f,                                                                    \
		.r_dt_over_l = 0.0f,                                                               \
		.dt_over_l = 0.0f,                                                                 \
		.l1_dt = 0.0f,                                                                     \
		.l2_dt = 0.0f,                                                                     \
		.pll_ki_dt = 0.0f,                                                                 \
		.pll_integral = 0.0f,                                                              \
		.settle_count = 0U,                                                                \
	}

#endif /* ZEPHYR_SUBSYS_MOTOR_ALGO_VELOCITY_OBSERVER_PRIV_H_ */
