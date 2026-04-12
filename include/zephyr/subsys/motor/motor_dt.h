/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_DT_H_
#define ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_DT_H_

/**
 * @file motor_dt.h
 * @brief Compile-time extraction of motor-controller parameters from Devicetree.
 *
 * Values are taken from a `zephyr,motor-controller` node using Zephyr @c DT_PROP
 * macros — they must be used with a **constant** node identifier (e.g.
 * @c DT_NODELABEL(motor_brushed)). There is no runtime DT walk.
 *
 * Used internally by @ref MOTOR_SUBSYS_DEFINE_DT; applications should not
 * need these macros unless declaring a custom registration path.
 */

#include <zephyr/devicetree.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/subsys/motor/motor_algo_dc_torque.h>

/** @internal 2π/60 — RPM to rad/s */
#define MOTOR_DT_RPM_TO_RADS (0.10471975512f)

/**
 * @brief Static initializer for @ref motor_ctrl_params from a motor-controller node.
 *
 * @param node_id Devicetree node identifier (e.g. @c DT_NODELABEL(motor0)).
 */
#define MOTOR_CTRL_PARAMS_INITIALIZER(node_id)                                                   \
	{                                                                                          \
		.limits =                                                                          \
			{                                                                          \
				.i_max_a = (float)DT_PROP(node_id, i_max_ma) / 1000.0f,            \
				.speed_max_rad_s = (float)DT_PROP(node_id, speed_max_rpm) *          \
						    MOTOR_DT_RPM_TO_RADS,                              \
				.vbus_derating_start = 0.0f,                                       \
				.temp_derating_start =                                               \
					(float)DT_PROP_OR(node_id, temp_derating_start_dc, 0) /    \
					100.0f,                                                      \
				.temp_fault =                                                      \
					(float)DT_PROP_OR(node_id, temp_fault_dc, 0) / 100.0f,     \
			},                                                                         \
		.timing =                                                                          \
			{                                                                          \
				.control_loop_dt_s =                                               \
					1.0f / (float)DT_PROP_OR(node_id, current_loop_rate_hz,    \
								 20000),                         \
			},                                                                         \
		.kt_nm_per_a = (float)DT_PROP_OR(node_id, dc_kt_mnm_per_a, 0) / 1000.0f,            \
		.pole_pairs = (uint8_t)DT_PROP(node_id, pole_pairs),                             \
		.speed_loop = {0},                                                                 \
		.accel = {0},                                                                      \
		.kp_pos = 0.0f,                                                                    \
		.kd_pos = 0.0f,                                                                    \
	}

/**
 * @brief Static initializer for @ref motor_algo_dc_torque_data from a motor-controller node.
 *
 * Inner-loop gains come from optional DT properties (milli-scaled); limits/timing/kt are
 * refreshed in @ref motor_algo_dc_torque init from @ref motor_ctrl_params.
 *
 * @param node_id Devicetree node identifier for the @c zephyr,motor-controller instance.
 */
#define MOTOR_DC_TORQUE_DATA_INITIALIZER(node_id)                                                \
	{                                                                                          \
		.i_integral = 0.0f,                                                              \
		.current_loop = {                                                                  \
			.kp = (float)DT_PROP_OR(node_id, dc_current_kp_milli, 500) / 1000.0f,   \
			.ki = (float)DT_PROP_OR(node_id, dc_current_ki_milli, 2000000) / 1000.0f, \
			.out_min = (float)DT_PROP_OR(node_id, dc_current_out_min_milli, (-1000)) / \
				   1000.0f,                                                        \
			.out_max = (float)DT_PROP_OR(node_id, dc_current_out_max_milli, (1000)) /  \
				   1000.0f,                                                        \
		},                                                                                 \
	}

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_DT_H_ */
