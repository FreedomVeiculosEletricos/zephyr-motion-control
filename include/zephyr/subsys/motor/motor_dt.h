/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_DT_H_
#define ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_DT_H_

/**
 * @file motor_dt.h
 * @brief Devicetree static initializers for algorithm instance data.
 *
 * Values are taken from a `zephyr,motor-controller` node using @c DT_PROP.
 * Used by @ref MOTOR_SUBSYS_DEFINE_DT. No shell layer — limits/timing/kt live in
 * the algorithm struct (here: @ref motor_algo_dc_current_data).
 */

#include <zephyr/devicetree.h>
#include <zephyr/subsys/motor/algorithms/dc_current/motor_algo_dc_current.h>

/** @internal 2π/60 — RPM to rad/s */
#define MOTOR_DT_RPM_TO_RADS (0.10471975512f)

/**
 * @brief Static initializer for @ref motor_algo_dc_current_data from a motor-controller node.
 *
 * Inner-loop gains, limits, timing, kt, and pole pairs come from DT — the sole
 * source for the DC current pipeline instance.
 *
 * @param node_id Devicetree node identifier (e.g. @c DT_NODELABEL(motor_brushed)).
 */
#define MOTOR_DC_CURRENT_DATA_INITIALIZER(node_id)                                                \
	{                                                                                          \
		MOTOR_ALGO_DC_CURRENT_BASE_INITIALIZER                                               \
		.i_integral = 0.0f,                                                              \
		.pi = {                                                                            \
			.kp = (float)DT_PROP(node_id, dc_current_kp_milli) / 1000.0f,            \
			.ki = (float)DT_PROP(node_id, dc_current_ki_milli) / 1000.0f,            \
			.out_min = (float)DT_PROP(node_id, dc_current_out_min_milli) / 1000.0f,  \
			.out_max = (float)DT_PROP(node_id, dc_current_out_max_milli) / 1000.0f,  \
		},                                                                                 \
		.limits =                                                                           \
			{                                                                          \
				.i_max_a = (float)DT_PROP(node_id, i_max_ma) / 1000.0f,            \
				.speed_max_rad_s = (float)DT_PROP(node_id, speed_max_rpm) *          \
						    MOTOR_DT_RPM_TO_RADS,                              \
				.vbus_derating_start = 0.0f,                                       \
				.temp_derating_start =                                               \
					(float)DT_PROP(node_id, temp_derating_start_dc) / 100.0f,    \
				.temp_fault =                                                      \
					(float)DT_PROP(node_id, temp_fault_dc) / 100.0f,           \
			},                                                                         \
		.timing =                                                                          \
			{                                                                          \
				.control_loop_dt_s =                                               \
					1.0f / (float)DT_PROP(node_id, current_loop_rate_hz),         \
			},                                                                         \
		.kt_nm_per_a = (float)DT_PROP(node_id, dc_kt_mnm_per_a) / 1000.0f,                  \
		.pole_pairs = (uint8_t)DT_PROP(node_id, pole_pairs),                             \
	}

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_DT_H_ */
