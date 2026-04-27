/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_DC_CURRENT_MOTOR_ALGO_DC_CURRENT_H_
#define ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_DC_CURRENT_MOTOR_ALGO_DC_CURRENT_H_

#include <zephyr/subsys/motor/motor_block.h>
#include <zephyr/subsys/motor/motor_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup motor_algo_dc_current DC current algorithm
 * @ingroup motor_control
 * @{
 *
 * One inner-loop block: scalar current PI, output as normalised duty to
 * @c MOTOR_ACTUATOR_CMD_DUTY_DIRECT.  Register with
 * @ref MOTOR_SUBSYS_DEFINE_DT / @ref MOTOR_SUBSYS_DEFINE_DT_DC_CURRENT and a
 * @c zephyr,motor-algorithm-dc-current node referenced by the controller's
 * @c algorithm phandle.  Devicetree expansion uses @ref MOTOR_DC_CURRENT_DATA_INITIALIZER.
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

struct motor_dc_current_timing {
	float control_loop_dt_s;
};

struct motor_algo_dc_current_data {
	struct motor_block base;
	struct k_spinlock current_pi_lock;
	float i_integral;
	struct motor_dc_current_pi pi;
	struct motor_dc_current_limits limits;
	struct motor_dc_current_timing timing;
};

void motor_dc_current_block_entry(struct motor_block *self, const struct motor_block_in *in,
				  struct motor_block_out *out);
void motor_dc_current_block_reset(struct motor_block *self);

#include <zephyr/subsys/motor/algorithms/dc_current/motor_algo_dc_current_priv.h>

#include <zephyr/devicetree.h>

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
				.vbus_derating_start = 0.0f,                                       \
				.temp_derating_start = 0.0f,                                       \
				.temp_fault = 0.0f,                                                \
			},                                                                         \
		.timing =                                                                          \
			{                                                                          \
				.control_loop_dt_s =                                               \
					1.0f / (float)DT_PROP(node_id, current_loop_rate_hz),         \
			},                                                                         \
	}

#define MOTOR_ALGO_DC_CURRENT_DATA_SIZE sizeof(struct motor_algo_dc_current_data)

int motor_algo_dc_current_set_pi_gains(motor_t motor, const struct motor_dc_current_pi *pi,
				       bool reset_integral);

int motor_algo_dc_current_set_limits(motor_t motor, const struct motor_dc_current_limits *limits);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_DC_CURRENT_MOTOR_ALGO_DC_CURRENT_H_ */
