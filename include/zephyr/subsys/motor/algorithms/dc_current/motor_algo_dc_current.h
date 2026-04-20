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
 * Predefined pipeline: one inner-loop block with scalar current PI and
 * @ref MOTOR_SUBSYS_DEFINE_DT for Devicetree-backed instances.
 *
 * Tunable parameters for this algorithm are defined below — not in
 * @ref motor_controller.h or @ref motor.h. Future algorithms (FOC, …) define
 * their own structs in their own headers.
 */

/**
 * Inner-loop current PI (normalised output / duty command).
 */
struct motor_dc_current_pi {
	float kp;
	float ki;
	float out_min;
	float out_max;
};

/**
 * Software limits and derating thresholds (algorithm-owned state).
 */
struct motor_dc_current_limits {
	float i_max_a;
	float speed_max_rad_s;
	float vbus_derating_start;
	float temp_derating_start;
	float temp_fault;
};

/**
 * Sample timing for the inner loop (ISR cadence).
 */
struct motor_dc_current_timing {
	float control_loop_dt_s;
};

/**
 * Algorithm state for the DC brushed scalar current loop.
 *
 * Embeds @ref motor_block as the first member so entries use @c CONTAINER_OF.
 * Fields are filled from DT via @ref MOTOR_DC_CURRENT_DATA_INITIALIZER or
 * by tests.
 */
struct motor_algo_dc_current_data {
	struct motor_block base;
	/** Protects @c pi and @c i_integral against the inner ISR. */
	struct k_spinlock current_pi_lock;
	float i_integral;
	struct motor_dc_current_pi pi;
	struct motor_dc_current_limits limits;
	struct motor_dc_current_timing timing;
	float kt_nm_per_a;
	uint8_t pole_pairs;
};

void motor_dc_current_block_entry(struct motor_block *self, const struct motor_block_in *in,
				  struct motor_block_out *out);
void motor_dc_current_block_reset(struct motor_block *self);

#include <zephyr/subsys/motor/algorithms/dc_current/motor_algo_dc_current_priv.h>

#define MOTOR_ALGO_DC_CURRENT_DATA_SIZE sizeof(struct motor_algo_dc_current_data)

/**
 * @brief Live-update inner-loop PI gains (thread context).
 *
 * Safe vs. the current-loop ISR. Fails if @a motor is not running the DC
 * current block (pipeline mismatch).
 *
 * @param reset_integral If true, clears the integrator (recommended after large gain steps).
 * @retval 0 on success, -EINVAL, or -ENOTSUP if the handle is not DC current.
 */
int motor_algo_dc_current_set_pi_gains(motor_t motor, const struct motor_dc_current_pi *pi,
				       bool reset_integral);

/**
 * @brief Update software current / speed / temperature limits carried in algorithm state.
 *
 * Thread context; same synchronisation as @ref motor_algo_dc_current_set_pi_gains.
 */
int motor_algo_dc_current_set_limits(motor_t motor, const struct motor_dc_current_limits *limits);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_DC_CURRENT_MOTOR_ALGO_DC_CURRENT_H_ */
