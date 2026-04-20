/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_BLOCK_H_
#define ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_BLOCK_H_

#include <stdint.h>

#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/subsys/motor/motor_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Skeleton for the upcoming N-block control pipeline.
 *
 * This header declares the @c motor_block contract that will replace the
 * 3-slot @c motor_algo_ops vtable. It is intentionally a *forward-declaration
 * + macro skeleton*: there is no runtime yet. It exists so out-of-tree
 * algorithm/pipeline authors can review the contract and so the migration
 * patches that follow are reviewable in isolation.
 *
 * Conceptual model
 * ----------------
 * An algorithm is a @ref motor_pipeline — an ordered array of @ref motor_block
 * instances. Each block:
 *
 *   - Embeds @c struct motor_block somewhere in its concrete state struct and
 *     reaches its private state via @c CONTAINER_OF inside its entry function.
 *   - Declares the @ref motor_pipeline_stage it belongs to and a unitless
 *     @c period_div: the block runs once every @c period_div ticks of its
 *     stage's counter (motor_ctrl maintains @c stage_tick[]). The base rate
 *     for @c MOTOR_STAGE_INNER_ISR is set by the actuator driver's PWM rate;
 *     other stages are paced by their thread.
 *   - Reads from @ref motor_block_in (sensor + setpoints + algo top pointer)
 *     and writes to @ref motor_block_out (setpoints and, only for the
 *     terminal block, the actuator command).
 *
 * Composition (sketch — not yet implemented):
 *
 * @code
 * struct pi_current_block {
 *     struct motor_block base;
 *     float kp, ki, integ, out_min, out_max;
 * };
 *
 * static void pi_current_entry(struct motor_block *self,
 *                              const struct motor_block_in *in,
 *                              struct motor_block_out *out)
 * {
 *     struct pi_current_block *p = CONTAINER_OF(self, struct pi_current_block, base);
 *     float err = in->sp->i_torque_a - in->sense->hot.i_phase[0];
 *     // ... PI math, write out->cmd ...
 * }
 *
 * MOTOR_BLOCK_DEFINE(my_pi, pi_current_entry,
 *                    .stage = MOTOR_STAGE_INNER_ISR, .period_div = 1);
 * MOTOR_PIPELINE_DEFINE(my_dc_torque, &my_pi.base);
 * @endcode
 *
 * @note Skeleton only. Until @c MOTOR_BLOCK_DEFINE / @c MOTOR_PIPELINE_DEFINE
 * are wired into motor_ctrl, algorithms still implement @ref motor_algo_ops.
 */

struct motor_block;

/**
 * @brief Entry point invoked by motor_ctrl when the block is due to run.
 *
 * @param self Pointer to the @c motor_block embedded in the concrete block
 *             state. Use @c CONTAINER_OF to recover the private struct.
 * @param in   Input bus (sensor + current setpoints + algo top pointer).
 * @param out  Output bus (setpoints to write; @c cmd non-NULL only for the
 *             terminal block at @c MOTOR_STAGE_INNER_ISR).
 */
typedef void (*motor_block_entry_t)(struct motor_block *self, const struct motor_block_in *in,
				    struct motor_block_out *out);

/**
 * @brief Optional per-block init invoked once when the controller enters RUN.
 */
typedef int (*motor_block_init_t)(struct motor_block *self,
				  const struct motor_ctrl_params *params);

/**
 * @brief Optional per-block reset invoked on STOP -> IDLE.
 */
typedef void (*motor_block_reset_t)(struct motor_block *self);

/**
 * @brief Optional per-block runtime parameter update.
 */
typedef void (*motor_block_set_params_t)(struct motor_block *self,
					 const struct motor_ctrl_params *params);

/**
 * @brief Generic pipeline block.
 *
 * Concrete blocks embed this struct; recover the concrete state with
 * @c CONTAINER_OF inside the entry function.
 */
struct motor_block {
	const char *name;
	enum motor_pipeline_stage stage;
	uint16_t period_div;
	motor_block_init_t init;
	motor_block_entry_t entry;
	motor_block_reset_t reset;
	motor_block_set_params_t set_params;
};

/**
 * @brief Static initializer for a @c motor_block.
 *
 * Use as the @c .base initializer of a concrete block:
 *
 * @code
 * static struct pi_current_block my_pi = {
 *     .base = MOTOR_BLOCK_INIT(.name = "pi_current",
 *                              .stage = MOTOR_STAGE_INNER_ISR,
 *                              .period_div = 1,
 *                              .entry = pi_current_entry),
 *     .kp = 0.5f, .ki = 2000.0f,
 * };
 * @endcode
 */
#define MOTOR_BLOCK_INIT(...) { __VA_ARGS__ }

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_BLOCK_H_ */
