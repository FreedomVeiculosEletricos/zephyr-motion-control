/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_PIPELINE_H_
#define ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_PIPELINE_H_

#include <stdint.h>

#include <zephyr/subsys/motor/motor_block.h>
#include <zephyr/subsys/motor/motor_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Skeleton for the upcoming N-block control pipeline.
 *
 * A @c motor_pipeline is an ordered array of @ref motor_block pointers, plus
 * optional pipeline-level hooks (init/reset/set_params) for cross-block
 * concerns. The first revision has no runtime: the structure and macros are
 * defined here so authors can model their algorithms against the final shape
 * while the controller still runs the @ref motor_algo_ops 3-slot vtable.
 *
 * Migration plan (future patches):
 *   1. Implement @c motor_pipeline_run_stage(pipeline, stage, in, out) in
 *      subsys/motor_pipeline.c.
 *   2. Make @ref motor_ctrl accept a @c motor_pipeline as @c algo_data and
 *      route motor_ctrl_run_inner/outer to motor_pipeline_run_stage.
 *   3. Migrate @c motor_algo_dc_torque to a single-block pipeline.
 *   4. Remove @c motor_algo_ops.
 */

struct motor_pipeline;

/** Optional pipeline-level hooks. */
typedef int (*motor_pipeline_init_t)(struct motor_pipeline *self,
				     const struct motor_ctrl_params *params);
typedef void (*motor_pipeline_reset_t)(struct motor_pipeline *self);
typedef void (*motor_pipeline_set_params_t)(struct motor_pipeline *self,
					    const struct motor_ctrl_params *params);

/**
 * @brief Pipeline = ordered array of blocks plus optional top-level hooks.
 *
 * @c blocks is sorted such that all blocks of the same stage are contiguous;
 * the controller iterates @c blocks at each stage tick and runs every block
 * whose @c period_div fires.
 */
struct motor_pipeline {
	const char *name;
	struct motor_block * const *blocks;
	uint8_t n_blocks;
	motor_pipeline_init_t init;
	motor_pipeline_reset_t reset;
	motor_pipeline_set_params_t set_params;
};

/**
 * @brief Define a @c motor_pipeline literal listing its blocks.
 *
 * Future macro shape (placeholder, expects the runtime to be implemented):
 *
 * @code
 * MOTOR_PIPELINE_DEFINE(my_dc_torque, &my_pi.base);
 * @endcode
 *
 * For now the macro is a thin wrapper around an aggregate initializer; the
 * generated symbol becomes useful once @c motor_pipeline_run_stage exists.
 */
#define MOTOR_PIPELINE_DEFINE(_name, ...)                                                          \
	static struct motor_block * const _name##_blocks[] = { __VA_ARGS__ };                      \
	static struct motor_pipeline _name = {                                                     \
		.name = #_name,                                                                    \
		.blocks = _name##_blocks,                                                          \
		.n_blocks = (uint8_t)ARRAY_SIZE(_name##_blocks),                                   \
	}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_PIPELINE_H_ */
