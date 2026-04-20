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
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Default implementation in @c motor_pipeline.c that walks @c blocks and calls
 * each @c .init is subsystem-internal (see motor_ctrl_priv.h). Optional
 * @c motor_pipeline::init / @c reset / @c set_params hooks are for custom pipelines.
 */

struct motor_pipeline;

typedef int (*motor_pipeline_init_t)(struct motor_pipeline *pipeline, void *ctx);
typedef void (*motor_pipeline_reset_t)(struct motor_pipeline *pipeline, void *ctx);
typedef void (*motor_pipeline_set_params_t)(struct motor_pipeline *pipeline, void *ctx);

/**
 * @brief Ordered list of @ref motor_block plus optional top-level hooks.
 *
 * @c blocks lists every block for all stages; @ref motor_pipeline_run_stage
 * filters by @c stage and @c period_div.
 */
struct motor_pipeline {
	const char *name;
	struct motor_block *const *blocks;
	uint8_t n_blocks;
	motor_pipeline_init_t init;
	motor_pipeline_reset_t reset;
	motor_pipeline_set_params_t set_params;
};

/**
 * @brief Run every block whose @c stage and @c period_div match this tick.
 *
 * @param stage_tick  Value of @c motor_ctrl::stage_tick[stage] for this invocation
 *                    (modulo scheduling; incremented by the controller after the pass).
 */
void motor_pipeline_run_stage(struct motor_pipeline *pipeline, void *ctx,
			      enum motor_pipeline_stage stage, uint32_t stage_tick,
			      const struct motor_block_in *in, struct motor_block_out *out);

/**
 * @brief Static pipeline instance: one block pointer list.
 */
#define MOTOR_PIPELINE_DEFINE(_name, ...)                                                          \
	static struct motor_block *const _name##_blocks[] = {__VA_ARGS__};                           \
	static struct motor_pipeline _name = {                                                     \
		.name = #_name,                                                                    \
		.blocks = _name##_blocks,                                                          \
		.n_blocks = (uint8_t)ARRAY_SIZE(_name##_blocks),                                   \
		.init = NULL,                                                                      \
		.reset = NULL,                                                                     \
		.set_params = NULL,                                                                \
	}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_PIPELINE_H_ */
