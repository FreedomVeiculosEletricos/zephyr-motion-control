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
 * @brief Ordered list of blocks; each @ref motor_block_in / @ref motor_block_out
 * hop is one schedulable step. Setpoints are written by @ref motor_set_current
 * on @c motor_ctrl::setpoints, not via the pipeline structure.
 */
struct motor_pipeline {
	const char *name;
	struct motor_block *const *blocks;
	uint8_t n_blocks;
};

void motor_pipeline_run_stage(struct motor_pipeline *pipeline, void *ctx,
			      enum motor_pipeline_stage stage, uint32_t stage_tick,
			      const struct motor_block_in *in, struct motor_block_out *out);

#define MOTOR_PIPELINE_DEFINE(_name, ...)                                                          \
	static struct motor_block *const _name##_blocks[] = {__VA_ARGS__};                           \
	static struct motor_pipeline _name = {                                                     \
		.name = #_name,                                                                    \
		.blocks = _name##_blocks,                                                          \
		.n_blocks = (uint8_t)ARRAY_SIZE(_name##_blocks),                                   \
	}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_PIPELINE_H_ */
