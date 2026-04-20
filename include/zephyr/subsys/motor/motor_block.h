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
 * @brief One schedulable unit inside a @ref motor_pipeline.
 *
 * Algorithms compose @c motor_block instances (each embedded in private state).
 * @ref motor_pipeline_run_stage dispatches by @c stage and @c period_div.
 * The @c .init pointer is invoked during pipeline bootstrap by the subsystem,
 * not from application code.
 */

struct motor_block;

typedef void (*motor_block_entry_t)(struct motor_block *self, const struct motor_block_in *in,
				    struct motor_block_out *out);

typedef int (*motor_block_init_t)(struct motor_block *self);

typedef void (*motor_block_reset_t)(struct motor_block *self);

typedef void (*motor_block_set_params_t)(struct motor_block *self);

struct motor_block {
	const char *name;
	enum motor_pipeline_stage stage;
	uint16_t period_div;
	motor_block_init_t init;
	motor_block_entry_t entry;
	motor_block_reset_t reset;
	motor_block_set_params_t set_params;
};

#define MOTOR_BLOCK_INIT(...) { __VA_ARGS__ }

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_BLOCK_H_ */
