/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_DC_CURRENT_MOTOR_ALGO_DC_CURRENT_PRIV_H_
#define ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_DC_CURRENT_MOTOR_ALGO_DC_CURRENT_PRIV_H_

/**
 * @file motor_algo_dc_current_priv.h
 *
 * Vtable symbols and @ref MOTOR_ALGO_DC_CURRENT_BASE_INITIALIZER for static
 * algorithm storage. Invoked only from @ref motor_pipeline_init — not
 * application code.
 */

#include <zephyr/subsys/motor/motor_block.h>

int motor_dc_current_block_init(struct motor_block *self);
void motor_dc_current_block_set_params(struct motor_block *self);

#define MOTOR_ALGO_DC_CURRENT_BASE_INITIALIZER                                                       \
	.base =                                                                                      \
		{                                                                                    \
			.name = "dc_current",                                                      \
			.stage = MOTOR_STAGE_INNER_ISR,                                            \
			.period_div = 1,                                                           \
			.init = motor_dc_current_block_init,                                       \
			.entry = motor_dc_current_block_entry,                                     \
			.reset = motor_dc_current_block_reset,                                     \
			.set_params = motor_dc_current_block_set_params,                             \
		},

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_DC_CURRENT_MOTOR_ALGO_DC_CURRENT_PRIV_H_ */
