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
 * Vtable fields and @ref MOTOR_ALGO_DC_CURRENT_BASE_INITIALIZER for static
 * algorithm storage. State is set by @ref MOTOR_DC_CURRENT_DATA_INITIALIZER
 * (Devicetree) or C initializers, not a block @c init.
 */

#include <zephyr/subsys/motor/motor_block.h>

void motor_dc_current_block_set_params(struct motor_block *self);

#define MOTOR_ALGO_DC_CURRENT_BASE_INITIALIZER                                                       \
	.base =                                                                                      \
		{                                                                                    \
			.name = "dc_current",                                                      \
			.stage = MOTOR_STAGE_INNER_ISR,                                            \
			.period_div = 1,                                                           \
			.entry = motor_dc_current_block_entry,                                     \
			.reset = motor_dc_current_block_reset,                                     \
			.set_params = motor_dc_current_block_set_params,                             \
		},

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_ALGORITHMS_DC_CURRENT_MOTOR_ALGO_DC_CURRENT_PRIV_H_ */
