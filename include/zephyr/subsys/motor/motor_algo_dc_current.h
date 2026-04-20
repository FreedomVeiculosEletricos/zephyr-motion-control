/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_ALGO_DC_CURRENT_H_
#define ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_ALGO_DC_CURRENT_H_

#include <zephyr/subsys/motor/motor_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Algorithm state for @ref motor_algo_dc_current.
 *
 * Inner-loop PI gains live here — not in @ref motor_ctrl_params — so the
 * controller shell stays independent of this control law.
 */
struct motor_algo_dc_current_data {
	float i_integral;
	struct motor_pi_gains current_loop;
	/** Mirrored from @ref motor_ctrl_params on init / set_params. */
	struct motor_ctrl_limits limits;
	struct motor_ctrl_timing timing;
	float kt_nm_per_a;
	uint8_t pole_pairs;
};

/** Algorithm vtable for DC brushed scalar current (torque) loop. */
extern const struct motor_algo_ops motor_algo_dc_current;

#define MOTOR_ALGO_DC_CURRENT_DATA_SIZE sizeof(struct motor_algo_dc_current_data)

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_ALGO_DC_CURRENT_H_ */
