/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_TYPES_H_
#define ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_TYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Motor Control Subsystem — Shared Types
 * @defgroup motor_types Motor Control Types
 * @ingroup motor_control
 * @{
 */

/* ------------------------------------------------------------------ */
/* Motor state machine (dc-current path: IDLE <-> RUN)                 */
/* ------------------------------------------------------------------ */

/**
 * @brief Controller operational state.
 */
enum motor_state {
	MOTOR_STATE_UNINIT = 0,
	MOTOR_STATE_IDLE = 1,
	MOTOR_STATE_RUN = 2,
	MOTOR_STATE_FAULT = 3,
};

/* ------------------------------------------------------------------ */
/* Control mode (extend when FOC/outer loops are added)                */
/* ------------------------------------------------------------------ */

enum motor_control_mode {
	MOTOR_MODE_CURRENT = 0,
};

/* ------------------------------------------------------------------ */
/* Fault flags (subset used by current DC + stage backends)              */
/* ------------------------------------------------------------------ */

#define MOTOR_FAULT_OVERCURRENT    BIT(0)
#define MOTOR_FAULT_OVERVOLTAGE    BIT(1)
#define MOTOR_FAULT_UNDERVOLTAGE   BIT(2)
#define MOTOR_FAULT_OVERTEMP_MOTOR BIT(3)
#define MOTOR_FAULT_OVERTEMP_BOARD BIT(4)
#define MOTOR_FAULT_ENCODER_ERROR  BIT(5)
#define MOTOR_FAULT_STALL          BIT(6)
#define MOTOR_FAULT_GATE_DRIVER    BIT(7)

/* ------------------------------------------------------------------ */
/* Drive / braking mode (power stage)                                 */
/* ------------------------------------------------------------------ */

enum motor_drive_mode {
	MOTOR_DRIVE_NORMAL = 0,
	MOTOR_DRIVE_COAST = 1,
	MOTOR_DRIVE_BRAKE = 2,
};

/**
 * @brief Pipeline stage for block scheduling (extend when outer blocks exist).
 */
enum motor_pipeline_stage {
	MOTOR_STAGE_INNER_ISR = 0,
	MOTOR_STAGE_COUNT,
};

struct motor_ctrl;

typedef struct motor_ctrl *motor_t;

typedef void (*motor_state_cb_t)(motor_t motor, enum motor_state state, void *user_data);
typedef void (*motor_fault_notify_cb_t)(motor_t motor, uint32_t faults, void *user_data);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_TYPES_H_ */
