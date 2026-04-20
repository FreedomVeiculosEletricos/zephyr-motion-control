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
 *
 * Units convention (SI throughout, no per-unit normalisation):
 *   Angle        : float, radians         [0, 2π)
 *   Angular vel  : float, rad/s
 *   Current      : float, amperes (A)
 *   Voltage      : float, volts (V)
 *   Torque       : float, newton-metres (N·m)
 *   Temperature  : float, degrees Celsius (°C)
 *   Duty cycle   : float, normalised [−1.0, +1.0]  (negative = reverse)
 *   Time         : uint32_t, nanoseconds (for deadtime, periods)
 *
 * Fixed-point note:
 *   Targets with FPU (Cortex-M4F/M7/M55) use float natively.
 *   For FPU-less targets the backend may use Q15/Q31 internally, but
 *   the public API surface is always float — conversion happens inside
 *   the backend init/update functions.
 */

/* ------------------------------------------------------------------ */
/* Motor state machine states                                          */
/* ------------------------------------------------------------------ */

/**
 * @brief Motor controller operational states.
 *
 * Transitions:
 *
 *   UNINIT ──init()──────► IDLE
 *   IDLE   ──enable()───► ALIGN  (incremental encoder: rotor search)
 *   IDLE   ──enable()───► RUN    (absolute position known: skip ALIGN)
 *   ALIGN  ──done────────► RUN
 *   RUN    ──disable()──► STOP   (controlled deceleration)
 *   STOP   ──complete───► IDLE
 *   ANY    ──sto_arm()──► STO    (Safe Torque Off — HW PWM inhibit)
 *   STO    ──sto_release► IDLE   (dual-channel confirmation required)
 *   ANY    ──fault───────► FAULT
 *   FAULT  ──clear()────► IDLE   (after fault cause resolved)
 */
enum motor_state {
	MOTOR_STATE_UNINIT = 0,
	MOTOR_STATE_IDLE = 1,
	MOTOR_STATE_ALIGN = 2, /* rotor pre-alignment / index search         */
	MOTOR_STATE_RUN = 3,
	MOTOR_STATE_STOP = 4, /* controlled deceleration in progress        */
	MOTOR_STATE_STO = 5,  /* Safe Torque Off — outputs HW-inhibited     */
	MOTOR_STATE_FAULT = 6,
};

/* ------------------------------------------------------------------ */
/* Motor control modes (interface A command modes)                     */
/* ------------------------------------------------------------------ */

enum motor_control_mode {
	MOTOR_MODE_OPEN_LOOP_VOLTAGE = 0, /* raw Vd/Vq or duty, no feedback  */
	MOTOR_MODE_CURRENT = 1,           /* torque control via Id/Iq targets */
	MOTOR_MODE_SPEED = 2,             /* outer speed loop → inner current */
	MOTOR_MODE_POSITION = 3,          /* outer position loop → speed → current */
};

/* ------------------------------------------------------------------ */
/* Fault flags (bitmask, fits uint32_t)                                */
/* ------------------------------------------------------------------ */

#define MOTOR_FAULT_OVERCURRENT    BIT(0)  /* phase current exceeded limit     */
#define MOTOR_FAULT_OVERVOLTAGE    BIT(1)  /* Vbus exceeded OV threshold       */
#define MOTOR_FAULT_UNDERVOLTAGE   BIT(2)  /* Vbus below UV threshold          */
#define MOTOR_FAULT_OVERTEMP_MOTOR BIT(3)  /* motor winding overtemperature    */
#define MOTOR_FAULT_OVERTEMP_BOARD BIT(4)  /* power stage PCB overtemperature  */
#define MOTOR_FAULT_PHASE_LOSS     BIT(5)  /* open phase detected              */
#define MOTOR_FAULT_ENCODER_ERROR  BIT(6)  /* position sensor lost/invalid     */
#define MOTOR_FAULT_DESYNC         BIT(7)  /* sensorless lost tracking         */
#define MOTOR_FAULT_STALL          BIT(8)  /* motor stalled                    */
#define MOTOR_FAULT_SHORT_HS       BIT(9)  /* high-side transistor short       */
#define MOTOR_FAULT_SHORT_LS       BIT(10) /* low-side transistor short        */
#define MOTOR_FAULT_SHORT_MULTI    BIT(11) /* multiple transistor shorts       */
#define MOTOR_FAULT_GATE_DRIVER    BIT(12) /* gate driver nFAULT asserted      */
#define MOTOR_FAULT_SW_WATCHDOG    BIT(13) /* control loop overrun             */
#define MOTOR_FAULT_STO_MISMATCH   BIT(14) /* STO dual-channel discrepancy     */

/* ------------------------------------------------------------------ */
/* Braking / drive modes for the power stage                           */
/* ------------------------------------------------------------------ */

enum motor_drive_mode {
	MOTOR_DRIVE_NORMAL = 0, /* normal PWM motoring                 */
	MOTOR_DRIVE_COAST = 1,  /* all switches off, freewheeling      */
	MOTOR_DRIVE_BRAKE = 2,  /* active short-circuit braking        */
	MOTOR_DRIVE_REGEN = 3,  /* regenerative braking (3-phase only) */
};

/* ------------------------------------------------------------------ */
/* Acceleration profile types                                          */
/* ------------------------------------------------------------------ */

enum motor_accel_profile {
	MOTOR_ACCEL_NONE = 0,        /* step change (open-loop / current mode) */
	MOTOR_ACCEL_TRAPEZOIDAL = 1, /* linear ramp up / constant / ramp down  */
	MOTOR_ACCEL_SCURVE = 2,      /* jerk-limited S-curve                   */
};

/**
 * @brief Pipeline stage at which a control block is dispatched.
 *
 * Each stage has an independent tick counter inside motor_ctrl. Blocks declare
 * @c period_div: a block runs once every @c period_div ticks of its own stage.
 * The base rate for INNER_ISR is set by the actuator driver (PWM ISR rate);
 * outer / supervision base rates are set by their respective threads.
 *
 * Today the controller implements three slots (inner, outer_0, outer_1) plus a
 * future supervision rate. This enum is the contract for the upcoming N-block
 * pipeline; the 3-slot vtable maps as:
 *
 *   motor_algo_ops.inner_step    -> MOTOR_STAGE_INNER_ISR
 *   motor_algo_ops.outer_step_0  -> MOTOR_STAGE_OUTER_FAST
 *   motor_algo_ops.outer_step_1  -> MOTOR_STAGE_OUTER_SLOW
 */
enum motor_pipeline_stage {
	MOTOR_STAGE_INNER_ISR = 0,
	MOTOR_STAGE_OUTER_FAST = 1,
	MOTOR_STAGE_OUTER_SLOW = 2,
	MOTOR_STAGE_SUPERVISION = 3,
	MOTOR_STAGE_COUNT,
};

struct motor_ctrl;

/**
 * @brief Opaque motor handle returned by motor_init().
 *
 * The application holds this pointer and passes it to all motor_*()
 * calls. Internal structure is not part of the public API.
 */
typedef struct motor_ctrl *motor_t;

/**
 * @brief State change notification callback.
 *
 * Invoked from the supervision thread when the controller state machine
 * transitions.
 */
typedef void (*motor_state_cb_t)(motor_t motor, enum motor_state state, void *user_data);

/**
 * @brief Fault notification callback.
 *
 * Invoked from the supervision thread when a new fault is detected; the
 * motor is already in MOTOR_STATE_FAULT.
 */
typedef void (*motor_fault_notify_cb_t)(motor_t motor, uint32_t faults, void *user_data);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_TYPES_H_ */
