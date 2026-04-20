/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_CONTROLLER_H_
#define ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_CONTROLLER_H_

#include <zephyr/subsys/motor/motor_types.h>
#include <zephyr/drivers/motor/motor_sensor.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/kernel.h>


#ifdef __cplusplus
extern "C" {
#endif

/* struct motor_ctrl is forward-declared in motor_types.h together with the
 * public motor_t and callback typedefs reused below.
 */

/**
 * @brief Motor Controller Layer (extension API)
 * @defgroup motor_controller Motor Controller Extension API
 * @ingroup motor_control
 * @{
 *
 * This header is the extension surface used by:
 *   - the motor subsystem implementation,
 *   - in-tree algorithm backends (@ref motor_pipeline + @ref motor_block), and
 *   - out-of-tree pipeline authors that compose the controller.
 *
 * Applications must use the @ref motor_app "motor_*()" facade in motor.h
 * instead. The motor_ctrl_*() entry points published here are not part of
 * the application API; their shape may evolve as the pipeline/N-block
 * architecture lands.
 *
 * Wiring (sensor, actuator, pipeline, algorithm, rates) is not configured here:
 * use @c MOTOR_SUBSYS_DEFINE / @c MOTOR_SUBSYS_DEFINE_DT and @ref motor_subsys_init
 * (usually via @c SYS_INIT); values come from Devicetree and static algorithm data.
 *
 * The controller layer sits between the sensor/power stage backends
 * (interfaces B and C) and the application (interface A).
 *
 * Multi-rate loop architecture
 * ----------------------------
 *
 *   Rate 0 — @ref motor_pipeline_run_stage @c MOTOR_STAGE_INNER_ISR (ISR / power-stage
 *     callback, ~20–100 kHz)
 *     Triggered by: motor_control_cb_t from power stage backend.
 *     Reads:  sensor_output_t.i_phase[], .theta_rad
 *     Writes: @ref motor_actuator_set_command (e.g. αβ, d/q, or duty)
 *     Algorithm: implementation-defined (FOC, six-step, DC current loop, V/f, …).
 *
 *   Rate 1 — outer thread, @c MOTOR_STAGE_OUTER_FAST (~1 kHz when enabled)
 *     Optional blocks; typical: speed PI, intermediate cascade.
 *
 *   Rate 2 — same thread, @c MOTOR_STAGE_OUTER_SLOW (~100–500 Hz)
 *     Optional blocks; typical: position / trajectory.
 *
 *   Rate 3 — Supervision (~10–100 Hz, low-priority thread)
 *     Fault policy, thermal derating, state machine transitions,
 *     watchdog feed, telemetry.
 *
 * Data handoff between ISR and outer thread
 * ------------------------------------------
 *   The ISR copies the latest sensor sample into a snapshot under a spinlock
 *   (short critical section), then may give a semaphore at a decimated rate
 *   (@c CONFIG_MOTOR_CTRL_OUTER_TARGET_HZ vs inner_rate_hz). Outer stages read
 *   the snapshot under the same lock.
 */

/* ------------------------------------------------------------------ */
/* Controller setpoints (written by outer steps or app)               */
/* ------------------------------------------------------------------ */

/**
 * @brief Internal controller setpoints — live state shared across rates.
 *
 * Naming is **physical / axis-neutral**: torque vs flux components, not FOC-only.
 * Each algorithm maps these to its internal representation (e.g. Id/Iq, Ia, duty).
 *
 * Inter-block bus
 * ---------------
 * This struct is the canonical wire between pipeline stages: outer blocks (or
 * the application) write the higher-level reference, intermediate blocks may
 * translate it (e.g. position_loop produces omega_mech_rad_s, speed_loop
 * produces i_torque_a), and the inner block consumes it. Producer/consumer
 * roles per field:
 *
 *   theta_mech_rad     produced by app/trajectory   consumed by position loop
 *   omega_mech_rad_s   produced by position loop    consumed by speed loop
 *   i_torque_a         produced by speed loop / app consumed by inner current
 *   i_flux_a           produced by app / FOC outer  consumed by inner current
 *   v_torque_v / v_flux_v   open-loop overrides (consumed by inner current)
 *   drive_mode         set by app or supervision    consumed by inner / power stage
 *
 * Written by outer loops or the application; read by inner-stage blocks in ISR context.
 */
struct motor_ctrl_setpoints {
	/**
	 * Torque-producing current reference (A).
	 * Examples: quadrature (Iq) current in rotating frame; DC brushed armature Ia.
	 */
	float i_torque_a;
	/**
	 * Flux / field-producing current reference (A).
	 * Examples: direct (Id) current for FOC field weakening; unused (0) for DC brushed.
	 */
	float i_flux_a;
	/** Mechanical speed setpoint (rad/s) — used when @ref motor_control_mode is speed. */
	float omega_mech_rad_s;
	/** Mechanical position setpoint (rad) — used in position mode. */
	float theta_mech_rad;
	/**
	 * Open-loop voltage: torque axis (V).
	 * Examples: Vq in rotating frame; scalar V for single-phase open-loop.
	 */
	float v_torque_v;
	/** Open-loop voltage: flux axis (V). Examples: Vd in rotating frame. */
	float v_flux_v;
	enum motor_drive_mode drive_mode;
};

/* ------------------------------------------------------------------ */
/* Pipeline IO bus (input/output of an algorithm step or block)        */
/* ------------------------------------------------------------------ */

/**
 * @brief Input bus for one algorithm/block step.
 *
 * Represents what the step consumes: latest sensor snapshot, current setpoints
 * (read-only at this point) and the top-level algorithm state pointer for any
 * cross-block context the step might need.
 */
struct motor_block_in {
	const struct motor_sensor_output *sense;
	const struct motor_ctrl_setpoints *sp;
	void *algo;
};

/**
 * @brief Output bus for one algorithm/block step.
 *
 * Represents what the step may produce: an updated setpoints object (for
 * outer / intermediate steps) and an actuator command (only the terminal,
 * inner step writes here; @p cmd is NULL for non-terminal steps).
 */
struct motor_block_out {
	struct motor_ctrl_setpoints *sp;
	struct motor_actuator_cmd *cmd;
};

struct motor_pipeline;

/* ------------------------------------------------------------------ */
/* Controller instance                                                 */
/* ------------------------------------------------------------------ */

/**
 * @brief Motor controller instance.
 *
 * One per motor.  Holds references to sensor and actuator backends,
 * the active algorithm, and the multi-rate scheduling state.
 *
 * Allocated statically — use MOTOR_CTRL_DEFINE() macro.
 */
struct motor_ctrl {
	/* Backends */
	const struct device *sensor;   /* implements motor_sensor_ops    */
	const struct device *actuator; /* implements motor_actuator_ops  */

	struct motor_pipeline *pipeline;
	/** Opaque root for @ref motor_block_in::algo (e.g. algorithm instance struct). */
	void *pipeline_ctx;

	/* State machine */
	enum motor_state state;
	enum motor_control_mode mode;
	uint32_t fault_flags; /* accumulated fault bitmask      */

	/* Setpoints — written by outer steps, read by inner-stage blocks (ISR) */
	struct motor_ctrl_setpoints setpoints;

	/* Per-stage tick counters (one per motor_pipeline_stage). Each stage's
	 * thread / ISR increments stage_tick[stage]; a block declared with
	 * period_div = N runs whenever stage_tick % N == 0. The N-block pipeline
	 * uses this same counter directly.
	 */
	uint32_t stage_tick[MOTOR_STAGE_COUNT];

	/* Supervision timer (Rate 3) — reserved; outer thread uses periodic wake */
	struct k_timer supervision_timer;

#if IS_ENABLED(CONFIG_MOTOR_CTRL_OUTER_LOOPS)
	struct k_sem outer_wake;
	struct k_spinlock outer_snap_lock;
	struct motor_sensor_output outer_snap;
	uint32_t outer_decim;
	uint32_t outer_isr_phase;
	struct k_thread outer_thread;
	K_KERNEL_STACK_MEMBER(outer_stack_mem, CONFIG_MOTOR_CTRL_OUTER_STACK_SIZE);
	bool outer_thread_started;
#endif

	/* Mutex protecting state transitions and param updates           */
	struct k_mutex lock;

	/* Watchdog: ISR must toggle this each cycle                      */
	atomic_t watchdog_cnt;

	/** Inner (ISR) rate in Hz (fixed at build time from DT via subsystem). */
	uint32_t inner_rate_hz;

	motor_state_cb_t app_state_cb;
	motor_fault_notify_cb_t app_fault_cb;
	void *app_cb_data;
};

/**
 * @brief Statically define a motor controller instance.
 *
 * @param name  C identifier for the instance.
 */
#define MOTOR_CTRL_DEFINE(name) static struct motor_ctrl name = {0}

/* ------------------------------------------------------------------ */
/* Controller lifecycle API                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief Run pre-run self-test sequence (blocking).
 *
 * Calls sensor calibration and actuator self-test.
 * Motor must be stopped and disconnected from load.
 *
 * @param ctrl   Controller instance.
 * @param flags  Output: fault flags from diagnostic.
 * @retval 0 No faults. -EIO on hardware error.
 */
int motor_ctrl_self_test(struct motor_ctrl *ctrl, uint32_t *flags);

/**
 * @brief Transition from IDLE to RUN (or ALIGN if needed).
 *
 * Enables the power stage and starts the control loop.
 *
 * @param ctrl  Controller instance.
 * @retval 0 on success, -EFAULT if in FAULT state, negative errno otherwise.
 */
int motor_ctrl_enable(struct motor_ctrl *ctrl);

/**
 * @brief Initiate controlled stop (RUN → STOP → IDLE).
 *
 * Disables the power stage and resets pipeline state (controlled deceleration
 * is algorithm-dependent).
 *
 * @param ctrl  Controller instance.
 * @retval 0 on success, negative errno on failure.
 */
int motor_ctrl_disable(struct motor_ctrl *ctrl);

/**
 * @brief Immediately disable outputs (any state → IDLE or FAULT).
 *
 * Does not decelerate — cuts power immediately.
 * Use for emergency stop only.
 *
 * @param ctrl  Controller instance.
 */
void motor_ctrl_estop(struct motor_ctrl *ctrl);

/**
 * @brief Clear fault and return to IDLE (if fault cause resolved).
 *
 * @param ctrl  Controller instance.
 * @retval 0 on success, -EBUSY if fault cause still present.
 */
int motor_ctrl_clear_fault(struct motor_ctrl *ctrl);

/**
 * @brief Read current controller state and fault flags.
 *
 * @param ctrl   Controller instance.
 * @param state  Output: current state.
 * @param faults Output: current fault bitmask.
 */
void motor_ctrl_get_status(const struct motor_ctrl *ctrl, enum motor_state *state,
			   uint32_t *faults);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_CONTROLLER_H_ */
