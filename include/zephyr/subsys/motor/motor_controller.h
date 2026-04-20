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
 *   - in-tree algorithm backends (motor_algo_ops vtables), and
 *   - out-of-tree pipeline/algorithm authors that compose the controller.
 *
 * Applications must use the @ref motor_app "motor_*()" facade in motor.h
 * instead. The motor_ctrl_*() entry points published here are not part of
 * the application API; their shape may evolve as the pipeline/N-block
 * architecture lands.
 *
 * The controller layer sits between the sensor/power stage backends
 * (interfaces B and C) and the application (interface A).
 *
 * Multi-rate loop architecture
 * ----------------------------
 *
 *   Rate 0 — @ref motor_algo_ops.inner_step (ISR / power-stage callback, ~20–100 kHz)
 *     Triggered by: motor_control_cb_t from power stage backend.
 *     Reads:  sensor_output_t.i_phase[], .theta_rad
 *     Writes: @ref motor_actuator_set_command (e.g. αβ, d/q, or duty)
 *     Algorithm: implementation-defined (FOC, six-step, DC current loop, V/f, …).
 *
 *   Rate 1 — @ref motor_algo_ops.outer_step_0 (~1 kHz, RTOS thread when enabled)
 *     Optional; NULL for single-rate algorithms.
 *     Typical: speed PI, intermediate cascade.
 *
 *   Rate 2 — @ref motor_algo_ops.outer_step_1 (~100–500 Hz, same thread as outer_step_0)
 *     Optional; NULL if unused.
 *     Typical: position / trajectory.
 *
 *   Rate 3 — Supervision (~10–100 Hz, low-priority thread)
 *     Fault policy, thermal derating, state machine transitions,
 *     watchdog feed, telemetry.
 *
 * Data handoff between ISR and threads
 * -------------------------------------
 *   The inner-step ISR writes to a double-buffer.  Outer steps
 *   consume the non-ISR buffer using atomic pointer swap.  No mutex
 *   in the ISR path.
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
 * Written by outer loops or the application; read by @ref motor_algo_ops.inner_step
 * in ISR context.
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
/* Controller parameters — shell only (algorithm-agnostic)             */
/* ------------------------------------------------------------------ */

/**
 * @brief PI controller gains (used by outer loops when present).
 */
struct motor_pi_gains {
	float kp;
	float ki;
	float out_min;
	float out_max;
};

/**
 * @brief Acceleration profile parameters (outer trajectory).
 */
struct motor_accel_params {
	enum motor_accel_profile type;
	float max_accel_rad_s2;
	float max_jerk_rad_s3;
};

/**
 * @brief Safety and operating limits (independent of control law).
 */
struct motor_ctrl_limits {
	float i_max_a;
	float speed_max_rad_s;
	float vbus_derating_start;
	float temp_derating_start;
	float temp_fault;
};

/**
 * @brief Timing for the controller shell (ISR cadence).
 */
struct motor_ctrl_timing {
	/** Inner (fast) loop sample period (s), e.g. 1/20000 for 20 kHz. */
	float control_loop_dt_s;
};

/**
 * @brief Parameters shared by @ref motor_ctrl for all algorithms.
 *
 * Holds **limits, timing, and application-facing motor constants** only.
 * Current-loop / FOC / observer gains belong in the algorithm’s private state,
 * not here — otherwise every motor type would carry unused fields.
 *
 * Updated at runtime via motor_ctrl_set_params() (thread context).
 */
struct motor_ctrl_params {
	struct motor_ctrl_limits limits;
	struct motor_ctrl_timing timing;

	/**
	 * Torque constant (N·m/A) for @ref motor_set_torque() → current reference.
	 * Set to 0 if torque-from-current is not used.
	 */
	float kt_nm_per_a;

	/** Pole pairs (mechanical ↔ electrical speed); use 1 for DC brushed. */
	uint8_t pole_pairs;

	/* --- Optional outer-loop tuning (when outer steps are enabled) --- */
	struct motor_pi_gains speed_loop;
	struct motor_accel_params accel;
	float kp_pos;
	float kd_pos;
};

/* ------------------------------------------------------------------ */
/* Algorithm backend vtable                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief Controller algorithm operations.
 *
 * Each algorithm (e.g. FOC, six-step, DC torque, V/f) implements
 * this vtable. Names are **scheduling-centric**, not physics-centric:
 * single-rate algorithms may set only @ref inner_step; multi-rate algorithms
 * may also use @ref outer_step_0 and @ref outer_step_1.
 *
 * Dispatch contract (when @ref CONFIG_MOTOR_CTRL_OUTER_LOOPS is enabled):
 *   - @ref inner_step — ISR (power-stage callback), highest rate.
 *   - @ref outer_step_0 — outer thread, faster cadence (e.g. ~1 kHz).
 *   - @ref outer_step_1 — same thread, lower cadence (e.g. ~100 Hz); may be NULL.
 */
struct motor_algo_ops {
	/**
	 * @brief Initialise algorithm state.
	 * Called once when entering RUN state.
	 *
	 * @param algo_data  Per-algorithm private state pointer.
	 * @param params     Initial parameter set.
	 * @retval 0 on success, negative errno on failure.
	 */
	int (*init)(void *algo_data, const struct motor_ctrl_params *params);

	/**
	 * @brief Inner / fast path step (ISR context, e.g. PWM period ~20–100 kHz).
	 *
	 * Computes @ref motor_actuator_cmd from setpoints and sensor. Required.
	 * Must set @p cmd->kind and the matching union arm.
	 *
	 * @param algo_data  Per-algorithm private state.
	 * @param sense      Latest sensor output.
	 * @param sp         Current setpoints.
	 * @param cmd        Output: command to the power stage (written by algo).
	 */
	void (*inner_step)(void *algo_data, const struct motor_sensor_output *sense,
			   const struct motor_ctrl_setpoints *sp, struct motor_actuator_cmd *cmd);

	/**
	 * @brief First outer step (thread context, higher priority outer work).
	 *
	 * Optional (NULL if single-rate). Typical use: speed or intermediate
	 * cascade; may update @p sp (e.g. inner setpoints).
	 */
	void (*outer_step_0)(void *algo_data, const struct motor_sensor_output *sense,
			     struct motor_ctrl_setpoints *sp);

	/**
	 * @brief Second outer step (thread context, lower cadence than outer_step_0).
	 *
	 * Optional (NULL). Typical use: position / trajectory; may update @p sp.
	 */
	void (*outer_step_1)(void *algo_data, const struct motor_sensor_output *sense,
			     struct motor_ctrl_setpoints *sp);

	/**
	 * @brief Update algorithm parameters at runtime.
	 *
	 * Called from thread context when new params are applied.
	 *
	 * @param algo_data  Per-algorithm private state.
	 * @param params     New parameter set.
	 */
	void (*set_params)(void *algo_data, const struct motor_ctrl_params *params);

	/**
	 * @brief Reset algorithm integrators (called on STOP → IDLE).
	 *
	 * @param algo_data  Per-algorithm private state.
	 */
	void (*reset)(void *algo_data);
};

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

	/* Active algorithm */
	const struct motor_algo_ops *algo;
	void *algo_data; /* algorithm private state        */

	/* State machine */
	enum motor_state state;
	enum motor_control_mode mode;
	uint32_t fault_flags; /* accumulated fault bitmask      */

	/* Setpoints — written by outer steps, read by inner_step ISR */
	struct motor_ctrl_setpoints setpoints;

	/* Parameters — updated via motor_ctrl_set_params()              */
	struct motor_ctrl_params params;

	/* Multi-rate scheduling (reserved for future divider-driven outers) */
	uint32_t inner_step_cnt; /* ISR counter for outer cadence        */
	uint32_t outer0_div;     /* inner steps per outer_step_0 tick  */
	uint32_t outer1_div;     /* inner steps per outer_step_1 tick    */

	/* Double-buffer for ISR → thread data handoff                   */
	struct motor_sensor_output sense_buf[2];
	atomic_t sense_buf_idx; /* index of the ISR-written buffer */

	/* Supervision timer (Rate 3) — reserved; outer thread uses periodic wake */
	struct k_timer supervision_timer;

#if IS_ENABLED(CONFIG_MOTOR_CTRL_OUTER_LOOPS)
	/* Outer steps: dedicated thread (outer_step_0 / outer_step_1) */
	struct k_thread outer_thread;
	K_KERNEL_STACK_MEMBER(outer_stack_mem, CONFIG_MOTOR_CTRL_OUTER_STACK_SIZE);
	bool outer_thread_started;
#endif

	/* Mutex protecting state transitions and param updates           */
	struct k_mutex lock;

	/* Watchdog: ISR must toggle this each cycle                      */
	atomic_t watchdog_cnt;

	/** Inner (ISR) rate in Hz; derived from @ref motor_ctrl_timing.control_loop_dt_s. */
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
 * @brief Initialise a motor controller instance.
 *
 * Wires sensor + actuator backends, sets initial params, registers
 * the control loop callback with the power stage backend.
 *
 * @param ctrl     Controller instance.
 * @param sensor   Sensor backend device.
 * @param actuator Power stage backend device.
 * @param algo     Algorithm vtable.
 * @param algo_data Algorithm private state buffer.
 * @param params   Initial parameters (required). For DT-driven motors use
 *                 @ref MOTOR_SUBSYS_DEFINE_DT or @ref MOTOR_CTRL_PARAMS_INITIALIZER.
 * @retval 0 on success, negative errno on failure.
 */
int motor_ctrl_init(struct motor_ctrl *ctrl, const struct device *sensor,
		    const struct device *actuator, const struct motor_algo_ops *algo,
		    void *algo_data, const struct motor_ctrl_params *params);

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
 * Decelerates according to the configured accel profile, then disables.
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
 * @brief Update controller parameters at runtime (thread context only).
 *
 * Parameters are double-buffered and applied before the next
 * current-loop execution.
 *
 * @param ctrl    Controller instance.
 * @param params  New parameters.
 * @retval 0 on success, negative errno on failure.
 */
int motor_ctrl_set_params(struct motor_ctrl *ctrl, const struct motor_ctrl_params *params);

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
