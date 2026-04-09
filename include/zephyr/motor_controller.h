/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MOTOR_MOTOR_CONTROLLER_H_
#define ZEPHYR_INCLUDE_DRIVERS_MOTOR_MOTOR_CONTROLLER_H_

#include <zephyr/drivers/motor/motor_types.h>
#include <zephyr/drivers/motor/motor_sensor.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Motor Controller Layer
 * @defgroup motor_controller Motor Controller API
 * @ingroup motor_control
 * @{
 *
 * The controller layer sits between the sensor/power stage backends
 * (interfaces B and C) and the application (interface A).
 *
 * Multi-rate loop architecture
 * ----------------------------
 *
 *   Rate 0 — Current loop (fires from PWM period ISR, ~20–100 kHz)
 *     Triggered by: motor_control_cb_t from power stage backend.
 *     Reads:  sensor_output_t.i_phase[], .theta_rad
 *     Writes: motor_actuator_set_vector(Vα, Vβ)
 *     Algorithm: Clarke → Park → Id/Iq PI → inv-Park → SVPWM
 *                OR 6-step commutation table
 *                OR duty passthrough (open-loop)
 *
 *   Rate 1 — Speed loop (~1 kHz, RTOS thread or SW timer)
 *     Triggered by: k_timer or count-down from current loop ISR.
 *     Reads:  sensor_output_t.omega_rad_s
 *     Writes: controller_state.iq_ref (current-loop setpoint)
 *     Algorithm: Speed PI with anti-windup + accel profile
 *
 *   Rate 2 — Position loop (~100–500 Hz, RTOS thread)
 *     Triggered by: k_timer
 *     Reads:  sensor_output_t.theta_mech (mechanical angle)
 *     Writes: controller_state.omega_ref (speed-loop setpoint)
 *     Algorithm: Position P/PD + trajectory generator
 *
 *   Rate 3 — Supervision (~10–100 Hz, low-priority thread)
 *     Fault policy, thermal derating, state machine transitions,
 *     watchdog feed, telemetry.
 *
 * Data handoff between ISR and threads
 * -------------------------------------
 *   The current loop ISR writes to a double-buffer.  Speed and
 *   position loops consume the non-ISR buffer using atomic pointer
 *   swap.  No mutex in the ISR path.
 */

/* ------------------------------------------------------------------ */
/* Controller setpoints (written by speed/position loops or app)      */
/* ------------------------------------------------------------------ */

/**
 * @brief Internal controller setpoints — live state shared across rates.
 *
 * Written by the outer loops (speed, position) or directly by the
 * application in current/open-loop modes.
 * Read by the inner current loop in ISR context.
 *
 * All float fields must be written atomically where the target
 * does not guarantee float atomicity — use the provided
 * motor_ctrl_set_*() accessors which apply the appropriate barrier.
 */
struct motor_ctrl_setpoints {
	float id_ref;    /* d-axis current setpoint (A) — FOC field weakening */
	float iq_ref;    /* q-axis current setpoint (A) — torque              */
	float omega_ref; /* speed setpoint (rad/s) — speed mode               */
	float theta_ref; /* position setpoint (rad) — position mode            */
	float vd_ref;    /* d-axis voltage (V) — open-loop voltage mode        */
	float vq_ref;    /* q-axis voltage (V) — open-loop voltage mode        */
	enum motor_drive_mode drive_mode;
};

/* ------------------------------------------------------------------ */
/* Controller parameters (gains, limits, profiles)                    */
/* ------------------------------------------------------------------ */

/**
 * @brief PI controller gains.
 */
struct motor_pi_gains {
	float kp;
	float ki;
	float out_min;
	float out_max;
};

/**
 * @brief Acceleration profile parameters.
 */
struct motor_accel_params {
	enum motor_accel_profile type;
	float max_accel_rad_s2; /* maximum angular acceleration (rad/s²) */
	float max_jerk_rad_s3;  /* S-curve jerk limit (rad/s³)           */
};

/**
 * @brief Full controller parameter set.
 *
 * May be updated at runtime via motor_ctrl_set_params() from thread
 * context.  Parameters are copied atomically into the controller
 * before the next current-loop execution.
 */
struct motor_ctrl_params {
	/* Current (torque) loop */
	struct motor_pi_gains id_loop;
	struct motor_pi_gains iq_loop;

	/* Speed loop */
	struct motor_pi_gains speed_loop;
	struct motor_accel_params accel;

	/* Position loop */
	float kp_pos;
	float kd_pos;

	/* Limits */
	float i_max_a;             /* peak phase current limit (A)          */
	float speed_max_rad_s;     /* maximum mechanical speed (rad/s)      */
	float vbus_derating_start; /* Vbus below this → derate torque (V)   */

	/* Thermal derating */
	float temp_derating_start; /* °C above which torque is derated      */
	float temp_fault;          /* °C at which OVERTEMP fault is raised  */

	/* Motor parameters (used by observers) */
	uint8_t pole_pairs;
	float Rs;           /* stator resistance (Ω)                 */
	float Ld;           /* d-axis inductance (H)                 */
	float Lq;           /* q-axis inductance (H)                 */
	float flux_linkage; /* permanent magnet flux linkage (Wb)    */
};

/* ------------------------------------------------------------------ */
/* Algorithm backend vtable                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief Controller algorithm operations.
 *
 * Each algorithm (FOC, 6-step, open-loop, V/f, DTC) implements
 * this vtable.  The motor subsystem dispatches to the active
 * algorithm without knowing its internals.
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
	 * @brief Execute current-loop step (ISR context, ~20–100 kHz).
	 *
	 * @param algo_data  Per-algorithm private state.
	 * @param sense      Latest sensor output.
	 * @param sp         Current setpoints.
	 * @param valpha     Output: alpha-axis voltage (normalised).
	 * @param vbeta      Output: beta-axis voltage  (normalised).
	 */
	void (*current_loop)(void *algo_data, const struct motor_sensor_output *sense,
			     const struct motor_ctrl_setpoints *sp, float *valpha, float *vbeta);

	/**
	 * @brief Execute speed-loop step (thread context, ~1 kHz).
	 *
	 * Updates sp->iq_ref based on omega error.
	 *
	 * @param algo_data  Per-algorithm private state.
	 * @param sense      Latest sensor output.
	 * @param sp         Setpoints struct (iq_ref written here).
	 */
	void (*speed_loop)(void *algo_data, const struct motor_sensor_output *sense,
			   struct motor_ctrl_setpoints *sp);

	/**
	 * @brief Execute position-loop step (thread context, ~100–500 Hz).
	 *
	 * Updates sp->omega_ref based on position error.
	 *
	 * @param algo_data  Per-algorithm private state.
	 * @param sense      Latest sensor output.
	 * @param sp         Setpoints struct (omega_ref written here).
	 */
	void (*position_loop)(void *algo_data, const struct motor_sensor_output *sense,
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

	/* Setpoints — written by outer loops, read by current loop ISR  */
	struct motor_ctrl_setpoints setpoints;

	/* Parameters — updated via motor_ctrl_set_params()              */
	struct motor_ctrl_params params;

	/* Multi-rate scheduling                                          */
	uint32_t current_loop_cnt;  /* ISR counter for speed divider    */
	uint32_t speed_loop_div;    /* current_loop ticks per speed tick */
	uint32_t position_loop_div; /* current_loop ticks per pos tick   */

	/* Double-buffer for ISR → thread data handoff                   */
	struct motor_sensor_output sense_buf[2];
	atomic_t sense_buf_idx; /* index of the ISR-written buffer */

	/* Supervision timer (Rate 3)                                     */
	struct k_timer supervision_timer;

	/* Mutex protecting state transitions and param updates           */
	struct k_mutex lock;

	/* Watchdog: ISR must toggle this each cycle                      */
	atomic_t watchdog_cnt;
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
 * @param params   Initial parameters (NULL = use defaults).
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

#endif /* ZEPHYR_INCLUDE_DRIVERS_MOTOR_MOTOR_CONTROLLER_H_ */
