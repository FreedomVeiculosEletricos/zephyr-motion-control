/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_H_
#define ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_H_

#include <zephyr/subsys/motor/motor_types.h>
#include <zephyr/subsys/motor/motor_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Motor Control Application API — Interface A
 * @defgroup motor_app Motor Application API
 * @ingroup motor_control
 * @{
 *
 * Use this for generic commands and state (enable, torque setpoint, faults).
 * Tunable parameters depend on the active pipeline: each predefined algorithm
 * module exposes its own public header (e.g. @ref motor_algo_dc_current.h under
 * @c motor/algorithms/dc_current/).
 * Custom pipelines use the same block/pipeline extension points without going
 * through this facade.
 *
 * It is motor-type agnostic at this layer — the same calls apply across
 * motor families where the installed pipeline supports them.
 *
 * The application does not use this header for algorithm tuning (gains, limits,
 * observers) — those live in each predefined algorithm’s public API.
 *
 * The application does not know about:
 *   - PWM timers, ADC channels, GPIO pins
 *   - Clarke/Park transforms, SVPWM, commutation tables
 *   - Sensor backends (encoder vs sensorless vs Hall)
 *   - Phase count or topology
 *
 * Instances are registered with @c MOTOR_SUBSYS_DEFINE_DT (or @c MOTOR_SUBSYS_DEFINE);
 * sensor, actuator, pipeline, algorithm data, and rates come from Devicetree and
 * static initialisers — there is no application-level “init” beyond Zephyr
 * @c SYS_INIT running @ref motor_subsys_init when @c CONFIG_MOTOR_SUBSYS_AUTO_INIT is set.
 * Obtain a handle with @ref motor_subsys_get_by_label or @ref motor_subsys_get_by_index.
 *
 * State machine summary:
 *
 *   motor_enable()  → IDLE    → ALIGN (if needed) → RUN
 *   motor_disable() → RUN     → STOP  → IDLE
 *   motor_estop()   → ANY     → IDLE  (output cut immediately)
 *   fault event     → ANY     → FAULT
 *   motor_clear_fault() → FAULT → IDLE (if cause resolved)
 *
 * Valid commands per state:
 *   IDLE   : motor_enable(), motor_self_test()
 *   ALIGN  : none (wait for motor_state_cb MOTOR_STATE_RUN)
 *   RUN    : motor_set_torque(), motor_set_drive_mode(),
 *            motor_disable(), motor_estop()
 *   STOP   : motor_estop()
 *   FAULT  : motor_clear_fault(), motor_estop()
 */

/* ------------------------------------------------------------------ */
/* Interface A — application API                                       */
/* ------------------------------------------------------------------ */

/**
 * @brief Register application callbacks.
 *
 * @param motor      Motor handle.
 * @param state_cb   State change callback (may be NULL).
 * @param fault_cb   Fault notification callback (may be NULL).
 * @param user_data  Passed to both callbacks.
 */
void motor_register_callbacks(motor_t motor, motor_state_cb_t state_cb,
			      motor_fault_notify_cb_t fault_cb, void *user_data);

/**
 * @brief Run pre-run self-test (blocking).
 *
 * Must be called from IDLE state with motor stopped and unloaded.
 *
 * @param motor   Motor handle.
 * @param faults  Output: MOTOR_FAULT_* bitmask of detected issues.
 * @retval 0 All checks passed.
 * @retval negative errno on hardware failure.
 */
int motor_self_test(motor_t motor, uint32_t *faults);

/**
 * @brief Enable the motor (IDLE → ALIGN → RUN).
 *
 * Non-blocking.  Completion notified via state_cb(MOTOR_STATE_RUN).
 *
 * @param motor  Motor handle.
 * @retval 0 on success, -EFAULT if in FAULT state.
 */
int motor_enable(motor_t motor);

/**
 * @brief Disable the motor with controlled deceleration (RUN → STOP → IDLE).
 *
 * Non-blocking.  Completion notified via state_cb(MOTOR_STATE_IDLE).
 *
 * @param motor  Motor handle.
 * @retval 0 on success, negative errno on failure.
 */
int motor_disable(motor_t motor);

/**
 * @brief Emergency stop — cut output immediately (ANY → IDLE).
 *
 * May be called from any context including ISR.
 *
 * @param motor  Motor handle.
 */
void motor_estop(motor_t motor);

/**
 * @brief Set torque command (current control mode).
 *
 * Maps torque to an inner-loop current reference according to the active
 * pipeline (e.g. DC current uses @c kt_nm_per_a in @ref motor_algo_dc_current_data).
 * Switches to @c MOTOR_MODE_CURRENT. Valid only in RUN state.
 *
 * @param motor      Motor handle.
 * @param torque_nm  Target torque (N·m). Positive = forward.
 * @retval 0 on success, -EINVAL if mapping is unavailable or invalid.
 */
int motor_set_torque(motor_t motor, float torque_nm);

/**
 * @brief Set drive mode (normal / coast / brake / regen).
 *
 * @param motor  Motor handle.
 * @param mode   Desired drive mode.
 * @retval 0 on success, -ENOTSUP if mode not supported by power stage.
 */
int motor_set_drive_mode(motor_t motor, enum motor_drive_mode mode);

/**
 * @brief Read current motor status.
 *
 * @param motor   Motor handle.
 * @param state   Output: current state machine state.
 * @param faults  Output: current fault bitmask.
 * @param sense   Output: sensor read via the backend (may be NULL; not synchronized with ISR).
 */
void motor_get_status(motor_t motor, enum motor_state *state, uint32_t *faults,
		      struct motor_sensor_output *sense);

/**
 * @brief Clear fault and attempt return to IDLE.
 *
 * @param motor  Motor handle.
 * @retval 0 on success, -EBUSY if fault cause still present.
 */
int motor_clear_fault(motor_t motor);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_H_ */
