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
 * This is the only API an application should use.
 * It is motor-type agnostic — the same calls work for DC brushed,
 * BLDC, PMSM, stepper, and AC induction motors.
 *
 * The application does not know about:
 *   - PWM timers, ADC channels, GPIO pins
 *   - Clarke/Park transforms, SVPWM, commutation tables
 *   - Sensor backends (encoder vs sensorless vs Hall)
 *   - Phase count or topology
 *
 * State machine summary:
 *
 *   motor_init()    → UNINIT  → IDLE
 *   motor_enable()  → IDLE    → ALIGN (if needed) → RUN
 *   motor_disable() → RUN     → STOP  → IDLE
 *   motor_estop()   → ANY     → IDLE  (output cut immediately)
 *   fault event     → ANY     → FAULT
 *   motor_clear_fault() → FAULT → IDLE (if cause resolved)
 *
 * Valid commands per state:
 *   IDLE   : motor_enable(), motor_self_test(), motor_set_params()
 *   ALIGN  : none (wait for motor_state_cb MOTOR_STATE_RUN)
 *   RUN    : motor_set_torque(), motor_set_drive_mode(),
 *            motor_disable(), motor_estop()
 *   STOP   : motor_estop()
 *   FAULT  : motor_clear_fault(), motor_estop()
 */

/* ------------------------------------------------------------------ */
/* Motor handle                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief Opaque motor handle returned by motor_init().
 *
 * The application holds this pointer and passes it to all motor_*()
 * calls.  Internal structure is not part of the public API.
 */
typedef struct motor_ctrl *motor_t;

/* ------------------------------------------------------------------ */
/* Application callbacks                                               */
/* ------------------------------------------------------------------ */

/**
 * @brief State change notification callback.
 *
 * Called from the supervision thread (Rate 3, ~10–100 Hz) when
 * the controller state machine transitions.
 *
 * @param motor   Motor handle.
 * @param state   New state.
 * @param user_data  Registered user pointer.
 */
typedef void (*motor_state_cb_t)(motor_t motor, enum motor_state state, void *user_data);

/**
 * @brief Fault notification callback.
 *
 * Called from the supervision thread when a new fault is detected.
 * The motor is already in FAULT state when this fires.
 *
 * @param motor      Motor handle.
 * @param faults     MOTOR_FAULT_* bitmask.
 * @param user_data  Registered user pointer.
 */
typedef void (*motor_fault_notify_cb_t)(motor_t motor, uint32_t faults, void *user_data);

/* ------------------------------------------------------------------ */
/* Interface A — application API                                       */
/* ------------------------------------------------------------------ */

/**
 * @brief Initialise a motor instance and return a handle.
 *
 * Wires the controller to its sensor and actuator backends (looked
 * up by device name from devicetree), applies initial params.
 *
 * @param ctrl      Pre-allocated controller instance (MOTOR_CTRL_DEFINE).
 * @param sensor    Sensor backend device.
 * @param actuator  Power stage backend device.
 * @param algo      Algorithm vtable (e.g. &motor_algo_foc, &motor_algo_6step).
 * @param algo_data Algorithm private state buffer.
 * @param params    Initial parameters (required); from Devicetree via subsystem macros.
 * @return          Motor handle on success, NULL on failure.
 */
motor_t motor_init(struct motor_ctrl *ctrl, const struct device *sensor,
		   const struct device *actuator, const struct motor_algo_ops *algo,
		   void *algo_data, const struct motor_ctrl_params *params);

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
 * Switches to MOTOR_MODE_CURRENT if not already.
 * Valid only in RUN state.
 *
 * @param motor      Motor handle.
 * @param torque_nm  Target torque (N·m). Positive = forward.
 * @retval 0 on success, -ENOTSUP if algorithm does not support torque mode.
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
 * @brief Update controller parameters at runtime.
 *
 * Thread context only. Parameters are applied before the next
 * current-loop execution via double-buffer swap.
 *
 * @param motor   Motor handle.
 * @param params  New parameters (full struct — partial updates not supported).
 * @retval 0 on success, negative errno on failure.
 */
int motor_set_params(motor_t motor, const struct motor_ctrl_params *params);

/**
 * @brief Read current motor status.
 *
 * @param motor   Motor handle.
 * @param state   Output: current state machine state.
 * @param faults  Output: current fault bitmask.
 * @param sense   Output: latest sensor snapshot (may be NULL).
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
