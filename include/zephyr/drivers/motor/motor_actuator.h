/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MOTOR_MOTOR_ACTUATOR_H_
#define ZEPHYR_INCLUDE_DRIVERS_MOTOR_MOTOR_ACTUATOR_H_

#include <zephyr/subsys/motor/motor_types.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Motor Power Stage Layer — Interface C
 * @defgroup motor_actuator Motor Power Stage API
 * @ingroup motor_control
 * @{
 *
 * The power stage layer abstracts the physical inverter or bridge.
 * The controller layer writes voltage vectors or normalised duty
 * cycles here and never touches PWM/GPIO peripherals directly.
 *
 * Backend implementations:
 *   - 1-phase H-bridge    (DC brushed)
 *   - Dual H-bridge       (stepper, 2-phase)
 *   - 3-phase inverter    (BLDC, PMSM, AC induction)
 *   - 6-phase dual 3-phase (advanced, future)
 *
 * Half-bridge power stages (STM32 reference): up to three complementary legs may
 * share one timer (one counter / period) so all legs stay time-aligned. The
 * static config field n_phases counts those legs for the active build (1 = one
 * half-bridge / unidirectional magnitude, 2 = full-bridge DC / bidirectional,
 * 3 = reserved for future multi-leg topologies on the same API).
 *
 * The controller does not need to know the phase count — it
 * writes a voltage vector (Vα, Vβ) and the backend maps this
 * to the appropriate number of phases via SVPWM or equivalent.
 * For single-phase backends, Vβ is ignored.
 */

/* ------------------------------------------------------------------ */
/* Power stage configuration (set at init, not changed at runtime)    */
/* ------------------------------------------------------------------ */

/**
 * @brief Identifiers for the physical power stage topology.
 */
enum motor_stage_topology {
	MOTOR_STAGE_HALF_BRIDGE = 0,  /* single half-bridge, 1 phase out    */
	MOTOR_STAGE_FULL_BRIDGE = 1,  /* H-bridge, 1-phase bidirectional    */
	MOTOR_STAGE_DUAL_HBRIDGE = 2, /* 2× H-bridge (stepper / 2-phase)   */
	MOTOR_STAGE_3PH_INVERTER = 3, /* 3-phase, 6-switch inverter         */
	MOTOR_STAGE_6PH_INVERTER = 4, /* dual 3-phase (future)              */
};

/**
 * @brief Static configuration for the power stage backend.
 *
 * Populated from devicetree at compile time.
 * The backend stores a pointer to this struct; it is ROM-resident.
 */
struct motor_stage_config {
	enum motor_stage_topology topology;
	uint8_t n_phases;       /* Half-bridge legs driven: 1, 2, or 3   */
	uint32_t pwm_period_ns; /* PWM period (sets switching frequency) */
	uint32_t deadtime_rising_ns;
	uint32_t deadtime_falling_ns;
	float v_bus_nominal;   /* nominal DC-link voltage (V)          */
	float v_bus_ov_thresh; /* overvoltage fault threshold (V)      */
	float v_bus_uv_thresh; /* undervoltage fault threshold (V)     */
	float i_peak_limit;    /* peak phase current limit (A)         */
};

/** Max phases for @ref motor_actuator_cmd DUTY_DIRECT payload. */
#define MOTOR_ACTUATOR_CMD_DUTY_MAX 6

/**
 * @brief Discriminant for @ref motor_actuator_cmd — how to interpret the union.
 *
 * Backends map each kind to PWM/SVPWM or pass-through duty. αβ and d/q are
 * typically normalised (e.g. ±1); exact scaling is backend-defined.
 */
enum motor_actuator_cmd_kind {
	/** Stationary frame (Clarke): two components, e.g. SVPWM or 1-phase magnitude. */
	MOTOR_ACTUATOR_CMD_ALPHA_BETA = 0,
	/** Rotating frame (Park): d/q voltage or duty command — requires backend support. */
	MOTOR_ACTUATOR_CMD_VD_VQ,
	/** Raw per-phase duty [0.0, 1.0], length @c u.duty.n (6-step, open-loop, etc.). */
	MOTOR_ACTUATOR_CMD_DUTY_DIRECT,
};

/**
 * @brief Unified actuator command (enum + union, no indirection).
 *
 * Produced by @ref motor_algo_ops.inner_step and consumed by @ref motor_actuator_set_command.
 */
struct motor_actuator_cmd {
	enum motor_actuator_cmd_kind kind;
	union {
		struct {
			float valpha;
			float vbeta;
		} ab;
		struct {
			float vd;
			float vq;
		} dq;
		struct {
			uint8_t n;
			float duty[MOTOR_ACTUATOR_CMD_DUTY_MAX];
		} duty;
	} u;
};

/* ------------------------------------------------------------------ */
/* Control loop callback (the heartbeat)                               */
/* ------------------------------------------------------------------ */

/**
 * @brief Control loop callback type.
 *
 * Called by the power stage backend at the start of each PWM period
 * (center-aligned: at the PWM counter underflow/overflow event).
 * Runs in ISR context at the current-loop rate (~20–100 kHz).
 *
 * The callback must:
 *   1. Call motor_sensor_update() to latch the ADC sample.
 *   2. Call motor_sensor_get() to read sensor_output_t.
 *   3. Run the current-loop algorithm.
 *   4. Fill @ref motor_actuator_cmd and call motor_actuator_set_command().
 *
 * @param dev        Power stage device handle.
 * @param user_data  Pointer registered with set_control_callback.
 */
typedef void (*motor_control_cb_t)(const struct device *dev, void *user_data);

/**
 * @brief Fault event callback type.
 *
 * Called from the hardware fault ISR (nFAULT GPIO, comparator trip).
 * The power stage has already disabled the output before this fires.
 * Runs in ISR context — keep it short.
 *
 * @param dev         Power stage device handle.
 * @param fault_flags MOTOR_FAULT_* bitmask describing the event.
 * @param user_data   Pointer registered with set_fault_callback.
 */
typedef void (*motor_fault_cb_t)(const struct device *dev, uint32_t fault_flags, void *user_data);

/* ------------------------------------------------------------------ */
/* Power stage operations vtable — Interface C implementation         */
/* ------------------------------------------------------------------ */

/**
 * @brief Power stage backend operations vtable.
 *
 * A backend (H-bridge driver, 3-phase inverter driver) registers
 * this struct as its Zephyr driver API.
 */
struct motor_actuator_ops {
	/**
	 * @brief Initialise the power stage hardware.
	 *
	 * Configures PWM timers, dead-time, gate driver, bootstrap
	 * charge sequence, and hardware fault inputs.
	 * Called once in thread context.
	 *
	 * @param dev  Power stage device.
	 * @retval 0 on success, negative errno on failure.
	 */
	int (*init)(const struct device *dev);

	/**
	 * @brief Enable the power stage outputs.
	 *
	 * Starts PWM generation.  Bootstrap charge must already be
	 * complete (handled inside init or a separate charge_bootstrap).
	 *
	 * @param dev  Power stage device.
	 * @retval 0 on success, negative errno on failure.
	 */
	int (*enable)(const struct device *dev);

	/**
	 * @brief Disable the power stage outputs gracefully.
	 *
	 * Stops PWM generation after the current period completes.
	 * Does not affect the hardware fault path.
	 *
	 * @param dev  Power stage device.
	 * @retval 0 on success, negative errno on failure.
	 */
	int (*disable)(const struct device *dev);

	/**
	 * @brief Apply a command to the power stage (ISR-safe).
	 *
	 * Dispatches on @ref motor_actuator_cmd.kind: αβ / d/q / raw duty per backend.
	 * Unsupported kinds return -ENOTSUP.
	 *
	 * @param dev  Power stage device.
	 * @param cmd  Command payload (must not be NULL).
	 * @retval 0 on success, negative errno on failure.
	 */
	int (*set_command)(const struct device *dev, const struct motor_actuator_cmd *cmd);

	/**
	 * @brief Apply per-phase duty cycles directly (ISR-safe).
	 *
	 * Bypasses SVPWM — used by 6-step commutation, stepper microstep
	 * tables, V/f scalar control.  Array length must match n_phases
	 * in the stage config.  Values normalised [0.0, 1.0].
	 *
	 * Updates take effect atomically at the next PWM reload event.
	 *
	 * @param dev    Power stage device.
	 * @param duty   Array of duty cycles, length = n_phases.
	 * @param n      Number of elements in @p duty.
	 * @retval 0 on success, negative errno on failure.
	 */
	int (*set_duty)(const struct device *dev, const float *duty, uint8_t n);

	/**
	 * @brief Set the drive mode (normal / coast / brake / regen).
	 *
	 * Transitions are applied at the next PWM reload.
	 *
	 * @param dev   Power stage device.
	 * @param mode  Desired drive mode.
	 * @retval 0 on success, -ENOTSUP if mode not supported by backend.
	 */
	int (*set_drive_mode)(const struct device *dev, enum motor_drive_mode mode);

	/**
	 * @brief Register the control loop heartbeat callback.
	 *
	 * The backend fires @p cb from the PWM period ISR.
	 * Only one callback may be registered per device instance.
	 *
	 * @param dev        Power stage device.
	 * @param cb         Callback function (ISR context).
	 * @param user_data  Passed to cb on each invocation.
	 * @retval 0 on success, negative errno on failure.
	 */
	int (*set_control_callback)(const struct device *dev, motor_control_cb_t cb,
				    void *user_data);

	/**
	 * @brief Register the hardware fault callback.
	 *
	 * @param dev        Power stage device.
	 * @param cb         Callback function (ISR context).
	 * @param user_data  Passed to cb on fault event.
	 * @retval 0 on success, negative errno on failure.
	 */
	int (*set_fault_callback)(const struct device *dev, motor_fault_cb_t cb, void *user_data);

	/**
	 * @brief Clear a latched hardware fault and re-arm protection.
	 *
	 * @param dev  Power stage device.
	 * @retval 0 on success, negative errno on failure.
	 */
	int (*clear_fault)(const struct device *dev);

	/**
	 * @brief Read current fault status.
	 *
	 * @param dev    Power stage device.
	 * @param flags  Output: MOTOR_FAULT_* bitmask.
	 * @retval 0 on success, negative errno on failure.
	 */
	int (*get_fault)(const struct device *dev, uint32_t *flags);

	/**
	 * @brief Run pre-run hardware diagnostics (blocking, motor stopped).
	 *
	 * Performs transistor short-circuit check and motor presence check.
	 * Sequence:
	 *   1. Bus voltage plausibility (multiple-short detect).
	 *   2. High-side transistor check via voltage measurement.
	 *   3. Low-side transistor check via GPIO.
	 *   4. Motor presence via alternating low-side GPIO sequence.
	 *
	 * @param dev    Power stage device.
	 * @param flags  Output: MOTOR_FAULT_* bitmask of detected faults.
	 * @retval 0 No faults detected.
	 * @retval -ENODEV Motor not connected.
	 * @retval -EIO    ADC/GPIO read failure.
	 * @retval negative errno on other failure.
	 */
	int (*self_test)(const struct device *dev, uint32_t *flags);

	/**
	 * @brief Return static power stage configuration.
	 *
	 * @param dev  Power stage device.
	 * @return Pointer to ROM-resident config struct.
	 */
	const struct motor_stage_config *(*get_config)(const struct device *dev);

	/**
	 * @brief Arm the Safe Torque Off (STO) path.
	 *
	 * Enables monitoring of the dual-channel STO inputs. When both
	 * channels assert, the power stage immediately inhibits all PWM
	 * outputs without CPU intervention, and the controller transitions
	 * to MOTOR_STATE_STO.
	 *
	 * Requires sto-gpios[2] to be defined in the DT node.
	 *
	 * @param dev  Power stage device.
	 * @retval 0 on success.
	 * @retval -ENOTSUP if no STO hardware is present (sto-gpios absent).
	 */
	int (*sto_arm)(const struct device *dev);

	/**
	 * @brief Release the STO condition and re-enable PWM outputs.
	 *
	 * Both STO channels must be de-asserted before this call succeeds.
	 * A self-test of the STO path (ASIL-B requirement) is performed
	 * internally before release. On success the controller returns to
	 * MOTOR_STATE_IDLE.
	 *
	 * @param dev    Power stage device.
	 * @param flags  Output: MOTOR_FAULT_STO_MISMATCH if dual-channel
	 *               discrepancy was detected during self-test.
	 * @retval 0 on success.
	 * @retval -EBUSY if STO inputs are still asserted.
	 * @retval -EIO   if STO self-test failed (path fault detected).
	 */
	int (*sto_release)(const struct device *dev, uint32_t *flags);

	/**
	 * @brief Invoke the registered control callback from an external sync ISR
	 * (e.g. ADC end-of-conversion). NULL if the heartbeat is only timer-driven.
	 */
	void (*invoke_control_callback)(const struct device *dev);
};

/* ------------------------------------------------------------------ */
/* Public actuator API (called by the motor subsystem, not the app)   */
/* ------------------------------------------------------------------ */

static inline int motor_actuator_enable(const struct device *dev)
{
	const struct motor_actuator_ops *ops = dev->api;

	return ops->enable(dev);
}

static inline int motor_actuator_disable(const struct device *dev)
{
	const struct motor_actuator_ops *ops = dev->api;

	return ops->disable(dev);
}

static inline int motor_actuator_set_command(const struct device *dev,
					     const struct motor_actuator_cmd *cmd)
{
	const struct motor_actuator_ops *ops = dev->api;

	return ops->set_command(dev, cmd);
}

/**
 * @brief Convenience: αβ command (normalised, same as legacy set_vector).
 */
static inline int motor_actuator_set_vector(const struct device *dev, float valpha, float vbeta)
{
	const struct motor_actuator_cmd cmd = {
		.kind = MOTOR_ACTUATOR_CMD_ALPHA_BETA,
		.u.ab.valpha = valpha,
		.u.ab.vbeta = vbeta,
	};

	return motor_actuator_set_command(dev, &cmd);
}

static inline int motor_actuator_set_duty(const struct device *dev, const float *duty, uint8_t n)
{
	const struct motor_actuator_ops *ops = dev->api;

	return ops->set_duty(dev, duty, n);
}

static inline int motor_actuator_set_drive_mode(const struct device *dev,
						enum motor_drive_mode mode)
{
	const struct motor_actuator_ops *ops = dev->api;

	return ops->set_drive_mode(dev, mode);
}

static inline int motor_actuator_set_control_callback(const struct device *dev,
						      motor_control_cb_t cb, void *user_data)
{
	const struct motor_actuator_ops *ops = dev->api;

	return ops->set_control_callback(dev, cb, user_data);
}

static inline int motor_actuator_set_fault_callback(const struct device *dev, motor_fault_cb_t cb,
						    void *user_data)
{
	const struct motor_actuator_ops *ops = dev->api;

	return ops->set_fault_callback(dev, cb, user_data);
}

static inline int motor_actuator_clear_fault(const struct device *dev)
{
	const struct motor_actuator_ops *ops = dev->api;

	return ops->clear_fault(dev);
}

static inline int motor_actuator_get_fault(const struct device *dev, uint32_t *flags)
{
	const struct motor_actuator_ops *ops = dev->api;

	return ops->get_fault(dev, flags);
}

static inline int motor_actuator_self_test(const struct device *dev, uint32_t *flags)
{
	const struct motor_actuator_ops *ops = dev->api;

	return ops->self_test(dev, flags);
}

static inline int motor_actuator_sto_arm(const struct device *dev)
{
	const struct motor_actuator_ops *ops = dev->api;

	return ops->sto_arm(dev);
}

static inline int motor_actuator_sto_release(const struct device *dev, uint32_t *flags)
{
	const struct motor_actuator_ops *ops = dev->api;

	return ops->sto_release(dev, flags);
}

static inline void motor_actuator_invoke_control_callback(const struct device *dev)
{
	const struct motor_actuator_ops *ops = dev->api;

	if ((ops != NULL) && (ops->invoke_control_callback != NULL)) {
		ops->invoke_control_callback(dev);
	}
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MOTOR_MOTOR_ACTUATOR_H_ */
