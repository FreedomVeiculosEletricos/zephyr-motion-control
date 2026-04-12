/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MOTOR_MOTOR_SENSOR_H_
#define ZEPHYR_INCLUDE_DRIVERS_MOTOR_MOTOR_SENSOR_H_

#include <zephyr/subsys/motor/motor_types.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Motor Sensor / Feedback Layer — Interface B
 * @defgroup motor_sensor Motor Sensor API
 * @ingroup motor_control
 * @{
 *
 * This is the contract between the sensor/feedback layer and the
 * controller layer.  Every backend (quadrature encoder, Hall×3,
 * resolver, SPI absolute encoder, BEMF observer, HFI) must produce
 * exactly this struct.  The controller never accesses hardware
 * peripherals directly.
 *
 * Two independent backend trees converge here:
 *   1. Position/speed  → hot.theta_rad, hot.omega_rad_s
 *   2. Current sense   → hot.i_phase[3]
 *   3. Bus monitoring  → cold.v_bus, cold.temp_motor, cold.temp_board
 *   4. Fault/validity  → cold.fault_flags, cold.valid_*, cold.stale
 *
 * The struct is split into HOT and COLD regions:
 *   - HOT  (~24 bytes, one 32-byte cache line): read every ISR cycle.
 *   - COLD (~20 bytes): read by supervision thread only, not ISR-copied.
 */

/* ------------------------------------------------------------------ */
/* Interface B output contract                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief Hot region — copied by the current-loop ISR every PWM period.
 *
 * Kept to one 32-byte D-cache line. Do not add fields without profiling
 * the impact on ISR copy latency.
 */
struct motor_sensor_hot {
	float theta_rad;   /* electrical angle [0, 2π)            */
	float omega_rad_s; /* electrical angular velocity (rad/s)  */
	float i_phase[3];  /* [0]=Ia, [1]=Ib, [2]=Ic  (A)
			    * Ic may be reconstructed: Ic = −Ia−Ib  */
	bool stale;        /* true if update() not called this cycle */
	bool valid_theta;
	bool valid_omega;
	bool valid_current;
	uint8_t _pad[1]; /* explicit padding to 4-byte boundary  */
};

/**
 * @brief Cold region — read by the supervision thread (~10–100 Hz).
 *
 * Not copied in the ISR hot path. Backend updates these fields on
 * each sensor_ops.update() call; supervision thread reads from the
 * non-ISR double-buffer copy.
 */
struct motor_sensor_cold {
	float v_bus;          /* DC-link voltage (V)                  */
	float temp_motor;     /* motor winding temperature (°C)       */
	float temp_board;     /* power stage PCB temperature (°C)     */
	uint32_t fault_flags; /* MOTOR_FAULT_* bitmask                */
	bool valid_vbus;
	bool valid_temp_motor;
	bool valid_temp_board;
	uint8_t _pad[1];
};

/**
 * @brief Unified sensor output — the only struct the controller reads.
 *
 * The current-loop ISR accesses only the @p hot sub-struct.
 * The supervision thread reads both sub-structs via the double-buffer.
 * All fields use SI units as defined in motor_types.h.
 */
struct motor_sensor_output {
	struct motor_sensor_hot hot;   /**< ISR-copied region (~24 bytes) */
	struct motor_sensor_cold cold; /**< Supervision-only region       */
};

/* ------------------------------------------------------------------ */
/* Position/speed backend types                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief Position and speed feedback source identifiers.
 *
 * Used by the controller to know what accuracy / latency to expect,
 * and to decide whether alignment is required at startup.
 */
enum motor_pos_backend_type {
	MOTOR_POS_QUADRATURE_ENCODER = 0, /* incremental, needs index or align */
	MOTOR_POS_HALL_SENSORS = 1,       /* 60° resolution, no align needed   */
	MOTOR_POS_ABS_ENCODER_SPI = 2,    /* absolute, no align (AS5047, RLS)  */
	MOTOR_POS_RESOLVER = 3,           /* SIN/COS ADC + atan2               */
	MOTOR_POS_BEMF_OBSERVER = 4,      /* sensorless, valid only above ~5%  */
	MOTOR_POS_HFI_OBSERVER = 5,       /* sensorless at standstill          */
	MOTOR_POS_STEP_COUNTER = 6,       /* open-loop step count (stepper)    */
	MOTOR_POS_NONE = 7,               /* no position feedback              */
};

/* ------------------------------------------------------------------ */
/* Current-sense backend types                                         */
/* ------------------------------------------------------------------ */

enum motor_cur_backend_type {
	MOTOR_CUR_3SHUNT = 0,        /* Ia, Ib, Ic direct — 3× ADC + DMA    */
	MOTOR_CUR_2SHUNT = 1,        /* Ia, Ib direct; Ic = −Ia−Ib          */
	MOTOR_CUR_1SHUNT_DCLINK = 2, /* DC-link shunt; reconstruct phases    */
	MOTOR_CUR_STALLGUARD = 3,    /* stepper back-EMF load estimate only  */
	MOTOR_CUR_NONE = 4,          /* no current sensing                   */
};

/* ------------------------------------------------------------------ */
/* Sensor backend lifecycle and vtable (sensor_ops_t)                  */
/* ------------------------------------------------------------------ */

/**
 * @brief Sensor backend calibration parameters (passed at init).
 *
 * Each backend inspects only the fields relevant to it.
 */
struct motor_sensor_cal {
	float current_offset[3]; /* ADC zero-current offset (A)          */
	float current_gain[3];   /* ADC gain correction (dimensionless)  */
	float encoder_cpr;       /* counts per revolution (quad encoder) */
	uint8_t pole_pairs;      /* electrical/mechanical angle ratio    */
	float resolver_offset;   /* resolver electrical offset (rad)     */
};

/**
 * @brief Sensor backend operations vtable — Interface B implementation.
 *
 * A backend registers itself by populating this struct.
 * The motor subsystem calls these functions; hardware drivers
 * (ADC, SPI, TIM) are accessed only inside the backend.
 */
struct motor_sensor_ops {
	/**
	 * @brief Initialise hardware and apply calibration.
	 * Called once from motor_sensor_init() in thread context.
	 *
	 * @param dev   Sensor backend device handle.
	 * @param cal   Calibration parameters (may be NULL for defaults).
	 * @retval 0 on success, negative errno on failure.
	 */
	int (*init)(const struct device *dev, const struct motor_sensor_cal *cal);

	/**
	 * @brief Acquire a new sample from hardware.
	 *
	 * Must be callable from ISR context (current loop, ~20–100 kHz).
	 * Implementations must be non-blocking.
	 *
	 * @param dev  Sensor backend device handle.
	 * @retval 0 on success, negative errno on failure.
	 */
	int (*update)(const struct device *dev);

	/**
	 * @brief Return the latest sensor output.
	 *
	 * Copies the most recent sample into @p out.
	 * Must be callable from ISR context.
	 *
	 * @param dev  Sensor backend device handle.
	 * @param out  Destination struct — filled by this call.
	 * @retval 0 on success, negative errno on failure.
	 */
	int (*get)(const struct device *dev, struct motor_sensor_output *out);

	/**
	 * @brief Run offset/gain calibration sequence.
	 *
	 * Called pre-run with motor stopped.  Updates internal cal state.
	 * Blocking — may take up to several PWM cycles.
	 *
	 * @param dev  Sensor backend device handle.
	 * @retval 0 on success, negative errno on failure.
	 */
	int (*calibrate)(const struct device *dev);

	/**
	 * @brief Report backend type identifiers.
	 *
	 * Allows the controller to query what backends are active and
	 * choose appropriate algorithms (e.g. skip ALIGN for abs encoder).
	 *
	 * @param dev       Sensor backend device handle.
	 * @param pos_type  Output: position backend type.
	 * @param cur_type  Output: current backend type.
	 */
	void (*get_backend_types)(const struct device *dev, enum motor_pos_backend_type *pos_type,
				  enum motor_cur_backend_type *cur_type);
};

/* ------------------------------------------------------------------ */
/* Public sensor API (called by the motor subsystem, not the app)     */
/* ------------------------------------------------------------------ */

/**
 * @brief Initialise a sensor backend.
 *
 * Wraps sensor_ops->init().  Called during motor controller init.
 *
 * @param dev  Sensor backend device.
 * @param cal  Calibration parameters, or NULL.
 * @retval 0 on success, negative errno on failure.
 */
static inline int motor_sensor_init(const struct device *dev, const struct motor_sensor_cal *cal)
{
	const struct motor_sensor_ops *ops = dev->api;

	return ops->init(dev, cal);
}

/**
 * @brief Trigger a sensor sample (ISR-safe).
 *
 * @param dev  Sensor backend device.
 * @retval 0 on success, negative errno on failure.
 */
static inline int motor_sensor_update(const struct device *dev)
{
	const struct motor_sensor_ops *ops = dev->api;

	return ops->update(dev);
}

/**
 * @brief Read the latest sensor output (ISR-safe).
 *
 * @param dev  Sensor backend device.
 * @param out  Output struct to fill.
 * @retval 0 on success, negative errno on failure.
 */
static inline int motor_sensor_get(const struct device *dev, struct motor_sensor_output *out)
{
	const struct motor_sensor_ops *ops = dev->api;

	return ops->get(dev, out);
}

/**
 * @brief Run sensor calibration sequence (blocking, pre-run only).
 *
 * @param dev  Sensor backend device.
 * @retval 0 on success, negative errno on failure.
 */
static inline int motor_sensor_calibrate(const struct device *dev)
{
	const struct motor_sensor_ops *ops = dev->api;

	return ops->calibrate(dev);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MOTOR_MOTOR_SENSOR_H_ */
