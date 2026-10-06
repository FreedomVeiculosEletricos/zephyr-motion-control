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

/**
 * @brief Motor controller extension API (subsystem + algorithm authors).
 * @defgroup motor_controller Motor Controller Extension API
 * @ingroup motor_control
 * @{
 *
 * Application code uses @ref motor.h.  The inner (current) loop runs in the
 * power-stage PWM ISR using an N-1 sample cadence: @c inner_rate_hz matches
 * the actuator PWM frequency from @ref motor_stage_config.pwm_period_ns.
 * Outer blocks run on the slow thread at PWM / @c slow-sample-div.
 */

struct motor_block;

struct motor_sense_bundle {
	float current_amps[MOTOR_SENSOR_CURRENT_MAX];
	size_t current_count;
	/* Signed bridge current from @ref MOTOR_SENSOR_CHAN_CURRENT_BRIDGE: the
	 * reading of the clamped leg, with the sign the applied direction
	 * implies. Only the sensor knows which entry of current_amps that is,
	 * since it moves with the command.
	 */
	float bridge_current_a;
	bool bridge_current_valid;
	float angle_rad;
	bool angle_valid;
};

/**
 * @brief Measured current through the bridge, in amps.
 *
 * Prefers the signed reading the sensor resolved from the leg the command
 * clamped, since that is the one whose low side carries the load current for the
 * whole PWM period and the one that knows which way it flows. Falls back to the
 * first composite entry for sensors that cannot pair a reading with a leg, and to
 * zero when there is no reading at all.
 */
static inline float motor_sense_current_a(const struct motor_sense_bundle *sense)
{
	if (sense == NULL) {
		return 0.0f;
	}

	if (sense->bridge_current_valid) {
		return sense->bridge_current_a;
	}

	return (sense->current_count > 0U) ? sense->current_amps[0] : 0.0f;
}

struct motor_block_in {
	const struct motor_sense_bundle *sense;
	float current_ref_a;
	bool has_current_ref;
	/* Signed normalized command applied on the previous inner tick. Blocks
	 * that model the plant need u(k-1), since i(k) is its consequence.
	 */
	float applied_u;
	bool has_applied_u;
};

struct motor_block_out {
	float duty[MOTOR_ACTUATOR_DUTY_MAX];
	uint8_t n_duty;
	float current_ref_a;
	bool has_current_ref;
	/* Signed normalized command behind @ref duty, before the topology
	 * mapping makes it ambiguous (bipolar vs sign-magnitude).
	 */
	float applied_u;
	bool has_applied_u;
};

struct motor_pipeline;

struct motor_ctrl {
	const struct device *sensor;
	const struct device *actuator;

	struct motor_pipeline *pipeline;
	void *pipeline_ctx;

	enum motor_state state;
	struct motor_sense_bundle last_sense;

	uint32_t inner_stage_tick;
	uint32_t slow_stage_tick;

	/* Cross-rate hop: OUTER writes, INNER reads (float32 store is atomic). */
	float cascaded_current_ref_a;
	bool cascaded_current_ref_valid;

	/* Inner-tick hop: written at the end of a tick, read on the next one. */
	float last_applied_u;
	bool last_applied_u_valid;

	struct k_mutex lock;
	struct k_sem slow_tick_sem;
	struct k_thread slow_thread;
	K_KERNEL_STACK_MEMBER(slow_thread_stack, 1024);
	bool slow_thread_started;

	uint32_t inner_rate_hz;

	motor_state_cb_t app_state_cb;
	motor_fault_notify_cb_t app_fault_cb;
	void *app_cb_data;
};

#define MOTOR_CTRL_DEFINE(name) static struct motor_ctrl name = {0}

int motor_ctrl_self_test(struct motor_ctrl *ctrl, uint32_t *flags);
int motor_ctrl_enable(struct motor_ctrl *ctrl);
int motor_ctrl_disable(struct motor_ctrl *ctrl);
void motor_ctrl_estop(struct motor_ctrl *ctrl);
int motor_ctrl_clear_fault(struct motor_ctrl *ctrl);
void motor_ctrl_get_status(const struct motor_ctrl *ctrl, enum motor_state *state,
			   uint32_t *faults);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_CONTROLLER_H_ */
