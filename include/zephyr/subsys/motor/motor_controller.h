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
 * Additional @ref motor_pipeline_stage values are reserved for future outer
 * or multi-rate blocks.
 */

struct motor_block;

struct motor_sense_bundle {
	float current_amps[MOTOR_SENSOR_CURRENT_MAX];
	size_t current_count;
	float angle_rad;
	bool angle_valid;
};

struct motor_block_in {
	const struct motor_sense_bundle *sense;
};

struct motor_block_out {
	float duty[MOTOR_ACTUATOR_DUTY_MAX];
	uint8_t n_duty;
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
