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
 * Application code uses @ref motor.h.  The current loop runs in the power-stage
 * callback at @c inner_rate_hz.  Additional @ref motor_pipeline_stage values
 * are reserved for future outer or multi-rate blocks.
 */

struct motor_block;

struct motor_ctrl_setpoints {
	/** Inner-loop current reference (A) — e.g. DC armature Ia, FOC Iq. */
	float i_ref_a;
};

struct motor_block_in {
	const struct motor_sensor_output *sense;
	const struct motor_ctrl_setpoints *sp;
};

struct motor_block_out {
	struct motor_actuator_cmd *cmd;
};

struct motor_pipeline;

struct motor_ctrl {
	const struct device *sensor;
	const struct device *actuator;

	struct motor_pipeline *pipeline;
	void *pipeline_ctx;

	enum motor_state state;
	struct motor_ctrl_setpoints setpoints;

	uint32_t inner_stage_tick;

	struct k_mutex lock;

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
