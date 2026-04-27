/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_CTRL_PRIV_H_
#define ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_CTRL_PRIV_H_

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/subsys/motor/motor_pipeline.h>

struct motor_ctrl;

int motor_ctrl_init(struct motor_ctrl *ctrl, const struct device *sensor,
		    const struct device *actuator, struct motor_pipeline *pipeline,
		    void *pipeline_ctx, uint32_t inner_rate_hz);

int motor_pipeline_init(struct motor_pipeline *pipeline, void *ctx);

void motor_pipeline_reset(struct motor_pipeline *pipeline, void *ctx);

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_CTRL_PRIV_H_ */
