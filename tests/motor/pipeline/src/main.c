/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fake sensor + fake actuator: exercise motor_ctrl ISR path
 * (current loop -> set_duty).
 */

#include <zephyr/device.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/subsys/motor/motor.h>
#include <zephyr/subsys/motor/motor_block.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/subsys/motor/motor_pipeline.h>
#include <zephyr/ztest.h>

#include "motor_ctrl_priv.h"
#include "motor_test_fake.h"

static struct motor_ctrl ctrl;
static void pipeline_test_block_entry(struct motor_block *self, const struct motor_block_in *in,
				      struct motor_block_out *out)
{
	ARG_UNUSED(self);
	ARG_UNUSED(in);

	out->n_duty = 1U;
	out->duty[0] = 0.5f;
}

static struct motor_block pipeline_block = {
	.name = "pipeline_test_block",
	.stage = MOTOR_STAGE_INNER_ISR,
	.period_div = 1U,
	.entry = pipeline_test_block_entry,
};
static struct motor_block *const pipeline_blocks[] = { &pipeline_block };
static struct motor_pipeline pipeline_pl = {
	.name = "pipe_test",
	.blocks = pipeline_blocks,
	.n_blocks = 1,
};

MOTOR_TEST_FAKE_DEFINE(pipeline_motor);

ZTEST_SUITE(motor_pipeline_suite, NULL, NULL, NULL, NULL, NULL);

ZTEST(motor_pipeline_suite, test_inner_step_reaches_actuator)
{
	const struct device *sens = DEVICE_GET(pipeline_motor_sensor);
	const struct device *act = DEVICE_GET(pipeline_motor_actuator);
	motor_t m;

	pipeline_motor.sensor_samples[0] = 0.0f;
	pipeline_motor.sensor_sample_count = 1U;
	pipeline_motor.has_last_duty = false;

	zassert_true(device_is_ready(sens), NULL);
	zassert_true(device_is_ready(act), NULL);

	zassert_equal(motor_ctrl_init(&ctrl, sens, act, &pipeline_pl, &pipeline_block), 0, NULL);
	m = &ctrl;

	zassert_equal(motor_enable(m), 0, NULL);

	motor_test_fake_sync_fire(act);

	zassert_true(pipeline_motor.has_last_duty, NULL);
	zassert_equal(pipeline_motor.last_n_duty, 1U, NULL);
	zassert_within(pipeline_motor.last_duty[0], 0.5f, 1e-5f, NULL);

	motor_estop(m);
}
