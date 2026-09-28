/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fake sensor + fake actuator: exercise motor_ctrl ISR path and same-stage
 * current_ref hop between pipeline blocks.
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

static float hop_current_ref;
static bool hop_saw_ref;

static void outer_block_entry(struct motor_block *self, const struct motor_block_in *in,
			      struct motor_block_out *out)
{
	ARG_UNUSED(self);
	ARG_UNUSED(in);

	out->n_duty = 0U;
	out->has_current_ref = true;
	out->current_ref_a = 1.25f;
}

static void inner_block_entry(struct motor_block *self, const struct motor_block_in *in,
			      struct motor_block_out *out)
{
	ARG_UNUSED(self);

	hop_saw_ref = in->has_current_ref;
	hop_current_ref = in->current_ref_a;

	out->n_duty = 1U;
	out->duty[0] = 0.5f;
	out->has_current_ref = false;
}

static struct motor_block outer_block = {
	.name = "outer_test",
	.stage = MOTOR_STAGE_OUTER,
	.period_div = 1U,
	.entry = outer_block_entry,
};

static struct motor_block inner_block = {
	.name = "inner_test",
	.stage = MOTOR_STAGE_INNER_ISR,
	.period_div = 1U,
	.entry = inner_block_entry,
};

static struct motor_block *const pipeline_blocks[] = {&outer_block, &inner_block};
static struct motor_pipeline pipeline_pl = {
	.name = "pipe_test",
	.blocks = pipeline_blocks,
	.n_blocks = 2,
};

static struct motor_ctrl ctrl;

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
	hop_saw_ref = false;

	zassert_true(device_is_ready(sens), NULL);
	zassert_true(device_is_ready(act), NULL);

	zassert_equal(motor_ctrl_init(&ctrl, sens, act, &pipeline_pl, &inner_block), 0, NULL);
	m = &ctrl;

	/* Re-apply after init clears cascaded hop state. */
	ctrl.cascaded_current_ref_valid = true;
	ctrl.cascaded_current_ref_a = 0.75f;

	zassert_equal(motor_enable(m), 0, NULL);

	motor_test_fake_sync_fire(act);

	zassert_true(pipeline_motor.has_last_duty, NULL);
	zassert_equal(pipeline_motor.last_n_duty, 1U, NULL);
	zassert_within(pipeline_motor.last_duty[0], 0.5f, 1e-5f, NULL);
	zassert_true(hop_saw_ref, NULL);
	zassert_within(hop_current_ref, 0.75f, 1e-5f, NULL);

	motor_estop(m);
}

ZTEST(motor_pipeline_suite, test_same_stage_current_ref_hop)
{
	struct motor_block producer = {
		.name = "producer",
		.stage = MOTOR_STAGE_INNER_ISR,
		.period_div = 1U,
		.entry = outer_block_entry,
	};
	struct motor_block consumer = {
		.name = "consumer",
		.stage = MOTOR_STAGE_INNER_ISR,
		.period_div = 1U,
		.entry = inner_block_entry,
	};
	struct motor_block *const blocks[] = {&producer, &consumer};
	struct motor_pipeline pipe = {
		.name = "hop",
		.blocks = blocks,
		.n_blocks = 2,
	};
	struct motor_sense_bundle sense = {0};
	struct motor_block_in in = {.sense = &sense};
	struct motor_block_out out = {0};

	hop_saw_ref = false;
	hop_current_ref = 0.0f;

	motor_pipeline_run_stage(&pipe, NULL, MOTOR_STAGE_INNER_ISR, 0U, &in, &out);

	zassert_true(hop_saw_ref, NULL);
	zassert_within(hop_current_ref, 1.25f, 1e-5f, NULL);
	zassert_equal(out.n_duty, 1U, NULL);
}
