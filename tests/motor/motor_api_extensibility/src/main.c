/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fake sensor/actuator + custom pipeline block: ISR path uses stub entry outputs.
 */

#include <string.h>

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

MOTOR_TEST_FAKE_DEFINE(motor_ext);

static void stub_block_entry(struct motor_block *self, const struct motor_block_in *in,
			     struct motor_block_out *out)
{
	ARG_UNUSED(self);
	ARG_UNUSED(in);

	out->n_duty = 1U;
	out->duty[0] = 0.5f;
}

static struct motor_block stub_template = {
	.name = "stub",
	.stage = MOTOR_STAGE_INNER_ISR,
	.period_div = 1,
	.entry = stub_block_entry,
	.reset = NULL,
	.set_params = NULL,
};

static struct motor_block stub_state;

static struct motor_block *const stub_blocks[] = { &stub_state };
static struct motor_pipeline stub_pipeline = {
	.name = "stub_pl",
	.blocks = stub_blocks,
	.n_blocks = 1,
};

ZTEST_SUITE(motor_api_extensibility_suite, NULL, NULL, NULL, NULL, NULL);

ZTEST(motor_api_extensibility_suite, test_motor_ctrl_init_with_custom_block)
{
	const struct device *sens = DEVICE_GET(motor_ext_sensor);
	const struct device *act = DEVICE_GET(motor_ext_actuator);

	memcpy(&stub_state, &stub_template, sizeof(stub_state));

	zassert_equal(motor_ctrl_init(&ctrl, sens, act, &stub_pipeline, &stub_state), 0, NULL);
}

ZTEST(motor_api_extensibility_suite, test_isr_uses_stub_inner_step)
{
	const struct device *sens = DEVICE_GET(motor_ext_sensor);
	const struct device *act = DEVICE_GET(motor_ext_actuator);
	motor_t m;

	memcpy(&stub_state, &stub_template, sizeof(stub_state));
	motor_ext.has_last_duty = false;

	memset(&ctrl, 0, sizeof(ctrl));

	zassert_equal(motor_ctrl_init(&ctrl, sens, act, &stub_pipeline, &stub_state), 0, NULL);
	m = &ctrl;

	zassert_equal(motor_enable(m), 0, NULL);

	motor_test_fake_sync_fire(act);

	zassert_true(motor_ext.has_last_duty, NULL);
	zassert_equal(motor_ext.last_n_duty, 1U, NULL);
	zassert_within(motor_ext.last_duty[0], 0.5f, 1e-5f, NULL);

	motor_estop(m);
}
