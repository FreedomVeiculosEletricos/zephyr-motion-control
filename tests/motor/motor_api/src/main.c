/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/subsys/motor/motor_block.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/subsys/motor/motor.h>
#include <zephyr/subsys/motor/motor_pipeline.h>
#include <zephyr/ztest.h>

#include "motor_ctrl_priv.h"
#include "motor_test_fake.h"

MOTOR_TEST_FAKE_DEFINE(motor_pwmchk);

static struct motor_ctrl ctrl;
static struct motor_block test_block = {
	.name = "api_test_block",
	.stage = MOTOR_STAGE_INNER_ISR,
	.period_div = 1U,
};
static struct motor_block *const test_blocks[] = { &test_block };
static struct motor_pipeline test_pipeline = {
	.name = "api_test",
	.blocks = test_blocks,
	.n_blocks = 1,
};

ZTEST_SUITE(motor_api_suite, NULL, NULL, NULL, NULL, NULL);

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_null_ctrl)
{
	zassert_equal(motor_ctrl_init(NULL, NULL, NULL, NULL, NULL), -EINVAL);
}

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_null_sensor)
{
	zassert_equal(motor_ctrl_init(&ctrl, NULL, NULL, &test_pipeline, &test_block), -EINVAL);
}

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_null_actuator)
{
	zassert_equal(
		motor_ctrl_init(&ctrl, (const struct device *)&ctrl, NULL, &test_pipeline, &test_block),
		-EINVAL);
}

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_null_pipeline)
{
	zassert_equal(motor_ctrl_init(&ctrl, (const struct device *)&ctrl,
				      (const struct device *)&ctrl, NULL, &test_block),
		      -EINVAL);
}

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_null_pipeline_ctx)
{
	zassert_equal(motor_ctrl_init(&ctrl, (const struct device *)&ctrl,
				      (const struct device *)&ctrl, &test_pipeline, NULL),
		      -EINVAL);
}

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_pwm_period_not_divisor_of_1g)
{
	const struct device *sens = DEVICE_GET(motor_pwmchk_sensor);
	const struct device *act = DEVICE_GET(motor_pwmchk_actuator);

	motor_pwmchk.actuator_stage.pwm_period_ns = 30000U;
	zassert_equal(motor_ctrl_init(&ctrl, sens, act, &test_pipeline, &test_block), -EINVAL);
	motor_pwmchk.actuator_stage.pwm_period_ns = 50000U;
}

