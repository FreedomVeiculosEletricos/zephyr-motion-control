/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/subsys/motor/algorithms/dc_current/motor_algo_dc_current.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/subsys/motor/motor_ctrl_priv.h>
#include <zephyr/subsys/motor/motor.h>
#include <zephyr/subsys/motor/motor_pipeline.h>
#include <zephyr/ztest.h>

static struct motor_ctrl ctrl;
static struct motor_algo_dc_current_data algo_data = {
	MOTOR_ALGO_DC_CURRENT_BASE_INITIALIZER
	.pi =
		{
			.kp = 0.5f,
			.ki = 2000.0f,
			.out_min = -1.0f,
			.out_max = 1.0f,
		},
	.limits =
		{
			.i_max_a = 5.0f,
			.speed_max_rad_s = 100.0f,
			.vbus_derating_start = 0.0f,
			.temp_derating_start = 0.0f,
			.temp_fault = 150.0f,
		},
	.timing = {.control_loop_dt_s = 1.0f / 20000.0f},
	.kt_nm_per_a = 0.05f,
	.pole_pairs = 1U,
};
static struct motor_block *const test_blocks[] = { &algo_data.base };
static struct motor_pipeline test_pipeline = {
	.name = "api_test",
	.blocks = test_blocks,
	.n_blocks = 1,
};

ZTEST_SUITE(motor_api_suite, NULL, NULL, NULL, NULL, NULL);

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_null_ctrl)
{
	zassert_equal(motor_ctrl_init(NULL, NULL, NULL, NULL, NULL, 20000U), -EINVAL);
}

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_null_sensor)
{
	zassert_equal(motor_ctrl_init(&ctrl, NULL, NULL, &test_pipeline, &algo_data, 20000U),
		      -EINVAL);
}

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_null_actuator)
{
	zassert_equal(motor_ctrl_init(&ctrl, (const struct device *)&ctrl, NULL, &test_pipeline,
				      &algo_data, 20000U),
		      -EINVAL);
}

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_null_pipeline)
{
	zassert_equal(motor_ctrl_init(&ctrl, (const struct device *)&ctrl,
				      (const struct device *)&ctrl, NULL, &algo_data, 20000U),
		      -EINVAL);
}

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_null_pipeline_ctx)
{
	zassert_equal(motor_ctrl_init(&ctrl, (const struct device *)&ctrl,
				      (const struct device *)&ctrl, &test_pipeline, NULL, 20000U),
		      -EINVAL);
}

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_zero_inner_rate)
{
	zassert_equal(motor_ctrl_init(&ctrl, (const struct device *)&ctrl,
				      (const struct device *)&ctrl, &test_pipeline, &algo_data, 0U),
		      -EINVAL);
}

ZTEST(motor_api_suite, test_motor_set_torque_rejects_null_handle)
{
	zassert_equal(motor_set_torque(NULL, 0.0f), -EINVAL);
}
