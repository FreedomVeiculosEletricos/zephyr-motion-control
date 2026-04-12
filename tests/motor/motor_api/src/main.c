/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/subsys/motor/motor_algo_dc_torque.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/subsys/motor/motor.h>
#include <zephyr/ztest.h>

static struct motor_ctrl ctrl;
static struct motor_algo_dc_torque_data algo_data;

static const struct motor_ctrl_params motor_api_valid_params = {
	.limits = {.i_max_a = 5.0f},
	.timing = {.control_loop_dt_s = 1.0f / 20000.0f},
	.kt_nm_per_a = 0.05f,
	.pole_pairs = 1U,
};

ZTEST_SUITE(motor_api_suite, NULL, NULL, NULL, NULL, NULL);

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_null_ctrl)
{
	zassert_equal(motor_ctrl_init(NULL, NULL, NULL, NULL, NULL, &motor_api_valid_params),
		      -EINVAL);
}

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_null_sensor)
{
	zassert_equal(motor_ctrl_init(&ctrl, NULL, NULL, &motor_algo_dc_torque, &algo_data,
				      &motor_api_valid_params),
		      -EINVAL);
}

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_null_actuator)
{
	zassert_equal(motor_ctrl_init(&ctrl, (const struct device *)&ctrl, NULL,
				      &motor_algo_dc_torque, &algo_data, &motor_api_valid_params),
		      -EINVAL);
}

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_null_algo)
{
	zassert_equal(motor_ctrl_init(&ctrl, (const struct device *)&ctrl,
				      (const struct device *)&ctrl, NULL, &algo_data,
				      &motor_api_valid_params),
		      -EINVAL);
}

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_null_algo_data)
{
	zassert_equal(motor_ctrl_init(&ctrl, (const struct device *)&ctrl,
				      (const struct device *)&ctrl, &motor_algo_dc_torque, NULL,
				      &motor_api_valid_params),
		      -EINVAL);
}

ZTEST(motor_api_suite, test_motor_ctrl_init_rejects_null_params)
{
	zassert_equal(motor_ctrl_init(&ctrl, (const struct device *)&ctrl,
				      (const struct device *)&ctrl, &motor_algo_dc_torque, &algo_data,
				      NULL),
		      -EINVAL);
}

ZTEST(motor_api_suite, test_motor_ctrl_set_params_rejects_null)
{
	struct motor_ctrl_params p = {0};

	zassert_equal(motor_ctrl_set_params(NULL, &p), -EINVAL);
	zassert_equal(motor_ctrl_set_params(&ctrl, NULL), -EINVAL);
}

ZTEST(motor_api_suite, test_motor_set_torque_rejects_null_handle)
{
	zassert_equal(motor_set_torque(NULL, 0.0f), -EINVAL);
}

ZTEST(motor_api_suite, test_motor_init_rejects_null_controller_storage)
{
	zassert_is_null(motor_init(NULL, NULL, NULL, &motor_algo_dc_torque, &algo_data,
				   &motor_api_valid_params));
}

ZTEST(motor_api_suite, test_motor_init_rejects_null_params)
{
	zassert_is_null(motor_init(&ctrl, (const struct device *)&ctrl,
				   (const struct device *)&ctrl, &motor_algo_dc_torque, &algo_data,
				   NULL));
}
