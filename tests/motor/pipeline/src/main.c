/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fake sensor + fake actuator: exercise motor_ctrl ISR path
 * (current loop -> set_command).
 */

#include <zephyr/device.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/subsys/motor/motor.h>
#include <zephyr/subsys/motor/motor_algo_dc_torque.h>
#include <zephyr/ztest.h>

#include "motor_test_fake.h"

static struct motor_ctrl ctrl;
static const struct motor_ctrl_params pipeline_params = {
	.limits = {.i_max_a = 5.0f},
	.timing = {.control_loop_dt_s = 1.0f / 20000.0f},
	.kt_nm_per_a = 0.05f,
	.pole_pairs = 1U,
};
static struct motor_algo_dc_torque_data algo_data = {
	.current_loop =
		{
			.kp = 0.5f,
			.ki = 2000.0f,
			.out_min = -1.0f,
			.out_max = 1.0f,
		},
};

MOTOR_TEST_FAKE_DEFINE(pipeline_motor);

ZTEST_SUITE(motor_pipeline_suite, NULL, NULL, NULL, NULL, NULL);

ZTEST(motor_pipeline_suite, test_inner_step_reaches_actuator)
{
	const struct device *sens = DEVICE_GET(pipeline_motor_sensor);
	const struct device *act = DEVICE_GET(pipeline_motor_actuator);
	motor_t m;

	pipeline_motor.sense.hot.i_phase[0] = 0.0f;
	pipeline_motor.sense.hot.valid_current = true;
	pipeline_motor.has_last_cmd = false;

	zassert_true(device_is_ready(sens), NULL);
	zassert_true(device_is_ready(act), NULL);

	m = motor_init(&ctrl, sens, act, &motor_algo_dc_torque, &algo_data, &pipeline_params);
	zassert_not_null(m, NULL);

	zassert_equal(motor_enable(m), 0, NULL);

	/* torque_nm / kt_nm_per_a = i_torque_a; kt = 0.05 -> 0.01 N.m -> 0.2 A */
	zassert_equal(motor_set_torque(m, 0.01f), 0, NULL);

	motor_actuator_invoke_control_callback(act);

	zassert_true(pipeline_motor.has_last_cmd, NULL);
	zassert_equal(pipeline_motor.last_cmd.kind, MOTOR_ACTUATOR_CMD_ALPHA_BETA, NULL);
	zassert_within(pipeline_motor.last_cmd.u.ab.vbeta, 0.0f, 1e-5f, NULL);
	/* i_torque_a=0.2, ia=0 -> first PI step valpha ~= 0.12 */
	zassert_within(pipeline_motor.last_cmd.u.ab.valpha, 0.12f, 0.02f, NULL);

	motor_estop(m);
}
