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
#include <zephyr/subsys/motor/algorithms/dc_current/motor_algo_dc_current.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/subsys/motor/motor_ctrl_priv.h>
#include <zephyr/subsys/motor/motor_pipeline.h>
#include <zephyr/ztest.h>

#include "motor_test_fake.h"

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
static struct motor_block *const pipeline_blocks[] = { &algo_data.base };
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

	pipeline_motor.sense.hot.i_phase[0] = 0.0f;
	pipeline_motor.sense.hot.valid_current = true;
	pipeline_motor.has_last_cmd = false;

	zassert_true(device_is_ready(sens), NULL);
	zassert_true(device_is_ready(act), NULL);

	zassert_equal(motor_ctrl_init(&ctrl, sens, act, &pipeline_pl, &algo_data, 20000U), 0, NULL);
	m = &ctrl;

	zassert_equal(motor_enable(m), 0, NULL);

	zassert_equal(motor_set_torque(m, 0.01f), 0, NULL);

	motor_actuator_invoke_control_callback(act);

	zassert_true(pipeline_motor.has_last_cmd, NULL);
	zassert_equal(pipeline_motor.last_cmd.kind, MOTOR_ACTUATOR_CMD_ALPHA_BETA, NULL);
	zassert_within(pipeline_motor.last_cmd.u.ab.vbeta, 0.0f, 1e-5f, NULL);
	zassert_within(pipeline_motor.last_cmd.u.ab.valpha, 0.12f, 0.02f, NULL);

	motor_estop(m);
}
