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
#include <zephyr/subsys/motor/algorithms/dc_current/motor_algo_dc_current.h>
#include <zephyr/subsys/motor/motor_ctrl_priv.h>
#include <zephyr/subsys/motor/motor_block.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/subsys/motor/motor_pipeline.h>
#include <zephyr/ztest.h>

#include "motor_test_fake.h"

static struct motor_ctrl ctrl;

MOTOR_TEST_FAKE_DEFINE(motor_ext);

static void stub_block_entry(struct motor_block *self, const struct motor_block_in *in,
			     struct motor_block_out *out)
{
	ARG_UNUSED(self);
	ARG_UNUSED(in);

	out->cmd->kind = MOTOR_ACTUATOR_CMD_DUTY_DIRECT;
	out->cmd->u.duty.n = 1U;
	out->cmd->u.duty.duty[0] = 0.5f;
}

static struct motor_algo_dc_current_data stub_template = {
	.base =
		{
			.name = "stub",
			.stage = MOTOR_STAGE_INNER_ISR,
			.period_div = 1,
			.entry = stub_block_entry,
			.reset = NULL,
			.set_params = NULL,
		},
	.i_integral = 0.0f,
	.pi =
		{
			.kp = 0.5f,
			.ki = 1.0f,
			.out_min = -1.0f,
			.out_max = 1.0f,
		},
	.limits =
		{
			.i_max_a = 5.0f,
			.vbus_derating_start = 0.0f,
			.temp_derating_start = 0.0f,
			.temp_fault = 150.0f,
		},
	.timing = {.control_loop_dt_s = 1.0f / 20000.0f},
};

static struct motor_algo_dc_current_data stub_state;

static struct motor_block *const stub_blocks[] = { &stub_state.base };
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

	zassert_equal(motor_ctrl_init(&ctrl, sens, act, &stub_pipeline, &stub_state, 20000U), 0, NULL);
}

ZTEST(motor_api_extensibility_suite, test_isr_uses_stub_inner_step)
{
	const struct device *sens = DEVICE_GET(motor_ext_sensor);
	const struct device *act = DEVICE_GET(motor_ext_actuator);
	motor_t m;

	memcpy(&stub_state, &stub_template, sizeof(stub_state));
	motor_ext.has_last_cmd = false;

	memset(&ctrl, 0, sizeof(ctrl));

	zassert_equal(motor_ctrl_init(&ctrl, sens, act, &stub_pipeline, &stub_state, 20000U), 0, NULL);
	m = &ctrl;

	zassert_equal(motor_enable(m), 0, NULL);
	zassert_equal(motor_set_current(m, 0.2f), 0, NULL);

	motor_actuator_invoke_control_callback(act);

	zassert_true(motor_ext.has_last_cmd, NULL);
	zassert_equal(motor_ext.last_cmd.kind, MOTOR_ACTUATOR_CMD_DUTY_DIRECT, NULL);
	zassert_equal(motor_ext.last_cmd.u.duty.n, 1U, NULL);
	zassert_within(motor_ext.last_cmd.u.duty.duty[0], 0.5f, 1e-5f, NULL);

	motor_estop(m);
}
