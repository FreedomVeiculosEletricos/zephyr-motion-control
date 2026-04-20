/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fake sensor/actuator + non-default motor_algo_ops: the API accepts any
 * algorithm vtable; the ISR path uses stub inner_step outputs.
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/subsys/motor/motor.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/ztest.h>

#include "motor_test_fake.h"

static struct motor_ctrl ctrl;

MOTOR_TEST_FAKE_DEFINE(motor_ext);

static int g_stub_init_calls;

struct stub_algo_state {
	uint8_t pad;
};

static struct stub_algo_state stub_algo_data;

static const struct motor_ctrl_params stub_motor_params = {
	.limits = {.i_max_a = 5.0f},
	.timing = {.control_loop_dt_s = 1.0f / 20000.0f},
	.kt_nm_per_a = 0.05f,
	.pole_pairs = 1U,
};

static int stub_algo_init(void *algo, const struct motor_ctrl_params *params)
{
	ARG_UNUSED(algo);
	ARG_UNUSED(params);

	g_stub_init_calls++;
	return 0;
}

static void stub_algo_inner_step(void *algo, const struct motor_block_in *in,
				 struct motor_block_out *out)
{
	ARG_UNUSED(algo);
	ARG_UNUSED(in);

	out->cmd->kind = MOTOR_ACTUATOR_CMD_ALPHA_BETA;
	out->cmd->u.ab.valpha = 1.0f;
	out->cmd->u.ab.vbeta = 2.0f;
}

static void stub_algo_set_params(void *algo, const struct motor_ctrl_params *params)
{
	ARG_UNUSED(algo);
	ARG_UNUSED(params);
}

static void stub_algo_reset(void *algo)
{
	ARG_UNUSED(algo);
}

static const struct motor_algo_ops stub_motor_algo = {
	.init = stub_algo_init,
	.inner_step = stub_algo_inner_step,
	.outer_step_0 = NULL,
	.outer_step_1 = NULL,
	.set_params = stub_algo_set_params,
	.reset = stub_algo_reset,
};

ZTEST_SUITE(motor_api_extensibility_suite, NULL, NULL, NULL, NULL, NULL);

ZTEST(motor_api_extensibility_suite, test_stub_algo_init_invoked)
{
	const struct device *sens = DEVICE_GET(motor_ext_sensor);
	const struct device *act = DEVICE_GET(motor_ext_actuator);

	g_stub_init_calls = 0;

	zassert_equal(motor_ctrl_init(&ctrl, sens, act, &stub_motor_algo, &stub_algo_data,
				      &stub_motor_params),
		      0, NULL);
	zassert_equal(g_stub_init_calls, 1, NULL);
}

ZTEST(motor_api_extensibility_suite, test_isr_uses_stub_inner_step)
{
	const struct device *sens = DEVICE_GET(motor_ext_sensor);
	const struct device *act = DEVICE_GET(motor_ext_actuator);
	motor_t m;

	g_stub_init_calls = 0;
	motor_ext.has_last_cmd = false;

	memset(&ctrl, 0, sizeof(ctrl));

	m = motor_init(&ctrl, sens, act, &stub_motor_algo, &stub_algo_data, &stub_motor_params);
	zassert_not_null(m, NULL);
	zassert_equal(g_stub_init_calls, 1, NULL);

	zassert_equal(motor_enable(m), 0, NULL);
	zassert_equal(motor_set_torque(m, 0.05f), 0, NULL);

	motor_actuator_invoke_control_callback(act);

	zassert_true(motor_ext.has_last_cmd, NULL);
	zassert_equal(motor_ext.last_cmd.kind, MOTOR_ACTUATOR_CMD_ALPHA_BETA, NULL);
	zassert_within(motor_ext.last_cmd.u.ab.valpha, 1.0f, 1e-5f, NULL);
	zassert_within(motor_ext.last_cmd.u.ab.vbeta, 2.0f, 1e-5f, NULL);

	motor_estop(m);
}

ZTEST(motor_api_extensibility_suite, test_motor_set_torque_rejects_zero_kt)
{
	const struct device *sens = DEVICE_GET(motor_ext_sensor);
	const struct device *act = DEVICE_GET(motor_ext_actuator);
	static struct motor_ctrl_params bad_kt = {
		.limits = {.i_max_a = 3.0f},
		.timing = {.control_loop_dt_s = 1.0f / 20000.0f},
		.kt_nm_per_a = 0.0f,
		.pole_pairs = 1U,
	};
	motor_t m;

	memset(&ctrl, 0, sizeof(ctrl));

	m = motor_init(&ctrl, sens, act, &stub_motor_algo, &stub_algo_data, &bad_kt);
	zassert_not_null(m, NULL);
	zassert_equal(motor_enable(m), 0, NULL);
	zassert_equal(motor_set_torque(m, 0.05f), -EINVAL, NULL);

	motor_estop(m);
}
