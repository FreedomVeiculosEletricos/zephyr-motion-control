/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zephyr/device.h>
#include <zephyr/subsys/motor/algorithms/adrc_current/motor_algo_adrc_current.h>
#include <zephyr/subsys/motor/motor.h>
#include <zephyr/subsys/motor/motor_subsys.h>
#include <zephyr/ztest.h>

#include "motor_test_fake.h"

static struct motor_test_fake adrc_current_fake;

DEVICE_DT_DEFINE(DT_NODELABEL(adrc_current_sensor), motor_test_fake_sensor_init, NULL,
		 &adrc_current_fake, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		 &motor_test_fake_sensor_ops);
DEVICE_DT_DEFINE(DT_NODELABEL(adrc_current_actuator), motor_test_fake_actuator_init, NULL,
		 &adrc_current_fake, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		 &motor_test_fake_actuator_ops);

static void *suite_setup(void)
{
	zassert_equal(motor_subsys_init(), 0, NULL);
	return NULL;
}

ZTEST_SUITE(motor_algo_adrc_current_suite, NULL, suite_setup, NULL, NULL, NULL);

ZTEST(motor_algo_adrc_current_suite, test_bypass_unit_gain_and_value_passthrough)
{
	motor_t motor = motor_subsys_get_by_label("adrc_current_motor");
	struct motor_adrc_current_params params = {
		.kp = 1.0f,
		.b0 = 1.0f,
		.beta1 = 0.0f,
		.beta2 = 0.0f,
		.out_min = -1.0f,
		.out_max = 1.0f,
	};
	struct motor_adrc_current_state state;

	zassert_not_null(motor, NULL);

	adrc_current_fake.sensor_samples[0] = 0.0f;
	adrc_current_fake.sensor_sample_count = 1U;
	adrc_current_fake.has_last_duty = false;

	zassert_equal(motor_enable(motor), 0, NULL);
	zassert_equal(motor_algo_adrc_current_set_params(motor, &params, true), 0, NULL);
	zassert_equal(motor_algo_adrc_current_set_current(motor, 0.2f), 0, NULL);

	motor_test_fake_sync_fire(DEVICE_DT_GET(DT_NODELABEL(adrc_current_actuator)));

	zassert_true(adrc_current_fake.has_last_duty, NULL);
	zassert_equal(adrc_current_fake.last_n_duty, 2U, NULL);
	zassert_within(adrc_current_fake.last_duty[0], 0.6f, 1e-4f, NULL);
	zassert_within(adrc_current_fake.last_duty[1], 0.4f, 1e-4f, NULL);

	zassert_equal(motor_algo_adrc_current_get_state(motor, &state), 0, NULL);
	zassert_within(state.i_ref_a, 0.2f, 1e-6f, NULL);
	zassert_within(state.i_meas_a, 0.0f, 1e-6f, NULL);
	zassert_within(state.error_a, 0.2f, 1e-6f, NULL);
	zassert_within(state.u, 0.2f, 1e-4f, NULL);
	zassert_within(state.duty, 0.6f, 1e-4f, NULL);

	motor_estop(motor);
}

ZTEST(motor_algo_adrc_current_suite, test_rejects_invalid_params)
{
	motor_t motor = motor_subsys_get_by_label("adrc_current_motor");
	struct motor_adrc_current_params invalid = {
		.kp = 1.0f,
		.b0 = 0.0f,
		.beta1 = 10.0f,
		.beta2 = 100.0f,
		.out_min = -1.0f,
		.out_max = 1.0f,
	};
	struct motor_adrc_current_limits invalid_limits = {
		.i_max_a = 0.0f,
	};

	zassert_not_null(motor, NULL);
	zassert_equal(motor_enable(motor), 0, NULL);
	zassert_equal(motor_algo_adrc_current_set_params(motor, &invalid, false), -EINVAL, NULL);
	zassert_equal(motor_algo_adrc_current_set_limits(motor, &invalid_limits), -EINVAL, NULL);
	motor_estop(motor);
}

ZTEST(motor_algo_adrc_current_suite, test_step_response_converges)
{
	motor_t motor = motor_subsys_get_by_label("adrc_current_motor");
	struct motor_adrc_current_params params = {
		.kp = 1.0f,
		.b0 = 1.0f,
		.beta1 = 2.0f,
		.beta2 = 10.0f,
		.out_min = -1.0f,
		.out_max = 1.0f,
	};
	struct motor_adrc_current_state state;
	const float ref = 0.7f;
	float plant = 0.0f;
	float start_err = fabsf(ref - plant);
	float final_err;

	zassert_not_null(motor, NULL);
	zassert_equal(motor_enable(motor), 0, NULL);
	zassert_equal(motor_algo_adrc_current_set_params(motor, &params, true), 0, NULL);
	zassert_equal(motor_algo_adrc_current_set_current(motor, ref), 0, NULL);

	for (int i = 0; i < 120; i++) {
		float u;

		adrc_current_fake.sensor_samples[0] = plant;
		adrc_current_fake.sensor_sample_count = 1U;
		motor_test_fake_sync_fire(DEVICE_DT_GET(DT_NODELABEL(adrc_current_actuator)));
		u = (2.0f * adrc_current_fake.last_duty[0]) - 1.0f;
		plant = u;

	}

	final_err = fabsf(ref - plant);
	zassert_true(final_err < start_err, NULL);
	zassert_within(plant, ref, 0.1f, NULL);

	zassert_equal(motor_algo_adrc_current_get_state(motor, &state), 0, NULL);
	zassert_within(state.i_meas_a, plant, 0.1f, NULL);
	zassert_within(state.i_ref_a, ref, 1e-6f, NULL);

	motor_estop(motor);
}
