/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/subsys/motor/algorithms/dc_current/motor_algo_dc_current.h>
#include <zephyr/subsys/motor/motor.h>
#include <zephyr/subsys/motor/motor_subsys.h>
#include <zephyr/ztest.h>

#include "motor_test_fake.h"

static struct motor_test_fake dc_current_fake;

DEVICE_DT_DEFINE(DT_NODELABEL(dc_current_sensor), motor_test_fake_sensor_init, NULL,
		 &dc_current_fake, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		 &motor_test_fake_sensor_ops);
DEVICE_DT_DEFINE(DT_NODELABEL(dc_current_actuator), motor_test_fake_actuator_init, NULL,
		 &dc_current_fake, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		 &motor_test_fake_actuator_ops);

static void *suite_setup(void)
{
	zassert_equal(motor_subsys_init(), 0, NULL);
	return NULL;
}

ZTEST_SUITE(motor_algo_dc_current_suite, NULL, suite_setup, NULL, NULL, NULL);

ZTEST(motor_algo_dc_current_suite, test_current_command_updates_state_and_duty)
{
	motor_t motor = motor_subsys_get_by_label("dc_current_motor");
	struct motor_dc_current_state state;

	zassert_not_null(motor, NULL);

	dc_current_fake.sensor_samples[0] = 0.0f;
	dc_current_fake.sensor_sample_count = 1U;
	dc_current_fake.has_last_duty = false;

	zassert_equal(motor_enable(motor), 0, NULL);
	zassert_equal(motor_algo_dc_current_set_current(motor, 1.0f), 0, NULL);
	motor_test_fake_sync_fire(DEVICE_DT_GET(DT_NODELABEL(dc_current_actuator)));

	/* PI: u = kp * err = 0.5 * 1.0 = 0.5; mapped duty d = (u+1)/2 = 0.75. */
	zassert_true(dc_current_fake.has_last_duty, NULL);
	zassert_equal(dc_current_fake.last_n_duty, 2U, NULL);
	zassert_within(dc_current_fake.last_duty[0], 0.75f, 1e-4f, NULL);
	zassert_within(dc_current_fake.last_duty[1], 0.25f, 1e-4f, NULL);

	zassert_equal(motor_algo_dc_current_get_state(motor, &state), 0, NULL);
	zassert_within(state.i_ref_a, 1.0f, 1e-6f, NULL);
	zassert_within(state.i_meas_a, 0.0f, 1e-6f, NULL);
	zassert_within(state.error_a, 1.0f, 1e-6f, NULL);
	zassert_within(state.duty, 0.75f, 1e-4f, NULL);

	zassert_equal(motor_algo_dc_current_set_current(motor, 500.0f), 0, NULL);
	motor_test_fake_sync_fire(DEVICE_DT_GET(DT_NODELABEL(dc_current_actuator)));
	zassert_within(dc_current_fake.last_duty[0], 1.0f, 1e-4f, NULL);
	zassert_within(dc_current_fake.last_duty[1], 0.0f, 1e-4f, NULL);

	motor_estop(motor);
}
