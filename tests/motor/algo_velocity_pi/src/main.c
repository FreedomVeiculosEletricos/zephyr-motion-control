/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/subsys/motor/algorithms/velocity_pi/motor_algo_velocity_pi.h>
#include <zephyr/subsys/motor/motor.h>
#include <zephyr/subsys/motor/motor_subsys.h>
#include <zephyr/ztest.h>

#include "motor_algo_velocity_pi_priv.h"
#include "motor_test_fake.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static struct motor_test_fake velocity_pi_fake;

DEVICE_DT_DEFINE(DT_NODELABEL(velocity_pi_sensor), motor_test_fake_sensor_init, NULL,
		 &velocity_pi_fake, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		 &motor_test_fake_sensor_ops);
DEVICE_DT_DEFINE(DT_NODELABEL(velocity_pi_actuator), motor_test_fake_actuator_init, NULL,
		 &velocity_pi_fake, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		 &motor_test_fake_actuator_ops);

static void *suite_setup(void)
{
	zassert_equal(motor_subsys_init(), 0, NULL);
	return NULL;
}

ZTEST_SUITE(motor_algo_velocity_pi_suite, NULL, suite_setup, NULL, NULL, NULL);

ZTEST(motor_algo_velocity_pi_suite, test_rejects_invalid_limits)
{
	motor_t motor = motor_subsys_get_by_label("velocity_pi_motor");
	struct motor_velocity_pi_limits bad = {
		.i_max_a = 0.0f,
	};

	zassert_not_null(motor, NULL);
	zassert_equal(motor_enable(motor), 0, NULL);
	zassert_equal(motor_algo_velocity_pi_set_limits(motor, &bad), -EINVAL, NULL);
	motor_estop(motor);
}

ZTEST(motor_algo_velocity_pi_suite, test_set_velocity_and_status)
{
	motor_t motor = motor_subsys_get_by_label("velocity_pi_motor");
	struct motor_velocity_pi_state state;

	zassert_not_null(motor, NULL);
	zassert_equal(motor_enable(motor), 0, NULL);
	zassert_equal(motor_algo_velocity_pi_set_velocity(motor, 10.0f), 0, NULL);
	zassert_equal(motor_algo_velocity_pi_get_state(motor, &state), 0, NULL);
	zassert_within(state.v_ref_rad_s, 10.0f, 1e-6f, NULL);
	motor_estop(motor);
}

ZTEST(motor_algo_velocity_pi_suite, test_pll_tracks_constant_speed_angle)
{
	struct motor_algo_velocity_pi_data vel = {
		MOTOR_ALGO_VELOCITY_PI_BASE_INITIALIZER
		.state = {0},
		.params =
			{
				.kp = 0.1f,
				.ki = 0.0f,
				.pll_kp = 40.0f,
				.pll_ki = 400.0f,
			},
		.limits = {.i_max_a = 3.0f},
		.timing = {.control_loop_dt_s = 10.0f / 20000.0f},
	};
	struct motor_sense_bundle sense = {0};
	struct motor_block_in in = {.sense = &sense};
	struct motor_block_out out = {0};
	const float omega = 5.0f;
	const float dt = vel.timing.control_loop_dt_s;
	float theta = 0.0f;

	motor_velocity_pi_block_set_params(&vel.base);
	vel.state.v_ref_rad_s = omega;

	for (int i = 0; i < 200; i++) {
		theta += omega * dt;
		while (theta > (float)M_PI) {
			theta -= 2.0f * (float)M_PI;
		}
		sense.angle_rad = theta;
		sense.angle_valid = true;
		motor_velocity_pi_block_entry(&vel.base, &in, &out);
	}

	zassert_true(out.has_current_ref, NULL);
	zassert_within(vel.state.omega_hat_rad_s, omega, 1.0f, NULL);
}

static struct motor_algo_velocity_pi_data make_open_loop_startup_vel(float startup_a)
{
	struct motor_algo_velocity_pi_data vel = {
		MOTOR_ALGO_VELOCITY_PI_BASE_INITIALIZER
		.state = {0},
		.params =
			{
				.kp = 0.1f,
				.ki = 1.0f,
				.pll_kp = 40.0f,
				.pll_ki = 400.0f,
			},
		.limits = {.i_max_a = 3.0f, .open_loop_startup_a = startup_a},
		.timing = {.control_loop_dt_s = 0.001f},
	};

	motor_velocity_pi_block_set_params(&vel.base);

	return vel;
}

ZTEST(motor_algo_velocity_pi_suite, test_open_loop_startup_follows_command_sign)
{
	struct motor_algo_velocity_pi_data vel = make_open_loop_startup_vel(0.8f);
	struct motor_sense_bundle sense = {.angle_valid = false};
	struct motor_block_in in = {.sense = &sense};
	struct motor_block_out out = {0};

	vel.state.v_ref_rad_s = 50.0f;
	motor_velocity_pi_block_entry(&vel.base, &in, &out);
	zassert_true(out.has_current_ref, NULL);
	zassert_within(out.current_ref_a, 0.8f, 1e-6f, NULL);

	vel.state.v_ref_rad_s = -50.0f;
	motor_velocity_pi_block_entry(&vel.base, &in, &out);
	zassert_within(out.current_ref_a, -0.8f, 1e-6f, NULL);

	vel.state.v_ref_rad_s = 0.0f;
	motor_velocity_pi_block_entry(&vel.base, &in, &out);
	zassert_within(out.current_ref_a, 0.0f, 1e-6f, "a zero command must not push current");
}

ZTEST(motor_algo_velocity_pi_suite, test_open_loop_startup_hands_over_to_closed_loop)
{
	struct motor_algo_velocity_pi_data vel = make_open_loop_startup_vel(0.8f);
	struct motor_sense_bundle sense = {.angle_valid = false};
	struct motor_block_in in = {.sense = &sense};
	struct motor_block_out out = {0};

	vel.state.v_ref_rad_s = 50.0f;
	for (int i = 0; i < 100; i++) {
		motor_velocity_pi_block_entry(&vel.base, &in, &out);
	}
	zassert_within(vel.i_integral, 0.0f, 1e-6f, "the open loop must not wind up");

	sense.angle_valid = true;
	sense.angle_rad = 0.1f;
	motor_velocity_pi_block_entry(&vel.base, &in, &out);

	zassert_true(vel.pll_primed, "the PLL must re-prime on the first valid angle");
	zassert_within(vel.state.theta_hat_rad, 0.1f, 1e-6f, NULL);
}

ZTEST(motor_algo_velocity_pi_suite, test_without_startup_current_reference_is_held)
{
	struct motor_algo_velocity_pi_data vel = make_open_loop_startup_vel(0.0f);
	struct motor_sense_bundle sense = {.angle_valid = false};
	struct motor_block_in in = {.sense = &sense};
	struct motor_block_out out = {0};

	vel.state.i_ref_a = 1.25f;
	vel.state.v_ref_rad_s = 50.0f;

	motor_velocity_pi_block_entry(&vel.base, &in, &out);

	zassert_within(out.current_ref_a, 1.25f, 1e-6f, NULL);
}

ZTEST(motor_algo_velocity_pi_suite, test_rejects_startup_current_above_limit)
{
	motor_t motor = motor_subsys_get_by_label("velocity_pi_motor");
	struct motor_velocity_pi_limits limits = {
		.i_max_a = 3.0f,
		.open_loop_startup_a = 3.5f,
	};

	zassert_not_null(motor, NULL);
	zassert_equal(motor_algo_velocity_pi_set_limits(motor, &limits), -EINVAL, NULL);

	limits.open_loop_startup_a = -0.1f;
	zassert_equal(motor_algo_velocity_pi_set_limits(motor, &limits), -EINVAL, NULL);

	limits.open_loop_startup_a = 1.0f;
	zassert_equal(motor_algo_velocity_pi_set_limits(motor, &limits), 0, NULL);
}

ZTEST(motor_algo_velocity_pi_suite, test_pi_saturates_current_ref)
{
	struct motor_algo_velocity_pi_data vel = {
		MOTOR_ALGO_VELOCITY_PI_BASE_INITIALIZER
		.state = {0},
		.params =
			{
				.kp = 10.0f,
				.ki = 0.0f,
				.pll_kp = 0.0f,
				.pll_ki = 0.0f,
			},
		.limits = {.i_max_a = 1.5f},
		.timing = {.control_loop_dt_s = 0.001f},
		.pll_primed = true,
	};
	struct motor_sense_bundle sense = {
		.angle_rad = 0.0f,
		.angle_valid = true,
	};
	struct motor_block_in in = {.sense = &sense};
	struct motor_block_out out = {0};

	motor_velocity_pi_block_set_params(&vel.base);
	vel.state.v_ref_rad_s = 100.0f;
	vel.state.theta_hat_rad = 0.0f;
	vel.state.omega_hat_rad_s = 0.0f;

	motor_velocity_pi_block_entry(&vel.base, &in, &out);

	zassert_true(out.has_current_ref, NULL);
	zassert_within(out.current_ref_a, 1.5f, 1e-4f, NULL);
}
