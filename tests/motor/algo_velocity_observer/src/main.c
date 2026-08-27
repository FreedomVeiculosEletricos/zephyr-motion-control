/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/motor/motor_encoder_observer.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/subsys/motor/algorithms/velocity_observer/motor_algo_velocity_observer.h>
#include <zephyr/subsys/motor/motor.h>
#include <zephyr/subsys/motor/motor_subsys.h>
#include <zephyr/ztest.h>

#include "motor_algo_velocity_observer_priv.h"
#include "motor_test_fake.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TEST_DT_S (1.0f / 20000.0f)
#define TEST_R_OHM 1.0f
#define TEST_L_H 0.001f
#define TEST_KE 0.05f
#define TEST_VBUS 12.0f

static struct motor_test_fake observer_fake;

DEVICE_DT_DEFINE(DT_NODELABEL(observer_sensor), motor_test_fake_sensor_init, NULL, &observer_fake,
		 NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		 &motor_test_fake_sensor_ops);
DEVICE_DT_DEFINE(DT_NODELABEL(observer_actuator), motor_test_fake_actuator_init, NULL,
		 &observer_fake, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		 &motor_test_fake_actuator_ops);

static const struct device *const virtual_encoder = DEVICE_DT_GET(DT_NODELABEL(observer_encoder));

static struct motor_algo_velocity_observer_data make_observer(void)
{
	struct motor_algo_velocity_observer_data obs = {
		MOTOR_ALGO_VELOCITY_OBSERVER_BASE_INITIALIZER
		.state = {0},
		.params =
			{
				.r_ohm = TEST_R_OHM,
				.l_h = TEST_L_H,
				.ke_v_per_rad_s = TEST_KE,
				.vbus_v = TEST_VBUS,
				.bandwidth_rad_s = 3000.0f,
				.pll_kp = 0.1f,
				.pll_ki = 200.0f,
			},
		.limits =
			{
				.valid_threshold_rad_s = 20.0f,
				.valid_release_rad_s = 15.0f,
				.settle_ticks = 0U,
			},
		.timing = {.control_loop_dt_s = TEST_DT_S},
		.sink = NULL,
	};

	motor_velocity_observer_block_set_params(&obs.base);

	return obs;
}

/*
 * Drive the observer with the current a real motor would draw while spinning at
 * a fixed speed under a fixed command, integrating the electrical equation with
 * the same step as the control loop.
 */
static void run_dc_plant(struct motor_algo_velocity_observer_data *obs, float omega, float u,
			 int ticks)
{
	struct motor_sense_bundle sense = {.current_count = 1U};
	struct motor_block_in in = {
		.sense = &sense,
		.applied_u = u,
		.has_applied_u = true,
	};
	struct motor_block_out out = {0};
	float i_plant = 0.0f;

	for (int k = 0; k < ticks; k++) {
		sense.current_amps[0] = i_plant;
		motor_velocity_observer_block_entry(&obs->base, &in, &out);
		i_plant += TEST_DT_S *
			   (u * TEST_VBUS - TEST_R_OHM * i_plant - TEST_KE * omega) / TEST_L_H;
	}
}

/*
 * Force a single exact omega through the estimator so the validity band can be
 * exercised on its own: with a settled current the back-EMF state is carried
 * through untouched, and a purely proportional tracking loop starting from zero
 * reproduces it as-is.
 */
static void step_to_omega(struct motor_algo_velocity_observer_data *obs, float omega)
{
	struct motor_sense_bundle sense = {.current_count = 1U, .current_amps = {0.0f}};
	struct motor_block_in in = {
		.sense = &sense,
		.applied_u = 0.0f,
		.has_applied_u = true,
	};
	struct motor_block_out out = {0};

	obs->state.i_hat_a = 0.0f;
	obs->state.emf_hat_v = (omega / obs->params.pll_kp) * TEST_KE;
	obs->state.omega_hat_rad_s = 0.0f;
	obs->pll_integral = 0.0f;

	motor_velocity_observer_block_entry(&obs->base, &in, &out);
}

static void *suite_setup(void)
{
	zassert_equal(motor_subsys_init(), 0, NULL);
	return NULL;
}

ZTEST_SUITE(motor_algo_velocity_observer_suite, NULL, suite_setup, NULL, NULL, NULL);

ZTEST(motor_algo_velocity_observer_suite, test_converges_to_constant_speed)
{
	struct motor_algo_velocity_observer_data obs = make_observer();

	run_dc_plant(&obs, 100.0f, 0.5f, 4000);

	zassert_within(obs.state.omega_hat_rad_s, 100.0f, 2.0f, NULL);
	zassert_true(obs.state.valid, NULL);
}

ZTEST(motor_algo_velocity_observer_suite, test_tracks_speed_step)
{
	struct motor_algo_velocity_observer_data obs = make_observer();

	run_dc_plant(&obs, 100.0f, 0.5f, 4000);
	run_dc_plant(&obs, 250.0f, 1.0f, 4000);

	zassert_within(obs.state.omega_hat_rad_s, 250.0f, 5.0f, NULL);
}

ZTEST(motor_algo_velocity_observer_suite, test_sign_follows_rotation)
{
	struct motor_algo_velocity_observer_data obs = make_observer();

	run_dc_plant(&obs, -100.0f, -0.5f, 4000);

	zassert_within(obs.state.omega_hat_rad_s, -100.0f, 2.0f, NULL);
	zassert_true(obs.state.valid, NULL);
}

ZTEST(motor_algo_velocity_observer_suite, test_validity_hysteresis)
{
	struct motor_algo_velocity_observer_data obs = make_observer();

	obs.params.pll_ki = 0.0f;
	motor_velocity_observer_block_set_params(&obs.base);

	step_to_omega(&obs, 18.0f);
	zassert_false(obs.state.valid, "below the threshold it must stay invalid");

	step_to_omega(&obs, 25.0f);
	zassert_true(obs.state.valid, NULL);

	step_to_omega(&obs, 17.0f);
	zassert_true(obs.state.valid, "inside the band it must latch");

	step_to_omega(&obs, 10.0f);
	zassert_false(obs.state.valid, NULL);
}

ZTEST(motor_algo_velocity_observer_suite, test_settle_ticks_hold_invalid)
{
	struct motor_algo_velocity_observer_data obs = make_observer();

	obs.limits.settle_ticks = 5000U;

	run_dc_plant(&obs, 100.0f, 0.5f, 4000);
	zassert_false(obs.state.valid, "validity must wait out the settle window");

	run_dc_plant(&obs, 100.0f, 0.5f, 1001);
	zassert_true(obs.state.valid, NULL);
}

ZTEST(motor_algo_velocity_observer_suite, test_reset_clears_estimate)
{
	struct motor_algo_velocity_observer_data obs = make_observer();

	run_dc_plant(&obs, 100.0f, 0.5f, 4000);
	zassert_true(obs.state.valid, NULL);

	motor_velocity_observer_block_reset(&obs.base);

	zassert_false(obs.state.valid, NULL);
	zassert_within(obs.state.omega_hat_rad_s, 0.0f, 1e-6f, NULL);
	zassert_within(obs.state.emf_hat_v, 0.0f, 1e-6f, NULL);
}

ZTEST(motor_algo_velocity_observer_suite, test_missing_current_keeps_estimate_invalid)
{
	struct motor_algo_velocity_observer_data obs = make_observer();
	struct motor_sense_bundle empty = {.current_count = 0U};
	struct motor_block_in no_sense = {.sense = NULL};
	struct motor_block_in no_current = {.sense = &empty};
	struct motor_block_out out = {0};

	for (int k = 0; k < 1000; k++) {
		motor_velocity_observer_block_entry(&obs.base, &no_sense, &out);
	}
	zassert_false(obs.state.valid, NULL);

	for (int k = 0; k < 1000; k++) {
		motor_velocity_observer_block_entry(&obs.base, &no_current, &out);
	}
	zassert_false(obs.state.valid, NULL);
}

ZTEST(motor_algo_velocity_observer_suite, test_block_drives_nothing)
{
	struct motor_algo_velocity_observer_data obs = make_observer();
	struct motor_sense_bundle sense = {.current_count = 1U, .current_amps = {1.0f}};
	struct motor_block_in in = {.sense = &sense, .applied_u = 0.5f, .has_applied_u = true};
	struct motor_block_out out = {
		.n_duty = 2U,
		.has_current_ref = true,
		.has_applied_u = true,
	};

	motor_velocity_observer_block_entry(&obs.base, &in, &out);

	zassert_equal(out.n_duty, 0U, NULL);
	zassert_false(out.has_current_ref, NULL);
	zassert_false(out.has_applied_u, NULL);
}

ZTEST(motor_algo_velocity_observer_suite, test_rejects_invalid_params)
{
	motor_t motor = motor_subsys_get_by_label("observer_motor");
	struct motor_velocity_observer_params params;

	zassert_not_null(motor, NULL);
	zassert_equal(motor_algo_velocity_observer_get_params(motor, &params), 0, NULL);
	zassert_within(params.r_ohm, TEST_R_OHM, 1e-6f, NULL);
	zassert_within(params.ke_v_per_rad_s, TEST_KE, 1e-9f, NULL);

	params.l_h = 0.0f;
	zassert_equal(motor_algo_velocity_observer_set_params(motor, &params, false), -EINVAL,
		      NULL);

	params.l_h = TEST_L_H;
	params.ke_v_per_rad_s = 0.0f;
	zassert_equal(motor_algo_velocity_observer_set_params(motor, &params, false), -EINVAL,
		      NULL);

	params.ke_v_per_rad_s = TEST_KE;
	params.bandwidth_rad_s = 0.0f;
	zassert_equal(motor_algo_velocity_observer_set_params(motor, &params, false), -EINVAL,
		      NULL);

	zassert_equal(motor_algo_velocity_observer_set_params(motor, NULL, false), -EINVAL, NULL);
}

ZTEST(motor_algo_velocity_observer_suite, test_rejects_unstable_tracking_gains)
{
	motor_t motor = motor_subsys_get_by_label("observer_motor");
	struct motor_velocity_observer_params params;

	zassert_not_null(motor, NULL);
	zassert_equal(motor_algo_velocity_observer_get_params(motor, &params), 0, NULL);

	params.pll_kp = 1.0f;
	zassert_equal(motor_algo_velocity_observer_set_params(motor, &params, false), -EINVAL,
		      "a unit proportional gain diverges");

	params.pll_kp = 0.1f;
	params.pll_ki = 1.0e6f;
	zassert_equal(motor_algo_velocity_observer_set_params(motor, &params, false), -EINVAL,
		      NULL);
}

ZTEST(motor_algo_velocity_observer_suite, test_rejects_inverted_hysteresis)
{
	motor_t motor = motor_subsys_get_by_label("observer_motor");
	struct motor_velocity_observer_limits limits;

	zassert_not_null(motor, NULL);
	zassert_equal(motor_algo_velocity_observer_get_limits(motor, &limits), 0, NULL);
	zassert_within(limits.valid_threshold_rad_s, 20.0f, 1e-6f, NULL);
	zassert_within(limits.valid_release_rad_s, 15.0f, 1e-6f, NULL);

	limits.valid_release_rad_s = 25.0f;
	zassert_equal(motor_algo_velocity_observer_set_limits(motor, &limits), -EINVAL, NULL);

	limits.valid_release_rad_s = 15.0f;
	limits.valid_threshold_rad_s = 0.0f;
	zassert_equal(motor_algo_velocity_observer_set_limits(motor, &limits), -EINVAL, NULL);
}

ZTEST(motor_algo_velocity_observer_suite, test_virtual_encoder_reports_no_data_while_invalid)
{
	struct sensor_value val;

	zassert_true(device_is_ready(virtual_encoder), NULL);

	motor_encoder_observer_publish(virtual_encoder, 50.0f, 1.0f, false);
	zassert_equal(sensor_sample_fetch(virtual_encoder), -ENODATA, NULL);

	motor_encoder_observer_publish(virtual_encoder, 0.0f, (float)M_PI / 2.0f, true);
	zassert_equal(sensor_sample_fetch(virtual_encoder), 0, NULL);
	zassert_equal(sensor_channel_get(virtual_encoder, SENSOR_CHAN_ROTATION, &val), 0, NULL);
	zassert_within(sensor_value_to_double(&val), 90.0, 0.01, "rotation must be degrees");
}

ZTEST(motor_algo_velocity_observer_suite, test_virtual_encoder_reports_rpm)
{
	struct sensor_value val;

	motor_encoder_observer_publish(virtual_encoder, 2.0f * (float)M_PI, 0.0f, true);
	zassert_equal(sensor_sample_fetch(virtual_encoder), 0, NULL);
	zassert_equal(sensor_channel_get(virtual_encoder, SENSOR_CHAN_RPM, &val), 0, NULL);
	zassert_within(sensor_value_to_double(&val), 60.0, 0.01, NULL);

	zassert_equal(sensor_channel_get(virtual_encoder, SENSOR_CHAN_DIE_TEMP, &val), -ENOTSUP,
		      NULL);
	zassert_equal(sensor_sample_fetch_chan(virtual_encoder, SENSOR_CHAN_DIE_TEMP), -ENOTSUP,
		      NULL);
}

ZTEST(motor_algo_velocity_observer_suite, test_observer_publishes_through_pipeline)
{
	motor_t motor = motor_subsys_get_by_label("observer_motor");
	struct motor_velocity_observer_state state;

	zassert_not_null(motor, NULL);
	zassert_equal(motor_enable(motor), 0, NULL);

	observer_fake.sensor_sample_count = 1U;
	observer_fake.sensor_samples[0] = 1.0f;

	for (int k = 0; k < 200; k++) {
		motor_test_fake_sync_fire(DEVICE_DT_GET(DT_NODELABEL(observer_actuator)));
	}

	zassert_equal(motor_algo_velocity_observer_get_state(motor, &state), 0, NULL);
	/* settle-ticks is 100 in the overlay, so the window is already over. */
	zassert_equal(sensor_sample_fetch(virtual_encoder), state.valid ? 0 : -ENODATA, NULL);

	motor_estop(motor);
	zassert_equal(sensor_sample_fetch(virtual_encoder), -ENODATA,
		      "stopping must invalidate the facade");
}
