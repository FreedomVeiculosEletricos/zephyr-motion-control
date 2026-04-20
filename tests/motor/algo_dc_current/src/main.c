/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/subsys/motor/algorithms/dc_current/motor_algo_dc_current.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/ztest.h>

ZTEST_SUITE(motor_algo_dc_current_suite, NULL, NULL, NULL, NULL, NULL);

static void run_inner(struct motor_algo_dc_current_data *st,
		      const struct motor_sensor_output *sense,
		      const struct motor_ctrl_setpoints *sp,
		      struct motor_actuator_cmd *cmd)
{
	struct motor_block_in in = {
		.sense = sense,
		.sp = sp,
		.algo = st,
	};
	struct motor_block_out out = {
		.sp = (struct motor_ctrl_setpoints *)sp,
		.cmd = cmd,
	};

	motor_dc_current_block_entry(&st->base, &in, &out);
}

#define DC_TEST_COMMON_FIELDS                                                                      \
	.limits =                                                                                  \
		{                                                                                  \
			.i_max_a = 5.0f,                                                           \
			.speed_max_rad_s = 100.0f,                                                \
			.vbus_derating_start = 0.0f,                                             \
			.temp_derating_start = 0.0f,                                             \
			.temp_fault = 150.0f,                                                    \
		},                                                                                 \
	.timing = {.control_loop_dt_s = 1.0f / 20000.0f}, .kt_nm_per_a = 0.05f, .pole_pairs = 1U,

ZTEST(motor_algo_dc_current_suite, test_pi_step_zero_current)
{
	struct motor_algo_dc_current_data st = {
		MOTOR_ALGO_DC_CURRENT_BASE_INITIALIZER
		.pi =
			{
				.kp = 0.5f,
				.ki = 2000.0f,
				.out_min = -1.0f,
				.out_max = 1.0f,
			},
		DC_TEST_COMMON_FIELDS
	};
	struct motor_sensor_output sense;
	struct motor_ctrl_setpoints sp;
	struct motor_actuator_cmd cmd;

	memset(&sense, 0, sizeof(sense));
	sense.hot.i_phase[0] = 0.0f;
	sp.i_torque_a = 1.0f;
	sp.i_flux_a = 0.0f;

	zassert_equal(motor_dc_current_block_init(&st.base), 0);

	run_inner(&st, &sense, &sp, &cmd);

	zassert_equal(cmd.kind, MOTOR_ACTUATOR_CMD_ALPHA_BETA, NULL);
	zassert_within(cmd.u.ab.valpha, 0.6f, 1e-4f, NULL);
	zassert_within(cmd.u.ab.vbeta, 0.0f, 1e-6f, NULL);
}

ZTEST(motor_algo_dc_current_suite, test_output_saturates_high)
{
	struct motor_algo_dc_current_data st = {
		MOTOR_ALGO_DC_CURRENT_BASE_INITIALIZER
		.pi =
			{
				.kp = 0.5f,
				.ki = 2000.0f,
				.out_min = -1.0f,
				.out_max = 1.0f,
			},
		DC_TEST_COMMON_FIELDS
	};
	struct motor_sensor_output sense;
	struct motor_ctrl_setpoints sp;
	struct motor_actuator_cmd cmd;

	memset(&sense, 0, sizeof(sense));
	sense.hot.i_phase[0] = 0.0f;
	sp.i_torque_a = 500.0f;

	zassert_equal(motor_dc_current_block_init(&st.base), 0);

	run_inner(&st, &sense, &sp, &cmd);

	zassert_equal(cmd.kind, MOTOR_ACTUATOR_CMD_ALPHA_BETA, NULL);
	zassert_within(cmd.u.ab.valpha, 1.0f, 1e-4f, NULL);
}

ZTEST(motor_algo_dc_current_suite, test_reset_clears_integral)
{
	struct motor_algo_dc_current_data st = {
		MOTOR_ALGO_DC_CURRENT_BASE_INITIALIZER
		.pi =
			{
				.kp = 0.5f,
				.ki = 2000.0f,
				.out_min = -1.0f,
				.out_max = 1.0f,
			},
		DC_TEST_COMMON_FIELDS
	};
	struct motor_sensor_output sense;
	struct motor_ctrl_setpoints sp;
	struct motor_actuator_cmd cmd;

	memset(&sense, 0, sizeof(sense));
	sense.hot.i_phase[0] = 0.0f;
	sp.i_torque_a = 1.0f;

	zassert_equal(motor_dc_current_block_init(&st.base), 0);
	run_inner(&st, &sense, &sp, &cmd);
	zassert_true(cmd.u.ab.valpha > 0.5f, NULL);

	motor_dc_current_block_reset(&st.base);
	run_inner(&st, &sense, &sp, &cmd);

	zassert_equal(cmd.kind, MOTOR_ACTUATOR_CMD_ALPHA_BETA, NULL);
	zassert_within(cmd.u.ab.valpha, 0.6f, 1e-4f, NULL);
}
