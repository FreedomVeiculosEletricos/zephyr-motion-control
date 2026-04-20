/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/subsys/motor/motor_algo_dc_torque.h>
#include <zephyr/ztest.h>

static const struct motor_ctrl_params test_ctrl_params = {
	.limits = {.i_max_a = 5.0f},
	.timing = {.control_loop_dt_s = 1.0f / 20000.0f},
	.kt_nm_per_a = 0.05f,
	.pole_pairs = 1U,
};

ZTEST_SUITE(motor_algo_dc_torque_suite, NULL, NULL, NULL, NULL, NULL);

static void run_inner(struct motor_algo_dc_torque_data *st,
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

	motor_algo_dc_torque.inner_step(st, &in, &out);
}

/* Default gains: kp=0.5, ki=2000, dt=1/20000, out [-1,1] — explicit (no DT in unit test). */
ZTEST(motor_algo_dc_torque_suite, test_pi_step_zero_current)
{
	struct motor_algo_dc_torque_data st;
	struct motor_sensor_output sense;
	struct motor_ctrl_setpoints sp;
	struct motor_actuator_cmd cmd;

	memset(&st, 0, sizeof(st));
	st.current_loop.kp = 0.5f;
	st.current_loop.ki = 2000.0f;
	st.current_loop.out_min = -1.0f;
	st.current_loop.out_max = 1.0f;

	memset(&sense, 0, sizeof(sense));
	sense.hot.i_phase[0] = 0.0f;
	sp.i_torque_a = 1.0f;
	sp.i_flux_a = 0.0f;

	zassert_equal(motor_algo_dc_torque.init(&st, &test_ctrl_params), 0);

	run_inner(&st, &sense, &sp, &cmd);

	zassert_equal(cmd.kind, MOTOR_ACTUATOR_CMD_ALPHA_BETA, NULL);
	/* First step: int += 2000 * 1 * (1/20000) = 0.1; u = 0.5*1 + 0.1 = 0.6 */
	zassert_within(cmd.u.ab.valpha, 0.6f, 1e-4f, NULL);
	zassert_within(cmd.u.ab.vbeta, 0.0f, 1e-6f, NULL);
}

ZTEST(motor_algo_dc_torque_suite, test_output_saturates_high)
{
	struct motor_algo_dc_torque_data st;
	struct motor_sensor_output sense;
	struct motor_ctrl_setpoints sp;
	struct motor_actuator_cmd cmd;

	memset(&st, 0, sizeof(st));
	st.current_loop.kp = 0.5f;
	st.current_loop.ki = 2000.0f;
	st.current_loop.out_min = -1.0f;
	st.current_loop.out_max = 1.0f;

	memset(&sense, 0, sizeof(sense));
	sense.hot.i_phase[0] = 0.0f;
	sp.i_torque_a = 500.0f;

	zassert_equal(motor_algo_dc_torque.init(&st, &test_ctrl_params), 0);

	run_inner(&st, &sense, &sp, &cmd);

	zassert_equal(cmd.kind, MOTOR_ACTUATOR_CMD_ALPHA_BETA, NULL);
	zassert_within(cmd.u.ab.valpha, 1.0f, 1e-4f, NULL);
}

ZTEST(motor_algo_dc_torque_suite, test_reset_clears_integral)
{
	struct motor_algo_dc_torque_data st;
	struct motor_sensor_output sense;
	struct motor_ctrl_setpoints sp;
	struct motor_actuator_cmd cmd;

	memset(&st, 0, sizeof(st));
	st.current_loop.kp = 0.5f;
	st.current_loop.ki = 2000.0f;
	st.current_loop.out_min = -1.0f;
	st.current_loop.out_max = 1.0f;

	memset(&sense, 0, sizeof(sense));
	sense.hot.i_phase[0] = 0.0f;
	sp.i_torque_a = 1.0f;

	zassert_equal(motor_algo_dc_torque.init(&st, &test_ctrl_params), 0);
	run_inner(&st, &sense, &sp, &cmd);
	zassert_true(cmd.u.ab.valpha > 0.5f, NULL);

	motor_algo_dc_torque.reset(&st);
	run_inner(&st, &sense, &sp, &cmd);

	zassert_equal(cmd.kind, MOTOR_ACTUATOR_CMD_ALPHA_BETA, NULL);
	/* Same as first step after reset */
	zassert_within(cmd.u.ab.valpha, 0.6f, 1e-4f, NULL);
}
