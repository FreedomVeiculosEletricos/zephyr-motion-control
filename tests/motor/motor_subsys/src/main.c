/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/subsys/motor/motor.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/subsys/motor/motor_ctrl_priv.h>
#include <zephyr/subsys/motor/algorithms/dc_current/motor_algo_dc_current.h>
#include <zephyr/subsys/motor/motor_subsys.h>
#include <zephyr/ztest.h>

#include "motor_test_fake.h"

MOTOR_TEST_FAKE_DEFINE(motor_a);
MOTOR_TEST_FAKE_DEFINE(motor_b);

static struct motor_ctrl ctrl_a;
static struct motor_ctrl ctrl_b;
static struct motor_algo_dc_current_data algo_a = {
	MOTOR_ALGO_DC_CURRENT_BASE_INITIALIZER
	.pi = {
		.kp = 0.5f,
		.ki = 2000.0f,
		.out_min = -1.0f,
		.out_max = 1.0f,
	},
	.limits = {
		.i_max_a = 5.0f,
		.vbus_derating_start = 0.0f,
		.temp_derating_start = 0.0f,
		.temp_fault = 150.0f,
	},
	.timing = {.control_loop_dt_s = 1.0f / 20000.0f},
};
static struct motor_algo_dc_current_data algo_b = {
	MOTOR_ALGO_DC_CURRENT_BASE_INITIALIZER
	.pi = {
		.kp = 0.5f,
		.ki = 2000.0f,
		.out_min = -1.0f,
		.out_max = 1.0f,
	},
	.limits = {
		.i_max_a = 5.0f,
		.vbus_derating_start = 0.0f,
		.temp_derating_start = 0.0f,
		.temp_fault = 150.0f,
	},
	.timing = {.control_loop_dt_s = 1.0f / 20000.0f},
};
static struct motor_block *const blocks_a[] = { &algo_a.base };
static struct motor_block *const blocks_b[] = { &algo_b.base };
static struct motor_pipeline pl_a = {
	.name = "subsys_a",
	.blocks = blocks_a,
	.n_blocks = 1,
};
static struct motor_pipeline pl_b = {
	.name = "subsys_b",
	.blocks = blocks_b,
	.n_blocks = 1,
};

static motor_t make_motor_a(void)
{
	motor_a.sense.hot.i_phase[0] = 0.0f;
	motor_a.sense.hot.valid_current = true;

	zassert_equal(motor_ctrl_init(&ctrl_a, DEVICE_GET(motor_a_sensor), DEVICE_GET(motor_a_actuator),
				    &pl_a, &algo_a, 20000U),
		      0);
	return &ctrl_a;
}

static motor_t make_motor_b(void)
{
	motor_b.sense.hot.i_phase[0] = 0.0f;
	motor_b.sense.hot.valid_current = true;

	zassert_equal(motor_ctrl_init(&ctrl_b, DEVICE_GET(motor_b_sensor), DEVICE_GET(motor_b_actuator),
				    &pl_b, &algo_b, 20000U),
		      0);
	return &ctrl_b;
}

MOTOR_GROUP_DEFINE(g_null, "g_null");
MOTOR_GROUP_DEFINE(g_twice, "g_twice");
MOTOR_GROUP_DEFINE(g_life, "g_life");
MOTOR_GROUP_DEFINE(g_async, "g_async");
MOTOR_GROUP_DEFINE(g_pair, "g_pair");
MOTOR_GROUP_DEFINE(g_empty, "g_empty");

static K_SEM_DEFINE(async_sem, 0, 1);
static enum motor_group_state async_seen_state;
static uint32_t async_seen_fault_mask;

static void group_async_handler(struct motor_group *group, enum motor_group_state state,
				uint32_t fault_mask, void *user_data)
{
	ARG_UNUSED(group);
	ARG_UNUSED(user_data);

	async_seen_state = state;
	async_seen_fault_mask = fault_mask;
	k_sem_give(&async_sem);
}

static void *suite_setup(void)
{
	zassert_equal(motor_subsys_init(), 0, NULL);
	return NULL;
}

ZTEST_SUITE(motor_subsys_suite, NULL, suite_setup, NULL, NULL, NULL);

ZTEST(motor_subsys_suite, test_motor_subsys_init_idempotent)
{
	zassert_equal(motor_subsys_init(), 0, NULL);
}

ZTEST(motor_subsys_suite, test_motor_subsys_empty_registry)
{
	zassert_equal(motor_subsys_count(), 0U, NULL);
	zassert_is_null(motor_subsys_get_by_label("none"), NULL);
	zassert_is_null(motor_subsys_get_by_index(0), NULL);
}

ZTEST(motor_subsys_suite, test_motor_subsys_get_by_label_null)
{
	zassert_is_null(motor_subsys_get_by_label(NULL), NULL);
}

ZTEST(motor_subsys_suite, test_motor_group_null_and_bad_args)
{
	motor_t m = make_motor_a();

	zassert_not_null(m, NULL);

	zassert_equal(motor_group_add(NULL, &m, 1, MOTOR_GROUP_FAULT_ESTOP_ALL), -EINVAL);
	zassert_equal(motor_group_add(&g_null, NULL, 1, MOTOR_GROUP_FAULT_ESTOP_ALL), -EINVAL);
	zassert_equal(motor_group_add(&g_null, &m, 0, MOTOR_GROUP_FAULT_ESTOP_ALL), -EINVAL);

	motor_estop(m);
}

ZTEST(motor_subsys_suite, test_motor_group_add_twice_busy)
{
	motor_t m = make_motor_a();

	zassert_not_null(m, NULL);
	zassert_equal(motor_group_add(&g_twice, &m, 1, MOTOR_GROUP_FAULT_ESTOP_ALL), 0);
	zassert_equal(motor_group_add(&g_twice, &m, 1, MOTOR_GROUP_FAULT_ESTOP_ALL), -EBUSY);

	motor_estop(m);
}

ZTEST(motor_subsys_suite, test_motor_group_self_test_bad_args)
{
	uint32_t faults = 0U;

	zassert_equal(motor_group_self_test(NULL, &faults), -EINVAL);
	zassert_equal(motor_group_self_test(&g_empty, NULL), -EINVAL);
}

ZTEST(motor_subsys_suite, test_motor_group_single_lifecycle)
{
	motor_t m = make_motor_a();
	uint32_t faults[1];
	enum motor_group_state gst;
	uint32_t fm;

	zassert_not_null(m, NULL);
	zassert_equal(motor_group_add(&g_life, &m, 1, MOTOR_GROUP_FAULT_ESTOP_ALL), 0);
	zassert_equal(motor_group_self_test(&g_life, faults), 0);

	motor_group_get_status(&g_life, &gst, &fm);
	zassert_equal(gst, MOTOR_GROUP_IDLE, NULL);
	zassert_equal(fm, 0U, NULL);

	zassert_equal(motor_group_set_current(&g_life, (float[]){0.01f}), -ENOEXEC);

	zassert_equal(motor_group_enable(&g_life, K_SECONDS(2)), 0, NULL);
	zassert_equal(motor_group_set_current(&g_life, (float[]){0.01f}), 0, NULL);
	zassert_equal(motor_group_set_drive_mode(&g_life, MOTOR_DRIVE_NORMAL), 0, NULL);

	zassert_equal(motor_group_disable(&g_life, K_SECONDS(2)), 0, NULL);

	motor_group_get_status(&g_life, &gst, &fm);
	zassert_equal(gst, MOTOR_GROUP_IDLE, NULL);
}

ZTEST(motor_subsys_suite, test_motor_group_enable_async_callback)
{
	motor_t m = make_motor_a();

	zassert_not_null(m, NULL);
	zassert_equal(motor_group_add(&g_async, &m, 1, MOTOR_GROUP_FAULT_ESTOP_ALL), 0);

	async_seen_state = MOTOR_GROUP_IDLE;
	async_seen_fault_mask = 99U;
	k_sem_reset(&async_sem);

	motor_group_register_cb(&g_async, group_async_handler, NULL);
	zassert_equal(motor_group_enable_async(&g_async), 0, NULL);
	zassert_equal(k_sem_take(&async_sem, K_SECONDS(3)), 0, NULL);
	zassert_equal(async_seen_state, MOTOR_GROUP_RUN, NULL);
	zassert_equal(async_seen_fault_mask, 0U, NULL);

	zassert_equal(motor_group_disable(&g_async, K_SECONDS(2)), 0, NULL);
}

ZTEST(motor_subsys_suite, test_motor_group_two_members)
{
	motor_t ma = make_motor_a();
	motor_t mb = make_motor_b();
	motor_t members[2];
	float i_ref[2] = {0.01f, 0.02f};

	zassert_not_null(ma, NULL);
	zassert_not_null(mb, NULL);

	members[0] = ma;
	members[1] = mb;

	zassert_equal(motor_group_add(&g_pair, members, 2, MOTOR_GROUP_FAULT_DISABLE_ALL), 0);

	zassert_equal(motor_group_enable(&g_pair, K_SECONDS(2)), 0, NULL);
	zassert_equal(motor_group_set_current(&g_pair, i_ref), 0, NULL);
	zassert_equal(motor_group_disable(&g_pair, K_SECONDS(2)), 0, NULL);

	motor_estop(ma);
	motor_estop(mb);
}

ZTEST(motor_subsys_suite, test_motor_group_register_cb_null_group)
{
	motor_group_register_cb(NULL, group_async_handler, NULL);
}

ZTEST(motor_subsys_suite, test_motor_group_estop_null)
{
	motor_group_estop(NULL);
}
