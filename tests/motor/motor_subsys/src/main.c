/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/drivers/motor/motor_sensor.h>
#include <zephyr/subsys/motor/motor_algo_dc_torque.h>
#include <zephyr/subsys/motor/motor.h>
#include <zephyr/subsys/motor/motor_subsys.h>
#include <zephyr/ztest.h>

/* --- Fake sensor/actuator (same pattern as tests/motor/pipeline) --- */

static struct motor_sensor_output fake_out_a;
static struct motor_sensor_output fake_out_b;

static int fs_init(const struct device *dev, const struct motor_sensor_cal *cal)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cal);
	return 0;
}

static int fs_update(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int fs_get_a(const struct device *dev, struct motor_sensor_output *out)
{
	ARG_UNUSED(dev);
	*out = fake_out_a;
	return 0;
}

static int fs_get_b(const struct device *dev, struct motor_sensor_output *out)
{
	ARG_UNUSED(dev);
	*out = fake_out_b;
	return 0;
}

static int fs_cal(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static void fs_types(const struct device *dev, enum motor_pos_backend_type *pos_type,
		     enum motor_cur_backend_type *cur_type)
{
	ARG_UNUSED(dev);
	if (pos_type != NULL) {
		*pos_type = MOTOR_POS_NONE;
	}
	if (cur_type != NULL) {
		*cur_type = MOTOR_CUR_1SHUNT_DCLINK;
	}
}

static const struct motor_sensor_ops fake_sensor_ops_a = {
	.init = fs_init,
	.update = fs_update,
	.get = fs_get_a,
	.calibrate = fs_cal,
	.get_backend_types = fs_types,
};

static const struct motor_sensor_ops fake_sensor_ops_b = {
	.init = fs_init,
	.update = fs_update,
	.get = fs_get_b,
	.calibrate = fs_cal,
	.get_backend_types = fs_types,
};

static struct {
	int dummy;
} fs_data_a, fs_data_b;

static motor_control_cb_t g_cb_a;
static void *g_ud_a;
static motor_control_cb_t g_cb_b;
static void *g_ud_b;

static int fa_dev_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int fa_enable(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int fa_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int fa_cmd(const struct device *dev, const struct motor_actuator_cmd *cmd)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cmd);
	return 0;
}

static int fa_duty(const struct device *dev, const float *duty, uint8_t n)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(duty);
	ARG_UNUSED(n);
	return -ENOTSUP;
}

static int fa_mode(const struct device *dev, enum motor_drive_mode mode)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(mode);
	return 0;
}

static int fa_set_cb_a(const struct device *dev, motor_control_cb_t cb, void *user_data)
{
	ARG_UNUSED(dev);
	g_cb_a = cb;
	g_ud_a = user_data;
	return 0;
}

static int fa_set_cb_b(const struct device *dev, motor_control_cb_t cb, void *user_data)
{
	ARG_UNUSED(dev);
	g_cb_b = cb;
	g_ud_b = user_data;
	return 0;
}

static int fa_fault_cb(const struct device *dev, motor_fault_cb_t cb, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(user_data);
	return 0;
}

static int fa_clr_fault(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int fa_get_fault(const struct device *dev, uint32_t *flags)
{
	ARG_UNUSED(dev);
	if (flags != NULL) {
		*flags = 0U;
	}
	return 0;
}

static int fa_self_test(const struct device *dev, uint32_t *flags)
{
	ARG_UNUSED(dev);
	if (flags != NULL) {
		*flags = 0U;
	}
	return 0;
}

static const struct motor_stage_config fake_cfg = {
	.topology = MOTOR_STAGE_FULL_BRIDGE,
	.n_phases = 1U,
	.pwm_period_ns = 50000U,
	.deadtime_rising_ns = 0U,
	.deadtime_falling_ns = 0U,
	.v_bus_nominal = 12.0f,
	.v_bus_ov_thresh = 30.0f,
	.v_bus_uv_thresh = 6.0f,
	.i_peak_limit = 20.0f,
};

static const struct motor_stage_config *fa_cfg(const struct device *dev)
{
	ARG_UNUSED(dev);
	return &fake_cfg;
}

static int fa_sto_arm(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -ENOTSUP;
}

static int fa_sto_rel(const struct device *dev, uint32_t *flags)
{
	ARG_UNUSED(dev);
	if (flags != NULL) {
		*flags = 0U;
	}
	return -ENOTSUP;
}

static void fa_invoke_a(const struct device *dev)
{
	if (g_cb_a != NULL) {
		g_cb_a(dev, g_ud_a);
	}
}

static void fa_invoke_b(const struct device *dev)
{
	if (g_cb_b != NULL) {
		g_cb_b(dev, g_ud_b);
	}
}

static const struct motor_actuator_ops fake_actuator_ops_a = {
	.init = fa_dev_init,
	.enable = fa_enable,
	.disable = fa_disable,
	.set_command = fa_cmd,
	.set_duty = fa_duty,
	.set_drive_mode = fa_mode,
	.set_control_callback = fa_set_cb_a,
	.set_fault_callback = fa_fault_cb,
	.clear_fault = fa_clr_fault,
	.get_fault = fa_get_fault,
	.self_test = fa_self_test,
	.get_config = fa_cfg,
	.sto_arm = fa_sto_arm,
	.sto_release = fa_sto_rel,
	.invoke_control_callback = fa_invoke_a,
};

static const struct motor_actuator_ops fake_actuator_ops_b = {
	.init = fa_dev_init,
	.enable = fa_enable,
	.disable = fa_disable,
	.set_command = fa_cmd,
	.set_duty = fa_duty,
	.set_drive_mode = fa_mode,
	.set_control_callback = fa_set_cb_b,
	.set_fault_callback = fa_fault_cb,
	.clear_fault = fa_clr_fault,
	.get_fault = fa_get_fault,
	.self_test = fa_self_test,
	.get_config = fa_cfg,
	.sto_arm = fa_sto_arm,
	.sto_release = fa_sto_rel,
	.invoke_control_callback = fa_invoke_b,
};

static struct {
	int dummy;
} fa_data_a, fa_data_b;

DEVICE_DEFINE(fake_motor_sensor_a, "fake_motor_sensor_a", NULL, NULL, &fs_data_a, NULL, POST_KERNEL,
	      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &fake_sensor_ops_a);

DEVICE_DEFINE(fake_motor_actuator_a, "fake_motor_actuator_a", fa_dev_init, NULL, &fa_data_a, NULL,
	      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &fake_actuator_ops_a);

DEVICE_DEFINE(fake_motor_sensor_b, "fake_motor_sensor_b", NULL, NULL, &fs_data_b, NULL, POST_KERNEL,
	      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &fake_sensor_ops_b);

DEVICE_DEFINE(fake_motor_actuator_b, "fake_motor_actuator_b", fa_dev_init, NULL, &fa_data_b, NULL,
	      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &fake_actuator_ops_b);

/* --- Controllers / params --- */

static struct motor_ctrl ctrl_a;
static struct motor_ctrl ctrl_b;
static struct motor_algo_dc_torque_data algo_a = {
	.current_loop =
		{
			.kp = 0.5f,
			.ki = 2000.0f,
			.out_min = -1.0f,
			.out_max = 1.0f,
		},
};
static struct motor_algo_dc_torque_data algo_b = {
	.current_loop =
		{
			.kp = 0.5f,
			.ki = 2000.0f,
			.out_min = -1.0f,
			.out_max = 1.0f,
		},
};

static const struct motor_ctrl_params test_params = {
	.limits = {.i_max_a = 5.0f},
	.timing = {.control_loop_dt_s = 1.0f / 20000.0f},
	.kt_nm_per_a = 0.05f,
	.pole_pairs = 1U,
};

static motor_t make_motor_a(void)
{
	const struct device *s = DEVICE_GET(fake_motor_sensor_a);
	const struct device *a = DEVICE_GET(fake_motor_actuator_a);

	memset(&fake_out_a, 0, sizeof(fake_out_a));
	fake_out_a.hot.i_phase[0] = 0.0f;
	fake_out_a.hot.valid_current = true;

	return motor_init(&ctrl_a, s, a, &motor_algo_dc_torque, &algo_a, &test_params);
}

static motor_t make_motor_b(void)
{
	const struct device *s = DEVICE_GET(fake_motor_sensor_b);
	const struct device *act = DEVICE_GET(fake_motor_actuator_b);

	memset(&fake_out_b, 0, sizeof(fake_out_b));
	fake_out_b.hot.i_phase[0] = 0.0f;
	fake_out_b.hot.valid_current = true;

	return motor_init(&ctrl_b, s, act, &motor_algo_dc_torque, &algo_b, &test_params);
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

	zassert_equal(motor_group_set_torque(&g_life, (float[]){0.01f}), -ENOEXEC);

	zassert_equal(motor_group_enable(&g_life, K_SECONDS(2)), 0, NULL);
	zassert_equal(motor_group_set_torque(&g_life, (float[]){0.01f}), 0, NULL);
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
	float tau[2] = {0.01f, 0.02f};

	zassert_not_null(ma, NULL);
	zassert_not_null(mb, NULL);

	members[0] = ma;
	members[1] = mb;

	zassert_equal(motor_group_add(&g_pair, members, 2, MOTOR_GROUP_FAULT_DISABLE_ALL), 0);

	zassert_equal(motor_group_enable(&g_pair, K_SECONDS(2)), 0, NULL);
	zassert_equal(motor_group_set_torque(&g_pair, tau), 0, NULL);
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
