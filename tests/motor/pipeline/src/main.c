/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fake sensor + fake actuator: exercise motor_ctrl ISR path (current loop → set_command).
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/subsys/motor/motor_algo_dc_torque.h>
#include <zephyr/drivers/motor/motor_sensor.h>
#include <zephyr/subsys/motor/motor.h>
#include <zephyr/ztest.h>

static struct motor_ctrl ctrl;
static const struct motor_ctrl_params pipeline_params = {
	.limits = {.i_max_a = 5.0f},
	.timing = {.control_loop_dt_s = 1.0f / 20000.0f},
	.kt_nm_per_a = 0.05f,
	.pole_pairs = 1U,
};
static struct motor_algo_dc_torque_data algo_data = {
	.current_loop =
		{
			.kp = 0.5f,
			.ki = 2000.0f,
			.out_min = -1.0f,
			.out_max = 1.0f,
		},
};

static struct motor_sensor_output fake_sensor_out;

static int fake_sensor_init(const struct device *dev, const struct motor_sensor_cal *cal)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cal);

	memset(&fake_sensor_out, 0, sizeof(fake_sensor_out));
	fake_sensor_out.hot.i_phase[0] = 0.0f;
	fake_sensor_out.hot.valid_current = true;
	return 0;
}

static int fake_sensor_update(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int fake_sensor_get(const struct device *dev, struct motor_sensor_output *out)
{
	ARG_UNUSED(dev);
	*out = fake_sensor_out;
	return 0;
}

static int fake_sensor_calibrate(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static void fake_sensor_get_backend_types(const struct device *dev,
					  enum motor_pos_backend_type *pos_type,
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

static const struct motor_sensor_ops fake_sensor_ops = {
	.init = fake_sensor_init,
	.update = fake_sensor_update,
	.get = fake_sensor_get,
	.calibrate = fake_sensor_calibrate,
	.get_backend_types = fake_sensor_get_backend_types,
};

static struct {
	int dummy;
} fake_sensor_drv_data;

static float g_last_valpha;
static float g_last_vbeta;

static motor_control_cb_t g_control_cb;
static void *g_control_cb_user;

static int fake_actuator_device_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int fake_actuator_enable(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int fake_actuator_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int fake_actuator_set_command(const struct device *dev, const struct motor_actuator_cmd *cmd)
{
	ARG_UNUSED(dev);
	if (cmd == NULL) {
		return -EINVAL;
	}
	if (cmd->kind == MOTOR_ACTUATOR_CMD_ALPHA_BETA) {
		g_last_valpha = cmd->u.ab.valpha;
		g_last_vbeta = cmd->u.ab.vbeta;
	} else {
		g_last_valpha = 0.0f;
		g_last_vbeta = 0.0f;
	}
	return 0;
}

static int fake_actuator_set_duty(const struct device *dev, const float *duty, uint8_t n)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(duty);
	ARG_UNUSED(n);
	return -ENOTSUP;
}

static int fake_actuator_set_drive_mode(const struct device *dev, enum motor_drive_mode mode)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(mode);
	return 0;
}

static int fake_actuator_set_control_callback(const struct device *dev, motor_control_cb_t cb,
					      void *user_data)
{
	ARG_UNUSED(dev);
	g_control_cb = cb;
	g_control_cb_user = user_data;
	return 0;
}

static int fake_actuator_set_fault_callback(const struct device *dev, motor_fault_cb_t cb,
					    void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(user_data);
	return 0;
}

static int fake_actuator_clear_fault(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int fake_actuator_get_fault(const struct device *dev, uint32_t *flags)
{
	ARG_UNUSED(dev);
	if (flags != NULL) {
		*flags = 0U;
	}
	return 0;
}

static int fake_actuator_self_test(const struct device *dev, uint32_t *flags)
{
	ARG_UNUSED(dev);
	if (flags != NULL) {
		*flags = 0U;
	}
	return 0;
}

static const struct motor_stage_config fake_stage_cfg = {
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

static const struct motor_stage_config *fake_actuator_get_config(const struct device *dev)
{
	ARG_UNUSED(dev);
	return &fake_stage_cfg;
}

static int fake_actuator_sto_arm(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -ENOTSUP;
}

static int fake_actuator_sto_release(const struct device *dev, uint32_t *flags)
{
	ARG_UNUSED(dev);
	if (flags != NULL) {
		*flags = 0U;
	}
	return -ENOTSUP;
}

static void fake_actuator_invoke_control_callback(const struct device *dev)
{
	if (g_control_cb != NULL) {
		g_control_cb(dev, g_control_cb_user);
	}
}

static const struct motor_actuator_ops fake_actuator_ops = {
	.init = fake_actuator_device_init,
	.enable = fake_actuator_enable,
	.disable = fake_actuator_disable,
	.set_command = fake_actuator_set_command,
	.set_duty = fake_actuator_set_duty,
	.set_drive_mode = fake_actuator_set_drive_mode,
	.set_control_callback = fake_actuator_set_control_callback,
	.set_fault_callback = fake_actuator_set_fault_callback,
	.clear_fault = fake_actuator_clear_fault,
	.get_fault = fake_actuator_get_fault,
	.self_test = fake_actuator_self_test,
	.get_config = fake_actuator_get_config,
	.sto_arm = fake_actuator_sto_arm,
	.sto_release = fake_actuator_sto_release,
	.invoke_control_callback = fake_actuator_invoke_control_callback,
};

static struct {
	int dummy;
} fake_actuator_drv_data;

DEVICE_DEFINE(fake_motor_sensor, "fake_motor_sensor", NULL, NULL, &fake_sensor_drv_data, NULL,
	      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &fake_sensor_ops);

DEVICE_DEFINE(fake_motor_actuator, "fake_motor_actuator", fake_actuator_device_init, NULL,
	      &fake_actuator_drv_data, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
	      &fake_actuator_ops);

ZTEST_SUITE(motor_pipeline_suite, NULL, NULL, NULL, NULL, NULL);

ZTEST(motor_pipeline_suite, test_inner_step_reaches_actuator)
{
	const struct device *sens = DEVICE_GET(fake_motor_sensor);
	const struct device *act = DEVICE_GET(fake_motor_actuator);
	motor_t m;

	g_last_valpha = 0.0f;
	g_last_vbeta = 999.0f;
	g_control_cb = NULL;
	g_control_cb_user = NULL;

	zassert_true(device_is_ready(sens), NULL);
	zassert_true(device_is_ready(act), NULL);

	m = motor_init(&ctrl, sens, act, &motor_algo_dc_torque, &algo_data, &pipeline_params);
	zassert_not_null(m, NULL);

	zassert_equal(motor_enable(m), 0, NULL);

	/* torque_nm / kt_nm_per_a = i_torque_a; default kt = 0.05 → 0.01 Nm → 0.2 A */
	zassert_equal(motor_set_torque(m, 0.01f), 0, NULL);

	motor_actuator_invoke_control_callback(act);

	zassert_within(g_last_vbeta, 0.0f, 1e-5f, NULL);
	/* i_torque_a=0.2, ia=0 → first PI step valpha ≈ 0.12 */
	zassert_within(g_last_valpha, 0.12f, 0.02f, NULL);

	motor_estop(m);
}
