/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fake sensor/actuator + non-default motor_algo_ops: API accepts any algorithm
 * vtable; ISR path uses stub inner_step outputs.
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/subsys/motor/motor_controller.h>
#include <zephyr/drivers/motor/motor_sensor.h>
#include <zephyr/subsys/motor/motor.h>
#include <zephyr/ztest.h>

static struct motor_ctrl ctrl;

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

static int stub_algo_init(void *algo_data, const struct motor_ctrl_params *params)
{
	ARG_UNUSED(algo_data);
	ARG_UNUSED(params);

	g_stub_init_calls++;
	return 0;
}

static void stub_algo_inner_step(void *algo_data, const struct motor_sensor_output *sense,
				 const struct motor_ctrl_setpoints *sp, struct motor_actuator_cmd *cmd)
{
	ARG_UNUSED(algo_data);
	ARG_UNUSED(sense);
	ARG_UNUSED(sp);

	cmd->kind = MOTOR_ACTUATOR_CMD_ALPHA_BETA;
	cmd->u.ab.valpha = 1.0f;
	cmd->u.ab.vbeta = 2.0f;
}

static void stub_algo_set_params(void *algo_data, const struct motor_ctrl_params *params)
{
	ARG_UNUSED(algo_data);
	ARG_UNUSED(params);
}

static void stub_algo_reset(void *algo_data)
{
	ARG_UNUSED(algo_data);
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
	const struct device *sens = DEVICE_GET(fake_motor_sensor);
	const struct device *act = DEVICE_GET(fake_motor_actuator);

	g_stub_init_calls = 0;

	zassert_equal(motor_ctrl_init(&ctrl, sens, act, &stub_motor_algo, &stub_algo_data,
				    &stub_motor_params),
		      0, NULL);
	zassert_equal(g_stub_init_calls, 1, NULL);
}

ZTEST(motor_api_extensibility_suite, test_isr_uses_stub_inner_step)
{
	const struct device *sens = DEVICE_GET(fake_motor_sensor);
	const struct device *act = DEVICE_GET(fake_motor_actuator);
	motor_t m;

	g_stub_init_calls = 0;
	g_last_valpha = 0.0f;
	g_last_vbeta = 0.0f;
	g_control_cb = NULL;

	memset(&ctrl, 0, sizeof(ctrl));

	m = motor_init(&ctrl, sens, act, &stub_motor_algo, &stub_algo_data, &stub_motor_params);
	zassert_not_null(m, NULL);
	zassert_equal(g_stub_init_calls, 1, NULL);

	zassert_equal(motor_enable(m), 0, NULL);
	zassert_equal(motor_set_torque(m, 0.05f), 0, NULL);

	motor_actuator_invoke_control_callback(act);

	zassert_within(g_last_valpha, 1.0f, 1e-5f, NULL);
	zassert_within(g_last_vbeta, 2.0f, 1e-5f, NULL);

	motor_estop(m);
}

ZTEST(motor_api_extensibility_suite, test_motor_set_torque_rejects_zero_kt)
{
	const struct device *sens = DEVICE_GET(fake_motor_sensor);
	const struct device *act = DEVICE_GET(fake_motor_actuator);
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
