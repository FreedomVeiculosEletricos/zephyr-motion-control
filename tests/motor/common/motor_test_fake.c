/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_test_fake.h"

#include <errno.h>
#include <string.h>

#include <zephyr/sys/util.h>

static int fake_sensor_init(const struct device *dev, const struct motor_sensor_cal *cal)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cal);
	return 0;
}

static int fake_sensor_update(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int fake_sensor_get(const struct device *dev, struct motor_sensor_output *out)
{
	const struct motor_test_fake *fake = dev->data;

	*out = fake->sense;
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

const struct motor_sensor_ops motor_test_fake_sensor_ops = {
	.init = fake_sensor_init,
	.update = fake_sensor_update,
	.get = fake_sensor_get,
	.calibrate = fake_sensor_calibrate,
	.get_backend_types = fake_sensor_get_backend_types,
};

int motor_test_fake_actuator_init(const struct device *dev)
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
	struct motor_test_fake *fake = dev->data;

	if (cmd == NULL) {
		return -EINVAL;
	}

	fake->last_cmd = *cmd;
	fake->has_last_cmd = true;
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
	struct motor_test_fake *fake = dev->data;

	fake->drive_mode = mode;
	return 0;
}

static int fake_actuator_set_control_callback(const struct device *dev, motor_control_cb_t cb,
					      void *user_data)
{
	struct motor_test_fake *fake = dev->data;

	fake->control_cb = cb;
	fake->control_user_data = user_data;
	return 0;
}

static int fake_actuator_set_fault_callback(const struct device *dev, motor_fault_cb_t cb,
					    void *user_data)
{
	struct motor_test_fake *fake = dev->data;

	fake->fault_cb = cb;
	fake->fault_user_data = user_data;
	return 0;
}

static int fake_actuator_clear_fault(const struct device *dev)
{
	struct motor_test_fake *fake = dev->data;

	fake->fault_flags = 0U;
	return 0;
}

static int fake_actuator_get_fault(const struct device *dev, uint32_t *flags)
{
	struct motor_test_fake *fake = dev->data;

	if (flags != NULL) {
		*flags = fake->fault_flags;
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
	struct motor_test_fake *fake = dev->data;

	if (fake->control_cb != NULL) {
		fake->control_cb(dev, fake->control_user_data);
	}
}

const struct motor_actuator_ops motor_test_fake_actuator_ops = {
	.init = motor_test_fake_actuator_init,
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
