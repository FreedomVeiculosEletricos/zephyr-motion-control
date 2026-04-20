/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-S3 H-bridge via MCPWM HAL (no Zephyr PWM driver on the hot path).
 */

#define DT_DRV_COMPAT zephyr_motor_stage_hbridge_esp32s3

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include <hal/mcpwm_ll.h>
#include <esp_clk_tree.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>

#include "motor_actuator_common.h"

#define DUTY_MODE_ACTIVE_HIGH 0
#define DUTY_MODE_FORCE_LOW   2
#define DUTY_MODE_FORCE_HIGH  3

struct motor_hbridge_esp32s3_config {
	mcpwm_dev_t *mcpwm;
	uint8_t mcpwm_index;
	uint8_t timer_id;
	uint32_t pwm_freq_hz;
	uint32_t prescale;
	uint8_t prescale_timer;
	uint32_t deadtime_ns;
	struct gpio_dt_spec nfault;
	struct gpio_dt_spec nsleep;
	bool has_nfault;
	bool has_nsleep;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	struct motor_stage_config stage_cfg;
};

struct motor_hbridge_esp32s3_data {
	const struct device *self;
	uint32_t mcpwm_clk_hz;
	motor_control_cb_t control_cb;
	void *control_user_data;
	motor_fault_cb_t fault_cb;
	void *fault_user_data;
	struct gpio_callback nfault_cb;
	uint32_t fault_flags;
	enum motor_drive_mode drive_mode;
	uint8_t operator_id;
	uint8_t gen_left;
	uint8_t gen_right;
};

static void nfault_gpio_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct motor_hbridge_esp32s3_data *data =
		CONTAINER_OF(cb, struct motor_hbridge_esp32s3_data, nfault_cb);

	ARG_UNUSED(dev);
	ARG_UNUSED(pins);

	data->fault_flags |= MOTOR_FAULT_GATE_DRIVER;
	if (data->fault_cb != NULL) {
		data->fault_cb(data->self, MOTOR_FAULT_GATE_DRIVER, data->fault_user_data);
	}
}

static void hbridge_esp32s3_invoke_cb(const struct device *dev)
{
	struct motor_hbridge_esp32s3_data *data = dev->data;

	if (data->control_cb != NULL) {
		data->control_cb(dev, data->control_user_data);
	}
}

static void apply_pwm_pair(struct motor_hbridge_esp32s3_data *data,
			   const struct motor_hbridge_esp32s3_config *cfg, int duty_mode_left,
			   int duty_mode_right, uint32_t cmp_left, uint32_t cmp_right)
{
	mcpwm_dev_t *mcpwm = cfg->mcpwm;
	uint8_t op = data->operator_id;
	uint8_t gl = data->gen_left;
	uint8_t gr = data->gen_right;

	mcpwm_ll_operator_set_compare_value(mcpwm, op, gl, cmp_left);
	mcpwm_ll_operator_set_compare_value(mcpwm, op, gr, cmp_right);
	mcpwm_ll_operator_enable_update_compare_on_tez(mcpwm, op, gl, true);
	mcpwm_ll_operator_enable_update_compare_on_tez(mcpwm, op, gr, true);

	if (duty_mode_left == DUTY_MODE_ACTIVE_HIGH) {
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gl, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
			MCPWM_GEN_ACTION_HIGH);
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gl, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_FULL,
			MCPWM_GEN_ACTION_KEEP);
		mcpwm_ll_generator_set_action_on_compare_event(
			mcpwm, op, gl, MCPWM_TIMER_DIRECTION_UP, gl, MCPWM_GEN_ACTION_LOW);
	} else if (duty_mode_left == DUTY_MODE_FORCE_LOW) {
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gl, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
			MCPWM_GEN_ACTION_LOW);
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gl, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_FULL,
			MCPWM_GEN_ACTION_LOW);
		mcpwm_ll_generator_set_action_on_compare_event(
			mcpwm, op, gl, MCPWM_TIMER_DIRECTION_UP, gl, MCPWM_GEN_ACTION_LOW);
	} else {
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gl, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
			MCPWM_GEN_ACTION_HIGH);
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gl, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_FULL,
			MCPWM_GEN_ACTION_HIGH);
		mcpwm_ll_generator_set_action_on_compare_event(
			mcpwm, op, gl, MCPWM_TIMER_DIRECTION_UP, gl, MCPWM_GEN_ACTION_HIGH);
	}

	if (duty_mode_right == DUTY_MODE_ACTIVE_HIGH) {
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gr, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
			MCPWM_GEN_ACTION_HIGH);
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gr, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_FULL,
			MCPWM_GEN_ACTION_KEEP);
		mcpwm_ll_generator_set_action_on_compare_event(
			mcpwm, op, gr, MCPWM_TIMER_DIRECTION_UP, gr, MCPWM_GEN_ACTION_LOW);
	} else if (duty_mode_right == DUTY_MODE_FORCE_LOW) {
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gr, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
			MCPWM_GEN_ACTION_LOW);
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gr, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_FULL,
			MCPWM_GEN_ACTION_LOW);
		mcpwm_ll_generator_set_action_on_compare_event(
			mcpwm, op, gr, MCPWM_TIMER_DIRECTION_UP, gr, MCPWM_GEN_ACTION_LOW);
	} else {
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gr, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
			MCPWM_GEN_ACTION_HIGH);
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gr, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_FULL,
			MCPWM_GEN_ACTION_HIGH);
		mcpwm_ll_generator_set_action_on_compare_event(
			mcpwm, op, gr, MCPWM_TIMER_DIRECTION_UP, gr, MCPWM_GEN_ACTION_HIGH);
	}

	ARG_UNUSED(cfg);
}

static int hbridge_esp32s3_set_vector(const struct device *dev, float valpha, float vbeta)
{
	const struct motor_hbridge_esp32s3_config *cfg = dev->config;
	struct motor_hbridge_esp32s3_data *data = dev->data;
	float a = valpha;
	float mag;
	uint32_t peak;
	uint32_t timer_clk_hz;
	uint32_t cmp;

	ARG_UNUSED(vbeta);

	if (data->fault_flags != 0U) {
		return -EIO;
	}

	if (a > 1.0f) {
		a = 1.0f;
	} else if (a < -1.0f) {
		a = -1.0f;
	}

	timer_clk_hz = data->mcpwm_clk_hz / (cfg->prescale + 1U) / (cfg->prescale_timer + 1U);
	peak = timer_clk_hz / cfg->pwm_freq_hz;
	if (peak < 2U) {
		return -EINVAL;
	}

	mag = fabsf(a) * (float)peak;
	cmp = (uint32_t)mag;
	if (cmp > peak) {
		cmp = peak;
	}

	if (data->drive_mode == MOTOR_DRIVE_COAST) {
		apply_pwm_pair(data, cfg, DUTY_MODE_FORCE_LOW, DUTY_MODE_FORCE_LOW, 0U, 0U);
		return 0;
	}

	if (a >= 0.0f) {
		apply_pwm_pair(data, cfg, DUTY_MODE_ACTIVE_HIGH, DUTY_MODE_FORCE_LOW, cmp, 0U);
	} else {
		apply_pwm_pair(data, cfg, DUTY_MODE_FORCE_LOW, DUTY_MODE_ACTIVE_HIGH, 0U, cmp);
	}

	return 0;
}

static int hbridge_esp32s3_set_duty(const struct device *dev, const float *duty, uint8_t n)
{
	float v = (n >= 1U) ? duty[0] : 0.0f;

	return hbridge_esp32s3_set_vector(dev, v * 2.0f - 1.0f, 0.0f);
}

static int hbridge_esp32s3_set_command(const struct device *dev,
				       const struct motor_actuator_cmd *cmd)
{
	return motor_actuator_dispatch_cmd(dev, cmd, hbridge_esp32s3_set_vector,
					   hbridge_esp32s3_set_duty);
}

static int hbridge_esp32s3_set_drive_mode(const struct device *dev, enum motor_drive_mode mode)
{
	struct motor_hbridge_esp32s3_data *data = dev->data;

	data->drive_mode = mode;
	if (mode == MOTOR_DRIVE_BRAKE || mode == MOTOR_DRIVE_REGEN) {
		return -ENOTSUP;
	}

	return 0;
}

static int hbridge_esp32s3_set_control_callback(const struct device *dev, motor_control_cb_t cb,
						void *user_data)
{
	struct motor_hbridge_esp32s3_data *data = dev->data;

	data->control_cb = cb;
	data->control_user_data = user_data;

	return 0;
}

static int hbridge_esp32s3_set_fault_callback(const struct device *dev, motor_fault_cb_t cb,
					      void *user_data)
{
	struct motor_hbridge_esp32s3_data *data = dev->data;

	data->fault_cb = cb;
	data->fault_user_data = user_data;

	return 0;
}

static int hbridge_esp32s3_clear_fault(const struct device *dev)
{
	struct motor_hbridge_esp32s3_data *data = dev->data;

	data->fault_flags = 0U;
	return 0;
}

static int hbridge_esp32s3_get_fault(const struct device *dev, uint32_t *flags)
{
	struct motor_hbridge_esp32s3_data *data = dev->data;

	if (flags == NULL) {
		return -EINVAL;
	}

	*flags = data->fault_flags;
	return 0;
}

static int hbridge_esp32s3_self_test(const struct device *dev, uint32_t *flags)
{
	ARG_UNUSED(dev);

	if (flags != NULL) {
		*flags = 0U;
	}

	return 0;
}

static const struct motor_stage_config *hbridge_esp32s3_get_config(const struct device *dev)
{
	const struct motor_hbridge_esp32s3_config *cfg = dev->config;

	return &cfg->stage_cfg;
}

static int hbridge_esp32s3_sto_arm(const struct device *dev)
{
	ARG_UNUSED(dev);

	return -ENOTSUP;
}

static int hbridge_esp32s3_sto_release(const struct device *dev, uint32_t *flags)
{
	ARG_UNUSED(dev);

	if (flags != NULL) {
		*flags = 0U;
	}

	return -ENOTSUP;
}

static int hbridge_esp32s3_disable_outputs(const struct device *dev)
{
	const struct motor_hbridge_esp32s3_config *cfg = dev->config;
	struct motor_hbridge_esp32s3_data *data = dev->data;

	apply_pwm_pair(data, cfg, DUTY_MODE_FORCE_LOW, DUTY_MODE_FORCE_LOW, 0U, 0U);
	return 0;
}

static int hbridge_esp32s3_disable(const struct device *dev)
{
	const struct motor_hbridge_esp32s3_config *cfg = dev->config;
	struct motor_hbridge_esp32s3_data *data = dev->data;

	(void)hbridge_esp32s3_disable_outputs(dev);
	mcpwm_ll_timer_set_start_stop_command(cfg->mcpwm, cfg->timer_id, MCPWM_TIMER_STOP_EMPTY);

	if (cfg->has_nsleep) {
		(void)gpio_pin_set_dt(&cfg->nsleep, 0);
	}

	data->drive_mode = MOTOR_DRIVE_NORMAL;

	return 0;
}

static int hbridge_esp32s3_enable(const struct device *dev)
{
	const struct motor_hbridge_esp32s3_config *cfg = dev->config;
	struct motor_hbridge_esp32s3_data *data = dev->data;
	int err;

	if (cfg->has_nsleep) {
		err = gpio_pin_set_dt(&cfg->nsleep, 1);
		if (err != 0) {
			return err;
		}
	}

	mcpwm_ll_timer_set_start_stop_command(cfg->mcpwm, cfg->timer_id,
					      MCPWM_TIMER_START_NO_STOP);
	data->drive_mode = MOTOR_DRIVE_NORMAL;

	return 0;
}

static int hbridge_esp32s3_api_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static int motor_hbridge_esp32s3_init_hw(const struct device *dev)
{
	const struct motor_hbridge_esp32s3_config *cfg = dev->config;
	struct motor_hbridge_esp32s3_data *data = dev->data;
	int err;
	uint32_t timer_clk_hz;
	uint32_t peak;

	err = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (err < 0) {
		return err;
	}

	mcpwm_ll_group_enable_clock(cfg->mcpwm_index, true);
	mcpwm_ll_group_set_clock_source(cfg->mcpwm_index, MCPWM_TIMER_CLK_SRC_DEFAULT);
	esp_clk_tree_src_get_freq_hz(MCPWM_TIMER_CLK_SRC_DEFAULT,
				     ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &data->mcpwm_clk_hz);

	mcpwm_ll_group_set_clock_prescale(cfg->mcpwm_index, cfg->prescale);
	mcpwm_ll_group_enable_shadow_mode(cfg->mcpwm);
	mcpwm_ll_group_flush_shadow(cfg->mcpwm);

	mcpwm_ll_timer_set_clock_prescale(cfg->mcpwm, cfg->timer_id, cfg->prescale_timer);
	mcpwm_ll_timer_set_count_mode(cfg->mcpwm, cfg->timer_id, MCPWM_TIMER_COUNT_MODE_UP);
	mcpwm_ll_timer_update_period_at_once(cfg->mcpwm, cfg->timer_id);

	timer_clk_hz = data->mcpwm_clk_hz / (cfg->prescale + 1U) / (cfg->prescale_timer + 1U);
	peak = timer_clk_hz / cfg->pwm_freq_hz;
	if (peak < 2U) {
		return -EINVAL;
	}

	mcpwm_ll_timer_set_peak(cfg->mcpwm, cfg->timer_id, peak, false);

	mcpwm_ll_operator_connect_timer(cfg->mcpwm, data->operator_id, cfg->timer_id);

	apply_pwm_pair(data, cfg, DUTY_MODE_FORCE_LOW, DUTY_MODE_FORCE_LOW, 0U, 0U);

	mcpwm_ll_timer_set_start_stop_command(cfg->mcpwm, cfg->timer_id, MCPWM_TIMER_STOP_EMPTY);

	ARG_UNUSED(cfg->deadtime_ns);

	return 0;
}

static int motor_hbridge_esp32s3_init(const struct device *dev)
{
	const struct motor_hbridge_esp32s3_config *cfg = dev->config;
	struct motor_hbridge_esp32s3_data *data = dev->data;
	int err;

	data->self = dev;
	data->drive_mode = MOTOR_DRIVE_NORMAL;
	data->operator_id = 0U;
	data->gen_left = 0U;
	data->gen_right = 1U;

	err = motor_hbridge_esp32s3_init_hw(dev);
	if (err != 0) {
		return err;
	}

	if (cfg->has_nfault) {
		if (!gpio_is_ready_dt(&cfg->nfault)) {
			return -ENODEV;
		}

		err = gpio_pin_configure_dt(&cfg->nfault, GPIO_INPUT);
		if (err != 0) {
			return err;
		}

		gpio_init_callback(&data->nfault_cb, nfault_gpio_cb, BIT(cfg->nfault.pin));
		err = gpio_add_callback_dt(&cfg->nfault, &data->nfault_cb);
		if (err != 0) {
			return err;
		}

		err = gpio_pin_interrupt_configure_dt(&cfg->nfault, GPIO_INT_EDGE_TO_ACTIVE);
		if (err != 0) {
			return err;
		}
	}

	if (cfg->has_nsleep) {
		if (!gpio_is_ready_dt(&cfg->nsleep)) {
			return -ENODEV;
		}

		err = gpio_pin_configure_dt(&cfg->nsleep, GPIO_OUTPUT_INACTIVE);
		if (err != 0) {
			return err;
		}
	}

	return 0;
}

static const struct motor_actuator_ops motor_hbridge_esp32s3_api = {
	.init = hbridge_esp32s3_api_init,
	.enable = hbridge_esp32s3_enable,
	.disable = hbridge_esp32s3_disable,
	.set_command = hbridge_esp32s3_set_command,
	.set_duty = hbridge_esp32s3_set_duty,
	.set_drive_mode = hbridge_esp32s3_set_drive_mode,
	.set_control_callback = hbridge_esp32s3_set_control_callback,
	.set_fault_callback = hbridge_esp32s3_set_fault_callback,
	.clear_fault = hbridge_esp32s3_clear_fault,
	.get_fault = hbridge_esp32s3_get_fault,
	.self_test = hbridge_esp32s3_self_test,
	.get_config = hbridge_esp32s3_get_config,
	.sto_arm = hbridge_esp32s3_sto_arm,
	.sto_release = hbridge_esp32s3_sto_release,
	.invoke_control_callback = hbridge_esp32s3_invoke_cb,
};

#define MCPWM_FROM_PHANDLE(inst) ((mcpwm_dev_t *)DT_REG_ADDR(DT_INST_PHANDLE(inst, mcpwm)))

#define HBRIDGE_ESP32S3_DEFINE(inst)                                                                 \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static const struct motor_hbridge_esp32s3_config motor_hbridge_esp32s3_cfg_##inst = {      \
		.mcpwm = MCPWM_FROM_PHANDLE(inst),                                             \
		.mcpwm_index = (uint8_t)DT_INST_PROP(inst, mcpwm_index),                         \
		.timer_id = (uint8_t)DT_INST_PROP(inst, timer_id),                               \
		.pwm_freq_hz = (uint32_t)DT_INST_PROP(inst, pwm_frequency),                    \
		.prescale = (uint32_t)DT_INST_PROP(inst, prescale),                              \
		.prescale_timer = (uint8_t)DT_INST_PROP_OR(inst, prescale_timer0, 0),            \
		.deadtime_ns = (uint32_t)DT_INST_PROP(inst, deadtime_ns),                        \
		.nfault = GPIO_DT_SPEC_INST_GET_OR(inst, nfault_gpios, {0}),                     \
		.nsleep = GPIO_DT_SPEC_INST_GET_OR(inst, nsleep_gpios, {0}),                     \
		.has_nfault = DT_NODE_HAS_PROP(DT_DRV_INST(inst), nfault_gpios),                 \
		.has_nsleep = DT_NODE_HAS_PROP(DT_DRV_INST(inst), nsleep_gpios),                 \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PHANDLE(inst, mcpwm))),          \
		.clock_subsys =                                                                     \
			(clock_control_subsys_t)DT_CLOCKS_CELL(DT_INST_PHANDLE(inst, mcpwm), offset), \
		.stage_cfg =                                                                      \
			{                                                                         \
				.topology = MOTOR_STAGE_FULL_BRIDGE,                                \
				.n_phases = 1U,                                                     \
				.pwm_period_ns = (uint32_t)(NSEC_PER_SEC /                          \
							    (uint32_t)DT_INST_PROP(inst, pwm_frequency)), \
				.deadtime_rising_ns = (uint32_t)DT_INST_PROP(inst, deadtime_ns),   \
				.deadtime_falling_ns = (uint32_t)DT_INST_PROP(inst, deadtime_ns),  \
				.v_bus_nominal = 12.0f,                                            \
				.v_bus_ov_thresh = 30.0f,                                          \
				.v_bus_uv_thresh = 6.0f,                                           \
				.i_peak_limit = 20.0f,                                             \
			},                                                                        \
	};                                                                                         \
	static struct motor_hbridge_esp32s3_data motor_hbridge_esp32s3_data_##inst;                  \
	static int motor_hbridge_esp32s3_init_wrap_##inst(const struct device *dev)              \
	{                                                                                        \
		int e = pinctrl_apply_state(PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                  \
					    PINCTRL_STATE_DEFAULT);                                \
		if (e != 0) {                                                                    \
			return e;                                                                \
		}                                                                                \
		return motor_hbridge_esp32s3_init(dev);                                          \
	}                                                                                        \
	DEVICE_DT_DEFINE(DT_DRV_INST(inst), motor_hbridge_esp32s3_init_wrap_##inst, NULL,        \
			 &motor_hbridge_esp32s3_data_##inst, &motor_hbridge_esp32s3_cfg_##inst,    \
			 POST_KERNEL, CONFIG_MOTOR_STAGE_HBRIDGE_ESP32S3_INIT_PRIORITY,            \
			 &motor_hbridge_esp32s3_api);

DT_INST_FOREACH_STATUS_OKAY(HBRIDGE_ESP32S3_DEFINE)
