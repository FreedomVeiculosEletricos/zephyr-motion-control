/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_motor_stage_hbridge_espressif_mcpwm

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <esp_attr.h>
#include <hal/mcpwm_ll.h>
#include <esp_clk_tree.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

#include "motor_actuator_common.h"

#define DUTY_MODE_ACTIVE_HIGH 0
#define DUTY_MODE_FORCE_LOW   2
#define DUTY_MODE_FORCE_HIGH  3

/* Custom pinctrl state IDs picked up by Z_PINCTRL_STATE_ID via "complementary"
 * and "single-ended" entries in pinctrl-names.
 */
#define PINCTRL_STATE_COMPLEMENTARY PINCTRL_STATE_PRIV_START
#define PINCTRL_STATE_SINGLE_ENDED  (PINCTRL_STATE_PRIV_START + 1U)

struct motor_espressif_mcpwm_config {
	mcpwm_dev_t *mcpwm;
	uint8_t mcpwm_index;
	uint8_t timer_id;
	uint32_t pwm_freq_hz;
	uint32_t prescale;
	uint8_t prescale_timer;
	uint32_t deadtime_ns;
	bool single_ended;
	struct gpio_dt_spec nfault;
	struct gpio_dt_spec nsleep;
	struct gpio_dt_spec en;
	bool has_nfault;
	bool has_nsleep;
	bool has_en;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct device *slow_timer;
	uint32_t slow_sample_div;
	struct motor_stage_config stage_cfg;
};

struct motor_espressif_mcpwm_data {
	const struct device *self;
	uint32_t mcpwm_clk_hz;
	motor_actuator_callback_t actuator_cb;
	void *actuator_user_data;
	motor_actuator_callback_t slow_cb;
	void *slow_user_data;
	intr_handle_t intr;
	motor_fault_cb_t fault_cb;
	void *fault_user_data;
	struct gpio_callback nfault_cb;
	atomic_t slow_timer_running;
	uint32_t slow_timer_ticks;
	/* Cached at init: peak counter value (timer top) for PWM compares. */
	uint32_t peak;
	uint32_t fault_flags;
	enum motor_drive_mode drive_mode;
	bool running;
	uint8_t operator_id;
	uint8_t gen_high;
	uint8_t gen_low;
};

static void nfault_gpio_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct motor_espressif_mcpwm_data *data =
		CONTAINER_OF(cb, struct motor_espressif_mcpwm_data, nfault_cb);

	ARG_UNUSED(dev);
	ARG_UNUSED(pins);

	data->fault_flags |= MOTOR_FAULT_GATE_DRIVER;
	if (data->fault_cb != NULL) {
		data->fault_cb(data->self, MOTOR_FAULT_GATE_DRIVER, data->fault_user_data);
	}
}

static void apply_leg(struct motor_espressif_mcpwm_data *data,
		      const struct motor_espressif_mcpwm_config *cfg, int mode_hs, uint32_t cmp)
{
	mcpwm_dev_t *mcpwm = cfg->mcpwm;
	uint8_t op = data->operator_id;
	uint8_t gh = data->gen_high;
	uint8_t gl = data->gen_low;

	mcpwm_ll_operator_set_compare_value(mcpwm, op, gh, cmp);
	mcpwm_ll_operator_enable_update_compare_on_tez(mcpwm, op, gh, true);

	if (mode_hs == DUTY_MODE_ACTIVE_HIGH) {
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gh, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
			MCPWM_GEN_ACTION_HIGH);
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gh, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_FULL,
			MCPWM_GEN_ACTION_KEEP);
		mcpwm_ll_generator_set_action_on_compare_event(
			mcpwm, op, gh, MCPWM_TIMER_DIRECTION_UP, gh, MCPWM_GEN_ACTION_LOW);

		/* LS is complementary of HS (no dead-time generator yet). */
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gl, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
			MCPWM_GEN_ACTION_LOW);
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gl, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_FULL,
			MCPWM_GEN_ACTION_KEEP);
		mcpwm_ll_generator_set_action_on_compare_event(
			mcpwm, op, gl, MCPWM_TIMER_DIRECTION_UP, gh, MCPWM_GEN_ACTION_HIGH);
	} else {
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gh, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
			MCPWM_GEN_ACTION_LOW);
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gh, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_FULL,
			MCPWM_GEN_ACTION_LOW);
		mcpwm_ll_generator_set_action_on_compare_event(
			mcpwm, op, gh, MCPWM_TIMER_DIRECTION_UP, gh, MCPWM_GEN_ACTION_LOW);

		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gl, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
			MCPWM_GEN_ACTION_HIGH);
		mcpwm_ll_generator_set_action_on_timer_event(
			mcpwm, op, gl, MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_FULL,
			MCPWM_GEN_ACTION_HIGH);
		mcpwm_ll_generator_set_action_on_compare_event(
			mcpwm, op, gl, MCPWM_TIMER_DIRECTION_UP, gh, MCPWM_GEN_ACTION_HIGH);
	}
}

static int set_duty(const struct device *dev, const float *duty, uint8_t n)
{
	const struct motor_espressif_mcpwm_config *cfg = dev->config;
	struct motor_espressif_mcpwm_data *data = dev->data;
	uint32_t peak = data->peak;
	float d;
	uint32_t cmp;

	if (data->fault_flags != 0U) {
		return -EIO;
	}

	if (data->drive_mode == MOTOR_DRIVE_COAST) {
		apply_leg(data, cfg, DUTY_MODE_FORCE_LOW, 0U);
		return 0;
	}

	d = (n >= 1U) ? duty[0] : 0.0f;
	if (d > 1.0f) {
		d = 1.0f;
	} else if (d < 0.0f) {
		d = 0.0f;
	}
	cmp = (uint32_t)(d * (float)peak);
	apply_leg(data, cfg, DUTY_MODE_ACTIVE_HIGH, cmp);
	return 0;
}

static int set_drive_mode(const struct device *dev, enum motor_drive_mode mode)
{
	struct motor_espressif_mcpwm_data *data = dev->data;

	data->drive_mode = mode;
	if (mode == MOTOR_DRIVE_BRAKE || mode == MOTOR_DRIVE_REGEN) {
		return -ENOTSUP;
	}

	return 0;
}

static void slow_alarm_cb(const struct device *counter_dev, uint8_t chan_id, uint32_t ticks,
			  void *user_data)
{
	const struct device *dev = user_data;
	struct motor_espressif_mcpwm_data *data = dev->data;

	ARG_UNUSED(counter_dev);
	ARG_UNUSED(chan_id);
	ARG_UNUSED(ticks);

	if (data->slow_cb != NULL) {
		data->slow_cb(dev, data->slow_user_data);
	}
	atomic_clear(&data->slow_timer_running);
}

static void arm_slow_timer(const struct device *dev)
{
	const struct motor_espressif_mcpwm_config *cfg = dev->config;
	struct motor_espressif_mcpwm_data *data = dev->data;
	struct counter_alarm_cfg alarm = {
		.callback = slow_alarm_cb,
		.ticks = data->slow_timer_ticks,
		.user_data = (void *)dev,
		.flags = 0U,
	};
	int err;

	if (!data->running || data->slow_timer_ticks == 0U) {
		return;
	}

	if (!atomic_cas(&data->slow_timer_running, 0, 1)) {
		return;
	}

	err = counter_set_channel_alarm(cfg->slow_timer, 0U, &alarm);
	if (err != 0) {
		atomic_clear(&data->slow_timer_running);
	}
}

static int set_callback(const struct device *dev, motor_actuator_callback_t fast_cb,
			void *fast_user_data, motor_actuator_callback_t slow_cb, void *slow_user_data)
{
	struct motor_espressif_mcpwm_data *data = dev->data;

	data->actuator_cb = fast_cb;
	data->actuator_user_data = fast_user_data;
	data->slow_cb = slow_cb;
	data->slow_user_data = slow_user_data;

	return 0;
}

static int set_fault_callback(const struct device *dev, motor_fault_cb_t cb, void *user_data)
{
	struct motor_espressif_mcpwm_data *data = dev->data;

	data->fault_cb = cb;
	data->fault_user_data = user_data;

	return 0;
}

static int clear_fault(const struct device *dev)
{
	struct motor_espressif_mcpwm_data *data = dev->data;

	data->fault_flags = 0U;
	return 0;
}

static int get_fault(const struct device *dev, uint32_t *flags)
{
	struct motor_espressif_mcpwm_data *data = dev->data;

	if (flags == NULL) {
		return -EINVAL;
	}

	*flags = data->fault_flags;
	return 0;
}

static const struct motor_stage_config *get_stage_config(const struct device *dev)
{
	const struct motor_espressif_mcpwm_config *cfg = dev->config;

	return &cfg->stage_cfg;
}

static void disable_outputs(const struct device *dev)
{
	const struct motor_espressif_mcpwm_config *cfg = dev->config;
	struct motor_espressif_mcpwm_data *data = dev->data;

	apply_leg(data, cfg, DUTY_MODE_FORCE_LOW, 0U);
}

static int disable(const struct device *dev)
{
	const struct motor_espressif_mcpwm_config *cfg = dev->config;
	struct motor_espressif_mcpwm_data *data = dev->data;

	data->running = false;
	(void)counter_cancel_channel_alarm(cfg->slow_timer, 0U);
	atomic_clear(&data->slow_timer_running);
	disable_outputs(dev);
	mcpwm_ll_timer_set_start_stop_command(cfg->mcpwm, cfg->timer_id, MCPWM_TIMER_STOP_EMPTY);

	if (cfg->has_en) {
		(void)gpio_pin_set_dt(&cfg->en, 0);
	}
	if (cfg->has_nsleep) {
		(void)gpio_pin_set_dt(&cfg->nsleep, 0);
	}

	data->drive_mode = MOTOR_DRIVE_NORMAL;

	return 0;
}

static int enable(const struct device *dev)
{
	const struct motor_espressif_mcpwm_config *cfg = dev->config;
	struct motor_espressif_mcpwm_data *data = dev->data;
	int err;

	if (cfg->has_nsleep) {
		err = gpio_pin_set_dt(&cfg->nsleep, 1);
		if (err != 0) {
			return err;
		}
	}
	if (cfg->has_en) {
		err = gpio_pin_set_dt(&cfg->en, 1);
		if (err != 0) {
			return err;
		}
	}

	mcpwm_ll_timer_set_start_stop_command(cfg->mcpwm, cfg->timer_id,
					      MCPWM_TIMER_START_NO_STOP);
	data->drive_mode = MOTOR_DRIVE_NORMAL;
	data->running = true;

	return 0;
}

static int init_hw(const struct device *dev)
{
	const struct motor_espressif_mcpwm_config *cfg = dev->config;
	struct motor_espressif_mcpwm_data *data = dev->data;
	int err;
	uint64_t slow_ticks;
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
	data->peak = peak;

	if (!device_is_ready(cfg->slow_timer)) {
		return -ENODEV;
	}

	slow_ticks = ((uint64_t)counter_get_frequency(cfg->slow_timer) *
		      (uint64_t)cfg->stage_cfg.pwm_period_ns * (uint64_t)cfg->slow_sample_div) /
		     (uint64_t)NSEC_PER_SEC;
	if ((slow_ticks == 0ULL) || (slow_ticks > counter_get_top_value(cfg->slow_timer))) {
		return -EINVAL;
	}
	data->slow_timer_ticks = (uint32_t)slow_ticks;

	err = counter_start(cfg->slow_timer);
	if ((err != 0) && (err != -EALREADY)) {
		return err;
	}

	mcpwm_ll_timer_set_peak(cfg->mcpwm, cfg->timer_id, peak, false);

	mcpwm_ll_operator_connect_timer(cfg->mcpwm, data->operator_id, cfg->timer_id);

	disable_outputs(dev);

	mcpwm_ll_timer_set_start_stop_command(cfg->mcpwm, cfg->timer_id, MCPWM_TIMER_STOP_EMPTY);

	mcpwm_ll_intr_enable(cfg->mcpwm, MCPWM_LL_EVENT_TIMER_EMPTY(cfg->timer_id), true);
	mcpwm_ll_intr_clear_status(cfg->mcpwm, MCPWM_LL_EVENT_TIMER_EMPTY(cfg->timer_id));

	/* TODO: wire mcpwm dead-time generator from cfg->deadtime_ns when needed. */
	ARG_UNUSED(cfg->deadtime_ns);

	return 0;
}

static int dev_init(const struct device *dev)
{
	const struct motor_espressif_mcpwm_config *cfg = dev->config;
	struct motor_espressif_mcpwm_data *data = dev->data;
	int err;

	data->self = dev;
	data->drive_mode = MOTOR_DRIVE_NORMAL;
	data->running = false;
	atomic_clear(&data->slow_timer_running);
	data->operator_id = 0U;
	data->gen_high = 0U;
	data->gen_low = 1U;

	err = init_hw(dev);
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

	if (cfg->has_en) {
		if (!gpio_is_ready_dt(&cfg->en)) {
			return -ENODEV;
		}

		err = gpio_pin_configure_dt(&cfg->en, GPIO_OUTPUT_INACTIVE);
		if (err != 0) {
			return err;
		}
	}

	return 0;
}

const struct motor_actuator_ops motor_espressif_mcpwm_api = {
	.enable = enable,
	.disable = disable,
	.set_duty = set_duty,
	.set_drive_mode = set_drive_mode,
	.set_callback = set_callback,
	.set_fault_callback = set_fault_callback,
	.clear_fault = clear_fault,
	.get_fault = get_fault,
	.self_test = motor_actuator_self_test_noop,
	.get_config = get_stage_config,
	.sto_arm = motor_actuator_sto_arm_unsupported,
	.sto_release = motor_actuator_sto_release_unsupported,
};

#define MCPWM_FROM_PHANDLE(inst) ((mcpwm_dev_t *)DT_REG_ADDR(DT_INST_PHANDLE(inst, mcpwm)))
#define MCPWM_IRQ_CELL(inst, cell) DT_IRQ_BY_IDX(DT_INST_PHANDLE(inst, mcpwm), 0, cell)

#define MOTOR_ACTUATOR_ESPRESSIF_MCPWM_DEFINE(inst)                                                \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static void IRAM_ATTR motor_espressif_mcpwm_pwm_isr_##inst(void *arg)                      \
	{                                                                                            \
		const struct device *dev = arg;                                                     \
		const struct motor_espressif_mcpwm_config *cfg = dev->config;                       \
		struct motor_espressif_mcpwm_data *data = dev->data;                                 \
		mcpwm_dev_t *mcpwm = cfg->mcpwm;                                                     \
		uint32_t st = mcpwm_ll_intr_get_status(mcpwm);                                       \
		uint32_t ev = MCPWM_LL_EVENT_TIMER_EMPTY(cfg->timer_id);                             \
                                                                                                 \
		if ((st & ev) != 0U) {                                                               \
			mcpwm_ll_intr_clear_status(mcpwm, st & ev);                                   \
			if (data->actuator_cb != NULL) {                                             \
				data->actuator_cb(dev, data->actuator_user_data);                    \
			}                                                                            \
			arm_slow_timer(dev);                                                        \
		}                                                                                    \
	}                                                                                            \
	static const struct motor_espressif_mcpwm_config motor_espressif_mcpwm_cfg_##inst = {      \
		.mcpwm = MCPWM_FROM_PHANDLE(inst),                                             \
		.mcpwm_index = (uint8_t)DT_INST_PROP(inst, mcpwm_index),                         \
		.timer_id = (uint8_t)DT_INST_PROP(inst, timer_id),                               \
		.pwm_freq_hz = (uint32_t)DT_INST_PROP(inst, pwm_frequency),                    \
		.prescale = (uint32_t)DT_INST_PROP(inst, prescale),                              \
		.prescale_timer = (uint8_t)DT_INST_PROP_OR(inst, prescale_timer0, 0),            \
		.deadtime_ns = (uint32_t)DT_INST_PROP(inst, deadtime_ns),                        \
		.single_ended = DT_INST_PROP_OR(inst, single_ended, 0),                          \
		.nfault = GPIO_DT_SPEC_INST_GET_OR(inst, nfault_gpios, {0}),                     \
		.nsleep = GPIO_DT_SPEC_INST_GET_OR(inst, nsleep_gpios, {0}),                     \
		.en = GPIO_DT_SPEC_INST_GET_OR(inst, en_gpios, {0}),                             \
		.has_nfault = DT_NODE_HAS_PROP(DT_DRV_INST(inst), nfault_gpios),                 \
		.has_nsleep = DT_NODE_HAS_PROP(DT_DRV_INST(inst), nsleep_gpios),                 \
		.has_en = DT_NODE_HAS_PROP(DT_DRV_INST(inst), en_gpios),                         \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PHANDLE(inst, mcpwm))),          \
		.clock_subsys =                                                                     \
			(clock_control_subsys_t)DT_CLOCKS_CELL(DT_INST_PHANDLE(inst, mcpwm), offset), \
		.slow_timer = DEVICE_DT_GET(DT_INST_PHANDLE(inst, slow_timer)),                  \
		.slow_sample_div = (uint32_t)DT_INST_PROP(inst, slow_sample_div),                \
		.stage_cfg =                                                                      \
			{                                                                         \
				.topology = MOTOR_STAGE_HALF_BRIDGE,                                \
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
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, pinctrl_0) == 2U,                                      \
		     "pinctrl-0 (complementary) length must equal 2 (HS + LS)");                  \
	BUILD_ASSERT(!DT_INST_PROP(inst, single_ended) ||                                          \
			     (COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, pinctrl_1),                  \
					  (DT_INST_PROP_LEN(inst, pinctrl_1)), (0U)) == 1U),    \
		     "single-ended requires pinctrl-1 with 1 HS pin");                            \
	static struct motor_espressif_mcpwm_data motor_espressif_mcpwm_data_##inst;                  \
	static int motor_espressif_mcpwm_init_wrap_##inst(const struct device *dev)              \
	{                                                                                        \
		const struct motor_espressif_mcpwm_config *cfg_ = dev->config;                     \
		struct motor_espressif_mcpwm_data *d = dev->data;                                  \
		uint8_t pinctrl_state_ = cfg_->single_ended                                        \
			? PINCTRL_STATE_SINGLE_ENDED                                               \
			: PINCTRL_STATE_COMPLEMENTARY;                                             \
		int e = pinctrl_apply_state(PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                  \
					    pinctrl_state_);                                       \
		if (e != 0) {                                                                    \
			return e;                                                                \
		}                                                                                \
		e = dev_init(dev);                                                               \
		if (e != 0) {                                                                    \
			return e;                                                                \
		}                                                                                \
		e = esp_intr_alloc(MCPWM_IRQ_CELL(inst, irq),                                      \
				   ESP_PRIO_TO_FLAGS(MCPWM_IRQ_CELL(inst, priority)) |             \
					   ESP_INT_FLAGS_CHECK(MCPWM_IRQ_CELL(inst, flags)) |      \
					   ESP_INTR_FLAG_IRAM,                                   \
				   (intr_handler_t)motor_espressif_mcpwm_pwm_isr_##inst,           \
				   (void *)dev, &d->intr);                                         \
		return (e == 0) ? 0 : -EIO;                                                      \
	}                                                                                        \
	DEVICE_DT_DEFINE(DT_DRV_INST(inst), motor_espressif_mcpwm_init_wrap_##inst, NULL,          \
			 &motor_espressif_mcpwm_data_##inst, &motor_espressif_mcpwm_cfg_##inst,    \
			 POST_KERNEL, CONFIG_MOTOR_STAGE_HBRIDGE_ESPRESSIF_MCPWM_INIT_PRIORITY,  \
			 &motor_espressif_mcpwm_api);

DT_INST_FOREACH_STATUS_OKAY(MOTOR_ACTUATOR_ESPRESSIF_MCPWM_DEFINE)
