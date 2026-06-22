/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * STM32 motor power stage: ST LL timer only (no project HAL file). Advanced
 * timers (TIM1/TIM8) use complementary CHx/CHxN; general-purpose timers with
 * single-ended route one HS PWM pin per leg (gate driver generates LS).
 * actuator callback runs from the timer update interrupt
 * (interrupt-names "up" or "global" on the st,pwm-timer node).
 */

#define DT_DRV_COMPAT zephyr_motor_stage_hbridge_stm32

#include <errno.h>
#include <stdint.h>

#include <stm32_ll_bus.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_tim.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "motor_actuator_common.h"

#define HB_MAX 3U
#if defined(__LL_RCC_CALC_HCLK_FREQ)
#define MOTOR_RCC_CALC_HCLK_FREQ(sysclk, hpre) __LL_RCC_CALC_HCLK_FREQ((sysclk), (hpre))
#else
#define MOTOR_RCC_CALC_HCLK_FREQ(sysclk, hpre) LL_RCC_CALC_HCLK_FREQ((sysclk), (hpre))
#endif

#if defined(__LL_RCC_CALC_PCLK1_FREQ)
#define MOTOR_RCC_CALC_PCLK1_FREQ(hclk, ppre) __LL_RCC_CALC_PCLK1_FREQ((hclk), (ppre))
#else
#define MOTOR_RCC_CALC_PCLK1_FREQ(hclk, ppre) LL_RCC_CALC_PCLK1_FREQ((hclk), (ppre))
#endif

#if defined(__LL_RCC_CALC_PCLK2_FREQ)
#define MOTOR_RCC_CALC_PCLK2_FREQ(hclk, ppre) __LL_RCC_CALC_PCLK2_FREQ((hclk), (ppre))
#else
#define MOTOR_RCC_CALC_PCLK2_FREQ(hclk, ppre) LL_RCC_CALC_PCLK2_FREQ((hclk), (ppre))
#endif

/* Custom pinctrl state IDs picked up by Z_PINCTRL_STATE_ID via "complementary"
 * and "single-ended" entries in pinctrl-names.
 */
#define PINCTRL_STATE_COMPLEMENTARY PINCTRL_STATE_PRIV_START
#define PINCTRL_STATE_SINGLE_ENDED  (PINCTRL_STATE_PRIV_START + 1U)

struct motor_actuator_stm32_config {
	TIM_TypeDef *tim;
	const uint8_t *pwm_ch;
	uint8_t n_half_bridges;
	bool single_ended;
	uint32_t pwm_freq_hz;
	uint32_t trgo;
	uint32_t deadtime_ns;
	struct gpio_dt_spec nfault;
	struct gpio_dt_spec nsleep;
	struct gpio_dt_spec en;
	bool has_nfault;
	bool has_nsleep;
	bool has_en;
	const struct device *slow_timer;
	uint32_t slow_sample_div;
	struct motor_stage_config stage_cfg;
};

struct motor_actuator_stm32_data {
	const struct device *self;
	motor_actuator_callback_t actuator_cb;
	void *actuator_user_data;
	motor_actuator_callback_t slow_cb;
	void *slow_user_data;
	motor_fault_cb_t fault_cb;
	void *fault_user_data;
	struct gpio_callback nfault_cb;
	atomic_t slow_timer_running;
	uint32_t slow_timer_ticks;
	/* Cached at init: ARR + 1 = full-scale duty for oc_set_compare(). */
	uint32_t max_duty;
	uint32_t fault_flags;
	enum motor_drive_mode drive_mode;
	bool running;
};

static void nfault_gpio_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct motor_actuator_stm32_data *data = CONTAINER_OF(cb, struct motor_actuator_stm32_data, nfault_cb);

	ARG_UNUSED(dev);
	ARG_UNUSED(pins);

	data->fault_flags |= MOTOR_FAULT_GATE_DRIVER;
	if (data->fault_cb != NULL) {
		data->fault_cb(data->self, MOTOR_FAULT_GATE_DRIVER, data->fault_user_data);
	}
}

#define MOTOR_ACTUATOR_STM32_TRGO_FROM_ENUM(inst)                                                         \
	((DT_ENUM_IDX(DT_DRV_INST(inst), trgo_source) == 0)                                           \
		 ? LL_TIM_TRGO_UPDATE                                                               \
		 : (DT_ENUM_IDX(DT_DRV_INST(inst), trgo_source) == 1)                                  \
				 ? LL_TIM_TRGO_RESET                                                \
				 : (DT_ENUM_IDX(DT_DRV_INST(inst), trgo_source) == 2)                    \
						 ? LL_TIM_TRGO_ENABLE                               \
						 : LL_TIM_TRGO_CC1IF)

static bool tim_is_advanced(const TIM_TypeDef *tim)
{
	return (tim == TIM1) || (tim == TIM8) || (tim == TIM15) || (tim == TIM16) ||
	       (tim == TIM17);
}

static uint32_t tim_clock_hz(const TIM_TypeDef *tim)
{
	uint32_t hclk;
	uint32_t pclk;
	uint32_t apre;

	hclk = MOTOR_RCC_CALC_HCLK_FREQ(SystemCoreClock, LL_RCC_GetAHBPrescaler());

	if ((tim == TIM1) || (tim == TIM8) || (tim == TIM15) || (tim == TIM16) || (tim == TIM17)) {
		apre = LL_RCC_GetAPB2Prescaler();
		pclk = MOTOR_RCC_CALC_PCLK2_FREQ(hclk, apre);
		if (apre != LL_RCC_APB2_DIV_1) {
			return pclk * 2U;
		}
		return pclk;
	}

	apre = LL_RCC_GetAPB1Prescaler();
	pclk = MOTOR_RCC_CALC_PCLK1_FREQ(hclk, apre);
	if (apre != LL_RCC_APB1_DIV_1) {
		return pclk * 2U;
	}
	return pclk;
}

/** Encode dead-time for BDTR DTG[7:0] — t_dt = DTG * t_dts, t_dts = 1 / tim_clk. */
static uint8_t encode_deadtime(TIM_TypeDef *tim, uint32_t deadtime_ns)
{
	uint64_t clk = (uint64_t)tim_clock_hz((const TIM_TypeDef *)tim);
	uint64_t ticks = (deadtime_ns * clk) / 1000000000ULL;

	if (ticks > 255ULL) {
		return 255U;
	}
	return (uint8_t)ticks;
}

static uint32_t tim_ch_mask(uint8_t ch)
{
	switch (ch) {
	case 1U:
		return LL_TIM_CHANNEL_CH1;
	case 2U:
		return LL_TIM_CHANNEL_CH2;
	case 3U:
		return LL_TIM_CHANNEL_CH3;
	case 4U:
		return LL_TIM_CHANNEL_CH4;
	default:
		return 0U;
	}
}

static uint32_t tim_ch_pair_mask(uint8_t ch)
{
	switch (ch) {
	case 1U:
		return LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N;
	case 2U:
		return LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N;
	case 3U:
		return LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N;
	case 4U:
		return LL_TIM_CHANNEL_CH4;
	default:
		return 0U;
	}
}

static void oc_set_compare(TIM_TypeDef *tim, uint8_t ch, uint32_t pulse)
{
	switch (ch) {
	case 1U:
		LL_TIM_OC_SetCompareCH1(tim, pulse);
		break;
	case 2U:
		LL_TIM_OC_SetCompareCH2(tim, pulse);
		break;
	case 3U:
		LL_TIM_OC_SetCompareCH3(tim, pulse);
		break;
	case 4U:
		LL_TIM_OC_SetCompareCH4(tim, pulse);
		break;
	default:
		break;
	}
}

static void oc_set_single_ended_pwm(TIM_TypeDef *tim, uint8_t ch, uint32_t pulse)
{
	LL_TIM_OC_SetMode(tim, tim_ch_mask(ch), LL_TIM_OCMODE_PWM1);
	oc_set_compare(tim, ch, pulse);
	LL_TIM_CC_EnableChannel(tim, tim_ch_mask(ch));
}

static void oc_set_complementary_pwm(TIM_TypeDef *tim, uint8_t ch, uint32_t pulse)
{
	uint32_t pair = tim_ch_pair_mask(ch);

	LL_TIM_OC_SetMode(tim, tim_ch_mask(ch), LL_TIM_OCMODE_PWM1);
	oc_set_compare(tim, ch, pulse);
	LL_TIM_CC_EnableChannel(tim, pair);
}

static int tim_base_init(TIM_TypeDef *tim, uint32_t freq_hz, uint32_t *arr_out)
{
	uint32_t tim_clk = tim_clock_hz((const TIM_TypeDef *)tim);
	uint32_t psc = 0U;
	uint32_t arr;

	if ((tim == NULL) || (freq_hz == 0U)) {
		return -EINVAL;
	}

	arr = tim_clk / freq_hz;
	if (arr == 0U) {
		return -EINVAL;
	}
	arr -= 1U;

	while (arr > 0xFFFFUL) {
		psc++;
		arr = (tim_clk / (freq_hz * (psc + 1U))) - 1U;
		if (psc > 0xFFFFUL) {
			return -EINVAL;
		}
	}

	LL_TIM_SetPrescaler(tim, psc);
	LL_TIM_SetAutoReload(tim, arr);
	LL_TIM_SetCounterMode(tim, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetClockDivision(tim, LL_TIM_CLOCKDIVISION_DIV1);
	LL_TIM_EnableARRPreload(tim);

	if (arr_out != NULL) {
		*arr_out = arr;
	}

	return 0;
}

static int tim_gp_pwm_init(TIM_TypeDef *tim, uint32_t freq_hz, const uint8_t *ch, uint8_t n_ch,
			   uint32_t trgo, uint32_t *max_duty_out)
{
	int err;
	uint32_t arr;
	uint8_t i;

	err = tim_base_init(tim, freq_hz, &arr);
	if (err != 0) {
		return err;
	}

	for (i = 0U; i < n_ch; i++) {
		oc_set_single_ended_pwm(tim, ch[i], 0U);
	}

	LL_TIM_SetTriggerOutput(tim, trgo);
	LL_TIM_GenerateEvent_UPDATE(tim);
	LL_TIM_EnableCounter(tim);

	if (max_duty_out != NULL) {
		*max_duty_out = arr + 1U;
	}

	return 0;
}

static int tim_hbridge_init(TIM_TypeDef *tim, uint32_t freq_hz, const uint8_t *ch, uint8_t n_ch,
			    uint32_t deadtime_ns, uint32_t trgo, uint32_t *max_duty_out)
{
	int err;
	uint32_t arr;
	uint8_t dtg;
	uint8_t i;

	err = tim_base_init(tim, freq_hz, &arr);
	if (err != 0) {
		return err;
	}

	dtg = encode_deadtime(tim, deadtime_ns);
	LL_TIM_OC_SetDeadTime(tim, dtg);

	for (i = 0U; i < n_ch; i++) {
		oc_set_complementary_pwm(tim, ch[i], 0U);
	}

	LL_TIM_SetTriggerOutput(tim, trgo);
	LL_TIM_GenerateEvent_UPDATE(tim);
	LL_TIM_EnableCounter(tim);

	if (max_duty_out != NULL) {
		*max_duty_out = arr + 1U;
	}

	return 0;
}

static void tim_enable_main_output(TIM_TypeDef *tim, bool enable)
{
	if (!tim_is_advanced(tim)) {
		return;
	}

	if (enable) {
		LL_TIM_EnableAllOutputs(tim);
	} else {
		LL_TIM_DisableAllOutputs(tim);
	}
}

static int motor_actuator_stm32_set_duty(const struct device *dev, const float *duty, uint8_t n)
{
	const struct motor_actuator_stm32_config *cfg = dev->config;
	struct motor_actuator_stm32_data *data = dev->data;
	uint32_t max_duty = data->max_duty;
	uint8_t i;
	uint8_t limit;

	if (data->fault_flags != 0U) {
		return -EIO;
	}

	if (data->drive_mode == MOTOR_DRIVE_COAST) {
		for (i = 0U; i < cfg->n_half_bridges; i++) {
			oc_set_compare(cfg->tim, cfg->pwm_ch[i], 0U);
		}
		return 0;
	}

	limit = (n < cfg->n_half_bridges) ? n : cfg->n_half_bridges;
	for (i = 0U; i < limit; i++) {
		float d = duty[i];
		uint32_t cmp;

		if (d > 1.0f) {
			d = 1.0f;
		} else if (d < 0.0f) {
			d = 0.0f;
		}
		cmp = (uint32_t)(d * (float)max_duty);
		oc_set_compare(cfg->tim, cfg->pwm_ch[i], cmp);
	}

	return 0;
}

static int motor_actuator_stm32_set_drive_mode(const struct device *dev, enum motor_drive_mode mode)
{
	struct motor_actuator_stm32_data *data = dev->data;

	data->drive_mode = mode;
	if (mode == MOTOR_DRIVE_BRAKE || mode == MOTOR_DRIVE_REGEN) {
		return -ENOTSUP;
	}

	return 0;
}

static void motor_actuator_stm32_slow_alarm_cb(const struct device *counter_dev, uint8_t chan_id,
					       uint32_t ticks, void *user_data)
{
	const struct device *dev = user_data;
	struct motor_actuator_stm32_data *data = dev->data;

	ARG_UNUSED(counter_dev);
	ARG_UNUSED(chan_id);
	ARG_UNUSED(ticks);

	if (data->slow_cb != NULL) {
		data->slow_cb(dev, data->slow_user_data);
	}
	atomic_clear(&data->slow_timer_running);
}

static void motor_actuator_stm32_arm_slow_timer(const struct device *dev)
{
	const struct motor_actuator_stm32_config *cfg = dev->config;
	struct motor_actuator_stm32_data *data = dev->data;
	struct counter_alarm_cfg alarm = {
		.callback = motor_actuator_stm32_slow_alarm_cb,
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

static int motor_actuator_stm32_set_callback(const struct device *dev,
					     motor_actuator_callback_t fast_cb,
					     void *fast_user_data,
					     motor_actuator_callback_t slow_cb,
					     void *slow_user_data)
{
	struct motor_actuator_stm32_data *data = dev->data;

	data->actuator_cb = fast_cb;
	data->actuator_user_data = fast_user_data;
	data->slow_cb = slow_cb;
	data->slow_user_data = slow_user_data;

	return 0;
}

static int motor_actuator_stm32_set_fault_callback(const struct device *dev, motor_fault_cb_t cb,
					    void *user_data)
{
	struct motor_actuator_stm32_data *data = dev->data;

	data->fault_cb = cb;
	data->fault_user_data = user_data;

	return 0;
}

static int motor_actuator_stm32_clear_fault(const struct device *dev)
{
	struct motor_actuator_stm32_data *data = dev->data;

	data->fault_flags = 0U;
	return 0;
}

static int motor_actuator_stm32_get_fault(const struct device *dev, uint32_t *flags)
{
	struct motor_actuator_stm32_data *data = dev->data;

	if (flags == NULL) {
		return -EINVAL;
	}

	*flags = data->fault_flags;
	return 0;
}

static const struct motor_stage_config *motor_actuator_stm32_get_config(const struct device *dev)
{
	const struct motor_actuator_stm32_config *cfg = dev->config;

	return &cfg->stage_cfg;
}

static void motor_actuator_stm32_disable_outputs(const struct device *dev)
{
	const struct motor_actuator_stm32_config *cfg = dev->config;
	uint8_t i;

	for (i = 0U; i < cfg->n_half_bridges; i++) {
		oc_set_compare(cfg->tim, cfg->pwm_ch[i], 0U);
	}
}

static int motor_actuator_stm32_disable(const struct device *dev)
{
	const struct motor_actuator_stm32_config *cfg = dev->config;
	struct motor_actuator_stm32_data *data = dev->data;

	data->running = false;
	(void)counter_cancel_channel_alarm(cfg->slow_timer, 0U);
	atomic_clear(&data->slow_timer_running);
	motor_actuator_stm32_disable_outputs(dev);
	tim_enable_main_output(cfg->tim, false);
	LL_TIM_DisableCounter(cfg->tim);

	if (cfg->has_en) {
		(void)gpio_pin_set_dt(&cfg->en, 0);
	}
	if (cfg->has_nsleep) {
		(void)gpio_pin_set_dt(&cfg->nsleep, 0);
	}

	data->drive_mode = MOTOR_DRIVE_NORMAL;

	return 0;
}

static int motor_actuator_stm32_enable(const struct device *dev)
{
	const struct motor_actuator_stm32_config *cfg = dev->config;
	struct motor_actuator_stm32_data *data = dev->data;
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

	tim_enable_main_output(cfg->tim, true);
	LL_TIM_EnableCounter(cfg->tim);
	data->drive_mode = MOTOR_DRIVE_NORMAL;
	data->running = true;

	return 0;
}

static int motor_actuator_stm32_hw_init(const struct device *dev)
{
	const struct motor_actuator_stm32_config *cfg = dev->config;
	struct motor_actuator_stm32_data *data = dev->data;
	uint64_t slow_ticks;
	int err;

	data->self = dev;
	data->drive_mode = MOTOR_DRIVE_NORMAL;
	data->running = false;
	atomic_clear(&data->slow_timer_running);

	if (cfg->single_ended && !tim_is_advanced(cfg->tim)) {
		err = tim_gp_pwm_init(cfg->tim, cfg->pwm_freq_hz, cfg->pwm_ch, cfg->n_half_bridges,
				      cfg->trgo, &data->max_duty);
	} else {
		err = tim_hbridge_init(cfg->tim, cfg->pwm_freq_hz, cfg->pwm_ch, cfg->n_half_bridges,
				       cfg->deadtime_ns, cfg->trgo, &data->max_duty);
	}
	if (err != 0) {
		return err;
	}

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

	motor_actuator_stm32_disable_outputs(dev);

	LL_TIM_EnableIT_UPDATE(cfg->tim);

	return 0;
}

const struct motor_actuator_ops motor_actuator_stm32_api = {
	.enable = motor_actuator_stm32_enable,
	.disable = motor_actuator_stm32_disable,
	.set_duty = motor_actuator_stm32_set_duty,
	.set_drive_mode = motor_actuator_stm32_set_drive_mode,
	.set_callback = motor_actuator_stm32_set_callback,
	.set_fault_callback = motor_actuator_stm32_set_fault_callback,
	.clear_fault = motor_actuator_stm32_clear_fault,
	.get_fault = motor_actuator_stm32_get_fault,
	.self_test = motor_actuator_self_test_noop,
	.get_config = motor_actuator_stm32_get_config,
	.sto_arm = motor_actuator_sto_arm_unsupported,
	.sto_release = motor_actuator_sto_release_unsupported,
};

#define TIM_FROM_PHANDLE(inst) ((TIM_TypeDef *)DT_REG_ADDR(DT_INST_PHANDLE(inst, st_pwm_timer)))
#define MOTOR_ACTUATOR_STM32_TIMER(inst) DT_INST_PHANDLE(inst, st_pwm_timer)
#define MOTOR_ACTUATOR_STM32_IRQ_FLAGS IRQ_ZERO_LATENCY

#define MOTOR_ACTUATOR_STM32_DEFINE(inst)                                                                 \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	ISR_DIRECT_DECLARE(motor_actuator_stm32_pwm_isr_##inst)                                     \
	{                                                                                            \
		const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(inst));                          \
		const struct motor_actuator_stm32_config *cfg = dev->config;                          \
		struct motor_actuator_stm32_data *data = dev->data;                                   \
		TIM_TypeDef *tim = cfg->tim;                                                         \
                                                                                                 \
		if ((LL_TIM_IsActiveFlag_UPDATE(tim) != 0UL) &&                                      \
		    (LL_TIM_IsEnabledIT_UPDATE(tim) != 0U)) {                                        \
			LL_TIM_ClearFlag_UPDATE(tim);                                               \
			if (data->actuator_cb != NULL) {                                             \
				data->actuator_cb(dev, data->actuator_user_data);                    \
			}                                                                            \
			motor_actuator_stm32_arm_slow_timer(dev);                                    \
		}                                                                                    \
		return 0;                                                                            \
	}                                                                                            \
	static const uint8_t pwm_ch_array_##inst[] = DT_INST_PROP(inst, pwm_channels);               \
	static const struct motor_actuator_stm32_config motor_actuator_stm32_cfg_##inst = {          \
		.tim = TIM_FROM_PHANDLE(inst),                                                     \
		.pwm_ch = pwm_ch_array_##inst,                                                     \
		.n_half_bridges = (uint8_t)ARRAY_SIZE(pwm_ch_array_##inst),                        \
		.single_ended = DT_INST_PROP_OR(inst, single_ended, 0),                            \
		.pwm_freq_hz = (uint32_t)DT_INST_PROP(inst, pwm_frequency),                       \
		.trgo = MOTOR_ACTUATOR_STM32_TRGO_FROM_ENUM(inst),                                          \
		.deadtime_ns = (uint32_t)DT_INST_PROP(inst, deadtime_ns),                         \
		.nfault = GPIO_DT_SPEC_INST_GET_OR(inst, nfault_gpios, {0}),                        \
		.nsleep = GPIO_DT_SPEC_INST_GET_OR(inst, nsleep_gpios, {0}),                        \
		.en = GPIO_DT_SPEC_INST_GET_OR(inst, en_gpios, {0}),                                \
		.has_nfault = DT_NODE_HAS_PROP(DT_DRV_INST(inst), nfault_gpios),                   \
		.has_nsleep = DT_NODE_HAS_PROP(DT_DRV_INST(inst), nsleep_gpios),                   \
		.has_en = DT_NODE_HAS_PROP(DT_DRV_INST(inst), en_gpios),                            \
		.slow_timer = DEVICE_DT_GET(DT_INST_PHANDLE(inst, slow_timer)),                    \
		.slow_sample_div = (uint32_t)DT_INST_PROP(inst, slow_sample_div),                  \
		.stage_cfg =                                                                        \
			{                                                                             \
				.topology = (ARRAY_SIZE(pwm_ch_array_##inst) <= 1U)                        \
						    ? MOTOR_STAGE_HALF_BRIDGE                              \
						    : MOTOR_STAGE_FULL_BRIDGE,                           \
				.n_phases = (uint8_t)ARRAY_SIZE(pwm_ch_array_##inst),                 \
				.pwm_period_ns =                                                      \
					(uint32_t)(NSEC_PER_SEC / (uint32_t)DT_INST_PROP(inst,          \
										     pwm_frequency)), \
				.deadtime_rising_ns = (uint32_t)DT_INST_PROP(inst, deadtime_ns),      \
				.deadtime_falling_ns = (uint32_t)DT_INST_PROP(inst, deadtime_ns),     \
				.v_bus_nominal = 12.0f,                                              \
				.v_bus_ov_thresh = 30.0f,                                            \
				.v_bus_uv_thresh = 6.0f,                                             \
				.i_peak_limit = 20.0f,                                               \
			},                                                                            \
	};                                                                                           \
	BUILD_ASSERT(ARRAY_SIZE(pwm_ch_array_##inst) >= 1U &&                                      \
			     ARRAY_SIZE(pwm_ch_array_##inst) <= HB_MAX,                              \
		     "pwm-channels: length must be 1..3");                                         \
	BUILD_ASSERT(DT_INST_PROP(inst, single_ended) ||                                           \
			     DT_IRQ_HAS_NAME(MOTOR_ACTUATOR_STM32_TIMER(inst), brk),                \
		     "complementary mode requires advanced timer with \"brk\" interrupt");           \
	BUILD_ASSERT(!DT_INST_PROP(inst, single_ended) ||                                          \
			     DT_IRQ_HAS_NAME(MOTOR_ACTUATOR_STM32_TIMER(inst), up) ||                \
			     DT_IRQ_HAS_NAME(MOTOR_ACTUATOR_STM32_TIMER(inst), global),            \
		     "single-ended timer needs \"up\" or \"global\" interrupt");                   \
	BUILD_ASSERT(DT_INST_PROP(inst, single_ended) ||                                          \
			     (DT_INST_PROP_LEN(inst, pinctrl_0) ==                                  \
			      (2U * ARRAY_SIZE(pwm_ch_array_##inst))),                             \
		     "pinctrl-0 (complementary) length must equal 2 * len(pwm-channels)");        \
	BUILD_ASSERT(!DT_INST_PROP(inst, single_ended) ||                                          \
			     (COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, pinctrl_1),                  \
					  (DT_INST_PROP_LEN(inst, pinctrl_1)), (0U)) ==        \
			      ARRAY_SIZE(pwm_ch_array_##inst)),                                    \
		     "single-ended requires pinctrl-1 with len(pwm-channels) HS-only pins");      \
	static struct motor_actuator_stm32_data motor_actuator_stm32_data_##inst;                    \
	static int motor_actuator_stm32_dev_init_##inst(const struct device *dev)                       \
	{                                                                                            \
		const struct motor_actuator_stm32_config *cfg_ = dev->config;                       \
		uint8_t pinctrl_state_ = cfg_->single_ended                                         \
			? PINCTRL_STATE_SINGLE_ENDED                                                \
			: PINCTRL_STATE_COMPLEMENTARY;                                              \
		int err_ = pinctrl_apply_state(PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                 \
					       pinctrl_state_);                                       \
		if (err_ != 0) {                                                                     \
			return err_;                                                                 \
		}                                                                                    \
		err_ = motor_actuator_stm32_hw_init(dev);                                            \
		if (err_ != 0) {                                                                     \
			return err_;                                                                 \
		}                                                                                    \
		COND_CODE_1(DT_IRQ_HAS_NAME(MOTOR_ACTUATOR_STM32_TIMER(inst), up),                           \
			    (IRQ_DIRECT_CONNECT(DT_IRQ_BY_NAME(MOTOR_ACTUATOR_STM32_TIMER(inst), up, irq),  \
						DT_IRQ_BY_NAME(MOTOR_ACTUATOR_STM32_TIMER(inst), up, priority), \
						motor_actuator_stm32_pwm_isr_##inst,                    \
						MOTOR_ACTUATOR_STM32_IRQ_FLAGS);                        \
			     irq_enable(DT_IRQ_BY_NAME(MOTOR_ACTUATOR_STM32_TIMER(inst), up, irq));),       \
			    (COND_CODE_1(                                                             \
				    DT_IRQ_HAS_NAME(MOTOR_ACTUATOR_STM32_TIMER(inst), global),              \
				    (IRQ_DIRECT_CONNECT(                                              \
					     DT_IRQ_BY_NAME(MOTOR_ACTUATOR_STM32_TIMER(inst), global, irq), \
					     DT_IRQ_BY_NAME(MOTOR_ACTUATOR_STM32_TIMER(inst), global,       \
							     priority),                            \
					     motor_actuator_stm32_pwm_isr_##inst,                           \
					     MOTOR_ACTUATOR_STM32_IRQ_FLAGS);                        \
				     irq_enable(DT_IRQ_BY_NAME(MOTOR_ACTUATOR_STM32_TIMER(inst), global,  \
							       irq));),                            \
				    (BUILD_ASSERT(0,                                                 \
						  "st,pwm-timer needs interrupt-names "            \
						  "\"up\" or \"global\"")))));                       \
		return 0;                                                                            \
	}                                                                                            \
	DEVICE_DT_DEFINE(DT_DRV_INST(inst), motor_actuator_stm32_dev_init_##inst, NULL,                 \
			 &motor_actuator_stm32_data_##inst, &motor_actuator_stm32_cfg_##inst,          \
			 POST_KERNEL, CONFIG_MOTOR_STAGE_HBRIDGE_STM32_INIT_PRIORITY,                \
			 &motor_actuator_stm32_api);

DT_INST_FOREACH_STATUS_OKAY(MOTOR_ACTUATOR_STM32_DEFINE)
