/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * STM32G4 motor power stage: ST LL timer only (no project HAL file). Supports
 * up to three half-bridge legs on one timer (pwm-channels), independent PWM or
 * complementary CHx/CHxN on TIM1/TIM8. Control callback is invoked from the ADC
 * sync path (motor-sensor-encoder-stm32).
 */

#define DT_DRV_COMPAT zephyr_motor_stage_hbridge_stm32

#include <errno.h>
#include <math.h>
#include <stdint.h>

#include <stm32g4xx.h>
#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_rcc.h>
#include <stm32g4xx_ll_tim.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>

#define HB_MAX 3U

struct motor_hbridge_stm32_config {
	TIM_TypeDef *tim;
	const uint8_t *pwm_ch;
	uint8_t n_half_bridges;
	bool complementary;
	uint32_t pwm_freq_hz;
	uint32_t trgo;
	uint32_t deadtime_ns;
	struct gpio_dt_spec nfault;
	struct gpio_dt_spec nsleep;
	bool has_nfault;
	bool has_nsleep;
	struct motor_stage_config stage_cfg;
};

struct motor_hbridge_stm32_data {
	const struct device *self;
	motor_control_cb_t control_cb;
	void *control_user_data;
	motor_fault_cb_t fault_cb;
	void *fault_user_data;
	struct gpio_callback nfault_cb;
	uint32_t fault_flags;
	enum motor_drive_mode drive_mode;
};

static void nfault_gpio_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct motor_hbridge_stm32_data *data = CONTAINER_OF(cb, struct motor_hbridge_stm32_data, nfault_cb);

	ARG_UNUSED(dev);
	ARG_UNUSED(pins);

	data->fault_flags |= MOTOR_FAULT_GATE_DRIVER;
	if (data->fault_cb != NULL) {
		data->fault_cb(data->self, MOTOR_FAULT_GATE_DRIVER, data->fault_user_data);
	}
}

#define HBRIDGE_STM32_TRGO_FROM_ENUM(inst)                                                         \
	((DT_ENUM_IDX(DT_DRV_INST(inst), trgo_source) == 0)                                           \
		 ? LL_TIM_TRGO_UPDATE                                                               \
		 : (DT_ENUM_IDX(DT_DRV_INST(inst), trgo_source) == 1)                                  \
				 ? LL_TIM_TRGO_RESET                                                \
				 : (DT_ENUM_IDX(DT_DRV_INST(inst), trgo_source) == 2)                    \
						 ? LL_TIM_TRGO_ENABLE                               \
						 : LL_TIM_TRGO_CC1IF)

static uint32_t tim_clock_hz(const TIM_TypeDef *tim)
{
	uint32_t hclk;
	uint32_t pclk;
	uint32_t apre;

	hclk = __LL_RCC_CALC_HCLK_FREQ(SystemCoreClock, LL_RCC_GetAHBPrescaler());

	if ((tim == TIM1) || (tim == TIM8) || (tim == TIM15) || (tim == TIM16) || (tim == TIM17)) {
		apre = LL_RCC_GetAPB2Prescaler();
		pclk = __LL_RCC_CALC_PCLK2_FREQ(hclk, apre);
		if (apre != LL_RCC_APB2_DIV_1) {
			return pclk * 2U;
		}
		return pclk;
	}

	apre = LL_RCC_GetAPB1Prescaler();
	pclk = __LL_RCC_CALC_PCLK1_FREQ(hclk, apre);
	if (apre != LL_RCC_APB1_DIV_1) {
		return pclk * 2U;
	}
	return pclk;
}

static void tim_enable_rcc(TIM_TypeDef *tim)
{
	if (tim == TIM1) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
	} else if (tim == TIM2) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	} else if (tim == TIM3) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
	} else if (tim == TIM4) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	} else if (tim == TIM8) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
	}
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

static void oc_set_independent_pwm(TIM_TypeDef *tim, uint8_t ch, uint32_t pulse)
{
	switch (ch) {
	case 1U:
		LL_TIM_OC_SetMode(tim, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
		LL_TIM_OC_SetCompareCH1(tim, pulse);
		LL_TIM_CC_EnableChannel(tim, LL_TIM_CHANNEL_CH1);
		break;
	case 2U:
		LL_TIM_OC_SetMode(tim, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
		LL_TIM_OC_SetCompareCH2(tim, pulse);
		LL_TIM_CC_EnableChannel(tim, LL_TIM_CHANNEL_CH2);
		break;
	case 3U:
		LL_TIM_OC_SetMode(tim, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
		LL_TIM_OC_SetCompareCH3(tim, pulse);
		LL_TIM_CC_EnableChannel(tim, LL_TIM_CHANNEL_CH3);
		break;
	case 4U:
		LL_TIM_OC_SetMode(tim, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);
		LL_TIM_OC_SetCompareCH4(tim, pulse);
		LL_TIM_CC_EnableChannel(tim, LL_TIM_CHANNEL_CH4);
		break;
	default:
		break;
	}
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

static int tim_hbridge_init_independent(TIM_TypeDef *tim, uint32_t freq_hz,
					const uint8_t *ch, uint8_t n_ch, uint32_t trgo)
{
	int err;
	uint32_t arr;
	uint8_t i;

	err = tim_base_init(tim, freq_hz, &arr);
	if (err != 0) {
		return err;
	}

	for (i = 0U; i < n_ch; i++) {
		oc_set_independent_pwm(tim, ch[i], 0U);
	}

	LL_TIM_SetTriggerOutput(tim, trgo);
	LL_TIM_GenerateEvent_UPDATE(tim);
	LL_TIM_EnableCounter(tim);

	return 0;
}

static int tim_hbridge_init_complementary(TIM_TypeDef *tim, uint32_t freq_hz,
					  const uint8_t *ch, uint8_t n_ch, uint32_t deadtime_ns,
					  uint32_t trgo)
{
	int err;
	uint8_t dtg;
	uint8_t i;

	if ((tim != TIM1) && (tim != TIM8)) {
		return -EINVAL;
	}

	err = tim_base_init(tim, freq_hz, NULL);
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

	return 0;
}

static void tim_hbridge_set_vector_independent(TIM_TypeDef *tim, const uint8_t *ch, uint8_t n_ch,
					     float valpha, bool coast)
{
	uint32_t arr = LL_TIM_GetAutoReload(tim);
	uint32_t max_duty = arr + 1U;
	uint32_t mag;
	uint8_t i;

	if (coast) {
		mag = 0U;
	} else if (n_ch == 1U) {
		float a = valpha;

		if (a > 1.0f) {
			a = 1.0f;
		} else if (a < -1.0f) {
			a = -1.0f;
		}
		mag = (uint32_t)(fabsf(a) * (float)max_duty);
	} else {
		float a = valpha;

		if (a > 1.0f) {
			a = 1.0f;
		} else if (a < -1.0f) {
			a = -1.0f;
		}
		mag = (uint32_t)(fabsf(a) * (float)max_duty);
	}

	if (coast || (mag == 0U)) {
		for (i = 0U; i < n_ch; i++) {
			oc_set_compare(tim, ch[i], 0U);
		}
		return;
	}

	if (n_ch == 1U) {
		oc_set_compare(tim, ch[0], mag);
		return;
	}

	if (n_ch == 2U) {
		if (valpha >= 0.0f) {
			oc_set_compare(tim, ch[0], mag);
			oc_set_compare(tim, ch[1], 0U);
		} else {
			oc_set_compare(tim, ch[0], 0U);
			oc_set_compare(tim, ch[1], mag);
		}
		return;
	}

	/* Three legs: not modulated in this driver revision */
	for (i = 0U; i < n_ch; i++) {
		oc_set_compare(tim, ch[i], 0U);
	}
}

static void tim_hbridge_set_vector_complementary(TIM_TypeDef *tim, const uint8_t *ch, uint8_t n_ch,
						 float valpha, bool coast)
{
	/* Same compare mapping as independent; CHxN follows hardware. */
	tim_hbridge_set_vector_independent(tim, ch, n_ch, valpha, coast);
}

static void tim_enable_main_output(TIM_TypeDef *tim, bool enable)
{
	if ((tim == TIM1) || (tim == TIM8)) {
		if (enable) {
			LL_TIM_EnableAllOutputs(tim);
		} else {
			LL_TIM_DisableAllOutputs(tim);
		}
	}
}

static void hbridge_stm32_invoke_cb(const struct device *dev)
{
	struct motor_hbridge_stm32_data *data = dev->data;

	if (data->control_cb != NULL) {
		data->control_cb(dev, data->control_user_data);
	}
}

static int hbridge_stm32_set_vector(const struct device *dev, float valpha, float vbeta)
{
	const struct motor_hbridge_stm32_config *cfg = dev->config;
	struct motor_hbridge_stm32_data *data = dev->data;
	bool coast = false;

	ARG_UNUSED(vbeta);

	if (data->fault_flags != 0U) {
		return -EIO;
	}

	if (data->drive_mode == MOTOR_DRIVE_COAST) {
		coast = true;
	}

	if (cfg->complementary) {
		tim_hbridge_set_vector_complementary(cfg->tim, cfg->pwm_ch, cfg->n_half_bridges, valpha,
						       coast);
	} else {
		tim_hbridge_set_vector_independent(cfg->tim, cfg->pwm_ch, cfg->n_half_bridges, valpha,
						     coast);
	}

	return 0;
}

static int hbridge_stm32_set_duty(const struct device *dev, const float *duty, uint8_t n)
{
	float v = (n >= 1U) ? duty[0] : 0.0f;

	return hbridge_stm32_set_vector(dev, v * 2.0f - 1.0f, 0.0f);
}

static int hbridge_stm32_set_command(const struct device *dev, const struct motor_actuator_cmd *cmd)
{
	if (cmd == NULL) {
		return -EINVAL;
	}

	switch (cmd->kind) {
	case MOTOR_ACTUATOR_CMD_ALPHA_BETA:
		return hbridge_stm32_set_vector(dev, cmd->u.ab.valpha, cmd->u.ab.vbeta);
	case MOTOR_ACTUATOR_CMD_VD_VQ:
		return -ENOTSUP;
	case MOTOR_ACTUATOR_CMD_DUTY_DIRECT:
		if (cmd->u.duty.n == 0U || cmd->u.duty.n > MOTOR_ACTUATOR_CMD_DUTY_MAX) {
			return -EINVAL;
		}
		return hbridge_stm32_set_duty(dev, cmd->u.duty.duty, cmd->u.duty.n);
	default:
		return -EINVAL;
	}
}

static int hbridge_stm32_set_drive_mode(const struct device *dev, enum motor_drive_mode mode)
{
	struct motor_hbridge_stm32_data *data = dev->data;

	data->drive_mode = mode;
	if (mode == MOTOR_DRIVE_BRAKE || mode == MOTOR_DRIVE_REGEN) {
		return -ENOTSUP;
	}

	return 0;
}

static int hbridge_stm32_set_control_callback(const struct device *dev, motor_control_cb_t cb,
					      void *user_data)
{
	struct motor_hbridge_stm32_data *data = dev->data;

	data->control_cb = cb;
	data->control_user_data = user_data;

	return 0;
}

static int hbridge_stm32_set_fault_callback(const struct device *dev, motor_fault_cb_t cb,
					    void *user_data)
{
	struct motor_hbridge_stm32_data *data = dev->data;

	data->fault_cb = cb;
	data->fault_user_data = user_data;

	return 0;
}

static int hbridge_stm32_clear_fault(const struct device *dev)
{
	struct motor_hbridge_stm32_data *data = dev->data;

	data->fault_flags = 0U;
	return 0;
}

static int hbridge_stm32_get_fault(const struct device *dev, uint32_t *flags)
{
	struct motor_hbridge_stm32_data *data = dev->data;

	if (flags == NULL) {
		return -EINVAL;
	}

	*flags = data->fault_flags;
	return 0;
}

static int hbridge_stm32_self_test(const struct device *dev, uint32_t *flags)
{
	ARG_UNUSED(dev);

	if (flags != NULL) {
		*flags = 0U;
	}

	return 0;
}

static const struct motor_stage_config *hbridge_stm32_get_config(const struct device *dev)
{
	const struct motor_hbridge_stm32_config *cfg = dev->config;

	return &cfg->stage_cfg;
}

static int hbridge_stm32_sto_arm(const struct device *dev)
{
	ARG_UNUSED(dev);

	return -ENOTSUP;
}

static int hbridge_stm32_sto_release(const struct device *dev, uint32_t *flags)
{
	ARG_UNUSED(dev);

	if (flags != NULL) {
		*flags = 0U;
	}

	return -ENOTSUP;
}

static int hbridge_stm32_disable_outputs(const struct device *dev)
{
	const struct motor_hbridge_stm32_config *cfg = dev->config;

	if (cfg->complementary) {
		tim_hbridge_set_vector_complementary(cfg->tim, cfg->pwm_ch, cfg->n_half_bridges, 0.0f,
						     true);
	} else {
		tim_hbridge_set_vector_independent(cfg->tim, cfg->pwm_ch, cfg->n_half_bridges, 0.0f,
						   true);
	}

	return 0;
}

static int hbridge_stm32_disable(const struct device *dev)
{
	const struct motor_hbridge_stm32_config *cfg = dev->config;
	struct motor_hbridge_stm32_data *data = dev->data;

	(void)hbridge_stm32_disable_outputs(dev);
	tim_enable_main_output(cfg->tim, false);
	LL_TIM_DisableCounter(cfg->tim);

	if (cfg->has_nsleep) {
		(void)gpio_pin_set_dt(&cfg->nsleep, 0);
	}

	data->drive_mode = MOTOR_DRIVE_NORMAL;

	return 0;
}

static int hbridge_stm32_enable(const struct device *dev)
{
	const struct motor_hbridge_stm32_config *cfg = dev->config;
	struct motor_hbridge_stm32_data *data = dev->data;
	int err;

	if (cfg->has_nsleep) {
		err = gpio_pin_set_dt(&cfg->nsleep, 1);
		if (err != 0) {
			return err;
		}
	}

	tim_enable_main_output(cfg->tim, true);
	LL_TIM_EnableCounter(cfg->tim);
	data->drive_mode = MOTOR_DRIVE_NORMAL;

	return 0;
}

static int hbridge_stm32_api_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static int motor_hbridge_stm32_init(const struct device *dev)
{
	const struct motor_hbridge_stm32_config *cfg = dev->config;
	struct motor_hbridge_stm32_data *data = dev->data;
	int err;

	data->self = dev;
	data->drive_mode = MOTOR_DRIVE_NORMAL;

	tim_enable_rcc(cfg->tim);

	if (cfg->complementary) {
		err = tim_hbridge_init_complementary(cfg->tim, cfg->pwm_freq_hz, cfg->pwm_ch,
						     cfg->n_half_bridges, cfg->deadtime_ns, cfg->trgo);
	} else {
		err = tim_hbridge_init_independent(cfg->tim, cfg->pwm_freq_hz, cfg->pwm_ch,
						   cfg->n_half_bridges, cfg->trgo);
	}
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

	(void)hbridge_stm32_disable_outputs(dev);

	return 0;
}

static const struct motor_actuator_ops motor_hbridge_stm32_api = {
	.init = hbridge_stm32_api_init,
	.enable = hbridge_stm32_enable,
	.disable = hbridge_stm32_disable,
	.set_command = hbridge_stm32_set_command,
	.set_duty = hbridge_stm32_set_duty,
	.set_drive_mode = hbridge_stm32_set_drive_mode,
	.set_control_callback = hbridge_stm32_set_control_callback,
	.set_fault_callback = hbridge_stm32_set_fault_callback,
	.clear_fault = hbridge_stm32_clear_fault,
	.get_fault = hbridge_stm32_get_fault,
	.self_test = hbridge_stm32_self_test,
	.get_config = hbridge_stm32_get_config,
	.sto_arm = hbridge_stm32_sto_arm,
	.sto_release = hbridge_stm32_sto_release,
	.invoke_control_callback = hbridge_stm32_invoke_cb,
};

#define TIM_FROM_PHANDLE(inst) ((TIM_TypeDef *)DT_REG_ADDR(DT_INST_PHANDLE(inst, st_pwm_timer)))

#define HBRIDGE_STM32_DEFINE(inst)                                                                 \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static const uint8_t pwm_ch_array_##inst[] = DT_INST_PROP(inst, pwm_channels);               \
	static const struct motor_hbridge_stm32_config motor_hbridge_stm32_cfg_##inst = {          \
		.tim = TIM_FROM_PHANDLE(inst),                                                     \
		.pwm_ch = pwm_ch_array_##inst,                                                     \
		.n_half_bridges = (uint8_t)ARRAY_SIZE(pwm_ch_array_##inst),                        \
		.complementary = DT_INST_PROP_OR(inst, stm32_complementary_pwm, 0),                \
		.pwm_freq_hz = (uint32_t)DT_INST_PROP(inst, pwm_frequency),                       \
		.trgo = HBRIDGE_STM32_TRGO_FROM_ENUM(inst),                                          \
		.deadtime_ns = (uint32_t)DT_INST_PROP(inst, deadtime_ns),                         \
		.nfault = GPIO_DT_SPEC_INST_GET_OR(inst, nfault_gpios, {0}),                        \
		.nsleep = GPIO_DT_SPEC_INST_GET_OR(inst, nsleep_gpios, {0}),                        \
		.has_nfault = DT_NODE_HAS_PROP(DT_DRV_INST(inst), nfault_gpios),                   \
		.has_nsleep = DT_NODE_HAS_PROP(DT_DRV_INST(inst), nsleep_gpios),                   \
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
	static struct motor_hbridge_stm32_data motor_hbridge_stm32_data_##inst;                    \
	static int motor_hbridge_stm32_init_##inst(const struct device *dev)                       \
	{                                                                                            \
		int err_ = pinctrl_apply_state(PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                 \
					       PINCTRL_STATE_DEFAULT);                               \
		if (err_ != 0) {                                                                     \
			return err_;                                                                 \
		}                                                                                    \
		return motor_hbridge_stm32_init(dev);                                                \
	}                                                                                            \
	DEVICE_DT_DEFINE(DT_DRV_INST(inst), motor_hbridge_stm32_init_##inst, NULL,                 \
			 &motor_hbridge_stm32_data_##inst, &motor_hbridge_stm32_cfg_##inst,          \
			 POST_KERNEL, CONFIG_MOTOR_STAGE_HBRIDGE_STM32_INIT_PRIORITY,                \
			 &motor_hbridge_stm32_api);

DT_INST_FOREACH_STATUS_OKAY(HBRIDGE_STM32_DEFINE)
