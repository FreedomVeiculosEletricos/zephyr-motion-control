/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * STM32: injected ADC + TIM TRGO; ADC JEOC ISR latches current and invokes the
 * power-stage control callback (no adc_read in the fast path).
 */

#define DT_DRV_COMPAT zephyr_motor_sensor_encoder_stm32

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include <stm32g4xx.h>
#include <stm32g4xx_ll_adc.h>
#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_tim.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/drivers/motor/motor_sensor.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_COUNTER)
#include <zephyr/drivers/counter.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct motor_sensor_enc_stm32_config {
	ADC_TypeDef *adc;
	uint32_t adc_ch_decimal;
	/** LL_ADC_INJ_TRIG_* when set from DT; UINT32_MAX = derive from trigger_tim */
	uint32_t inj_trig_explicit;
	TIM_TypeDef *trigger_tim;
	const struct device *sync_actuator;
	const struct device *encoder;
	uint32_t encoder_cpr;
	uint8_t pole_pairs;
	float amps_per_volt;
};

struct motor_sensor_enc_stm32_data {
	struct motor_sensor_output last;
	float current_offset[3];
	uint32_t prev_cnt;
	uint64_t prev_cycles;
	atomic_t cal_done;
	uint16_t raw_latched;
};

static void adc_enable_rcc(ADC_TypeDef *adc)
{
	if ((adc == ADC1) || (adc == ADC2)) {
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
	}
}

static uint32_t inj_trig_from_tim(const TIM_TypeDef *tim)
{
	if (tim == TIM1) {
		return LL_ADC_INJ_TRIG_EXT_TIM1_TRGO;
	}
	if (tim == TIM2) {
		return LL_ADC_INJ_TRIG_EXT_TIM2_TRGO;
	}
	if (tim == TIM3) {
		return LL_ADC_INJ_TRIG_EXT_TIM3_TRGO;
	}
	if (tim == TIM4) {
		return LL_ADC_INJ_TRIG_EXT_TIM4_TRGO;
	}
	if (tim == TIM8) {
		return LL_ADC_INJ_TRIG_EXT_TIM8_TRGO;
	}
	return LL_ADC_INJ_TRIG_EXT_TIM3_TRGO;
}

static int adc_inj_init(ADC_TypeDef *adc, uint32_t adc_ch_decimal, uint32_t inj_trig)
{
	uint32_t ch = __LL_ADC_DECIMAL_NB_TO_CHANNEL(adc_ch_decimal);

	if (adc == NULL) {
		return -EINVAL;
	}

	adc_enable_rcc(adc);

	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(adc), LL_ADC_CLOCK_ASYNC_DIV1);

	LL_ADC_DisableDeepPowerDown(adc);
	LL_ADC_EnableInternalRegulator(adc);
	k_busy_wait(LL_ADC_DELAY_INTERNAL_REGUL_STAB_US);

	LL_ADC_StartCalibration(adc, LL_ADC_SINGLE_ENDED);
	while (LL_ADC_IsCalibrationOnGoing(adc) != 0UL) {
	}

	LL_ADC_Disable(adc);
	while (LL_ADC_IsDisableOngoing(adc)) {
	}

	LL_ADC_SetResolution(adc, LL_ADC_RESOLUTION_12B);
	LL_ADC_SetDataAlignment(adc, LL_ADC_DATA_ALIGN_RIGHT);
	LL_ADC_SetLowPowerMode(adc, LL_ADC_LP_MODE_NONE);
	LL_ADC_REG_SetOverrun(adc, LL_ADC_REG_OVR_DATA_OVERWRITTEN);

	LL_ADC_REG_SetSequencerLength(adc, LL_ADC_REG_SEQ_SCAN_DISABLE);

	LL_ADC_INJ_SetSequencerLength(adc, LL_ADC_INJ_SEQ_SCAN_DISABLE);
	LL_ADC_INJ_SetSequencerRanks(adc, LL_ADC_INJ_RANK_1, ch);
	LL_ADC_SetChannelSamplingTime(adc, ch, LL_ADC_SAMPLINGTIME_6CYCLES_5);
	LL_ADC_INJ_SetTrigAuto(adc, LL_ADC_INJ_TRIG_INDEPENDENT);

	LL_ADC_INJ_SetTriggerSource(adc, inj_trig);
	LL_ADC_INJ_SetTriggerEdge(adc, LL_ADC_INJ_TRIG_EXT_RISING);

	LL_ADC_Enable(adc);
	while (LL_ADC_IsActiveFlag_ADRDY(adc) == 0) {
	}
	LL_ADC_ClearFlag_ADRDY(adc);

	LL_ADC_EnableIT_JEOC(adc);

	return 0;
}

static int adc_inj_read_sw(ADC_TypeDef *adc, uint32_t adc_ch_decimal, uint16_t *raw_out)
{
	uint32_t ch = __LL_ADC_DECIMAL_NB_TO_CHANNEL(adc_ch_decimal);
	uint32_t saved_trig;

	if ((adc == NULL) || (raw_out == NULL)) {
		return -EINVAL;
	}

	saved_trig = LL_ADC_INJ_GetTriggerSource(adc);
	LL_ADC_INJ_SetTriggerSource(adc, LL_ADC_INJ_TRIG_SOFTWARE);

	LL_ADC_INJ_SetSequencerRanks(adc, LL_ADC_INJ_RANK_1, ch);

	LL_ADC_INJ_StartConversion(adc);
	while (LL_ADC_IsActiveFlag_JEOC(adc) == 0UL) {
	}
	LL_ADC_ClearFlag_JEOC(adc);

	*raw_out = LL_ADC_INJ_ReadConversionData12(adc, LL_ADC_INJ_RANK_1);

	LL_ADC_INJ_SetTriggerSource(adc, saved_trig);

	return 0;
}

static uint16_t adc_inj_data_raw(const ADC_TypeDef *adc)
{
	return LL_ADC_INJ_ReadConversionData12(adc, LL_ADC_INJ_RANK_1);
}

static void enc_stm32_latch_raw(struct motor_sensor_enc_stm32_data *data, uint16_t raw)
{
	data->raw_latched = raw;
}

static void adc_isr_cb(const void *arg)
{
	const struct device *dev = arg;
	const struct motor_sensor_enc_stm32_config *cfg = dev->config;
	struct motor_sensor_enc_stm32_data *data = dev->data;

	if (LL_ADC_IsActiveFlag_JEOC(cfg->adc) != 0UL) {
		LL_ADC_ClearFlag_JEOC(cfg->adc);
		enc_stm32_latch_raw(data, adc_inj_data_raw(cfg->adc));
		motor_actuator_invoke_control_callback(cfg->sync_actuator);
	}
}

static int sensor_enc_stm32_init(const struct device *dev, const struct motor_sensor_cal *cal)
{
	struct motor_sensor_enc_stm32_data *data = dev->data;

	memset(&data->last, 0, sizeof(data->last));
	data->current_offset[0] = 0.0f;
	data->current_offset[1] = 0.0f;
	data->current_offset[2] = 0.0f;
	data->prev_cnt = 0U;
	data->prev_cycles = 0U;
	atomic_set(&data->cal_done, 0);
	data->raw_latched = 0U;

	if (cal != NULL) {
		data->current_offset[0] = cal->current_offset[0];
		data->current_offset[1] = cal->current_offset[1];
		data->current_offset[2] = cal->current_offset[2];
	}

	return 0;
}

static void apply_raw_to_output(const struct motor_sensor_enc_stm32_config *cfg,
				struct motor_sensor_enc_stm32_data *data, uint16_t raw)
{
	float v_lsb = 3.3f / 4096.0f;
	float v = (float)raw * v_lsb;
	float ia = v * cfg->amps_per_volt;

	ia -= data->current_offset[0];

	data->last.hot.i_phase[0] = ia;
	data->last.hot.i_phase[1] = 0.0f;
	data->last.hot.i_phase[2] = 0.0f;
	data->last.hot.valid_current = true;
	data->last.hot.stale = false;
}

static int sensor_enc_stm32_update(const struct device *dev)
{
	const struct motor_sensor_enc_stm32_config *cfg = dev->config;
	struct motor_sensor_enc_stm32_data *data = dev->data;

	apply_raw_to_output(cfg, data, data->raw_latched);

#if defined(CONFIG_COUNTER)
	if (cfg->encoder != NULL) {
		uint32_t cnt;
		uint64_t now = k_cycle_get_64();
		int err;

		err = counter_get_value(cfg->encoder, &cnt);
		if (err == 0) {
			float mech = (2.0f * (float)M_PI * (float)cnt) / (float)cfg->encoder_cpr;
			float elec = mech * (float)cfg->pole_pairs;

			while (elec >= (float)(2.0 * M_PI)) {
				elec -= (float)(2.0 * M_PI);
			}
			while (elec < 0.0f) {
				elec += (float)(2.0 * M_PI);
			}

			data->last.hot.theta_rad = elec;
			data->last.hot.valid_theta = true;

			if (data->prev_cycles != 0ULL) {
				uint64_t dcyc = now - data->prev_cycles;
				float dt = (float)dcyc / (float)sys_clock_hw_cycles_per_sec();

				if (dt > 1e-9f && cfg->encoder_cpr > 0U) {
					int32_t dcnt = (int32_t)cnt - (int32_t)data->prev_cnt;

					data->last.hot.omega_rad_s =
						((float)dcnt * 2.0f * (float)M_PI * (float)cfg->pole_pairs) /
						((float)cfg->encoder_cpr * dt);
					data->last.hot.valid_omega = true;
				}
			} else {
				data->last.hot.valid_omega = false;
			}

			data->prev_cnt = cnt;
			data->prev_cycles = now;
		}
	} else {
		data->last.hot.valid_theta = false;
		data->last.hot.valid_omega = false;
	}
#else
	data->last.hot.valid_theta = false;
	data->last.hot.valid_omega = false;
#endif

	return 0;
}

static int sensor_enc_stm32_get(const struct device *dev, struct motor_sensor_output *out)
{
	const struct motor_sensor_enc_stm32_data *data = dev->data;

	if (out == NULL) {
		return -EINVAL;
	}

	*out = data->last;
	return 0;
}

static int sensor_enc_stm32_calibrate(const struct device *dev)
{
	const struct motor_sensor_enc_stm32_config *cfg = dev->config;
	struct motor_sensor_enc_stm32_data *data = dev->data;
	float sum = 0.0f;
	uint16_t raw;
	int err;

	for (int i = 0; i < 32; i++) {
		err = adc_inj_read_sw(cfg->adc, cfg->adc_ch_decimal, &raw);
		if (err != 0) {
			return err;
		}
		{
			float v_lsb = 3.3f / 4096.0f;
			float v = (float)raw * v_lsb;

			sum += v * cfg->amps_per_volt;
		}
		k_sleep(K_MSEC(1));
	}

	data->current_offset[0] = sum / 32.0f;
	atomic_set(&data->cal_done, 1);

	return 0;
}

static void sensor_enc_stm32_get_backend_types(const struct device *dev,
					      enum motor_pos_backend_type *pos_type,
					      enum motor_cur_backend_type *cur_type)
{
#if defined(CONFIG_COUNTER)
	const struct motor_sensor_enc_stm32_config *cfg = dev->config;
#endif

	if (pos_type != NULL) {
#if defined(CONFIG_COUNTER)
		*pos_type = (cfg->encoder != NULL) ? MOTOR_POS_QUADRATURE_ENCODER : MOTOR_POS_NONE;
#else
		ARG_UNUSED(dev);
		*pos_type = MOTOR_POS_NONE;
#endif
	}

	if (cur_type != NULL) {
		*cur_type = MOTOR_CUR_1SHUNT_DCLINK;
	}
}

static int sensor_enc_stm32_dev_init(const struct device *dev)
{
	const struct motor_sensor_enc_stm32_config *cfg = dev->config;
	int err;

	if (!device_is_ready(cfg->sync_actuator)) {
		return -ENODEV;
	}

#if defined(CONFIG_COUNTER)
	if ((cfg->encoder != NULL) && !device_is_ready(cfg->encoder)) {
		return -ENODEV;
	}
#endif

	uint32_t inj_trig = cfg->inj_trig_explicit;

	if (inj_trig == UINT32_MAX) {
		inj_trig = inj_trig_from_tim(cfg->trigger_tim);
	}

	err = adc_inj_init(cfg->adc, cfg->adc_ch_decimal, inj_trig);
	if (err != 0) {
		return err;
	}

	/*
	 * ARCH_IRQ_CONNECT expands to a braced block — must run inside a function
	 * (not at file scope). Single-instance: fixed inst 0 for IRQ line + DT prio.
	 */
	BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1U,
		     "motor_sensor_encoder_stm32: IRQ_CONNECT uses instance 0 only");
	IRQ_CONNECT(ADC1_2_IRQn,
		    DT_INST_PROP_OR(0, zephyr_adc_irq_priority, CONFIG_MOTOR_STM32_ADC_IRQ_PRIORITY),
		    adc_isr_cb, DEVICE_DT_GET(DT_DRV_INST(0)), 0);
	irq_enable(ADC1_2_IRQn);

	return 0;
}

static const struct motor_sensor_ops motor_sensor_enc_stm32_api = {
	.init = sensor_enc_stm32_init,
	.update = sensor_enc_stm32_update,
	.get = sensor_enc_stm32_get,
	.calibrate = sensor_enc_stm32_calibrate,
	.get_backend_types = sensor_enc_stm32_get_backend_types,
};

#define ADC_FROM_PHANDLE(inst) ((ADC_TypeDef *)DT_REG_ADDR(DT_INST_PHANDLE(inst, st_adc)))
#define TIM_FROM_PHANDLE(inst) ((TIM_TypeDef *)DT_REG_ADDR(DT_INST_PHANDLE(inst, adc_trigger_timer)))

#define AMPS_PER_VOLT_DEFAULT 1.0f

#define SENSOR_ENC_STM32_DEFINE(inst)                                                              \
	static const struct motor_sensor_enc_stm32_config sensor_enc_stm32_cfg_##inst = {          \
		.adc = ADC_FROM_PHANDLE(inst),                                                     \
		.adc_ch_decimal = (uint32_t)DT_INST_PROP(inst, adc_channel),                       \
		.inj_trig_explicit = COND_CODE_1(DT_NODE_HAS_PROP(DT_DRV_INST(inst),               \
								  adc_ext_trigger_inj),          \
						 (DT_INST_PROP(inst, adc_ext_trigger_inj)),                \
						 (UINT32_MAX)),                                            \
		.trigger_tim = TIM_FROM_PHANDLE(inst),                                             \
		.sync_actuator = DEVICE_DT_GET(DT_INST_PHANDLE(inst, sync_actuator)),              \
		.encoder = COND_CODE_1(DT_NODE_HAS_PROP(DT_DRV_INST(inst), encoder_timer),          \
				       (DEVICE_DT_GET(DT_INST_PHANDLE(inst, encoder_timer))),        \
				       (NULL)),                                                    \
		.encoder_cpr = (uint32_t)DT_INST_PROP(inst, encoder_cpr),                            \
		.pole_pairs = (uint8_t)DT_INST_PROP(inst, pole_pairs),                               \
		.amps_per_volt = AMPS_PER_VOLT_DEFAULT,                                              \
	};                                                                                         \
	static struct motor_sensor_enc_stm32_data sensor_enc_stm32_data_##inst;                    \
	DEVICE_DT_DEFINE(DT_DRV_INST(inst), sensor_enc_stm32_dev_init, NULL,                       \
			 &sensor_enc_stm32_data_##inst, &sensor_enc_stm32_cfg_##inst, POST_KERNEL, \
			 CONFIG_MOTOR_SENSOR_ENCODER_STM32_INIT_PRIORITY, &motor_sensor_enc_stm32_api)

DT_INST_FOREACH_STATUS_OKAY(SENSOR_ENC_STM32_DEFINE)
