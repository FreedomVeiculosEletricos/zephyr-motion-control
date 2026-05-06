/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_motor_sensor_stm32

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include <stm32g4xx.h>
#include <stm32g4xx_ll_adc.h>
#include <stm32g4xx_ll_bus.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/drivers/motor/motor_sensor.h>
#include <zephyr/irq.h>
#if IS_ENABLED(CONFIG_SENSOR)
#include <zephyr/drivers/sensor.h>
#endif
#include <zephyr/sys/util.h>

static int motor_sensor_stm32_check_adc_pwm_margin(const struct device *sync_actuator,
						   uint8_t n_adc_channels)
{
	const struct motor_stage_config *stage = motor_actuator_get_config(sync_actuator);
	uint64_t pwm_ns;
	uint64_t tconv_ns;
	uint32_t adc_ker_hz;
	uint32_t cyc;

	if (stage == NULL) {
		return -EINVAL;
	}

	pwm_ns = stage->pwm_period_ns;
	if (pwm_ns == 0U) {
		return -EINVAL;
	}

	if ((NSEC_PER_SEC % pwm_ns) != 0ULL) {
		return -EINVAL;
	}

	adc_ker_hz = SystemCoreClock;
	if (adc_ker_hz < 1000000U) {
		return -EINVAL;
	}

	cyc = 25U * (uint32_t)n_adc_channels;
	tconv_ns = ((uint64_t)cyc * (uint64_t)NSEC_PER_SEC) / (uint64_t)adc_ker_hz;

	if (tconv_ns >= pwm_ns) {
		return -EIO;
	}

	return 0;
}

struct motor_sensor_stm32_config {
	ADC_TypeDef *adc;
	const struct device *sync_actuator;
	const struct device *feedback_sensor;
	float amps_per_volt;
	uint32_t vref_mv;
	uint8_t resolution_bits;
};

struct motor_sensor_stm32_data {
	uint32_t adc_ch_decimal[MOTOR_SENSOR_CURRENT_MAX];
	uint8_t n_adc_channels;
	float current_amps[MOTOR_SENSOR_CURRENT_MAX];
	uint16_t raw_latched[MOTOR_SENSOR_CURRENT_MAX];
	float current_offset[MOTOR_SENSOR_CURRENT_MAX];
	/* Cached at init: (vref_v / counts) * amps_per_volt. Hot path uses
	 * this as the single per-channel scale factor.
	 */
	float amps_per_count;
	float angle_rad;
	motor_sensor_measurement_done_cb_t measurement_done_cb;
	void *measurement_done_user_data;
};

static uint32_t stm32_ll_adc_resolution_from_bits(uint8_t bits)
{
	switch (bits) {
	case 6U:
		return LL_ADC_RESOLUTION_6B;
	case 8U:
		return LL_ADC_RESOLUTION_8B;
	case 10U:
		return LL_ADC_RESOLUTION_10B;
	case 12U:
	default:
		return LL_ADC_RESOLUTION_12B;
	}
}

static const uint32_t inj_rank[4] = {
	LL_ADC_INJ_RANK_1,
	LL_ADC_INJ_RANK_2,
	LL_ADC_INJ_RANK_3,
	LL_ADC_INJ_RANK_4,
};

static void adc_enable_rcc(ADC_TypeDef *adc)
{
	if ((adc == ADC1) || (adc == ADC2)) {
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
	}
}

static uint32_t inj_seq_len_from_n(uint8_t n)
{
	switch (n) {
	case 1:
		return LL_ADC_INJ_SEQ_SCAN_DISABLE;
	case 2:
		return LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS;
	case 3:
		return LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS;
	case 4:
		return LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS;
	default:
		return LL_ADC_INJ_SEQ_SCAN_DISABLE;
	}
}

static int adc_inj_configure(ADC_TypeDef *adc, const uint32_t *ch_decimal, uint8_t n,
			     uint8_t resolution_bits)
{
	uint8_t i;

	if ((adc == NULL) || (ch_decimal == NULL) || (n == 0U) || (n > 4U)) {
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

	LL_ADC_SetResolution(adc, stm32_ll_adc_resolution_from_bits(resolution_bits));
	LL_ADC_SetDataAlignment(adc, LL_ADC_DATA_ALIGN_RIGHT);
	LL_ADC_SetLowPowerMode(adc, LL_ADC_LP_MODE_NONE);
	LL_ADC_REG_SetOverrun(adc, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
	LL_ADC_REG_SetSequencerLength(adc, LL_ADC_REG_SEQ_SCAN_DISABLE);

	LL_ADC_INJ_SetSequencerLength(adc, inj_seq_len_from_n(n));
	LL_ADC_INJ_SetTrigAuto(adc, LL_ADC_INJ_TRIG_INDEPENDENT);
	LL_ADC_INJ_SetTriggerSource(adc, LL_ADC_INJ_TRIG_SOFTWARE);
	LL_ADC_INJ_SetTriggerEdge(adc, LL_ADC_INJ_TRIG_EXT_RISING);

	for (i = 0U; i < n; i++) {
		uint32_t ch = __LL_ADC_DECIMAL_NB_TO_CHANNEL(ch_decimal[i]);

		LL_ADC_INJ_SetSequencerRanks(adc, inj_rank[i], ch);
		LL_ADC_SetChannelSamplingTime(adc, ch, LL_ADC_SAMPLINGTIME_6CYCLES_5);
	}

	LL_ADC_Enable(adc);
	while (LL_ADC_IsActiveFlag_ADRDY(adc) == 0) {
	}
	LL_ADC_ClearFlag_ADRDY(adc);
	LL_ADC_EnableIT_JEOC(adc);

	return 0;
}

static int adc_inj_sw_read(ADC_TypeDef *adc, uint8_t n, uint16_t *raw_out)
{
	uint8_t i;

	if ((adc == NULL) || (raw_out == NULL) || (n == 0U) || (n > 4U)) {
		return -EINVAL;
	}

	LL_ADC_INJ_StartConversion(adc);
	while (LL_ADC_IsActiveFlag_JEOC(adc) == 0UL) {
	}
	LL_ADC_ClearFlag_JEOC(adc);

	for (i = 0U; i < n; i++) {
		raw_out[i] = LL_ADC_INJ_ReadConversionData12(adc, inj_rank[i]);
	}

	return 0;
}

static void adc_latch_isr(const struct device *dev)
{
	const struct motor_sensor_stm32_config *cfg = dev->config;
	struct motor_sensor_stm32_data *data = dev->data;
	ADC_TypeDef *adc = cfg->adc;
	float scale = data->amps_per_count;
	uint8_t i;

	if (LL_ADC_IsActiveFlag_JEOC(adc) == 0UL) {
		return;
	}

	LL_ADC_ClearFlag_JEOC(adc);

	for (i = 0U; i < data->n_adc_channels; i++) {
		data->raw_latched[i] = LL_ADC_INJ_ReadConversionData12(adc, inj_rank[i]);
	}

	for (i = 0U; i < data->n_adc_channels; i++) {
		data->current_amps[i] = (float)data->raw_latched[i] * scale -
					data->current_offset[i];
	}

	if (data->measurement_done_cb != NULL) {
		data->measurement_done_cb(dev, data->measurement_done_user_data);
	}
}

static int motor_sensor_stm32_start_sample(const struct device *dev, enum motor_sensor_channel ch)
{
	__maybe_unused const struct motor_sensor_stm32_config *cfg = dev->config;
	__maybe_unused struct motor_sensor_stm32_data *data = dev->data;

	switch (ch) {
	case MOTOR_SENSOR_CHAN_CURRENT:
		LL_ADC_INJ_StartConversion(cfg->adc);
		return 0;
	case MOTOR_SENSOR_CHAN_ANGLE:
#if !IS_ENABLED(CONFIG_SENSOR)
		return -ENOTSUP;
#else
		if (cfg->feedback_sensor == NULL) {
			return -ENOTSUP;
		}
		if (k_is_in_isr()) {
			return MOTOR_SENSOR_ERR_CONTEXT;
		}
		{
			int err = sensor_sample_fetch(cfg->feedback_sensor);

			if (err != 0) {
				return err;
			}
			{
				struct sensor_value val;

				err = sensor_channel_get(cfg->feedback_sensor, SENSOR_CHAN_ROTATION, &val);
				if (err != 0) {
					return err;
				}
				data->angle_rad = (float)sensor_value_to_double(&val);
			}
		}
		return 0;
#endif
	default:
		return -EINVAL;
	}
}

static int motor_sensor_stm32_get_sample(const struct device *dev, enum motor_sensor_channel ch,
					 float *out, size_t out_len, size_t *got)
{
	__maybe_unused const struct motor_sensor_stm32_config *cfg = dev->config;
	struct motor_sensor_stm32_data *data = dev->data;
	uint8_t i;

	switch (ch) {
	case MOTOR_SENSOR_CHAN_CURRENT:
		if (out_len < data->n_adc_channels) {
			return -EINVAL;
		}
		for (i = 0U; i < data->n_adc_channels; i++) {
			out[i] = data->current_amps[i];
		}
		*got = data->n_adc_channels;
		return 0;
	case MOTOR_SENSOR_CHAN_ANGLE:
#if !IS_ENABLED(CONFIG_SENSOR)
		return -ENOTSUP;
#else
		if (cfg->feedback_sensor == NULL) {
			return -ENOTSUP;
		}
		if (out_len < 1U) {
			return -EINVAL;
		}
		out[0] = data->angle_rad;
		*got = 1U;
		return 0;
#endif
	default:
		return -EINVAL;
	}
}

static int motor_sensor_stm32_calibrate(const struct device *dev, enum motor_sensor_channel ch)
{
	const struct motor_sensor_stm32_config *cfg = dev->config;
	struct motor_sensor_stm32_data *data = dev->data;
	float scale = data->amps_per_count;
	uint8_t i;

	if (ch != MOTOR_SENSOR_CHAN_CURRENT) {
		return -ENOTSUP;
	}

	for (i = 0U; i < data->n_adc_channels; i++) {
		float sum = 0.0f;
		int s;

		for (s = 0; s < 32; s++) {
			uint16_t raw;
			int err = adc_inj_configure(cfg->adc, &data->adc_ch_decimal[i], 1,
						    cfg->resolution_bits);

			if (err != 0) {
				return err;
			}
			err = adc_inj_sw_read(cfg->adc, 1, &raw);
			if (err != 0) {
				return err;
			}
			sum += (float)raw * scale;
			k_sleep(K_MSEC(1));
		}

		data->current_offset[i] = sum * (1.0f / 32.0f);
	}

	return adc_inj_configure(cfg->adc, data->adc_ch_decimal, data->n_adc_channels,
				 cfg->resolution_bits);
}

static bool motor_sensor_stm32_channel_supported(const struct device *dev,
						 enum motor_sensor_channel ch)
{
	__maybe_unused const struct motor_sensor_stm32_config *cfg = dev->config;
	struct motor_sensor_stm32_data *data = dev->data;

	switch (ch) {
	case MOTOR_SENSOR_CHAN_CURRENT:
		return data->n_adc_channels > 0U;
	case MOTOR_SENSOR_CHAN_ANGLE:
#if !IS_ENABLED(CONFIG_SENSOR)
		return false;
#else
		return cfg->feedback_sensor != NULL;
#endif
	default:
		return false;
	}
}

static int motor_sensor_stm32_set_measurement_done_cb(const struct device *dev,
						      motor_sensor_measurement_done_cb_t cb,
						      void *user_data)
{
	struct motor_sensor_stm32_data *data = dev->data;

	data->measurement_done_cb = cb;
	data->measurement_done_user_data = user_data;
	return 0;
}

static const struct motor_sensor_ops motor_sensor_stm32_api = {
	.start_sample = motor_sensor_stm32_start_sample,
	.get_sample = motor_sensor_stm32_get_sample,
	.calibrate = motor_sensor_stm32_calibrate,
	.channel_supported = motor_sensor_stm32_channel_supported,
	.set_measurement_done_callback = motor_sensor_stm32_set_measurement_done_cb,
};

#define ADC_FROM_PHANDLE(inst) ((ADC_TypeDef *)DT_REG_ADDR(DT_INST_PHANDLE(inst, adc)))

#define SENSOR_STM32_DEFINE(inst)                                                                  \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, adc_channels) >= 1);                                     \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, adc_channels) <= MOTOR_SENSOR_CURRENT_MAX);              \
	static const uint32_t motor_sensor_stm32_adc_ch_##inst[] = DT_INST_PROP(inst, adc_channels); \
	BUILD_ASSERT(DT_INST_PROP(inst, adc_resolution_bits) == 6 ||                              \
			     DT_INST_PROP(inst, adc_resolution_bits) == 8 ||                       \
			     DT_INST_PROP(inst, adc_resolution_bits) == 10 ||                      \
			     DT_INST_PROP(inst, adc_resolution_bits) == 12,                        \
		     "adc-resolution-bits must be one of 6, 8, 10, 12 on STM32");                  \
	static const struct motor_sensor_stm32_config motor_sensor_stm32_cfg_##inst = {            \
		.adc = ADC_FROM_PHANDLE(inst),                                                     \
		.sync_actuator = DEVICE_DT_GET(DT_INST_PHANDLE(inst, sync_actuator)),              \
		.feedback_sensor =                                                                   \
			COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, feedback_sensor),                    \
				    (DEVICE_DT_GET(DT_INST_PHANDLE(inst, feedback_sensor))),         \
				    (NULL)),                                                         \
		.amps_per_volt =                                                                     \
			(float)DT_INST_PROP_OR(inst, amps_per_volt_milli, 1000) / 1000.0f,           \
		.vref_mv = (uint32_t)DT_INST_PROP(inst, adc_vref_mv),                                \
		.resolution_bits = (uint8_t)DT_INST_PROP(inst, adc_resolution_bits),                 \
	};                                                                                         \
	static struct motor_sensor_stm32_data motor_sensor_stm32_data_##inst;                    \
	ISR_DIRECT_DECLARE(motor_sensor_stm32_adc_isr_##inst)                                    \
	{                                                                                          \
		adc_latch_isr(DEVICE_DT_GET(DT_DRV_INST(inst)));                                  \
		return 0;                                                                          \
	}                                                                                          \
	static int motor_sensor_stm32_dev_init_##inst(const struct device *dev)                    \
	{                                                                                          \
		const struct motor_sensor_stm32_config *cfg = dev->config;                         \
		struct motor_sensor_stm32_data *data = dev->data;                                  \
		int err;                                                                           \
		uint8_t i;                                                                         \
		if (!device_is_ready(cfg->sync_actuator)) {                                        \
			return -ENODEV;                                                            \
		}                                                                                  \
		if ((cfg->feedback_sensor != NULL) && !device_is_ready(cfg->feedback_sensor)) {   \
			return -ENODEV;                                                            \
		}                                                                                  \
		data->n_adc_channels = (uint8_t)DT_INST_PROP_LEN(inst, adc_channels);                \
		for (i = 0U; i < data->n_adc_channels; i++) {                                      \
			data->adc_ch_decimal[i] = motor_sensor_stm32_adc_ch_##inst[i];              \
			data->current_offset[i] = 0.0f;                                             \
		}                                                                                  \
		data->amps_per_count = (((float)cfg->vref_mv * 1e-3f) /                              \
					 (float)(1U << cfg->resolution_bits)) *                       \
				       cfg->amps_per_volt;                                            \
		err = motor_sensor_stm32_check_adc_pwm_margin(cfg->sync_actuator,                    \
								data->n_adc_channels);                 \
		if (err != 0) {                                                                    \
			return err;                                                                \
		}                                                                                  \
		data->measurement_done_cb = NULL;                                                  \
		data->measurement_done_user_data = NULL;                                           \
		data->angle_rad = 0.0f;                                                            \
		err = adc_inj_configure(cfg->adc, data->adc_ch_decimal, data->n_adc_channels,        \
					cfg->resolution_bits);                                       \
		if (err != 0) {                                                                    \
			return err;                                                                \
		}                                                                                  \
		BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1U,                         \
			     "motor_sensor_stm32: single ADC1_2 IRQ bundle");                      \
		IRQ_DIRECT_CONNECT(ADC1_2_IRQn,                                                  \
				   DT_INST_PROP_OR(inst, zephyr_adc_irq_priority,                   \
						   CONFIG_MOTOR_STM32_ADC_IRQ_PRIORITY),            \
				   motor_sensor_stm32_adc_isr_##inst, IRQ_ZERO_LATENCY);          \
		irq_enable(ADC1_2_IRQn);                                                         \
		return 0;                                                                          \
	}                                                                                          \
	DEVICE_DT_DEFINE(DT_DRV_INST(inst), motor_sensor_stm32_dev_init_##inst, NULL,              \
			 &motor_sensor_stm32_data_##inst, &motor_sensor_stm32_cfg_##inst,            \
			 POST_KERNEL, CONFIG_MOTOR_SENSOR_STM32_INIT_PRIORITY,                      \
			 &motor_sensor_stm32_api)

DT_INST_FOREACH_STATUS_OKAY(SENSOR_STM32_DEFINE)
