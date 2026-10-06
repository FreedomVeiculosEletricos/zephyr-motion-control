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

#include <stm32_ll_adc.h>
#include <stm32_ll_system.h>
#include <stm32_ll_tim.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/drivers/motor/motor_sensor.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#if IS_ENABLED(CONFIG_SENSOR)
#include <zephyr/drivers/sensor.h>
#endif
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(motor_sensor_stm32, CONFIG_MOTOR_DRIVER_LOG_LEVEL);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Worst case is calibration on the slowest supported kernel clock, which still
 * completes in tens of microseconds. Anything longer means the ADC is not
 * clocked or is waiting on a trigger that will never come.
 */
#define MOTOR_STM32_ADC_TIMEOUT_US 1000U

#define MOTOR_STM32_ADC_WAIT_FOR(cond, what)                                                       \
	({                                                                                         \
		int _err = 0;                                                                      \
		uint32_t _left = MOTOR_STM32_ADC_TIMEOUT_US;                                       \
                                                                                                   \
		while (!(cond)) {                                                                  \
			if (_left == 0U) {                                                         \
				LOG_ERR("timed out waiting for %s", what);                         \
				_err = -ETIMEDOUT;                                                 \
				break;                                                             \
			}                                                                          \
			_left--;                                                                   \
			k_busy_wait(1);                                                            \
		}                                                                                  \
		_err;                                                                              \
	})

#if defined(LL_ADC_SAMPLINGTIME_6CYCLES_5)
#define MOTOR_STM32_ADC_SAMPLE_TIME LL_ADC_SAMPLINGTIME_6CYCLES_5
#elif defined(LL_ADC_SAMPLINGTIME_8CYCLES_5)
#define MOTOR_STM32_ADC_SAMPLE_TIME LL_ADC_SAMPLINGTIME_8CYCLES_5
#elif defined(LL_ADC_SAMPLINGTIME_12CYCLES_5)
#define MOTOR_STM32_ADC_SAMPLE_TIME LL_ADC_SAMPLINGTIME_12CYCLES_5
#else
#error "Unsupported STM32 LL ADC sampling time macro"
#endif

static int motor_sensor_stm32_check_adc_pwm_margin(const struct device *sync_actuator,
						   uint8_t n_adc_channels, uint32_t adc_ker_hz)
{
	const struct motor_stage_config *stage = motor_actuator_get_config(sync_actuator);
	uint64_t pwm_ns;
	uint64_t tconv_ns;
	uint32_t cyc;

	if (stage == NULL) {
		LOG_ERR("sync actuator has no stage config");
		return -EINVAL;
	}

	pwm_ns = stage->pwm_period_ns;
	if (pwm_ns == 0U) {
		LOG_ERR("sync actuator reports a zero PWM period");
		return -EINVAL;
	}

	if ((NSEC_PER_SEC % pwm_ns) != 0ULL) {
		LOG_ERR("PWM period %llu ns does not divide one second", pwm_ns);
		return -EINVAL;
	}

	if (adc_ker_hz < 1000000U) {
		LOG_ERR("ADC kernel clock %u Hz is too slow to sample current", adc_ker_hz);
		return -EINVAL;
	}

	cyc = 25U * (uint32_t)n_adc_channels;
	tconv_ns = ((uint64_t)cyc * (uint64_t)NSEC_PER_SEC) / (uint64_t)adc_ker_hz;

	if (tconv_ns >= pwm_ns) {
		LOG_ERR("conversion of %u channels takes %llu ns, PWM period is %llu ns",
			n_adc_channels, tconv_ns, pwm_ns);
		return -EIO;
	}

	return 0;
}

struct motor_sensor_stm32_config {
	ADC_TypeDef *adc;
	const struct device *sync_actuator;
	const struct device *feedback_sensor;
	const struct stm32_pclken *pclken;
	size_t pclk_len;
	uint32_t adc_common_clock;
	/* Timer behind sync_actuator, whose TRGO starts the conversion when
	 * hw_trigger is set. NULL when the paired stage has no timer to offer.
	 */
	TIM_TypeDef *sync_tim;
	bool hw_trigger;
	/* Which bridge leg each injected rank measures, or NULL when the board
	 * did not say. Indexes run parallel to adc_ch_decimal.
	 */
	const uint8_t *sense_leg;
	float amps_per_volt;
	uint32_t vref_mv;
	uint8_t resolution_bits;
};

/* The ADC is driven straight through LL here, so nothing else in the system
 * gates its clock: enabling it is part of this driver's init.
 */
static int motor_sensor_stm32_enable_adc_clock(const struct motor_sensor_stm32_config *cfg)
{
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	size_t i;
	int err;

	if (!device_is_ready(clk)) {
		LOG_ERR("clock controller not ready");
		return -ENODEV;
	}

	err = clock_control_on(clk, (clock_control_subsys_t)&cfg->pclken[0]);
	if (err != 0) {
		LOG_ERR("could not enable the ADC bus clock (%d)", err);
		return err;
	}

	/* Further entries select the kernel clock and its RCC prescaler. */
	for (i = 1U; i < cfg->pclk_len; i++) {
		err = clock_control_configure(clk, (clock_control_subsys_t)&cfg->pclken[i], NULL);
		if (err != 0) {
			LOG_ERR("could not configure ADC clock %u (%d)", (unsigned int)i, err);
			return err;
		}
	}

	return 0;
}

/* Divider the LL common-clock constant selects, so the kernel frequency can be
 * derived from the clock the RCC reports.
 */
static uint32_t motor_sensor_stm32_adc_clock_prescaler(uint32_t common_clock)
{
	switch (common_clock) {
#if defined(LL_ADC_CLOCK_SYNC_PCLK_DIV1)
	case LL_ADC_CLOCK_SYNC_PCLK_DIV1:
		return 1U;
	case LL_ADC_CLOCK_SYNC_PCLK_DIV2:
		return 2U;
	case LL_ADC_CLOCK_SYNC_PCLK_DIV4:
		return 4U;
#endif
	case LL_ADC_CLOCK_ASYNC_DIV1:
		return 1U;
#if defined(LL_ADC_CLOCK_ASYNC_DIV2)
	case LL_ADC_CLOCK_ASYNC_DIV2:
		return 2U;
	case LL_ADC_CLOCK_ASYNC_DIV4:
		return 4U;
	case LL_ADC_CLOCK_ASYNC_DIV6:
		return 6U;
	case LL_ADC_CLOCK_ASYNC_DIV8:
		return 8U;
	case LL_ADC_CLOCK_ASYNC_DIV10:
		return 10U;
	case LL_ADC_CLOCK_ASYNC_DIV12:
		return 12U;
	case LL_ADC_CLOCK_ASYNC_DIV16:
		return 16U;
	case LL_ADC_CLOCK_ASYNC_DIV32:
		return 32U;
	case LL_ADC_CLOCK_ASYNC_DIV64:
		return 64U;
	case LL_ADC_CLOCK_ASYNC_DIV128:
		return 128U;
	case LL_ADC_CLOCK_ASYNC_DIV256:
		return 256U;
#endif
	default:
		return 0U;
	}
}

/* H74x/H75x feed the ADC through a fixed extra /2 on every revision but Y.
 * It lives in no register, so it has to be applied by hand — adc_stm32.c does
 * the same thing in adc_stm32_get_input_freq_prescaler().
 */
static uint32_t motor_sensor_stm32_adc_extra_divider(void)
{
#if defined(CONFIG_SOC_SERIES_STM32H7X) && defined(ADC_VER_V5_X)
	return (LL_DBGMCU_GetRevisionID() <= 0x1003U) ? 1U : 2U;
#else
	return 1U;
#endif
}

/* Frequency the converter actually runs at, which is neither the core clock nor
 * the bus clock the devicetree names.
 */
static int motor_sensor_stm32_adc_kernel_freq(const struct motor_sensor_stm32_config *cfg,
					      uint32_t *freq_hz)
{
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	const struct stm32_pclken *src;
	uint32_t presc;
	uint32_t freq;
	bool is_sync;
	int err;

	presc = motor_sensor_stm32_adc_clock_prescaler(cfg->adc_common_clock);
	if (presc == 0U) {
		LOG_ERR("unsupported ADC common clock 0x%08x", cfg->adc_common_clock);
		return -EINVAL;
	}

#if defined(ADC_CCR_CKMODE)
	is_sync = (cfg->adc_common_clock & ADC_CCR_CKMODE) != 0U;
#else
	is_sync = false;
#endif

	/* Synchronous mode runs off the bus clock; asynchronous off the kernel
	 * clock, which is the second pclken entry when the board supplies one.
	 */
	src = (is_sync || (cfg->pclk_len < 2U)) ? &cfg->pclken[0] : &cfg->pclken[1];

	err = clock_control_get_rate(clk, (clock_control_subsys_t)src, &freq);
	if (err != 0) {
		LOG_ERR("could not read the ADC input clock (%d)", err);
		return err;
	}

	*freq_hz = freq / (presc * motor_sensor_stm32_adc_extra_divider());

	return 0;
}

#if defined(LL_ADC_BOOST_MODE_6MHZ25)
/* BOOST sets the bias current of the converter's internal amplifiers and has to
 * match the kernel frequency. Its reset value covers <= 6.25 MHz only, so
 * leaving it alone under-biases every ADC clocked faster than that: conversions
 * still complete, but the result carries an error that grows with how closely
 * they are spaced. A calibration taken from isolated conversions then fails to
 * cancel what a back-to-back injected sequence reads.
 */
static int motor_sensor_stm32_setup_boost(const struct motor_sensor_stm32_config *cfg,
					  ADC_TypeDef *adc)
{
	uint32_t freq;
	uint32_t boost;
	int err;

	err = motor_sensor_stm32_adc_kernel_freq(cfg, &freq);
	if (err != 0) {
		return err;
	}

	if (freq <= KHZ(6250)) {
		boost = LL_ADC_BOOST_MODE_6MHZ25;
	} else if (freq <= KHZ(12500)) {
		boost = LL_ADC_BOOST_MODE_12MHZ5;
	} else if (freq <= MHZ(20)) {
		boost = LL_ADC_BOOST_MODE_20MHZ;
	} else if (freq <= MHZ(25)) {
		boost = LL_ADC_BOOST_MODE_25MHZ;
	} else if (freq <= MHZ(50)) {
		boost = LL_ADC_BOOST_MODE_50MHZ;
	} else {
		LOG_ERR("ADC kernel clock is %u Hz, above the 50 MHz maximum", freq);
		return -ERANGE;
	}

	LL_ADC_SetBoostMode(adc, boost);

	return 0;
}
#endif /* LL_ADC_BOOST_MODE_6MHZ25 */

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
	/* Current through the leg that the applied command clamped, signed by
	 * the direction that command implies. Resolved on every latch, since
	 * the leg carrying the load current moves with the command.
	 */
	float bridge_current_a;
	bool bridge_current_valid;
	float angle_rad;
	motor_sensor_measurement_done_cb_t measurement_done_cb;
	void *measurement_done_user_data;
};

static int stm32_ll_adc_resolution_from_bits(uint8_t bits, uint32_t *out)
{
	switch (bits) {
#ifdef LL_ADC_RESOLUTION_6B
	case 6U:
		*out = LL_ADC_RESOLUTION_6B;
		return 0;
#endif
#ifdef LL_ADC_RESOLUTION_8B
	case 8U:
		*out = LL_ADC_RESOLUTION_8B;
		return 0;
#endif
#ifdef LL_ADC_RESOLUTION_10B
	case 10U:
		*out = LL_ADC_RESOLUTION_10B;
		return 0;
#endif
#ifdef LL_ADC_RESOLUTION_12B
	case 12U:
		*out = LL_ADC_RESOLUTION_12B;
		return 0;
#endif
#ifdef LL_ADC_RESOLUTION_14B
	case 14U:
		*out = LL_ADC_RESOLUTION_14B;
		return 0;
#endif
#ifdef LL_ADC_RESOLUTION_16B
	case 16U:
		*out = LL_ADC_RESOLUTION_16B;
		return 0;
#endif
	default:
		return -EINVAL;
	}
}

static const uint32_t inj_rank[4] = {
	LL_ADC_INJ_RANK_1,
	LL_ADC_INJ_RANK_2,
	LL_ADC_INJ_RANK_3,
	LL_ADC_INJ_RANK_4,
};

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

/* Which JEXTSEL value carries this timer's TRGO.
 *
 * Only a handful of timers are wired to the injected trigger mux, and the set
 * differs per series, so a stage timer that is perfectly good at making PWM may
 * still have no way to start a conversion. Saying so at init beats sampling
 * that silently never happens.
 */
static int inj_trigger_from_tim(const TIM_TypeDef *tim, uint32_t *out)
{
	if ((tim == NULL) || (out == NULL)) {
		return -EINVAL;
	}

#if defined(TIM1) && defined(LL_ADC_INJ_TRIG_EXT_TIM1_TRGO)
	if (tim == TIM1) {
		*out = LL_ADC_INJ_TRIG_EXT_TIM1_TRGO;
		return 0;
	}
#endif
#if defined(TIM2) && defined(LL_ADC_INJ_TRIG_EXT_TIM2_TRGO)
	if (tim == TIM2) {
		*out = LL_ADC_INJ_TRIG_EXT_TIM2_TRGO;
		return 0;
	}
#endif
#if defined(TIM3) && defined(LL_ADC_INJ_TRIG_EXT_TIM3_TRGO)
	if (tim == TIM3) {
		*out = LL_ADC_INJ_TRIG_EXT_TIM3_TRGO;
		return 0;
	}
#endif
#if defined(TIM4) && defined(LL_ADC_INJ_TRIG_EXT_TIM4_TRGO)
	if (tim == TIM4) {
		*out = LL_ADC_INJ_TRIG_EXT_TIM4_TRGO;
		return 0;
	}
#endif
#if defined(TIM6) && defined(LL_ADC_INJ_TRIG_EXT_TIM6_TRGO)
	if (tim == TIM6) {
		*out = LL_ADC_INJ_TRIG_EXT_TIM6_TRGO;
		return 0;
	}
#endif
#if defined(TIM8) && defined(LL_ADC_INJ_TRIG_EXT_TIM8_TRGO)
	if (tim == TIM8) {
		*out = LL_ADC_INJ_TRIG_EXT_TIM8_TRGO;
		return 0;
	}
#endif
#if defined(TIM15) && defined(LL_ADC_INJ_TRIG_EXT_TIM15_TRGO)
	if (tim == TIM15) {
		*out = LL_ADC_INJ_TRIG_EXT_TIM15_TRGO;
		return 0;
	}
#endif

	return -ENOTSUP;
}

static int adc_inj_set_trigger(const struct motor_sensor_stm32_config *cfg, ADC_TypeDef *adc,
			       bool hw_trigger)
{
	uint32_t trig;
	int err;

	if (!hw_trigger) {
		/* Conversions are started from the PWM update ISR, so JEXTEN must
		 * stay cleared. Programming a trigger edge here shares the same
		 * register field and would arm an external trigger that never
		 * fires.
		 */
		LL_ADC_INJ_SetTriggerSource(adc, LL_ADC_INJ_TRIG_SOFTWARE);
		return 0;
	}

	err = inj_trigger_from_tim(cfg->sync_tim, &trig);
	if (err != 0) {
		LOG_ERR("the stage timer has no TRGO this ADC can use as an injected trigger");
		return err;
	}

#if defined(LL_ADC_INJ_QUEUE_DISABLE)
	/* Out of reset the injected context queue is enabled: it serves the
	 * configured context once and then leaves the group with an empty queue,
	 * where every further trigger is ignored. One sequence lives here
	 * forever, so the queue has nothing to offer and only takes away.
	 */
	LL_ADC_INJ_SetQueueMode(adc, LL_ADC_INJ_QUEUE_DISABLE);
#endif

	/* Source before edge: both write JEXTEN, and doing it the other way
	 * around leaves the source overwritten by the edge write.
	 */
	LL_ADC_INJ_SetTriggerSource(adc, trig);
	LL_ADC_INJ_SetTriggerEdge(adc, LL_ADC_INJ_TRIG_EXT_RISING);

	return 0;
}

static int adc_inj_configure(const struct motor_sensor_stm32_config *cfg,
			     const uint32_t *ch_decimal, uint8_t n, uint8_t resolution_bits,
			     bool hw_trigger)
{
	ADC_TypeDef *adc = (cfg != NULL) ? cfg->adc : NULL;
	uint32_t common_clock;
	uint8_t i;
	uint32_t ll_resolution;
	int err;

	if ((adc == NULL) || (ch_decimal == NULL) || (n == 0U) || (n > 4U)) {
		return -EINVAL;
	}

	common_clock = cfg->adc_common_clock;
	if (stm32_ll_adc_resolution_from_bits(resolution_bits, &ll_resolution) != 0) {
		LOG_ERR("unsupported ADC resolution of %u bits", resolution_bits);
		return -EINVAL;
	}

	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(adc), common_clock);
	LL_ADC_DisableDeepPowerDown(adc);
	LL_ADC_EnableInternalRegulator(adc);
	k_busy_wait(LL_ADC_DELAY_INTERNAL_REGUL_STAB_US);

	/* An external trigger leaves the injected group armed indefinitely, and
	 * JADSTART has to be clear before the ADC can be disabled. calibrate()
	 * reconfigures while the stage timer keeps running, so disarming here is
	 * what stops that from racing conversions the hardware is still
	 * starting.
	 */
	if (LL_ADC_INJ_IsConversionOngoing(adc) != 0UL) {
		LL_ADC_INJ_StopConversion(adc);
		err = MOTOR_STM32_ADC_WAIT_FOR(LL_ADC_INJ_IsStopConversionOngoing(adc) == 0UL,
					       "the injected group to stop");
		if (err != 0) {
			return err;
		}
	}

	/* Calibration is only latched with ADEN cleared. Reconfiguring an ADC
	 * that is already converting (calibrate() comes back here per channel)
	 * would otherwise start a calibration that never runs.
	 */
	if (LL_ADC_IsEnabled(adc) != 0UL) {
		LL_ADC_Disable(adc);
	}
	err = MOTOR_STM32_ADC_WAIT_FOR(LL_ADC_IsDisableOngoing(adc) == 0UL, "the ADC to stop");
	if (err != 0) {
		return err;
	}

#if defined(LL_ADC_CALIB_OFFSET)
	LL_ADC_StartCalibration(adc, LL_ADC_CALIB_OFFSET, LL_ADC_SINGLE_ENDED);
#elif defined(LL_ADC_SINGLE_ENDED)
	LL_ADC_StartCalibration(adc, LL_ADC_SINGLE_ENDED);
#else
	LL_ADC_StartCalibration(adc);
#endif
	err = MOTOR_STM32_ADC_WAIT_FOR(LL_ADC_IsCalibrationOnGoing(adc) == 0UL, "ADC calibration");
	if (err != 0) {
		return err;
	}

	LL_ADC_SetResolution(adc, ll_resolution);
#if defined(CONFIG_SOC_SERIES_STM32H7X)
#if defined(LL_ADC_RIGHT_BIT_SHIFT_NONE)
	LL_ADC_SetDataRightShift(adc, LL_ADC_RIGHT_BIT_SHIFT_NONE);
#endif
#if defined(LL_ADC_BOOST_MODE_6MHZ25)
	/* Written here, with ADEN still clear, because BOOST is latched on
	 * enable. calibrate() comes back through this function per channel, so
	 * every reconfiguration reapplies it.
	 */
	err = motor_sensor_stm32_setup_boost(cfg, adc);
	if (err != 0) {
		return err;
	}
#endif
#else
	LL_ADC_SetDataAlignment(adc, LL_ADC_DATA_ALIGN_RIGHT);
	LL_ADC_SetLowPowerMode(adc, LL_ADC_LP_MODE_NONE);
#endif
	LL_ADC_REG_SetOverrun(adc, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
	LL_ADC_REG_SetSequencerLength(adc, LL_ADC_REG_SEQ_SCAN_DISABLE);

	LL_ADC_INJ_SetSequencerLength(adc, inj_seq_len_from_n(n));
	LL_ADC_INJ_SetTrigAuto(adc, LL_ADC_INJ_TRIG_INDEPENDENT);
	err = adc_inj_set_trigger(cfg, adc, hw_trigger);
	if (err != 0) {
		return err;
	}

	for (i = 0U; i < n; i++) {
		uint32_t ch = __LL_ADC_DECIMAL_NB_TO_CHANNEL(ch_decimal[i]);

		LL_ADC_INJ_SetSequencerRanks(adc, inj_rank[i], ch);
		LL_ADC_SetChannelSamplingTime(adc, ch, MOTOR_STM32_ADC_SAMPLE_TIME);
#if defined(CONFIG_SOC_SERIES_STM32H7X)
		/* On this series the preselection register is what connects the
		 * pad to the converter. Leave it clear and the channel still
		 * converts, on tap of nothing: the result is a parasitic level
		 * that ignores the pin, sits at roughly the same value on every
		 * channel, and does not change with sampling time. Every rank
		 * has to be enabled here.
		 */
		LL_ADC_SetChannelPreselection(adc, ch);
#endif
	}

	LL_ADC_Enable(adc);
	err = MOTOR_STM32_ADC_WAIT_FOR(LL_ADC_IsActiveFlag_ADRDY(adc) != 0UL, "the ADC to be ready");
	if (err != 0) {
		return err;
	}
	LL_ADC_ClearFlag_ADRDY(adc);
	LL_ADC_EnableIT_JEOC(adc);

	return 0;
}

static int adc_inj_sw_read(ADC_TypeDef *adc, uint8_t n, uint16_t *raw_out)
{
	bool jeoc_irq_enabled;
	uint8_t i;
	int err;

	if ((adc == NULL) || (raw_out == NULL) || (n == 0U) || (n > 4U)) {
		return -EINVAL;
	}

	/* The latch ISR acknowledges JEOC as soon as it fires, so it has to be
	 * masked while a thread polls the very same flag.
	 */
	jeoc_irq_enabled = LL_ADC_IsEnabledIT_JEOC(adc) != 0UL;
	if (jeoc_irq_enabled) {
		LL_ADC_DisableIT_JEOC(adc);
	}

	LL_ADC_ClearFlag_JEOC(adc);
	LL_ADC_INJ_StartConversion(adc);
	err = MOTOR_STM32_ADC_WAIT_FOR(LL_ADC_IsActiveFlag_JEOC(adc) != 0UL,
				       "the injected conversion");
	if (err == 0) {
		LL_ADC_ClearFlag_JEOC(adc);

		for (i = 0U; i < n; i++) {
			raw_out[i] = LL_ADC_INJ_ReadConversionData12(adc, inj_rank[i]);
		}
	}

	if (jeoc_irq_enabled) {
		LL_ADC_EnableIT_JEOC(adc);
	}

	return err;
}

/* Collapse the per-rank readings into the one signed number the control loop
 * wants.
 *
 * Low-side sensing (a shunt, or the Rds(on) of the low-side FET) only reports a
 * magnitude, and only the leg held at zero duty keeps its low side on for the
 * whole period, so only that leg carries the load current where the sampling
 * instant cannot miss it. Which leg that is flips with the
 * direction of travel, so the reading has to follow the command rather than a
 * fixed channel.
 */
static void bridge_current_update(const struct motor_sensor_stm32_config *cfg,
				  struct motor_sensor_stm32_data *data)
{
	uint8_t leg;
	uint8_t i;

	if (cfg->sense_leg == NULL) {
		return;
	}

	leg = motor_actuator_get_clamped_leg(cfg->sync_actuator);
	if (leg == MOTOR_ACTUATOR_LEG_NONE) {
		data->bridge_current_valid = false;
		return;
	}

	for (i = 0U; i < data->n_adc_channels; i++) {
		if (cfg->sense_leg[i] != leg) {
			continue;
		}

		/* Leg 0 clamped means the command drove the bridge the other
		 * way, matching the duty ordering sign-magnitude algorithms
		 * emit.
		 */
		data->bridge_current_a =
			(leg == 0U) ? -data->current_amps[i] : data->current_amps[i];
		data->bridge_current_valid = true;
		return;
	}

	data->bridge_current_valid = false;
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

	bridge_current_update(cfg, data);

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
		if (cfg->hw_trigger && (LL_ADC_INJ_IsConversionOngoing(cfg->adc) != 0UL)) {
			/* With an external trigger JADSTART arms the group rather
			 * than starting it, and stays set while armed. The timer
			 * drives every conversion from there, so the per-tick call
			 * has nothing left to do.
			 */
			return 0;
		}
		LL_ADC_INJ_StartConversion(cfg->adc);
		return 0;
	case MOTOR_SENSOR_CHAN_CURRENT_BRIDGE:
		/* Rides on the composite conversion; nothing separate to start. */
		return (cfg->sense_leg != NULL) ? 0 : -ENOTSUP;
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
				/* SENSOR_CHAN_ROTATION is degrees; store radians. */
				data->angle_rad = (float)sensor_value_to_double(&val) *
						  ((float)M_PI / 180.0f);
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
	case MOTOR_SENSOR_CHAN_CURRENT_BRIDGE:
		if (cfg->sense_leg == NULL) {
			return -ENOTSUP;
		}
		if (out_len < 1U) {
			return -EINVAL;
		}
		if (!data->bridge_current_valid) {
			return -EAGAIN;
		}
		out[0] = data->bridge_current_a;
		*got = 1U;
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
		int err;
		int s;

		/* adc_inj_sw_read() polls JEOC after starting the conversion
		 * itself, which an external trigger would never let happen.
		 */
		err = adc_inj_configure(cfg, &data->adc_ch_decimal[i], 1, cfg->resolution_bits,
					false);
		if (err != 0) {
			return err;
		}

		for (s = 0; s < 32; s++) {
			uint16_t raw;

			err = adc_inj_sw_read(cfg->adc, 1, &raw);
			if (err != 0) {
				return err;
			}
			sum += (float)raw * scale;
			k_sleep(K_MSEC(1));
		}

		data->current_offset[i] = sum * (1.0f / 32.0f);
	}

	return adc_inj_configure(cfg, data->adc_ch_decimal, data->n_adc_channels,
				 cfg->resolution_bits, cfg->hw_trigger);
}

static bool motor_sensor_stm32_channel_supported(const struct device *dev,
						 enum motor_sensor_channel ch)
{
	__maybe_unused const struct motor_sensor_stm32_config *cfg = dev->config;
	struct motor_sensor_stm32_data *data = dev->data;

	switch (ch) {
	case MOTOR_SENSOR_CHAN_CURRENT:
		return data->n_adc_channels > 0U;
	case MOTOR_SENSOR_CHAN_CURRENT_BRIDGE:
		return cfg->sense_leg != NULL;
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

/* Mirror what the Zephyr ADC driver derives from st,adc-clock-source and
 * st,adc-prescaler, so a board that clocks its ADC synchronously keeps that
 * setting instead of being forced onto an asynchronous source that may have no
 * PLL feeding it.
 */
#define SENSOR_STM32_ADC_CLOCK_PREFIX(inst)                                                        \
	COND_CODE_1(IS_EQ(DT_STRING_UPPER_TOKEN(DT_INST_PHANDLE(inst, adc), st_adc_clock_source),   \
			  SYNC),                                                                   \
		    (LL_ADC_CLOCK_SYNC_PCLK_DIV), (LL_ADC_CLOCK_ASYNC_DIV))

#define SENSOR_STM32_ADC_COMMON_CLOCK(inst)                                                        \
	COND_CODE_1(DT_NODE_HAS_PROP(DT_INST_PHANDLE(inst, adc), st_adc_clock_source),              \
		    (CONCAT(SENSOR_STM32_ADC_CLOCK_PREFIX(inst),                                   \
			    DT_PROP(DT_INST_PHANDLE(inst, adc), st_adc_prescaler))),                \
		    (LL_ADC_CLOCK_ASYNC_DIV1))
/* The stage keeps its timer private, but the node it came from is right there in
 * sync-actuator, so the trigger source can be resolved without a new op.
 */
#define SENSOR_STM32_SYNC_TIM(inst)                                                                \
	COND_CODE_1(DT_NODE_HAS_PROP(DT_INST_PHANDLE(inst, sync_actuator), st_pwm_timer),          \
		    ((TIM_TypeDef *)DT_REG_ADDR(                                                   \
			    DT_PHANDLE(DT_INST_PHANDLE(inst, sync_actuator), st_pwm_timer))),      \
		    (NULL))

#define SENSOR_STM32_HW_TRIGGER(inst) (DT_INST_ENUM_IDX(inst, adc_trigger) == 1)

#define SENSOR_STM32_SENSE_LEGS_DEFINE(inst)                                                       \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, current_sense_legs),                                \
		   (static const uint8_t motor_sensor_stm32_sense_leg_##inst[] =                   \
			    DT_INST_PROP(inst, current_sense_legs);))

#define SENSOR_STM32_SENSE_LEGS(inst)                                                              \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, current_sense_legs),                               \
		    (motor_sensor_stm32_sense_leg_##inst), (NULL))

#define SENSOR_STM32_ADC_IRQN(inst) DT_IRQ_BY_IDX(DT_INST_PHANDLE(inst, adc), 0, irq)
#define SENSOR_STM32_ADC_IRQ_PRIO(inst)                                                         \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, zephyr_adc_irq_priority),                      \
		    (DT_INST_PROP(inst, zephyr_adc_irq_priority)),                              \
		    (COND_CODE_1(DT_IRQ_HAS_CELL(DT_INST_PHANDLE(inst, adc), priority),         \
				 (DT_IRQ_BY_IDX(DT_INST_PHANDLE(inst, adc), 0, priority)),         \
				 (CONFIG_MOTOR_STM32_ADC_IRQ_PRIORITY))))

#define SENSOR_STM32_DEFINE(inst)                                                                  \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, adc_channels) >= 1);                                     \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, adc_channels) <= MOTOR_SENSOR_CURRENT_MAX);              \
	BUILD_ASSERT(COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, current_sense_legs),                    \
				 (DT_INST_PROP_LEN(inst, current_sense_legs) ==                      \
				  DT_INST_PROP_LEN(inst, adc_channels)),                             \
				 (1)),                                                               \
		     "current-sense-legs needs one entry per adc-channels entry");                  \
	static const uint32_t motor_sensor_stm32_adc_ch_##inst[] = DT_INST_PROP(inst, adc_channels); \
	SENSOR_STM32_SENSE_LEGS_DEFINE(inst)                                                         \
	static const struct stm32_pclken motor_sensor_stm32_pclken_##inst[] =                      \
		STM32_DT_CLOCKS(DT_INST_PHANDLE(inst, adc));                                       \
	static const struct motor_sensor_stm32_config motor_sensor_stm32_cfg_##inst = {            \
		.adc = ADC_FROM_PHANDLE(inst),                                                     \
		.sync_actuator = DEVICE_DT_GET(DT_INST_PHANDLE(inst, sync_actuator)),              \
		.pclken = motor_sensor_stm32_pclken_##inst,                                        \
		.pclk_len = ARRAY_SIZE(motor_sensor_stm32_pclken_##inst),                          \
		.adc_common_clock = SENSOR_STM32_ADC_COMMON_CLOCK(inst),                           \
		.sync_tim = SENSOR_STM32_SYNC_TIM(inst),                                           \
		.hw_trigger = SENSOR_STM32_HW_TRIGGER(inst),                                       \
		.sense_leg = SENSOR_STM32_SENSE_LEGS(inst),                                        \
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
		uint32_t adc_ker_hz;                                                               \
		int err;                                                                           \
		uint8_t i;                                                                         \
		if (!device_is_ready(cfg->sync_actuator)) {                                        \
			LOG_ERR("sync actuator %s is not ready", cfg->sync_actuator->name);         \
			return -ENODEV;                                                            \
		}                                                                                  \
		if ((cfg->feedback_sensor != NULL) && !device_is_ready(cfg->feedback_sensor)) {   \
			LOG_ERR("feedback sensor %s is not ready, check its init priority",        \
				cfg->feedback_sensor->name);                                       \
			return -ENODEV;                                                            \
		}                                                                                  \
		err = motor_sensor_stm32_enable_adc_clock(cfg);                                    \
		if (err != 0) {                                                                    \
			return err;                                                                \
		}                                                                                  \
		data->n_adc_channels = (uint8_t)DT_INST_PROP_LEN(inst, adc_channels);                \
		for (i = 0U; i < data->n_adc_channels; i++) {                                      \
			data->adc_ch_decimal[i] = motor_sensor_stm32_adc_ch_##inst[i];              \
			data->current_offset[i] = 0.0f;                                             \
		}                                                                                  \
		data->amps_per_count = (((float)cfg->vref_mv * 1e-3f) /                              \
					 (float)(1U << cfg->resolution_bits)) *                       \
				       cfg->amps_per_volt;                                            \
		err = motor_sensor_stm32_adc_kernel_freq(cfg, &adc_ker_hz);                         \
		if (err != 0) {                                                                    \
			return err;                                                                \
		}                                                                                  \
		LOG_INF("ADC kernel clock %u Hz (silicon rev 0x%04x)", adc_ker_hz,                  \
			(unsigned int)LL_DBGMCU_GetRevisionID());                                   \
		err = motor_sensor_stm32_check_adc_pwm_margin(cfg->sync_actuator,                    \
								data->n_adc_channels, adc_ker_hz);     \
		if (err != 0) {                                                                    \
			return err;                                                                \
		}                                                                                  \
		if (cfg->sense_leg != NULL) {                                                      \
			const struct motor_stage_config *stage_ =                                  \
				motor_actuator_get_config(cfg->sync_actuator);                     \
			for (i = 0U; i < data->n_adc_channels; i++) {                              \
				if (cfg->sense_leg[i] < stage_->n_phases) {                        \
					continue;                                                  \
				}                                                                  \
				LOG_ERR("current-sense-legs[%u] names leg %u, but the stage "       \
					"drives %u",                                               \
					i, cfg->sense_leg[i], stage_->n_phases);                   \
				return -EINVAL;                                                    \
			}                                                                          \
		}                                                                                  \
		data->measurement_done_cb = NULL;                                                  \
		data->measurement_done_user_data = NULL;                                           \
		data->angle_rad = 0.0f;                                                            \
		data->bridge_current_a = 0.0f;                                                     \
		data->bridge_current_valid = false;                                                \
		err = adc_inj_configure(cfg, data->adc_ch_decimal, data->n_adc_channels,            \
					cfg->resolution_bits, cfg->hw_trigger);                     \
		if (err != 0) {                                                                    \
			return err;                                                                \
		}                                                                                  \
		BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1U,                         \
			     "motor_sensor_stm32: only one instance is currently supported");       \
		IRQ_DIRECT_CONNECT(SENSOR_STM32_ADC_IRQN(inst),                                 \
				   SENSOR_STM32_ADC_IRQ_PRIO(inst),                                  \
				   motor_sensor_stm32_adc_isr_##inst, IRQ_ZERO_LATENCY);          \
		irq_enable(SENSOR_STM32_ADC_IRQN(inst));                                         \
		return 0;                                                                          \
	}                                                                                          \
	DEVICE_DT_DEFINE(DT_DRV_INST(inst), motor_sensor_stm32_dev_init_##inst, NULL,              \
			 &motor_sensor_stm32_data_##inst, &motor_sensor_stm32_cfg_##inst,            \
			 POST_KERNEL, CONFIG_MOTOR_SENSOR_STM32_INIT_PRIORITY,                      \
			 &motor_sensor_stm32_api)

DT_INST_FOREACH_STATUS_OKAY(SENSOR_STM32_DEFINE)
