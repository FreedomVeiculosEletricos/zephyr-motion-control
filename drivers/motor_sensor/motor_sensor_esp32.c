/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Espressif ESP32 family: SAR ADC digital + DMA EOF (GDMA or I2S/SPI-linked on legacy SoCs).
 * ADC digital path is implemented here (same idea as Zephyr adc_esp32_dma.c) so this
 * driver stays self-contained; only Espressif HAL/LL headers are used.
 */

#define DT_DRV_COMPAT zephyr_motor_sensor_esp32

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include <esp_clk_tree.h>
#include <esp_err.h>
#include <esp_intr_alloc.h>
#include <esp_private/adc_share_hw_ctrl.h>
#include <esp_private/sar_periph_ctrl.h>
#include <hal/adc_hal.h>
#include <hal/adc_ll.h>
#include <hal/adc_types.h>
#include <hal/mcpwm_ll.h>
#include <soc/soc.h>
#include <soc/soc_caps.h>

#if !defined(SOC_ADC_DIG_CTRL_SUPPORTED) || (SOC_ADC_DIG_CTRL_SUPPORTED != 1)
#error "motor_sensor_esp32: SoC must provide SAR ADC digital controller (SOC_ADC_DIG_CTRL_SUPPORTED)"
#endif

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#if SOC_GDMA_SUPPORTED
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_esp32.h>
#include <hal/adc_ll.h>
#elif defined(CONFIG_SOC_SERIES_ESP32)
#include <esp_private/periph_ctrl.h>
#include <hal/i2s_ll.h>
#define MOTOR_ADC_DMA_I2S_HOST 0
#define MOTOR_ADC_DMA_INTR_MASK I2S_LL_EVENT_RX_EOF
#elif defined(CONFIG_SOC_SERIES_ESP32S2)
#include <esp_private/periph_ctrl.h>
#include <hal/spi_ll.h>
#include <hal/spi_types.h>
#define MOTOR_ADC_DMA_SPI_HOST SPI3_HOST
#define MOTOR_ADC_DMA_INTR_MASK SPI_LL_INTR_IN_SUC_EOF
#endif
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/drivers/motor/motor_sensor.h>
#include <zephyr/kernel.h>
#if IS_ENABLED(CONFIG_SENSOR)
#include <zephyr/drivers/sensor.h>
#endif
#include <zephyr/sys/util.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define AMPS_PER_VOLT_DEFAULT 1.0f

#define UNIT_ATTEN_UNINIT UINT32_MAX

typedef void (*motor_enc_adc_eof_fn)(void *user, const uint8_t *dma_buf, uint32_t n_bytes);

struct motor_enc_adc_digi_ctx {
	adc_unit_t unit;
	const struct device *dma_dev;
	uint8_t dma_channel;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	adc_hal_dma_ctx_t adc_hal_dma_ctx;
	dma_descriptor_t *rx_desc;
	uint8_t *dma_buffer;
	uint32_t dma_buffer_bytes;
	uint32_t block_samples;
#if SOC_GDMA_SUPPORTED
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk;
#endif
	bool running;
	bool digi_started;
	motor_enc_adc_eof_fn eof_fn;
	void *eof_user;
#if !SOC_GDMA_SUPPORTED
	intr_handle_t irq_handle;
#endif
};

#if SOC_ADC_CALIBRATION_V1_SUPPORTED
static void motor_enc_adc_hw_calibration(adc_unit_t unit)
{
	adc_hal_calibration_init(unit);
	for (int j = 0; j < SOC_ADC_ATTEN_NUM; j++) {
		adc_calc_hw_calibration_code(unit, j);
#if SOC_ADC_CALIB_CHAN_COMPENS_SUPPORTED
		for (int k = 0; k < SOC_ADC_CHANNEL_NUM(unit); k++) {
			adc_load_hw_calibration_chan_compens(unit, k, j);
		}
#endif
	}
}
#else
static void motor_enc_adc_hw_calibration(adc_unit_t unit)
{
	ARG_UNUSED(unit);
}
#endif

#if SOC_GDMA_SUPPORTED

static int motor_enc_adc_dma_run(struct motor_enc_adc_digi_ctx *ctx)
{
	int err;

	err = dma_config(ctx->dma_dev, ctx->dma_channel, &ctx->dma_cfg);
	if (err != 0) {
		return err;
	}

	return dma_start(ctx->dma_dev, ctx->dma_channel);
}

static void IRAM_ATTR motor_enc_adc_dma_eof_cb(const struct device *dma_dev, void *user_data,
					       uint32_t channel, int status)
{
	struct motor_enc_adc_digi_ctx *ctx = user_data;
	int err;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (ctx->eof_fn == NULL) {
		return;
	}

	if (status != DMA_STATUS_COMPLETE) {
		return;
	}

	ctx->eof_fn(ctx->eof_user, ctx->dma_buffer, ctx->dma_buffer_bytes);

	err = dma_stop(ctx->dma_dev, ctx->dma_channel);
	if (err != 0) {
		return;
	}

	(void)motor_enc_adc_dma_run(ctx);
}

#else

static void motor_enc_adc_legacy_digi_arm(struct motor_enc_adc_digi_ctx *ctx)
{
#if defined(CONFIG_SOC_SERIES_ESP32)
	i2s_dev_t *i2s_dev = I2S_LL_GET_HW(MOTOR_ADC_DMA_I2S_HOST);

	i2s_ll_rx_stop_link(i2s_dev);
#elif defined(CONFIG_SOC_SERIES_ESP32S2)
	spi_dev_t *spi_dev = SPI_LL_GET_HW(MOTOR_ADC_DMA_SPI_HOST);

	spi_ll_disable_intr(spi_dev, MOTOR_ADC_DMA_INTR_MASK);
	spi_ll_clear_intr(spi_dev, MOTOR_ADC_DMA_INTR_MASK);
	spi_dma_ll_rx_stop(spi_dev, 0);
#endif

	adc_hal_digi_connect(false);

#if defined(CONFIG_SOC_SERIES_ESP32S2)
	spi_dma_ll_rx_reset(SPI_LL_GET_HW(MOTOR_ADC_DMA_SPI_HOST), 0);
#endif

	adc_hal_digi_reset();
	adc_hal_digi_dma_link(&ctx->adc_hal_dma_ctx, ctx->dma_buffer);

#if defined(CONFIG_SOC_SERIES_ESP32)
	{
		i2s_dev_t *i2s_dev = I2S_LL_GET_HW(MOTOR_ADC_DMA_I2S_HOST);

		i2s_ll_clear_intr_status(i2s_dev, MOTOR_ADC_DMA_INTR_MASK);
		i2s_ll_enable_intr(i2s_dev, MOTOR_ADC_DMA_INTR_MASK, true);
		i2s_ll_enable_dma(i2s_dev, true);
		i2s_ll_rx_start_link(i2s_dev, (uint32_t)ctx->adc_hal_dma_ctx.rx_desc);
	}
#elif defined(CONFIG_SOC_SERIES_ESP32S2)
	{
		spi_dev_t *spi_dev = SPI_LL_GET_HW(MOTOR_ADC_DMA_SPI_HOST);

		spi_ll_clear_intr(spi_dev, MOTOR_ADC_DMA_INTR_MASK);
		spi_ll_enable_intr(spi_dev, MOTOR_ADC_DMA_INTR_MASK);
		spi_dma_ll_rx_start(spi_dev, 0, (lldesc_t *)ctx->adc_hal_dma_ctx.rx_desc);
	}
#endif

	adc_hal_digi_connect(true);
	adc_hal_digi_enable(true);
}

static void IRAM_ATTR motor_enc_adc_legacy_dma_isr(void *arg)
{
	struct motor_enc_adc_digi_ctx *ctx = arg;

#if defined(CONFIG_SOC_SERIES_ESP32)
	i2s_dev_t *i2s_dev = I2S_LL_GET_HW(MOTOR_ADC_DMA_I2S_HOST);

	if ((i2s_ll_get_intr_status(i2s_dev) & MOTOR_ADC_DMA_INTR_MASK) == 0U) {
		return;
	}

	i2s_ll_clear_intr_status(i2s_dev, MOTOR_ADC_DMA_INTR_MASK);
#elif defined(CONFIG_SOC_SERIES_ESP32S2)
	spi_dev_t *spi_dev = SPI_LL_GET_HW(MOTOR_ADC_DMA_SPI_HOST);

	if ((spi_ll_get_intr(spi_dev, MOTOR_ADC_DMA_INTR_MASK)) == 0U) {
		return;
	}

	spi_ll_clear_intr(spi_dev, MOTOR_ADC_DMA_INTR_MASK);
#else
	return;
#endif

	if (ctx->eof_fn != NULL) {
		ctx->eof_fn(ctx->eof_user, ctx->dma_buffer, ctx->dma_buffer_bytes);
	}

	motor_enc_adc_legacy_digi_arm(ctx);
}

#endif /* SOC_GDMA_SUPPORTED */

static int motor_enc_adc_digi_hw_start(struct motor_enc_adc_digi_ctx *ctx,
				       adc_hal_digi_ctrlr_cfg_t *dig_cfg, uint32_t unit_atten,
				       uint32_t n_samples)
{
	__maybe_unused int err = 0;

	sar_periph_ctrl_adc_continuous_power_acquire();
	adc_lock_acquire(ctx->unit);

#if SOC_ADC_CALIBRATION_V1_SUPPORTED
	adc_set_hw_calibration_code(ctx->unit, unit_atten);
#endif

#if SOC_ADC_ARBITER_SUPPORTED
	if (ctx->unit == ADC_UNIT_2) {
		adc_arbiter_t arb = ADC_ARBITER_CONFIG_DEFAULT();

		adc_hal_arbiter_config(&arb);
	}
#endif

	adc_hal_dma_config_t hal_dma_cfg = {
		.eof_desc_num = 1,
		.eof_step = 1,
		.eof_num = n_samples,
	};

	adc_hal_dma_ctx_config(&ctx->adc_hal_dma_ctx, &hal_dma_cfg);
	adc_hal_set_controller(ctx->unit, ADC_HAL_CONTINUOUS_READ_MODE);
	adc_hal_digi_init(&ctx->adc_hal_dma_ctx);
	adc_hal_digi_controller_config(&ctx->adc_hal_dma_ctx, dig_cfg);

	adc_hal_digi_enable(false);

#if defined(CONFIG_SOC_SERIES_ESP32)
	i2s_dev_t *i2s_dev = I2S_LL_GET_HW(MOTOR_ADC_DMA_I2S_HOST);

	i2s_ll_rx_stop_link(i2s_dev);
#elif defined(CONFIG_SOC_SERIES_ESP32S2)
	spi_dev_t *spi_dev = SPI_LL_GET_HW(MOTOR_ADC_DMA_SPI_HOST);

	spi_ll_disable_intr(spi_dev, MOTOR_ADC_DMA_INTR_MASK);
	spi_ll_clear_intr(spi_dev, MOTOR_ADC_DMA_INTR_MASK);
	spi_dma_ll_rx_stop(spi_dev, 0);
#endif

	adc_hal_digi_connect(false);

#if defined(CONFIG_SOC_SERIES_ESP32S2)
	spi_dma_ll_rx_reset(spi_dev, 0);
#endif

	adc_hal_digi_reset();

#if SOC_GDMA_SUPPORTED
	err = motor_enc_adc_dma_run(ctx);
	if (err != 0) {
		adc_lock_release(ctx->unit);
		sar_periph_ctrl_adc_continuous_power_release();
		return err;
	}
#else
	adc_hal_digi_dma_link(&ctx->adc_hal_dma_ctx, ctx->dma_buffer);

#if defined(CONFIG_SOC_SERIES_ESP32)
	i2s_ll_clear_intr_status(i2s_dev, MOTOR_ADC_DMA_INTR_MASK);
	i2s_ll_enable_intr(i2s_dev, MOTOR_ADC_DMA_INTR_MASK, true);
	i2s_ll_enable_dma(i2s_dev, true);
	i2s_ll_rx_start_link(i2s_dev, (uint32_t)ctx->adc_hal_dma_ctx.rx_desc);
#elif defined(CONFIG_SOC_SERIES_ESP32S2)
	spi_ll_clear_intr(spi_dev, MOTOR_ADC_DMA_INTR_MASK);
	spi_ll_enable_intr(spi_dev, MOTOR_ADC_DMA_INTR_MASK);
	spi_dma_ll_rx_start(spi_dev, 0, (lldesc_t *)ctx->adc_hal_dma_ctx.rx_desc);
#endif
#endif

	adc_hal_digi_connect(true);
	adc_hal_digi_enable(true);

	ctx->digi_started = true;
	return 0;
}

static void motor_enc_adc_digi_hw_halt(struct motor_enc_adc_digi_ctx *ctx)
{
	if (!ctx->digi_started) {
		return;
	}

#if defined(CONFIG_SOC_SERIES_ESP32)
	i2s_dev_t *i2s_dev = I2S_LL_GET_HW(MOTOR_ADC_DMA_I2S_HOST);

	i2s_ll_enable_intr(i2s_dev, MOTOR_ADC_DMA_INTR_MASK, false);
	i2s_ll_clear_intr_status(i2s_dev, MOTOR_ADC_DMA_INTR_MASK);
	i2s_ll_rx_stop_link(i2s_dev);
#elif defined(CONFIG_SOC_SERIES_ESP32S2)
	spi_dev_t *spi_dev = SPI_LL_GET_HW(MOTOR_ADC_DMA_SPI_HOST);

	spi_ll_disable_intr(spi_dev, MOTOR_ADC_DMA_INTR_MASK);
	spi_ll_clear_intr(spi_dev, MOTOR_ADC_DMA_INTR_MASK);
	spi_dma_ll_rx_stop(spi_dev, 0);
#endif

#if !SOC_GDMA_SUPPORTED
	if (ctx->irq_handle != NULL) {
		(void)esp_intr_disable(ctx->irq_handle);
	}
#endif

	adc_hal_digi_enable(false);
	adc_hal_digi_connect(false);

#if defined(ADC_LL_WORKAROUND_CLEAR_EOF_COUNTER) && ADC_LL_WORKAROUND_CLEAR_EOF_COUNTER
	adc_hal_digi_clr_eof();
#endif

	adc_hal_digi_deinit();
	adc_lock_release(ctx->unit);
	sar_periph_ctrl_adc_continuous_power_release();

#if SOC_GDMA_SUPPORTED
	(void)dma_stop(ctx->dma_dev, ctx->dma_channel);
#endif

	ctx->digi_started = false;
}

static int motor_enc_adc_digi_init(struct motor_enc_adc_digi_ctx *ctx)
{
	int err;

	if (ctx->rx_desc != NULL || ctx->dma_buffer != NULL) {
		return -EINVAL;
	}

	ctx->adc_hal_dma_ctx.rx_desc =
		k_aligned_alloc(sizeof(uint32_t), sizeof(dma_descriptor_t));
	if (ctx->adc_hal_dma_ctx.rx_desc == NULL) {
		return -ENOMEM;
	}

	ctx->rx_desc = ctx->adc_hal_dma_ctx.rx_desc;

	ctx->dma_buffer = NULL;
	ctx->dma_buffer_bytes = 0U;
	ctx->running = false;
	ctx->digi_started = false;
	ctx->eof_fn = NULL;
	ctx->eof_user = NULL;

#if SOC_GDMA_SUPPORTED
	adc_ll_enable_bus_clock(true);
	adc_ll_reset_register();
#elif defined(CONFIG_SOC_SERIES_ESP32)
	ctx->irq_handle = NULL;

	periph_module_enable(PERIPH_I2S0_MODULE);

	{
		i2s_dev_t *i2s_dev = I2S_LL_GET_HW(MOTOR_ADC_DMA_I2S_HOST);

		i2s_ll_enable_core_clock(i2s_dev, true);
	}

	err = esp_intr_alloc(I2S0_INTR_SOURCE, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_INTRDISABLED,
			     motor_enc_adc_legacy_dma_isr, ctx, &ctx->irq_handle);
	if (err != 0) {
		k_free(ctx->adc_hal_dma_ctx.rx_desc);
		ctx->adc_hal_dma_ctx.rx_desc = NULL;
		ctx->rx_desc = NULL;
		return err;
	}
#elif defined(CONFIG_SOC_SERIES_ESP32S2)
	ctx->irq_handle = NULL;

	spi_ll_enable_bus_clock(MOTOR_ADC_DMA_SPI_HOST, true);
	spi_ll_reset_register(MOTOR_ADC_DMA_SPI_HOST);

	periph_module_enable(PERIPH_SPI2_DMA_MODULE);
	periph_module_enable(PERIPH_SPI3_DMA_MODULE);

	{
		spi_dev_t *spi_dev = SPI_LL_GET_HW(MOTOR_ADC_DMA_SPI_HOST);

		spi_dma_ll_rx_enable_burst_desc(spi_dev, 0, true);
	}

	err = esp_intr_alloc(SPI3_DMA_INTR_SOURCE,
			     ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_INTRDISABLED,
			     motor_enc_adc_legacy_dma_isr, ctx, &ctx->irq_handle);
	if (err != 0) {
		k_free(ctx->adc_hal_dma_ctx.rx_desc);
		ctx->adc_hal_dma_ctx.rx_desc = NULL;
		ctx->rx_desc = NULL;
		return err;
	}
#endif

	return 0;
}

static int motor_enc_adc_digi_start(struct motor_enc_adc_digi_ctx *ctx, uint8_t channel_id,
				    adc_atten_t atten, uint8_t resolution_bits, uint32_t sample_freq_hz,
				    uint32_t block_samples, motor_enc_adc_eof_fn eof_fn, void *eof_user)
{
	adc_digi_pattern_config_t pattern[1];
	adc_hal_digi_ctrlr_cfg_t dig_cfg = {0};
	uint32_t unit_atten = UNIT_ATTEN_UNINIT;
	soc_module_clk_t clk_src = ADC_DIGI_CLK_SRC_DEFAULT;
	uint32_t clk_src_freq_hz = 0;
	int err;

	if (block_samples == 0U || eof_fn == NULL) {
		return -EINVAL;
	}

	if (!SOC_ADC_DIG_SUPPORTED_UNIT(ctx->unit)) {
		return -EINVAL;
	}

	ctx->dma_buffer_bytes = block_samples * SOC_ADC_DIGI_DATA_BYTES_PER_CONV;

	if (ctx->dma_buffer != NULL) {
		k_free(ctx->dma_buffer);
		ctx->dma_buffer = NULL;
	}

	ctx->dma_buffer = k_aligned_alloc(sizeof(uint32_t), ctx->dma_buffer_bytes);
	if (ctx->dma_buffer == NULL) {
		return -ENOMEM;
	}

	err = clock_control_on(ctx->clock_dev, ctx->clock_subsys);
	if (err < 0) {
		k_free(ctx->dma_buffer);
		ctx->dma_buffer = NULL;
		return err;
	}

	motor_enc_adc_hw_calibration(ctx->unit);

	ctx->block_samples = block_samples;
	ctx->eof_fn = eof_fn;
	ctx->eof_user = eof_user;

	pattern[0].atten = atten;
	pattern[0].channel = channel_id;
	pattern[0].unit = (uint8_t)ctx->unit;
	pattern[0].bit_width = resolution_bits;

	err = esp_clk_tree_src_get_freq_hz(clk_src, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
					   &clk_src_freq_hz);
	if (err != ESP_OK) {
		k_free(ctx->dma_buffer);
		ctx->dma_buffer = NULL;
		return -EIO;
	}

	dig_cfg.conv_mode = (ctx->unit == ADC_UNIT_1) ? ADC_CONV_SINGLE_UNIT_1
						       : ADC_CONV_SINGLE_UNIT_2;
	dig_cfg.clk_src = clk_src;
	dig_cfg.clk_src_freq_hz = clk_src_freq_hz;
	dig_cfg.sample_freq_hz = sample_freq_hz;
	dig_cfg.adc_pattern = pattern;
	dig_cfg.adc_pattern_len = 1;

	unit_atten = (uint32_t)atten;

#if SOC_GDMA_SUPPORTED
	memset(&ctx->dma_cfg, 0, sizeof(ctx->dma_cfg));
	memset(&ctx->dma_blk, 0, sizeof(ctx->dma_blk));

	ctx->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	ctx->dma_cfg.dma_callback = motor_enc_adc_dma_eof_cb;
	ctx->dma_cfg.user_data = ctx;
	ctx->dma_cfg.dma_slot = ESP_GDMA_TRIG_PERIPH_ADC0;
	ctx->dma_cfg.block_count = 1;
	ctx->dma_cfg.head_block = &ctx->dma_blk;
	ctx->dma_blk.block_size = ctx->dma_buffer_bytes;
	ctx->dma_blk.dest_address = (uint32_t)ctx->dma_buffer;
#else
	err = esp_intr_enable(ctx->irq_handle);
	if (err != 0) {
		k_free(ctx->dma_buffer);
		ctx->dma_buffer = NULL;
		return err;
	}
#endif

	err = motor_enc_adc_digi_hw_start(ctx, &dig_cfg, unit_atten, block_samples);
	if (err != 0) {
#if !SOC_GDMA_SUPPORTED
		(void)esp_intr_disable(ctx->irq_handle);
#endif
		k_free(ctx->dma_buffer);
		ctx->dma_buffer = NULL;
		return err;
	}

	ctx->running = true;
	return 0;
}

static void motor_enc_adc_digi_stop(struct motor_enc_adc_digi_ctx *ctx)
{
	if (!ctx->running) {
		return;
	}

	motor_enc_adc_digi_hw_halt(ctx);

	if (ctx->dma_buffer != NULL) {
		k_free(ctx->dma_buffer);
		ctx->dma_buffer = NULL;
	}

	ctx->eof_fn = NULL;
	ctx->eof_user = NULL;
	ctx->running = false;
}

struct motor_sensor_esp32_config {
	mcpwm_dev_t *mcpwm_sync;
	uint8_t sync_timer_id;
	bool sync_is_tep;
	const struct device *sync_actuator;
	const struct device *feedback_sensor;
	adc_unit_t adc_unit;
	uint8_t adc_channel;
	adc_atten_t adc_atten;
	uint8_t adc_resolution_bits;
	uint32_t adc_vref_mv;
	uint32_t adc_sample_freq_hz;
	uint32_t digi_block_samples;
	const struct device *adc_clock_dev;
	clock_control_subsys_t adc_clock_subsys;
	const struct device *dma_dev;
	uint8_t dma_channel;
	float amps_per_volt;
};

struct motor_sensor_esp32_data {
	struct motor_enc_adc_digi_ctx digi;
	float current_amps[1];
	float current_offset[1];
	/* Cached at init: (vref_v / counts) * amps_per_volt. Hot path uses
	 * this as the single scale factor.
	 */
	float amps_per_count;
	uint16_t raw_latched;
	float angle_rad;
	intr_handle_t mcpwm_intr_handle;
	motor_sensor_measurement_done_cb_t measurement_done_cb;
	void *measurement_done_user_data;
};

static int motor_sensor_esp32_check_eof_pwm_grid(const struct motor_sensor_esp32_config *cfg)
{
	const struct motor_stage_config *stage = motor_actuator_get_config(cfg->sync_actuator);
	uint64_t pwm_ns;
	uint64_t eof_ns;

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

	if (cfg->adc_sample_freq_hz == 0U) {
		return -EINVAL;
	}

	eof_ns = (uint64_t)cfg->digi_block_samples * (uint64_t)NSEC_PER_SEC /
		 (uint64_t)cfg->adc_sample_freq_hz;

	if (eof_ns < pwm_ns) {
		return -EINVAL;
	}

	if ((eof_ns % pwm_ns) != 0ULL) {
		return -EINVAL;
	}

	return 0;
}

static adc_atten_t adc_atten_from_db(int db)
{
	switch (db) {
	case 0:
		return ADC_ATTEN_DB_0;
	case 2:
		return ADC_ATTEN_DB_2_5;
	case 6:
		return ADC_ATTEN_DB_6;
	case 11:
	default:
		return ADC_ATTEN_DB_12;
	}
}

static void IRAM_ATTR mcpwm_sync_isr(void *arg)
{
	const struct device *dev = arg;
	const struct motor_sensor_esp32_config *cfg = dev->config;
	mcpwm_dev_t *mcpwm = cfg->mcpwm_sync;
	uint32_t st = mcpwm_ll_intr_get_status(mcpwm);
	uint32_t ev = cfg->sync_is_tep ? MCPWM_LL_EVENT_TIMER_FULL(cfg->sync_timer_id)
				       : MCPWM_LL_EVENT_TIMER_EMPTY(cfg->sync_timer_id);

	if ((st & ev) != 0U) {
		mcpwm_ll_intr_clear_status(mcpwm, st & ev);
	}
}

static void latch_raw_to_current(const struct motor_sensor_esp32_config *cfg,
				 struct motor_sensor_esp32_data *data, uint16_t raw)
{
	ARG_UNUSED(cfg);

	data->current_amps[0] = (float)raw * data->amps_per_count - data->current_offset[0];
}

static void motor_enc_digi_eof(void *user, const uint8_t *buf, uint32_t n_bytes)
{
	const struct device *dev = user;
	const struct motor_sensor_esp32_config *cfg = dev->config;
	const adc_digi_output_data_t *d = (const adc_digi_output_data_t *)buf;
	struct motor_sensor_esp32_data *data = dev->data;
	uint32_t ns;

	if (n_bytes < SOC_ADC_DIGI_DATA_BYTES_PER_CONV) {
		return;
	}

	ns = n_bytes / SOC_ADC_DIGI_DATA_BYTES_PER_CONV;
#if SOC_GDMA_SUPPORTED
	data->raw_latched = (uint16_t)d[ns - 1U].type2.data;
#else
	data->raw_latched = (uint16_t)d[ns - 1U].type1.data;
#endif
	latch_raw_to_current(cfg, data, data->raw_latched);
	if (data->measurement_done_cb != NULL) {
		data->measurement_done_cb(dev, data->measurement_done_user_data);
	}
}

static void motor_sensor_esp32_data_default_init(struct motor_sensor_esp32_data *data)
{
	data->current_amps[0] = 0.0f;
	data->current_offset[0] = 0.0f;
	data->amps_per_count = 0.0f;
	data->raw_latched = 0U;
	data->angle_rad = 0.0f;
	data->measurement_done_cb = NULL;
	data->measurement_done_user_data = NULL;
}

static int motor_sensor_esp32_start_sample(const struct device *dev, enum motor_sensor_channel ch)
{
	const struct motor_sensor_esp32_config *cfg = dev->config;
	struct motor_sensor_esp32_data *data = dev->data;

	switch (ch) {
	case MOTOR_SENSOR_CHAN_CURRENT:
		ARG_UNUSED(data);
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

static int motor_sensor_esp32_get_sample(const struct device *dev, enum motor_sensor_channel ch,
					   float *out, size_t out_len, size_t *got)
{
	const struct motor_sensor_esp32_config *cfg = dev->config;
	struct motor_sensor_esp32_data *data = dev->data;

	switch (ch) {
	case MOTOR_SENSOR_CHAN_CURRENT:
		if (out_len < 1U) {
			return -EINVAL;
		}
		out[0] = data->current_amps[0];
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

static int motor_sensor_esp32_calibrate(const struct device *dev, enum motor_sensor_channel ch)
{
	ARG_UNUSED(dev);

	if (ch != MOTOR_SENSOR_CHAN_CURRENT) {
		return -ENOTSUP;
	}
	return 0;
}

static int motor_sensor_esp32_set_measurement_done_cb(const struct device *dev,
							motor_sensor_measurement_done_cb_t cb,
							void *user_data)
{
	struct motor_sensor_esp32_data *data = dev->data;

	data->measurement_done_cb = cb;
	data->measurement_done_user_data = user_data;
	return 0;
}

static bool motor_sensor_esp32_channel_supported(const struct device *dev,
						   enum motor_sensor_channel ch)
{
	const struct motor_sensor_esp32_config *cfg = dev->config;

	switch (ch) {
	case MOTOR_SENSOR_CHAN_CURRENT:
		return true;
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

static const struct motor_sensor_ops motor_sensor_esp32_api = {
	.start_sample = motor_sensor_esp32_start_sample,
	.get_sample = motor_sensor_esp32_get_sample,
	.calibrate = motor_sensor_esp32_calibrate,
	.channel_supported = motor_sensor_esp32_channel_supported,
	.set_measurement_done_callback = motor_sensor_esp32_set_measurement_done_cb,
};

#if SOC_GDMA_SUPPORTED
static int motor_sensor_esp32_dma_dev_check(const struct motor_sensor_esp32_config *cfg)
{
	if (!device_is_ready(cfg->dma_dev)) {
		return -ENODEV;
	}

	return 0;
}
#else
static int motor_sensor_esp32_dma_dev_check(const struct motor_sensor_esp32_config *cfg)
{
	ARG_UNUSED(cfg);
	return 0;
}
#endif

#define ADC_NODE(id) DT_INST_PHANDLE(id, adc)

#if SOC_GDMA_SUPPORTED
#define MOTOR_ESP32_INST_ASSERT_DMA(inst)                                                          \
	BUILD_ASSERT(DT_NODE_HAS_PROP(ADC_NODE(inst), dmas),                                        \
		     "motor_sensor_esp32: adc needs dmas on GDMA SoCs");
#else
#define MOTOR_ESP32_INST_ASSERT_DMA(inst)
#endif

#define SENSOR_ENC_IRQ_FLAGS_PRIO(inst)                                                          \
	(ESP_PRIO_TO_FLAGS(DT_INST_PROP(inst, zephyr_mcpwm_sync_irq_priority)) | ESP_INTR_FLAG_IRAM)

#define SENSOR_ENC_IRQ_FLAGS_DEFAULT(inst)                                                       \
	(ESP_PRIO_TO_FLAGS(DT_IRQ_BY_IDX(DT_INST_PHANDLE(inst, mcpwm_sync), 0, priority)) |        \
	 ESP_INT_FLAGS_CHECK(DT_IRQ_BY_IDX(DT_INST_PHANDLE(inst, mcpwm_sync), 0, flags)) |         \
	 ESP_INTR_FLAG_IRAM)

#define SENSOR_ESP32_DEFINE(inst)                                                                  \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, adc_channels) == 1, "motor_sensor_esp32: one adc-channels entry"); \
	MOTOR_ESP32_INST_ASSERT_DMA(inst)                                                          \
	static const struct motor_sensor_esp32_config sensor_esp32_cfg_##inst = {               \
		.mcpwm_sync = (mcpwm_dev_t *)DT_REG_ADDR(DT_INST_PHANDLE(inst, mcpwm_sync)),          \
		.sync_timer_id = (uint8_t)DT_INST_PROP(inst, sync_timer_id),                         \
		.sync_is_tep = (strcmp(DT_INST_PROP(inst, sync_edge), "tep") == 0),                  \
		.sync_actuator = DEVICE_DT_GET(DT_INST_PHANDLE(inst, sync_actuator)),               \
		.feedback_sensor =                                                                   \
			COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, feedback_sensor),                    \
				    (DEVICE_DT_GET(DT_INST_PHANDLE(inst, feedback_sensor))),         \
				    (NULL)),                                                         \
		.adc_unit = (adc_unit_t)(DT_PROP(ADC_NODE(inst), unit) - 1),                          \
		.adc_channel = (uint8_t)DT_INST_PROP_BY_IDX(inst, adc_channels, 0),                  \
		.adc_atten = adc_atten_from_db(DT_INST_PROP(inst, adc_channel_attenuation_db)),     \
		.adc_resolution_bits = (uint8_t)DT_INST_PROP(inst, adc_resolution_bits),             \
		.adc_vref_mv = (uint32_t)DT_INST_PROP(inst, adc_vref_mv),                            \
		.adc_sample_freq_hz = (uint32_t)DT_INST_PROP(inst, adc_digi_sample_freq_hz),          \
		.digi_block_samples = (uint32_t)DT_INST_PROP(inst, digi_block_samples),              \
		.adc_clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(ADC_NODE(inst))),                       \
		.adc_clock_subsys =                                                                    \
			(clock_control_subsys_t)DT_CLOCKS_CELL(ADC_NODE(inst), offset),               \
		.dma_dev = COND_CODE_1(DT_NODE_HAS_PROP(ADC_NODE(inst), dmas),                       \
				       (DEVICE_DT_GET(DT_DMAS_CTLR_BY_IDX(ADC_NODE(inst), 0))),       \
				       (NULL)),                                                        \
		.dma_channel = COND_CODE_1(DT_NODE_HAS_PROP(ADC_NODE(inst), dmas),                   \
					 (DT_DMAS_CELL_BY_IDX(ADC_NODE(inst), 0, channel)), (0)),    \
		.amps_per_volt =                                                                     \
			(float)DT_INST_PROP_OR(inst, amps_per_volt_milli, 1000) / 1000.0f,           \
	};                                                                                         \
	static struct motor_sensor_esp32_data sensor_esp32_data_##inst;                            \
	static int motor_sensor_esp32_dev_init_##inst(const struct device *dev)                    \
	{                                                                                            \
		const struct motor_sensor_esp32_config *cfg = dev->config;                           \
		struct motor_sensor_esp32_data *data = dev->data;                                    \
		struct motor_enc_adc_digi_ctx *digi = &data->digi;                                   \
		uint32_t ev;                                                                           \
		int err;                                                                               \
		int irq_flags;                                                                         \
		motor_sensor_esp32_data_default_init(data);                                          \
		data->amps_per_count = (((float)cfg->adc_vref_mv * 1e-3f) /                            \
					 (float)(1U << cfg->adc_resolution_bits)) *                    \
				       cfg->amps_per_volt;                                              \
		if (!device_is_ready(cfg->sync_actuator)) {                                            \
			return -ENODEV;                                                                \
		}                                                                                      \
		if ((cfg->feedback_sensor != NULL) && !device_is_ready(cfg->feedback_sensor)) {       \
			return -ENODEV;                                                                \
		}                                                                                      \
		err = motor_sensor_esp32_dma_dev_check(cfg);                                           \
		if (err != 0) {                                                                        \
			return err;                                                                    \
		}                                                                                      \
		if (!device_is_ready(cfg->adc_clock_dev)) {                                            \
			return -ENODEV;                                                                \
		}                                                                                      \
		err = motor_sensor_esp32_check_eof_pwm_grid(cfg);                                      \
		if (err != 0) {                                                                        \
			return err;                                                                    \
		}                                                                                      \
		digi->unit = cfg->adc_unit;                                                            \
		digi->dma_dev = cfg->dma_dev;                                                          \
		digi->dma_channel = cfg->dma_channel;                                                  \
		digi->clock_dev = cfg->adc_clock_dev;                                                  \
		digi->clock_subsys = cfg->adc_clock_subsys;                                            \
		err = motor_enc_adc_digi_init(digi);                                                   \
		if (err != 0) {                                                                        \
			return err;                                                                    \
		}                                                                                      \
		err = motor_enc_adc_digi_start(digi, cfg->adc_channel, cfg->adc_atten,                  \
					       cfg->adc_resolution_bits, cfg->adc_sample_freq_hz,        \
					       cfg->digi_block_samples, motor_enc_digi_eof, (void *)dev); \
		if (err != 0) {                                                                        \
			return err;                                                                    \
		}                                                                                      \
		ev = cfg->sync_is_tep ? MCPWM_LL_EVENT_TIMER_FULL(cfg->sync_timer_id)                  \
				      : MCPWM_LL_EVENT_TIMER_EMPTY(cfg->sync_timer_id);                \
		mcpwm_ll_intr_clear_status(cfg->mcpwm_sync, ev);                                       \
		mcpwm_ll_intr_enable(cfg->mcpwm_sync, ev, true);                                       \
		irq_flags = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, zephyr_mcpwm_sync_irq_priority), \
					(SENSOR_ENC_IRQ_FLAGS_PRIO(inst)),                           \
					(SENSOR_ENC_IRQ_FLAGS_DEFAULT(inst)));                       \
		err = esp_intr_alloc(DT_IRQ_BY_IDX(DT_INST_PHANDLE(inst, mcpwm_sync), 0, irq),         \
				     irq_flags, mcpwm_sync_isr, (void *)dev, &data->mcpwm_intr_handle);   \
		if (err != ESP_OK) {                                                                   \
			motor_enc_adc_digi_stop(digi);                                                 \
			return -EIO;                                                                   \
		}                                                                                      \
		return 0;                                                                              \
	}                                                                                            \
	DEVICE_DT_DEFINE(DT_DRV_INST(inst), motor_sensor_esp32_dev_init_##inst, NULL,              \
			 &sensor_esp32_data_##inst, &sensor_esp32_cfg_##inst, POST_KERNEL,            \
			 CONFIG_MOTOR_SENSOR_ESP32_INIT_PRIORITY, &motor_sensor_esp32_api)

DT_INST_FOREACH_STATUS_OKAY(SENSOR_ESP32_DEFINE)
