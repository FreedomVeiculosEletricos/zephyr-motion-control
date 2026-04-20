/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-S3: SAR ADC Digi + GDMA EOF and MCPWM TEZ/TEP ISR for sync (no ETM).
 * ADC digital path is implemented here (same idea as Zephyr adc_esp32_dma.c) so this
 * driver stays self-contained; only Espressif HAL/LL headers are used.
 */

#define DT_DRV_COMPAT zephyr_motor_sensor_encoder_esp32s3

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

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_esp32.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <zephyr/drivers/motor/motor_actuator.h>
#include <zephyr/drivers/motor/motor_sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_COUNTER)
#include <zephyr/drivers/counter.h>
#endif

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
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk;
	bool running;
	bool digi_started;
	motor_enc_adc_eof_fn eof_fn;
	void *eof_user;
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

static int motor_enc_adc_digi_hw_start(struct motor_enc_adc_digi_ctx *ctx,
				       adc_hal_digi_ctrlr_cfg_t *dig_cfg, uint32_t unit_atten,
				       uint32_t n_samples)
{
	int err;

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
	adc_hal_digi_connect(false);
	adc_hal_digi_reset();

	err = motor_enc_adc_dma_run(ctx);
	if (err != 0) {
		adc_lock_release(ctx->unit);
		sar_periph_ctrl_adc_continuous_power_release();
		return err;
	}

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

	adc_hal_digi_enable(false);
	adc_hal_digi_connect(false);

#if defined(ADC_LL_WORKAROUND_CLEAR_EOF_COUNTER) && ADC_LL_WORKAROUND_CLEAR_EOF_COUNTER
	adc_hal_digi_clr_eof();
#endif

	adc_hal_digi_deinit();
	adc_lock_release(ctx->unit);
	sar_periph_ctrl_adc_continuous_power_release();

	(void)dma_stop(ctx->dma_dev, ctx->dma_channel);

	ctx->digi_started = false;
}

static int motor_enc_adc_digi_init(struct motor_enc_adc_digi_ctx *ctx)
{
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

	err = motor_enc_adc_digi_hw_start(ctx, &dig_cfg, unit_atten, block_samples);
	if (err != 0) {
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

struct motor_sensor_enc_esp32s3_config {
	mcpwm_dev_t *mcpwm_sync;
	uint8_t sync_timer_id;
	bool sync_is_tep;
	const struct device *sync_actuator;
	const struct device *encoder;
	uint32_t encoder_cpr;
	uint8_t pole_pairs;
	adc_unit_t adc_unit;
	uint8_t adc_channel;
	adc_atten_t adc_atten;
	uint8_t adc_resolution_bits;
	uint32_t adc_sample_freq_hz;
	uint32_t digi_block_samples;
	const struct device *adc_clock_dev;
	clock_control_subsys_t adc_clock_subsys;
	const struct device *dma_dev;
	uint8_t dma_channel;
	float amps_per_volt;
};

struct motor_sensor_enc_esp32s3_data {
	struct motor_enc_adc_digi_ctx digi;
	struct motor_sensor_output last;
	float current_offset[3];
	uint32_t prev_cnt;
	uint64_t prev_cycles;
	atomic_t cal_done;
	uint16_t raw_latched;
	intr_handle_t mcpwm_intr_handle;
};

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
	const struct motor_sensor_enc_esp32s3_config *cfg = dev->config;
	mcpwm_dev_t *mcpwm = cfg->mcpwm_sync;
	uint32_t st = mcpwm_ll_intr_get_status(mcpwm);
	uint32_t ev = cfg->sync_is_tep ? MCPWM_LL_EVENT_TIMER_FULL(cfg->sync_timer_id)
				       : MCPWM_LL_EVENT_TIMER_EMPTY(cfg->sync_timer_id);

	if ((st & ev) != 0U) {
		mcpwm_ll_intr_clear_status(mcpwm, st & ev);
	}
}

static void motor_enc_digi_eof(void *user, const uint8_t *buf, uint32_t n_bytes)
{
	const struct device *dev = user;
	const struct motor_sensor_enc_esp32s3_config *cfg = dev->config;
	const adc_digi_output_data_t *d = (const adc_digi_output_data_t *)buf;
	struct motor_sensor_enc_esp32s3_data *data = dev->data;
	uint32_t ns;

	if (n_bytes < SOC_ADC_DIGI_DATA_BYTES_PER_CONV) {
		return;
	}

	ns = n_bytes / SOC_ADC_DIGI_DATA_BYTES_PER_CONV;
	data->raw_latched = (uint16_t)d[ns - 1U].type2.data;

	motor_actuator_invoke_control_callback(cfg->sync_actuator);
}

static void apply_raw_to_output(const struct motor_sensor_enc_esp32s3_config *cfg,
				struct motor_sensor_enc_esp32s3_data *data, uint16_t raw)
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

static int sensor_enc_esp32s3_init(const struct device *dev, const struct motor_sensor_cal *cal)
{
	struct motor_sensor_enc_esp32s3_data *data = dev->data;

	(void)memset(&data->last, 0, sizeof(data->last));
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

static int sensor_enc_esp32s3_update(const struct device *dev)
{
	const struct motor_sensor_enc_esp32s3_config *cfg = dev->config;
	struct motor_sensor_enc_esp32s3_data *data = dev->data;

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

static int sensor_enc_esp32s3_get(const struct device *dev, struct motor_sensor_output *out)
{
	const struct motor_sensor_enc_esp32s3_data *data = dev->data;

	if (out == NULL) {
		return -EINVAL;
	}

	*out = data->last;
	return 0;
}

static int sensor_enc_esp32s3_calibrate(const struct device *dev)
{
	struct motor_sensor_enc_esp32s3_data *data = dev->data;

	ARG_UNUSED(dev);

	atomic_set(&data->cal_done, 1);
	return 0;
}

static int sensor_enc_esp32s3_check_encoder(const struct motor_sensor_enc_esp32s3_config *cfg)
{
#if defined(CONFIG_COUNTER)
	if (cfg->encoder != NULL && !device_is_ready(cfg->encoder)) {
		return -ENODEV;
	}
#else
	ARG_UNUSED(cfg);
#endif
	return 0;
}

static void sensor_enc_esp32s3_get_backend_types(const struct device *dev,
						 enum motor_pos_backend_type *pos_type,
						 enum motor_cur_backend_type *cur_type)
{
#if defined(CONFIG_COUNTER)
	const struct motor_sensor_enc_esp32s3_config *cfg = dev->config;
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

static const struct motor_sensor_ops motor_sensor_enc_esp32s3_api = {
	.init = sensor_enc_esp32s3_init,
	.update = sensor_enc_esp32s3_update,
	.get = sensor_enc_esp32s3_get,
	.calibrate = sensor_enc_esp32s3_calibrate,
	.get_backend_types = sensor_enc_esp32s3_get_backend_types,
};

#define ADC_NODE(id) DT_INST_PHANDLE(id, st_adc)

#define SENSOR_ENC_IRQ_FLAGS_PRIO(inst)                                                          \
	(ESP_PRIO_TO_FLAGS(DT_INST_PROP(inst, zephyr_mcpwm_sync_irq_priority)) | ESP_INTR_FLAG_IRAM)

#define SENSOR_ENC_IRQ_FLAGS_DEFAULT(inst)                                                       \
	(ESP_PRIO_TO_FLAGS(DT_IRQ_BY_IDX(DT_INST_PHANDLE(inst, mcpwm_sync), 0, priority)) |        \
	 ESP_INT_FLAGS_CHECK(DT_IRQ_BY_IDX(DT_INST_PHANDLE(inst, mcpwm_sync), 0, flags)) |         \
	 ESP_INTR_FLAG_IRAM)

#define SENSOR_ENC_ESP32S3_DEFINE(inst)                                                            \
	static const struct motor_sensor_enc_esp32s3_config sensor_enc_esp32s3_cfg_##inst = {      \
		.mcpwm_sync = (mcpwm_dev_t *)DT_REG_ADDR(DT_INST_PHANDLE(inst, mcpwm_sync)),          \
		.sync_timer_id = (uint8_t)DT_INST_PROP(inst, sync_timer_id),                         \
		.sync_is_tep = (strcmp(DT_INST_PROP(inst, sync_edge), "tep") == 0),                  \
		.sync_actuator = DEVICE_DT_GET(DT_INST_PHANDLE(inst, sync_actuator)),               \
		.encoder = COND_CODE_1(DT_NODE_HAS_PROP(DT_DRV_INST(inst), encoder_timer),            \
				       (DEVICE_DT_GET(DT_INST_PHANDLE(inst, encoder_timer))),         \
				       (NULL)),                                                     \
		.encoder_cpr = (uint32_t)DT_INST_PROP(inst, encoder_cpr),                             \
		.pole_pairs = (uint8_t)DT_INST_PROP(inst, pole_pairs),                                \
		.adc_unit = (adc_unit_t)(DT_PROP(ADC_NODE(inst), unit) - 1),                          \
		.adc_channel = (uint8_t)DT_INST_PROP(inst, adc_channel),                              \
		.adc_atten = adc_atten_from_db(DT_INST_PROP(inst, adc_attenuation_db)),              \
		.adc_resolution_bits = 12,                                                           \
		.adc_sample_freq_hz = (uint32_t)DT_INST_PROP(inst, adc_digi_sample_freq_hz),          \
		.digi_block_samples = (uint32_t)DT_INST_PROP(inst, digi_block_samples),                \
		.adc_clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(ADC_NODE(inst))),                       \
		.adc_clock_subsys =                                                                    \
			(clock_control_subsys_t)DT_CLOCKS_CELL(ADC_NODE(inst), offset),               \
		.dma_dev = DEVICE_DT_GET(DT_DMAS_CTLR_BY_IDX(ADC_NODE(inst), 0)),                    \
		.dma_channel = DT_DMAS_CELL_BY_IDX(ADC_NODE(inst), 0, channel),                       \
		.amps_per_volt = AMPS_PER_VOLT_DEFAULT,                                               \
	};                                                                                         \
	static struct motor_sensor_enc_esp32s3_data sensor_enc_esp32s3_data_##inst;                  \
	static int sensor_enc_esp32s3_dev_init_##inst(const struct device *dev)                      \
	{                                                                                            \
		const struct motor_sensor_enc_esp32s3_config *cfg = dev->config;                       \
		struct motor_sensor_enc_esp32s3_data *data = dev->data;                                \
		struct motor_enc_adc_digi_ctx *digi = &data->digi;                                   \
		uint32_t ev;                                                                           \
		int err;                                                                               \
		int irq_flags;                                                                         \
		if (!device_is_ready(cfg->sync_actuator)) {                                            \
			return -ENODEV;                                                                \
		}                                                                                      \
		err = sensor_enc_esp32s3_check_encoder(cfg);                                           \
		if (err != 0) {                                                                        \
			return err;                                                                    \
		}                                                                                      \
		if (!device_is_ready(cfg->dma_dev)) {                                                  \
			return -ENODEV;                                                                \
		}                                                                                      \
		if (!device_is_ready(cfg->adc_clock_dev)) {                                            \
			return -ENODEV;                                                                \
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
	DEVICE_DT_DEFINE(DT_DRV_INST(inst), sensor_enc_esp32s3_dev_init_##inst, NULL,                \
			 &sensor_enc_esp32s3_data_##inst, &sensor_enc_esp32s3_cfg_##inst, POST_KERNEL, \
			 CONFIG_MOTOR_SENSOR_ENCODER_ESP32S3_INIT_PRIORITY, &motor_sensor_enc_esp32s3_api)

DT_INST_FOREACH_STATUS_OKAY(SENSOR_ENC_ESP32S3_DEFINE)
