/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/subsys/motor/motor_ctrl_priv.h>

int motor_pipeline_init(struct motor_pipeline *pipeline, void *ctx)
{
	ARG_UNUSED(ctx);

	if (pipeline == NULL) {
		return -EINVAL;
	}

	return 0;
}

void motor_pipeline_reset(struct motor_pipeline *pipeline, void *ctx)
{
	ARG_UNUSED(ctx);

	if (pipeline == NULL) {
		return;
	}

	for (uint8_t i = 0; i < pipeline->n_blocks; i++) {
		struct motor_block *b = pipeline->blocks[i];

		if ((b != NULL) && (b->reset != NULL)) {
			b->reset(b);
		}
	}
}

void motor_pipeline_run_stage(struct motor_pipeline *pipeline, void *ctx,
			      enum motor_pipeline_stage stage, uint32_t stage_tick,
			      const struct motor_block_in *in, struct motor_block_out *out)
{
	struct motor_block_in run_in;

	ARG_UNUSED(ctx);

	if ((pipeline == NULL) || (in == NULL) || (out == NULL)) {
		return;
	}

	if (stage != MOTOR_STAGE_INNER_ISR) {
		return;
	}

	run_in = *in;

	for (uint8_t i = 0; i < pipeline->n_blocks; i++) {
		struct motor_block *b = pipeline->blocks[i];

		if ((b == NULL) || (b->stage != stage)) {
			continue;
		}

		uint16_t div = (b->period_div != 0U) ? b->period_div : 1U;

		if ((stage_tick % div) != 0U) {
			continue;
		}

		if (b->entry != NULL) {
			b->entry(b, &run_in, out);
		}
	}
}
