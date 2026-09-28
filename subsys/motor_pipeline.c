/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_ctrl_priv.h"

#include <errno.h>
#include <zephyr/sys/util.h>

int motor_pipeline_init(struct motor_pipeline *pipeline, void *ctx)
{
	ARG_UNUSED(ctx);

	if (pipeline == NULL) {
		return -EINVAL;
	}

	for (uint8_t i = 0; i < pipeline->n_blocks; i++) {
		struct motor_block *b = pipeline->blocks[i];

		if ((b != NULL) && (b->set_params != NULL)) {
			b->set_params(b);
		}
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

	if (stage >= MOTOR_STAGE_COUNT) {
		return;
	}

	run_in = *in;
	out->n_duty = 0U;
	out->has_current_ref = false;
	out->has_applied_u = false;

	for (uint8_t i = 0; i < pipeline->n_blocks; i++) {
		struct motor_block *b = pipeline->blocks[i];
		struct motor_block_out block_out = {0};

		if ((b == NULL) || (b->stage != stage)) {
			continue;
		}

		uint16_t div = (b->period_div != 0U) ? b->period_div : 1U;

		if ((stage_tick % div) != 0U) {
			continue;
		}

		if (b->entry != NULL) {
			b->entry(b, &run_in, &block_out);
		}

		if (block_out.n_duty != 0U) {
			out->n_duty = block_out.n_duty;
			for (uint8_t d = 0; d < block_out.n_duty; d++) {
				out->duty[d] = block_out.duty[d];
			}
		}

		if (block_out.has_current_ref) {
			out->has_current_ref = true;
			out->current_ref_a = block_out.current_ref_a;
			run_in.has_current_ref = true;
			run_in.current_ref_a = block_out.current_ref_a;
		}

		/* Deliberately not fed back into run_in: a block must always see
		 * the command from the previous tick, whatever its position.
		 */
		if (block_out.has_applied_u) {
			out->has_applied_u = true;
			out->applied_u = block_out.applied_u;
		}
	}
}
