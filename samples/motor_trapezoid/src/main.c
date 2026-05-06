/*
 * Copyright (c) 2026 Freedom Veiculos Eletricos
 * SPDX-License-Identifier: UNLICENSED
 *
 * Shell-controlled trapezoidal current trajectory for the dc-current algorithm.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>

#include <zephyr/subsys/motor/algorithms/dc_current/motor_algo_dc_current.h>
#include <zephyr/subsys/motor/motor.h>
#include <zephyr/subsys/motor/motor_subsys.h>

LOG_MODULE_REGISTER(motor_trapezoid, LOG_LEVEL_INF);

BUILD_ASSERT(DT_NODE_EXISTS(DT_NODELABEL(motor_brushed)),
	     "Add motor_brushed zephyr,motor-controller in board overlay");

#define TRAJ_TICK_MS      10U
#define TRAJ_RAMP_STEPS   (5000U / TRAJ_TICK_MS)
#define TRAJ_HOLD_STEPS   (5000U / TRAJ_TICK_MS)
#define TRAJ_ZERO_STEPS   (1000U / TRAJ_TICK_MS)
#define TRAJ_STACK_SIZE   2048
#define TRAJ_THREAD_PRIO  5

static K_THREAD_STACK_DEFINE(traj_stack, TRAJ_STACK_SIZE);
static struct k_thread traj_thread;
static atomic_t traj_run_requested;
static atomic_t traj_thread_active;
static motor_t traj_motor;
static float traj_peak_a;

static bool traj_should_run(void)
{
	return atomic_get(&traj_run_requested) != 0;
}

static int traj_set_current(motor_t motor, float i_a)
{
	int err = motor_algo_dc_current_set_current(motor, i_a);

	if (err != 0) {
		LOG_ERR("motor_algo_dc_current_set_current failed: %d", err);
	}

	return err;
}

static int traj_hold(motor_t motor, float i_a, uint32_t steps)
{
	for (uint32_t step = 0U; step < steps; step++) {
		if (!traj_should_run()) {
			return -ECANCELED;
		}
		if (traj_set_current(motor, i_a) != 0) {
			return -EIO;
		}
		k_sleep(K_MSEC(TRAJ_TICK_MS));
	}

	return 0;
}

static int traj_ramp(motor_t motor, float from_a, float to_a, uint32_t steps)
{
	for (uint32_t step = 1U; step <= steps; step++) {
		float ratio;
		float i_a;

		if (!traj_should_run()) {
			return -ECANCELED;
		}

		ratio = (float)step / (float)steps;
		i_a = from_a + ((to_a - from_a) * ratio);
		if (traj_set_current(motor, i_a) != 0) {
			return -EIO;
		}
		k_sleep(K_MSEC(TRAJ_TICK_MS));
	}

	return 0;
}

static int traj_cycle(motor_t motor, float peak_a)
{
	int err;

	err = traj_ramp(motor, 0.0f, peak_a, TRAJ_RAMP_STEPS);
	if (err != 0) {
		return err;
	}
	err = traj_hold(motor, peak_a, TRAJ_HOLD_STEPS);
	if (err != 0) {
		return err;
	}
	err = traj_ramp(motor, peak_a, 0.0f, TRAJ_RAMP_STEPS);
	if (err != 0) {
		return err;
	}
	err = traj_hold(motor, 0.0f, TRAJ_ZERO_STEPS);
	if (err != 0) {
		return err;
	}
	err = traj_ramp(motor, 0.0f, -peak_a, TRAJ_RAMP_STEPS);
	if (err != 0) {
		return err;
	}
	err = traj_hold(motor, -peak_a, TRAJ_HOLD_STEPS);
	if (err != 0) {
		return err;
	}
	err = traj_ramp(motor, -peak_a, 0.0f, TRAJ_RAMP_STEPS);
	if (err != 0) {
		return err;
	}

	return traj_hold(motor, 0.0f, TRAJ_ZERO_STEPS);
}

static void traj_thread_fn(void *p1, void *p2, void *p3)
{
	motor_t motor = p1;
	struct motor_dc_current_limits limits;
	int err;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	err = motor_algo_dc_current_get_limits(motor, &limits);
	if (err != 0) {
		LOG_ERR("motor_algo_dc_current_get_limits failed: %d", err);
		goto out;
	}

	traj_peak_a = limits.i_max_a;
	err = motor_enable(motor);
	if (err != 0) {
		LOG_ERR("motor_enable failed: %d", err);
		goto out;
	}

	while (traj_should_run()) {
		err = traj_cycle(motor, traj_peak_a);
		if ((err != 0) && (err != -ECANCELED)) {
			LOG_ERR("trajectory failed: %d", err);
			break;
		}
	}

	(void)motor_algo_dc_current_set_current(motor, 0.0f);
	(void)motor_disable(motor);

out:
	atomic_clear(&traj_run_requested);
	atomic_clear(&traj_thread_active);
}

static int cmd_trapezoid_start(const struct shell *sh)
{
	motor_t motor;

	if (atomic_get(&traj_thread_active) != 0) {
		shell_print(sh, "trajectory already running");
		return 0;
	}

	motor = motor_subsys_get_by_label("motor_brushed");
	if (motor == NULL) {
		shell_error(sh, "motor_brushed not found");
		return -ENODEV;
	}

	traj_motor = motor;
	atomic_set(&traj_run_requested, 1);
	atomic_set(&traj_thread_active, 1);
	k_thread_create(&traj_thread, traj_stack, K_THREAD_STACK_SIZEOF(traj_stack), traj_thread_fn,
			motor, NULL, NULL, TRAJ_THREAD_PRIO, 0, K_NO_WAIT);

	shell_print(sh, "trajectory started");
	return 0;
}

static int cmd_trapezoid_stop(const struct shell *sh)
{
	if (atomic_get(&traj_thread_active) == 0) {
		shell_print(sh, "trajectory already stopped");
		return 0;
	}

	atomic_clear(&traj_run_requested);
	shell_print(sh, "trajectory stopping");
	return 0;
}

static int cmd_trapezoid_status(const struct shell *sh)
{
	struct motor_dc_current_state state;
	enum motor_state motor_state;
	uint32_t faults;
	int err;

	shell_print(sh, "requested=%d active=%d peak=%f A", (int)atomic_get(&traj_run_requested),
		    (int)atomic_get(&traj_thread_active), (double)traj_peak_a);

	if (traj_motor == NULL) {
		return 0;
	}

	motor_get_status(traj_motor, &motor_state, &faults);
	shell_print(sh, "motor_state=%d faults=0x%08x", (int)motor_state, faults);

	err = motor_algo_dc_current_get_state(traj_motor, &state);
	if (err == 0) {
		shell_print(sh, "i_ref=%f i_meas=%f err=%f duty=%f", (double)state.i_ref_a,
			    (double)state.i_meas_a, (double)state.error_a, (double)state.duty);
	}

	return err;
}

static int cmd_trapezoid(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "sub: start|stop|status");
		return -EINVAL;
	}

	if (strcmp(argv[1], "start") == 0) {
		return cmd_trapezoid_start(sh);
	}
	if (strcmp(argv[1], "stop") == 0) {
		return cmd_trapezoid_stop(sh);
	}
	if (strcmp(argv[1], "status") == 0) {
		return cmd_trapezoid_status(sh);
	}

	shell_error(sh, "unknown: %s", argv[1]);
	return -EINVAL;
}

SHELL_CMD_ARG_REGISTER(trapezoid, NULL, "Trapezoidal current trajectory: start|stop|status",
		       cmd_trapezoid, 2, 0);

int main(void)
{
	LOG_INF("Use `trapezoid start` to run the current trajectory");
	return 0;
}
