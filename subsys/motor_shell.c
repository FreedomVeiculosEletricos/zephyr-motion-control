/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>
#include <zephyr/subsys/motor/algorithms/dc_current/motor_algo_dc_current.h>
#include <zephyr/subsys/motor/motor.h>
#include <zephyr/subsys/motor/motor_subsys.h>

static struct motor_ctrl *motor_shell_ctx;

static int select_motor_by_token(const char *s, struct motor_ctrl **out)
{
	unsigned long u;
	char *end = NULL;
	struct motor_ctrl *m;

	if (s == NULL || out == NULL) {
		return -EINVAL;
	}

	u = strtoul(s, &end, 10);
	if ((end != s) && (*end == '\0') && (u < (unsigned long)motor_subsys_count())) {
		m = motor_subsys_get_by_index((uint8_t)u);
		if (m != NULL) {
			*out = m;
			return 0;
		}
	}

	m = motor_subsys_get_by_label(s);
	if (m != NULL) {
		*out = m;
		return 0;
	}

	return -ENOENT;
}

static void print_motor_status(const struct shell *sh, struct motor_ctrl *m)
{
	enum motor_state st;
	uint32_t fa;
	struct motor_dc_current_pi pi;
	struct motor_dc_current_limits lim;
	int r;

	motor_get_status(m, &st, &fa, NULL);
	shell_print(sh, "  state=%d faults=0x%08x", (int)st, fa);
	r = motor_algo_dc_current_get_pi_gains(m, &pi);
	if (r == 0) {
		shell_print(sh, "  DC PI: kp=%f ki=%f out[%.3f, %.3f]", (double)pi.kp, (double)pi.ki,
			    (double)pi.out_min, (double)pi.out_max);
	} else {
		shell_print(sh, "  DC PI: n/a (%d)", r);
	}
	r = motor_algo_dc_current_get_limits(m, &lim);
	if (r == 0) {
		shell_print(sh, "  DC limits: i_max_a=%f", (double)lim.i_max_a);
	} else {
		shell_print(sh, "  DC limits: n/a (%d)", r);
	}
}

static int cmd_motor(const struct shell *sh, size_t argc, char **argv)
{
	int err;
	struct motor_ctrl *m = motor_shell_ctx;
	const char *sub;
	uint32_t fl;
	struct motor_dc_current_pi pi;
	struct motor_dc_current_limits lim;
	bool reset_i;

	if (argc < 2) {
		shell_error(sh, "sub: list|use|enable|disable|estop|current|selftest|status|pi|imax");
		return -EINVAL;
	}

	sub = argv[1];

	if (strcmp(sub, "list") == 0) {
		uint8_t n = motor_subsys_count();

		shell_print(sh, "motors: %u", n);
		for (uint8_t i = 0; i < n; i++) {
			struct motor_ctrl *mm = motor_subsys_get_by_index(i);

			shell_print(sh, "  [%u] %s", i, motor_subsys_label_get(mm));
		}
		return 0;
	}

	if (strcmp(sub, "use") == 0) {
		if (argc < 3) {
			shell_error(sh, "use <index|label>");
			return -EINVAL;
		}
		err = select_motor_by_token(argv[2], &motor_shell_ctx);
		if (err != 0) {
			shell_error(sh, "not found (%d)", err);
			return err;
		}
		shell_print(sh, "selected: %s", motor_subsys_label_get(motor_shell_ctx));
		return 0;
	}

	if (m == NULL) {
		shell_error(sh, "no selection: motor use <index|label>");
		return -EINVAL;
	}

	if (strcmp(sub, "enable") == 0) {
		return motor_enable(m);
	}
	if (strcmp(sub, "disable") == 0) {
		return motor_disable(m);
	}
	if (strcmp(sub, "estop") == 0) {
		motor_estop(m);
		return 0;
	}

	if (strcmp(sub, "current") == 0) {
		if (argc < 3) {
			shell_error(sh, "current <A>");
			return -EINVAL;
		}
		return motor_set_current(m, strtof(argv[2], NULL));
	}

	if (strcmp(sub, "selftest") == 0) {
		err = motor_self_test(m, &fl);
		if (err == 0) {
			shell_print(sh, "selftest ok faults=0x%08x", fl);
		} else {
			shell_error(sh, "selftest err=%d faults=0x%08x", err, fl);
		}
		return err;
	}

	if (strcmp(sub, "status") == 0) {
		shell_print(sh, "motor: %s", motor_subsys_label_get(m));
		print_motor_status(sh, m);
		return 0;
	}

	if (strcmp(sub, "pi") == 0) {
		if (argc < 4) {
			shell_error(sh, "pi <Kp> <Ki> [reset: 0|1]");
			return -EINVAL;
		}
		err = motor_algo_dc_current_get_pi_gains(m, &pi);
		if (err != 0) {
			return err;
		}
		pi.kp = strtof(argv[2], NULL);
		pi.ki = strtof(argv[3], NULL);
		reset_i = (argc >= 5) && (strtoul(argv[4], NULL, 10) != 0U);
		return motor_algo_dc_current_set_pi_gains(m, &pi, reset_i);
	}

	if (strcmp(sub, "imax") == 0) {
		if (argc < 3) {
			shell_error(sh, "imax <A>");
			return -EINVAL;
		}
		err = motor_algo_dc_current_get_limits(m, &lim);
		if (err != 0) {
			return err;
		}
		lim.i_max_a = strtof(argv[2], NULL);
		return motor_algo_dc_current_set_limits(m, &lim);
	}

	shell_error(sh, "unknown: %s", sub);
	return -EINVAL;
}

SHELL_CMD_ARG_REGISTER(motor, NULL, "Motor control: list, use, enable, current, pi, imax, …", cmd_motor, 1,
		       8);
