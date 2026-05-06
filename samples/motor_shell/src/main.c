/*
 * Copyright (c) 2026 Freedom Veiculos Eletricos
 * SPDX-License-Identifier: UNLICENSED
 *
 * Boots the console shell. Motor instances come from Devicetree; use
 *   motor list
 *   motor use <index|label>
 *   motor selftest
 *   motor enable
 *   motor current <A>
 *   motor status
 *   motor pi <Kp> <Ki> [0|1]
 *   motor imax <A>
 * Type ``motor`` in the shell for the full command set.
 */

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include <zephyr/subsys/motor/motor_subsys.h>

BUILD_ASSERT(DT_NODE_EXISTS(DT_NODELABEL(motor_brushed)),
	     "Add motor_brushed zephyr,motor-controller in board overlay");

int main(void)
{
	k_sleep(K_SECONDS(1U));
	/* Shell runs on the console UART; `motor` commands require motor_subsys. */
	return 0;
}
