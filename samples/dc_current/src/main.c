/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal DC current loop demo: DT motor-controller node, subsystem registration,
 * calibrate shunt, enable bridge, command torque. Parameters from Devicetree.
 */

#include <stdio.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/subsys/motor/motor.h>
#include <zephyr/subsys/motor/motor_subsys.h>

BUILD_ASSERT(DT_NODE_EXISTS(DT_NODELABEL(motor_brushed)),
	     "Overlay must define motor_brushed zephyr,motor-controller");

MOTOR_CTRL_DEFINE(motor_ctrl_inst);

MOTOR_SUBSYS_DEFINE_DT(motor_brushed, &motor_ctrl_inst);

int main(void)
{
	motor_t m = motor_subsys_get_by_label("motor_brushed");
	uint32_t faults = 0U;
	int err;

	printf("Motor control DC current (motor_subsys)\n");

	if (m == NULL) {
		printf("motor_subsys_get_by_label failed\n");
		return 0;
	}

	err = motor_self_test(m, &faults);
	if (err != 0) {
		printf("motor_self_test failed: %d faults=0x%08x\n", err, faults);
		return 0;
	}

	err = motor_enable(m);
	if (err != 0) {
		printf("motor_enable failed: %d\n", err);
		return 0;
	}

	err = motor_set_torque(m, 0.05f);
	if (err != 0) {
		printf("motor_set_torque failed: %d\n", err);
	}

	k_sleep(K_SECONDS(2));

	(void)motor_disable(m);
	printf("Stopped.\n");

	return 0;
}
