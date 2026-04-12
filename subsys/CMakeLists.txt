# Copyright (c) 2026 Freedom Veiculos Eletricos
# SPDX-License-Identifier: UNLICENSED

zephyr_library_named(motor_control)

zephyr_library_sources_ifdef(CONFIG_MOTOR_CONTROL_LIB motor_ctrl.c motor.c motor_algo_dc_torque.c)
zephyr_library_sources_ifdef(CONFIG_MOTOR_SUBSYS motor_subsys.c)

zephyr_linker_sources_ifdef(CONFIG_MOTOR_SUBSYS SECTIONS linker/motor-subsys.ld)
