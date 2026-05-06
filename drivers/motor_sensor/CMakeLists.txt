# Copyright (c) 2026 Freedom Veiculos Eletricos
# SPDX-License-Identifier: UNLICENSED

zephyr_library_sources_ifdef(CONFIG_MOTOR_SENSOR_STM32 motor_sensor_stm32.c)
zephyr_library_sources_ifdef(CONFIG_MOTOR_SENSOR_ESP32 motor_sensor_esp32.c)
