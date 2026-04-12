# Copyright (c) 2026 Freedom Veiculos Eletricos
# SPDX-License-Identifier: UNLICENSED

zephyr_library_sources_ifdef(CONFIG_MOTOR_STAGE_HBRIDGE_STM32 motor_stage_hbridge_stm32.c)
zephyr_library_sources_ifdef(CONFIG_MOTOR_STAGE_HBRIDGE_ESP32S3 motor_stage_hbridge_esp32s3.c)
