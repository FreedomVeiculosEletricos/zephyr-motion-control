# Copyright (c) 2026 Freedom Veiculos Eletricos
# SPDX-License-Identifier: UNLICENSED

zephyr_library_sources_ifdef(CONFIG_MOTOR_SENSOR_ENCODER_STM32 motor_sensor_encoder_stm32.c)
zephyr_library_sources_ifdef(CONFIG_MOTOR_SENSOR_ENCODER_ESP32S3 motor_sensor_encoder_esp32s3.c)
