# Copyright (c) 2026 Freedom Veiculos Eletricos
# SPDX-License-Identifier: UNLICENSED

zephyr_library_sources_ifdef(CONFIG_MOTOR_ENCODER_OBSERVER motor_encoder_observer.c)
