# Copyright (c) 2026 Freedom Veiculos Eletricos
# SPDX-License-Identifier: UNLICENSED

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dc_current_full_bridge)

target_sources(app PRIVATE src/main.c)
