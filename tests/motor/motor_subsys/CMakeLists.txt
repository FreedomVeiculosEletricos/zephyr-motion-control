# Copyright (c) 2026 Zephyr Motor Control Contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor_subsys_test)

target_sources(app PRIVATE src/main.c ../common/motor_test_fake.c)
target_include_directories(app PRIVATE ../common)
