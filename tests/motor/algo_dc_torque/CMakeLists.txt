# Copyright (c) 2026 Freedom Veiculos Eletricos
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor_algo_dc_torque_test)

set(MOTOR_MODULE ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

target_sources(app PRIVATE src/main.c ${MOTOR_MODULE}/subsys/motor_algo_dc_torque.c)
target_include_directories(app PRIVATE ${MOTOR_MODULE}/include)
