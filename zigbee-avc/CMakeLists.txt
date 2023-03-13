#
# Copyright (c) 2020 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project("Zigbee Multifunction A/V Controller")

target_sources(app PRIVATE
  src/main.c
  src/cc1101.c
)
target_include_directories(app PRIVATE include)

# used when compiling zcl_el_measurement.c in the SDK
target_compile_definitions(zboss INTERFACE
  ZB_ZCL_SUPPORT_CLUSTER_ELECTRICAL_MEASUREMENT
)
