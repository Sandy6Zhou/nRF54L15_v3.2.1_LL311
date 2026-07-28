#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(LL311_BLE)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

target_include_directories(app PRIVATE
 ./inc
 ./src
 ./ext_module/QMI8658B/api
)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/gsensor_test_main.c
  src/my_shell.c
  src/my_gsensor.c
  ext_module/QMI8658B/api/imu_api.c
  ext_module/QMI8658B/port/qmi8658b_port.c
  ext_module/QMI8658B/driver/qmi8658b_driver.c
)

# NORDIC SDK APP END
