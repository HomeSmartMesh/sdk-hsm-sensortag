# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

set(BOARD nrf52840dk_nrf52840)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(openthread-node)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
