
set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(adc_battery)

target_sources(app PRIVATE src/main.c)
