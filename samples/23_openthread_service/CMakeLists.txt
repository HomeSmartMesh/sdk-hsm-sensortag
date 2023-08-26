
set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

cmake_minimum_required(VERSION 3.13.1)
set(BOARD nrf52840_sensortag)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(openthread_sensor_service)

target_sources(app PRIVATE src/main.c)
