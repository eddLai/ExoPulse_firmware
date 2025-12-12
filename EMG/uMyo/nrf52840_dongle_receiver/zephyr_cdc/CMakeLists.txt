# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf52840_dongle_cdc)

target_sources(app PRIVATE
    src/main.c
    src/urf_lib/urf_radio.c
    src/urf_lib/urf_star_protocol.c
    src/urf_lib/urf_timer.c
)

target_include_directories(app PRIVATE src/urf_lib)
