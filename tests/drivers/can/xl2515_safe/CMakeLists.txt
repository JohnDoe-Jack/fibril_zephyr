# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xl2515_safe)

target_sources(app PRIVATE src/main.c src/emul_xl2515.c)
