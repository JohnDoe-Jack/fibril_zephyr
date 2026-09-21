cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(gamepad_servo_protocol_test)
target_sources(app PRIVATE src/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../apps/gamepad_servo/src/can_servo_control.c)
target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../apps/gamepad_servo/src)
