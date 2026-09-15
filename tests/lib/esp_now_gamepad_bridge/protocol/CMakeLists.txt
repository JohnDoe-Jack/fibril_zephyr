cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(gamepad_protocol_test)
target_sources(app PRIVATE src/main.c
  ${ZEPHYR_FIBRIL_ZEPHYR_MODULE_DIR}/lib/esp_now_gamepad_bridge/gamepad_protocol.c)
target_include_directories(app PRIVATE
  ${ZEPHYR_FIBRIL_ZEPHYR_MODULE_DIR}/lib/esp_now_gamepad_bridge)
