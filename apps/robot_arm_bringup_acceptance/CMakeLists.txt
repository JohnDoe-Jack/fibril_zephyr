cmake_minimum_required(VERSION 3.20.0)
set(CONF_FILE "${CMAKE_CURRENT_LIST_DIR}/../robot_arm_bringup_full/prj.conf")
set(DTC_OVERLAY_FILE
    "${CMAKE_CURRENT_LIST_DIR}/../robot_arm_bringup_full/boards/waveshare_rp2350_can_rp2350a_m33.overlay")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(robot_arm_bringup_acceptance)
target_sources(app PRIVATE ../robot_arm/src/main.cpp ../robot_arm/src/input_logic.cpp)
target_compile_definitions(app PRIVATE ROBOT_ARM_BRINGUP=1 ROBOT_ARM_ACCEPTANCE=1)

