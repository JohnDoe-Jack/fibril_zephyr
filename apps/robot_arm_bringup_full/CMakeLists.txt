cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(robot_arm_bringup_full)
target_sources(app PRIVATE ../robot_arm/src/main.cpp ../robot_arm/src/input_logic.cpp)
target_compile_definitions(app PRIVATE ROBOT_ARM_BRINGUP=1)

