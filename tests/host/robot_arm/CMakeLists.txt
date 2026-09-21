cmake_minimum_required(VERSION 3.20)
project(robot_arm_input_tests LANGUAGES CXX)
enable_testing()
add_executable(robot_arm_input_tests
  main.cpp
  ../../../apps/robot_arm/src/input_logic.cpp
)
target_include_directories(robot_arm_input_tests PRIVATE ../../../apps/robot_arm/src)
target_compile_features(robot_arm_input_tests PRIVATE cxx_std_17)
target_compile_options(robot_arm_input_tests PRIVATE -Wall -Wextra -Werror -fno-exceptions -fno-rtti)
add_test(NAME robot_arm_input COMMAND robot_arm_input_tests)
