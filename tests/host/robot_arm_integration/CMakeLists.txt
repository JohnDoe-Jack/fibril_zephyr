cmake_minimum_required(VERSION 3.20)
project(robot_arm_integration_tests LANGUAGES CXX)
enable_testing()
add_executable(robot_arm_integration_tests
  main.cpp
  ../../../apps/robot_arm/src/input_logic.cpp
  ../../../lib/differential_screw/wrist.cpp
)
target_include_directories(robot_arm_integration_tests PRIVATE
  ../../../apps/robot_arm/src ../../../include)
target_compile_features(robot_arm_integration_tests PRIVATE cxx_std_17)
target_compile_options(robot_arm_integration_tests PRIVATE
  -Wall -Wextra -Werror -fno-exceptions -fno-rtti)
add_test(NAME robot_arm_integration COMMAND robot_arm_integration_tests)
