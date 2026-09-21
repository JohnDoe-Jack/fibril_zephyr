cmake_minimum_required(VERSION 3.20)
project(differential_screw_tests LANGUAGES CXX)

add_executable(differential_screw_tests
  main.cpp
  ../../../lib/differential_screw/wrist.cpp
)
target_include_directories(differential_screw_tests PRIVATE ../../../include)
target_compile_features(differential_screw_tests PRIVATE cxx_std_17)
target_compile_options(differential_screw_tests PRIVATE -Wall -Wextra -Werror -fno-exceptions -fno-rtti)

enable_testing()
add_test(NAME differential_screw COMMAND differential_screw_tests)
