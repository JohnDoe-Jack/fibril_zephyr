cmake_minimum_required(VERSION 3.20)
project(arm_control_tests LANGUAGES CXX)

enable_testing()

set(ARM_SOURCES
  ../../../lib/arm/kinematics.cpp
  ../../../lib/arm/motion.cpp
  ../../../lib/arm/transmission.cpp
  ../../../lib/arm/workspace.cpp
)
set(CONTROLLER_SOURCES
  ${ARM_SOURCES}
  ../../../lib/arm_control/controller.cpp
)

function(add_arm_test name)
  add_executable(${name} ${name}.cpp ${ARGN})
  target_include_directories(${name} PRIVATE . ../../../include)
  target_compile_features(${name} PRIVATE cxx_std_17)
  target_compile_options(${name} PRIVATE
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
    -Wdouble-promotion -Werror -fno-exceptions -fno-rtti)
  add_test(NAME ${name} COMMAND ${name})
endfunction()

add_arm_test(test_arm_kinematics ../../../lib/arm/kinematics.cpp)
add_arm_test(test_arm_transmission ../../../lib/arm/transmission.cpp)
add_arm_test(test_arm_workspace ../../../lib/arm/workspace.cpp)
add_arm_test(test_arm_motion ../../../lib/arm/motion.cpp)
add_arm_test(test_arm_continuity ${ARM_SOURCES})
add_arm_test(test_arm_controller ${CONTROLLER_SOURCES})
add_arm_test(test_arm_profile ../../../lib/arm/kinematics.cpp ../../../lib/arm/transmission.cpp)
add_arm_test(test_arm_site ../../../lib/arm/kinematics.cpp ../../../lib/arm/transmission.cpp ../../../lib/arm/workspace.cpp)
add_arm_test(test_arm_enable ${CONTROLLER_SOURCES} ../../../lib/arm_control/enable.cpp)
