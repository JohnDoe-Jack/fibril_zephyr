# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(wrist.cpp)
zephyr_library_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)
