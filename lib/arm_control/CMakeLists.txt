zephyr_library()
zephyr_library_sources(controller.cpp enable.cpp)
zephyr_library_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)
