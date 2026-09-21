zephyr_library()
zephyr_library_sources(kinematics.cpp motion.cpp transmission.cpp workspace.cpp)
zephyr_library_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)
