zephyr_library()
zephyr_library_sources(kinematics.cpp motion.cpp transmission.cpp workspace.cpp)
zephyr_library_compile_features(cxx_std_17)
