add_library(cpu-lib
    cpu.cc
    jit.cc)
target_link_libraries(cpu-lib
    emu)

if (BUILD_TESTING)
    add_executable(cpu-lib-check
        cpu_test.cc)
    target_link_libraries(cpu-lib-check
        cpu
        gtest gtest_main)
    add_test(cpu-lib-check cpu-lib-check)
endif (BUILD_TESTING)
