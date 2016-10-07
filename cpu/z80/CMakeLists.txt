
add_library(z80
    z80.cc)

target_link_libraries(z80
    emu)

if (BUILD_TESTING)
    add_executable(z80-check
        z80_test.cc)
    target_link_libraries(z80-check
        z80
        gtest gtest_main)
    add_test(z80-check z80-check)
endif (BUILD_TESTING)
