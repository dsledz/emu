
add_library(z80
    z80.cc
    z80v2.cc)

target_link_libraries(z80
    emu)

if (BUILD_TESTING)
    add_executable(z80-check
        z80_test.cc)
    target_link_libraries(z80-check
        z80
        gtest gtest_main)
    add_test(z80-check z80-check)
    add_executable(z80-check2
        z80_test2.cc)
    target_link_libraries(z80-check2
        z80
        gtest gtest_main)
    add_test(z80-check2 z80-check2)
endif (BUILD_TESTING)
