add_library(gb
    gb.cc
    gbgfx.cc
    gbmbc.cc
    lr35902.cc)

target_link_libraries(gb
    emu
    cpu)

if (BUILD_TESTING)
    add_executable(gb-check
        gb_test.cc)
    target_link_libraries(gb-check
        machine
        gtest gtest_main)
    add_test(gb-check gb-check)
endif (BUILD_TESTING)

