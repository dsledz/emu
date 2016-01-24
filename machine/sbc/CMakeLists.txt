add_library(sbc
    m6850.cc
    sbc.cc)

target_link_libraries(sbc
    emu
    cpu)

if (BUILD_TESTING)
    add_executable(sbc-check
        sbc_test.cc)
    target_link_libraries(sbc-check
        machine
        gtest
        gtest_main)
    add_test(sbc-check sbc-check)
endif (BUILD_TESTING)
