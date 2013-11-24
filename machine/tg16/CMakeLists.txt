add_library(tg16
    tg16.cc
    tg16_vdc.cc)
target_link_libraries(tg16
    emu
    cpu)

if (BUILD_TESTING)
    add_executable(tg16-check
        tg16_test.cc)
    target_link_libraries(tg16-check
        tg16
        gtest gtest_main)
    add_test(tg16-check tg16-check)
endif (BUILD_TESTING)
