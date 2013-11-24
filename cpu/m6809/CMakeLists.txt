add_library(m6809
    m6809.cc)

target_link_libraries(m6809
    emu)

if (BUILD_TESTING)
    add_executable(m6809-check
        m6809_test.cc)
    target_link_libraries(m6809-check
        m6809
        gtest gtest_main)
    add_test(m6809-check m6809-check)
endif (BUILD_TESTING)
