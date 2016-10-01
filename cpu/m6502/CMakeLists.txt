add_library(m6502
    m6502.cc
    m65c02.cc
    n2a03.cc
    HuC6280.cc)
target_link_libraries(m6502
    cpu-lib)

if (BUILD_TESTING)
    add_executable(m6502-check
        jit_test.cc)
    target_link_libraries(m6502-check
        m6502
        gtest gtest_main)
    add_test(m6502-check m6502-check)
endif (BUILD_TESTING)
