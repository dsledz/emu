add_subdirectory(m6502)
add_subdirectory(z80)

add_library(cpu
    m6502.cc
    n2a03.cc
    m65c02.cc
    hu6280.cc
    m6809.cc)
target_link_libraries(cpu
    m6502
    z80
    emu)

if (BUILD_TESTING)
    add_executable(cpu-check
        m6502_test.cc
        hu6280_test.cc
        m6809_test.cc)
    target_link_libraries(cpu-check
        cpu
        gtest gtest_main)
    add_test(cpu-check cpu-check)
endif (BUILD_TESTING)
