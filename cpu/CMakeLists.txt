add_subdirectory(lib)
add_subdirectory(m6502)
add_subdirectory(z80)

add_library(cpu
    cpu.cc)
target_link_libraries(cpu
    m6502
    z80
    emu)

