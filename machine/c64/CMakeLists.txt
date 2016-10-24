add_library(c64
    c64.cc
    c64_cia.cc
    c64_keyboard.cc
    m6510.cc
    vic2.cc)

target_link_libraries(c64
    emu
    cpu)


