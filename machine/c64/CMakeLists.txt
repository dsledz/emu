add_library(c64
    c64.cc
    m6510.cc
    vic2.cc)

target_link_libraries(c64
    emu
    cpu)


