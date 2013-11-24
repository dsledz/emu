add_library(nes
    nes.cc
    nes_mapper.cc
    nes_ppu.cc)

target_link_libraries(nes
    emu
    cpu)

