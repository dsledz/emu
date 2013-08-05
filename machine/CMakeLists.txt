add_library(machine
    galaga.cc
    namco51.cc
    namco06.cc
    gb.cc
    gbgfx.cc
    gbmbc.cc
    nes.cc
    nes_mapper.cc
    nes_ppu.cc
    tg16.cc
    tg16_vdc.cc)
target_link_libraries(machine
    emu
    cpu)

if (BUILD_TESTING)
    add_executable(machine-check
        galaga_test.cc
        gb_test.cc
        tg16_test.cc)
    target_link_libraries(machine-check
        machine
        gtest gtest_main)
    add_test(machine-check machine-check)
endif (BUILD_TESTING)
