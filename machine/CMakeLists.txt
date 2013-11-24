add_subdirectory(gb)

add_library(machine
    dkong.cc
    dkonggfx.cc
    galaga.cc
    namco51.cc
    namco06.cc
    pacman.cc
    i8257.cc
    nes.cc
    nes_mapper.cc
    nes_ppu.cc
    tg16.cc
    tg16_vdc.cc)
target_link_libraries(machine
    gb
    emu
    cpu)

if (BUILD_TESTING)
    add_executable(machine-check
        galaga_test.cc
        tg16_test.cc)
    target_link_libraries(machine-check
        machine
        gtest gtest_main)
    add_test(machine-check machine-check)
endif (BUILD_TESTING)
