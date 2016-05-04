add_library(nes
    nes.cc
    nes_mapper.cc
    nes_ppu.cc)

target_link_libraries(nes
    emu
    cpu)

if (BUILD_TESTING)
    add_executable(nes-check
        nes_test.cc)
    target_link_libraries(nes-check
        nes
        gtest gtest_main)
    add_test(nes-check nes-check)
endif (BUILD_TESTING)
