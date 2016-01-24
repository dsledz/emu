add_subdirectory(gb)
add_subdirectory(nes)
add_subdirectory(tg16)
add_subdirectory(sbc)

add_library(machine
    dkong.cc
    dkonggfx.cc
    galaga.cc
    galagagfx.cc
    namco51.cc
    namco06.cc
    pacman.cc
    pacmangfx.cc
    i8257.cc)
target_link_libraries(machine
    gb
    nes
    tg16
    sbc
    emu
    cpu)

if (BUILD_TESTING)
    add_executable(machine-check
        galaga_test.cc)
    target_link_libraries(machine-check
        machine
        gtest gtest_main)
    add_test(machine-check machine-check)
endif (BUILD_TESTING)
