add_library(arcade
    dkong.cc
    dkonggfx.cc
    galaga.cc
    galagagfx.cc
    namco51.cc
    namco06.cc
    pacman.cc
    pacmangfx.cc
    i8257.cc)
target_link_libraries(arcade
    emu
    cpu)

if (BUILD_TESTING)
    add_executable(arcade-check
        dkong_test.cc
        galaga_test.cc
        pacman_test.cc)
    target_link_libraries(arcade-check
        arcade
        gtest gtest_main)
    add_test(arcade-check arcade-check)
endif (BUILD_TESTING)
