add_library(emu
    emu.cc
    task.cc
    input.cc
    machine.cc
    ram.cc
    rom.cc
    video.cc
    dipswitch.cc)

if (BUILD_TESTING)
    add_executable(emu-check
        emu_test.cc
        time_test.cc
        debug_test.cc
        dipswitch_test.cc
        io_test.cc)
    target_link_libraries(emu-check
        emu
        gtest gtest_main)
    add_test(emu-check emu-check)
endif (BUILD_TESTING)
