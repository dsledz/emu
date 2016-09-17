add_library(emu
    clock.cc
    device.cc
    emu.cc
    emulator.cc
    gfx.cc
    timer_queue.cc
    input.cc
    machine.cc
    ram.cc
    rom.cc
    screen.cc
    video.cc
    dipswitch.cc)

target_link_libraries(emu
    core)

if (BUILD_TESTING)
    add_executable(emu-check
        debugger_test.cc
        dipswitch_test.cc
        emu_test.cc
        io_test.cc
        page_test.cc
        time_test.cc
        timer_queue_test.cc)
    target_link_libraries(emu-check
        emu
        gtest gtest_main)
    add_test(emu-check emu-check)
endif (BUILD_TESTING)
