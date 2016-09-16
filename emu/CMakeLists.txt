add_library(emu
    clock.cc
    device.cc
    emu.cc
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
        emu_test.cc
        time_test.cc
        timer_queue_test.cc
        dipswitch_test.cc
        io_test.cc)
    target_link_libraries(emu-check
        emu
        gtest gtest_main)
    add_test(emu-check emu-check)
    add_executable(mem-check
        page_test.cc)
    target_link_libraries(mem-check
        emu
        gtest gtest_main)
    add_test(mem-check mem-check)
    add_executable(debug-check
        debugger_test.cc)
    target_link_libraries(debug-check
        emu
        gtest gtest_main)
    add_test(debug-check debug-check)
endif (BUILD_TESTING)
