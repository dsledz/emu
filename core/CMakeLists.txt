ENABLE_LANGUAGE(ASM-ATT)

add_library(core
    debug.cc
    fiber.cc
    task.cc
    x64_asm.s)

if (BUILD_TESTING)
    add_executable(core-check
        fiber_test.cc
        task_test.cc
        debug_test.cc)
    target_link_libraries(core-check
        core
        gtest gtest_main)
    add_test(core-check core-check)
endif (BUILD_TESTING)
