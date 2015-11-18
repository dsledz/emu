
if (BUILD_JIT)
    ENABLE_LANGUAGE(ASM-ATT)
    set(JIT_HELPER x86_asm.s)
endif (BUILD_JIT)

add_library(core
    debug.cc
    fiber.cc
    task.cc
    ${JIT_HELPER})

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
