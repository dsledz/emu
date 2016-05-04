
if (BUILD_ASM)
    ENABLE_LANGUAGE(ASM-ATT)

    if (APPLE)
        set(SUPPORT_ASM osx_asm.s)
    else (APPLE)
        set(SUPPORT_ASM freebsd_asm.s)
    endif (APPLE)
endif (BUILD_ASM)

add_library(core
    debug.cc
    fiber.cc
    task.cc
    ${SUPPORT_ASM}
    )

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
