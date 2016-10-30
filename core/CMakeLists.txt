
if (BUILD_ASM)
    if (WIN32)
        ENABLE_LANGUAGE(ASM_MASM)
        set(SUPPORT_ASM win64.asm)
    else (WIN32)
        ENABLE_LANGUAGE(ASM-ATT)
        if (UNIX)
            set(SUPPORT_ASM linux_asm.s)
        endif (UNIX)
        if (APPLE)
            set(SUPPORT_ASM osx_asm.s)
        endif (APPLE)
	if (FREEBSD)
            set(SUPPORT_ASM freebsd_asm.s)
        endif (FREEBSD)
    endif (WIN32)
endif (BUILD_ASM)

add_library(core
    debug.cc
    fiber.cc
    rtc.cc
    task.cc
    ${SUPPORT_ASM}
    )

if (BUILD_TESTING)
    add_executable(core-check
        fiber_test.cc
        lock_test.cc
        task_test.cc
        debug_test.cc)
    target_link_libraries(core-check
        core
        gtest gtest_main)
    add_test(core-check core-check)
endif (BUILD_TESTING)
