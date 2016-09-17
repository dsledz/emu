find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)

add_library(driver
    driver.cc
    wrapper.cc
    opengl.cc)

target_link_libraries(driver
    emu
    cpu
    machine)

if (BUILD_TESTING)
    add_executable(emulator-check
        emulator_test.cc)
    target_link_libraries(emulator-check
        driver
        gtest gtest_main)
    add_test(emulator-check emulator-check)
endif (BUILD_TESTING)

include_directories(${OPENGL_INCLUDE_DIR})

    message("Building OSX support.")
if (BUILD_OSX)
    add_subdirectory(osx)
endif (BUILD_OSX)

if (BUILD_IOS)
    message("Building iOS support.")
    add_subdirectory(ios)
endif (BUILD_IOS)

if (BUILD_SDL)
    message("Building SDL support.")
    add_subdirectory(sdl)
endif (BUILD_SDL)

if (BUILD_SFML)
    message("Building SFML support.")
    add_subdirectory(sfml)
endif (BUILD_SFML)
