find_package(OpenGL REQUIRED)
add_library(driver
    wrapper.cc
    emulator.cc
    opengl.cc)
target_link_libraries(driver
    emu
    cpu
    machine)

if (BUILD_OSX)
    message("Building OSX support.")
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