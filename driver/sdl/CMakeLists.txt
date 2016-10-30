include(FindSDL2)
find_package(Threads REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIR})

include_directories(${GLEW_INCLUDE_DIR})

if (APPLE)
    add_framework(SDL2)
    add_framework(OpenGL)
endif (APPLE)

if (WIN32)
ELSE(WIN32)
    set(GLUT_LIBRARY glut)
endif (WIN32)

add_executable(sdlEmu
    main.cc)

target_link_libraries(sdlEmu
    ${GLEW_LIBRARY}
    ${GLUT_LIBRARY}
    ${OPENGL_LIBRARY}
    ${SDL2_LIBRARY}
    Threads::Threads
    driver)

