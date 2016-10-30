find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)
set(SFML_STATIC_LIBRARIES TRUE)
find_package(SFML 2 REQUIRED window system)
include_directories("/Users/dan/local/include")
link_directories("/Users/dan/local/lib")
include_directories(${SFML_INCLUDE_DIR})

add_executable(sfmlEmu
    main.cc)

if (APPLE)
    add_framework(freetype)
else (APPLE)
    if (UNIX)
        set(SFML_WINDOW_DEPENDENCIES ${SFML_WINDOW_DEPENDENCIES} "Xrandr")
    endif (UNIX)
endif (APPLE)

target_link_libraries(sfmlEmu
    ${GLEW_LIBRARY}
    ${SFML_LIBRARIES}
    ${SFML_WINDOW_DEPENDENCIES}
    ${OPENGL_LIBRARY}
    Threads::Threads
    driver)

