find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)
find_package(SFML 2 REQUIRED system window graphics network audio)
include_directories("/Users/dan/local/include")
link_directories("/Users/dan/local/lib")
include_directories(${SFML_INCLUDE_DIR})

add_executable(sfmlEmu
    main.cc)

if (APPLE)
        add_framework(freetype)
endif (APPLE)

target_link_libraries(sfmlEmu
    ${GLEW_LIBRARY}
    ${SFML_LIBRARIES}
    ${OPENGL_LIBRARY}
    Threads::Threads
    driver)

