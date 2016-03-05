find_package(OpenGL REQUIRED)
include_directories("/Users/dan/local/include")
link_directories("/Users/dan/local/lib")

add_executable(sfmlEmu
    main.cc)

add_framework(freetype)

target_link_libraries(sfmlEmu
    sfml-graphics
    sfml-window
    sfml-system
    ${OPENGL_LIBRARY}
    driver)

