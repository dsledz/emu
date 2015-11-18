
file(GLOB headers *.h)
file(GLOB sources *.m)
file(GLOB resources *.xib)

set(NAME iEmu)
add_executable(${NAME}
    ${APP_TYPE}
    ${headers}
    ${sources}
    ${resources}
    )

set(CMAKE_OSX_SYSROOT iphoneos4.3)

set_target_properties(${NAME}
    PROPERTIES RESOURCE ${resources})

set_target_properties(${NAME} PROPERTIES
    XCODE_ATTRIBUTE_IPHONEOS_DEPLOYMENT_TARGET 3.0)

set_target_properties(${NAME}
    PROPERTIES XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY
    "iPhone Developer: Dan Sledz")

# Force to link with clang++
set_source_files_properties(
    ${sources}
    PROPERTIES LANGUAGE C++
    )

target_link_libraries(${NAME}
    driver
    )

add_framework(CoreGraphics)
add_framework(Foundation)
add_framework(UIKit)
add_framework(GLKit)
add_framework(OpenGLES)
