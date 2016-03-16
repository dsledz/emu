add_library(lua_support
    lua_support.cc)

link_directories("/opt/local/lib")

target_link_libraries(lua_support
    lua)

if (BUILD_TESTING)
    add_executable(lua-check
        lua_test.cc)
    target_link_libraries(lua-check
        lua_support
        gtest gtest_main)
    add_test(lua-check lua-check)
endif (BUILD_TESTING)
