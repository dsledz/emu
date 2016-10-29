add_library(lua53
    lua53/lapi.c
    lua53/lcode.c
    lua53/lctype.c
    lua53/ldebug.c
    lua53/ldo.c
    lua53/ldump.c
    lua53/lfunc.c
    lua53/lgc.c
    lua53/llex.c
    lua53/lmem.c
    lua53/lobject.c
    lua53/lopcodes.c
    lua53/lparser.c
    lua53/lstate.c
    lua53/lstring.c
    lua53/ltable.c
    lua53/ltm.c
    lua53/lundump.c
    lua53/lvm.c
    lua53/lzio.c
    lua53/lauxlib.c
    lua53/lbaselib.c
    lua53/lbitlib.c
    lua53/lcorolib.c
    lua53/ldblib.c
    lua53/liolib.c
    lua53/lmathlib.c
    lua53/loslib.c
    lua53/lstrlib.c
    lua53/ltablib.c
    lua53/lutf8lib.c
    lua53/loadlib.c
    lua53/linit.c)

add_library(lua_support
    lua_support.cc)

include_directories(lua53)

target_link_libraries(lua_support
    lua53)

if (BUILD_TESTING)
    add_executable(lua-check
        lua_test.cc)
    target_link_libraries(lua-check
        lua_support
        gtest gtest_main)
    add_test(lua-check lua-check)
endif (BUILD_TESTING)
