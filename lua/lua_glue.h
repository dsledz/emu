/*
 * Copyright (c) 2016, Dan Sledz
 */
#pragma once

extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

namespace Lua {

class LuaSupport
{
public:
    LuaSupport();
    ~LuaSupport();

    LuaSupport(const LuaSupport &ls) = delete;

    inline operator lua_State*() {
        return m_L;
    }

private:

    void load_libraries(void);

private:

    lua_State *m_L;
};

};
