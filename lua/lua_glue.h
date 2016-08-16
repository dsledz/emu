/*
 * Copyright (c) 2016, Dan Sledz
 */
#pragma once

#include "core/exception.h"

extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

namespace Lua {

class LuaException: public Core::CoreException {
public:
    LuaException(const std::string &error):
        CoreException("Lua error: ")
    {
        std::stringstream ss;
        ss << error;
        msg += ss.str();
    }
};

typedef std::function<int(lua_State *) > lua_call_t;

class LuaSupport
{
public:
    LuaSupport();
    ~LuaSupport();

    LuaSupport(const LuaSupport &ls) = delete;

    inline operator lua_State*() {
        return m_L;
    }

    static int callback(lua_State *L) {
        void * var = lua_touserdata(L, lua_upvalueindex(1));
        auto call = static_cast<lua_call_t *>(var);
        return (*call)(L);
    }

    void add_callback(const std::string &name, lua_call_t *call) {
        lua_pushlightuserdata(m_L, call);
        lua_pushcclosure(m_L, LuaSupport::callback, 1);
        lua_setglobal(m_L, name.c_str());
    }

    void add(const std::string &name, lua_CFunction func) {
        lua_register(m_L, name.c_str(), func);
    }

    void remove(const std::string &name) {
        return;
        lua_pushnil(m_L);
        lua_setglobal(m_L, name.c_str());
    }

    int call(const std::string &code) {
        int status = luaL_loadstring(m_L, code.c_str());
        if (status)
            throw LuaException(lua_tostring(m_L, -1));

        status = lua_pcall(m_L, 0, LUA_MULTRET, 0);
        if (status)
            throw LuaException(lua_tostring(m_L, -1));

        return lua_gettop(m_L);
    }

private:

    void load_libraries(void);

private:

    lua_State *m_L;
};

struct LuaFunction {
    LuaFunction(LuaSupport &lua, const std::string &name, lua_call_t *call):
        m_lua(lua), m_name(name) {
        lua.add_callback(name, call);
    }
    ~LuaFunction() {
        m_lua.remove(m_name);
    }

    LuaSupport &m_lua;
    const std::string &m_name;
};

class LuaLibrary {
public:
    typedef std::pair<std::string, lua_call_t> method_t;
    typedef std::auto_ptr<method_t> method_ptr_t;

    LuaLibrary(LuaSupport &lua, const std::string &name):
        m_lua(lua),
        m_members(),
        m_reg(lua, name, LuaLibrary::open_lib)
    {
        m_members.push_back((struct luaL_reg){ NULL, NULL});
    }
    ~LuaLibrary() {
    }

    void add_method(const std::string &name, lua_call_t &call) {
        m_methods.push_back(new method_ptr_t(std::make_pair(name, call)))
    }

    int library_open(lua_State *L) {
        lua_newtable(L);
        for (auto it = m_methods.begin();
             it != m_methods.end(); it++) {
            lua_pushlightuserdata(L, it->second);
            lua_pushcclosure(L, LuaLibrary::library_call, 1);
            lua_setfield(L, -2, it->first.c_str());
        }
        return 1;
    }

    static int open_lib(lua_State *L) {
        void *ctx = lua_touserdata(L, lua_upvalueindex(1));
        auto self = static_cast<LuaLibrary *>(ctx);
        return self->library_open(L);
    }

    static int library_call(lua_State *L) {
        void *ctx = lua_touserdata(L, lua_upvalueindex(1));
        auto call = static_cast<lua_call_t *>(ctx);
        return (*call)(L);
    }

private:
    LuaSupport m_lua;
    LuaFunction m_reg;
    std::list<method_ptr_t> m_methods;
}

};
