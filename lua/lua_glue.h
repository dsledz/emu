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

class LuaCall {
public:
    LuaCall(const std::string &name, lua_call_t *call):
        m_name(name),
        m_call(call) {
    }
    ~LuaCall() {
    }
    const std::string & name() { return m_name; }
    lua_call_t *call() { return m_call; }

private:
    const std::string &m_name;
    lua_call_t *m_call;
};

static inline int
lua_specialized_call(lua_State *L, std::function<int (int)> closure)
{
    if (lua_gettop(L) != 1) {
        lua_pushstring(L, "Incorrect arguments, expected (int)");
        lua_error(L);
        return 0;
    }

    int arg1 = lua_tonumber(L, -1);
    int result;
    result = closure(arg1);
    lua_pushnumber(L, result);
    return 1;
}

static inline int
lua_specialized_call(lua_State *L, std::function<std::string (const std::string &)> closure)
{
    if (lua_gettop(L) != 1) {
        lua_pushstring(L, "Incorrect arguments, expected (string)");
        lua_error(L);
        return 0;
    }

    const char *arg1 = lua_tostring(L, -1);
    std::string result = closure(arg1);
    lua_pushstring(L, result.c_str());
    return 1;
}

static inline int
lua_specialized_call(lua_State *L, std::function<void (int)> closure)
{
    if (lua_gettop(L) != 1) {
        lua_pushstring(L, "Incorrect arguments, expected (int)");
        lua_error(L);
        return 0;
    }

    int arg1 = lua_tonumber(L, -1);
    closure(arg1);
    return 1;
}

static inline int
lua_specialized_call(lua_State *L, std::function<std::string (void)> closure)
{
    if (lua_gettop(L) != 0) {
        lua_pushstring(L, "Incorrect arguments, expected ()");
        lua_error(L);
        return 0;
    }

    std::string result = closure();
    lua_pushstring(L, result.c_str());
    return 1;
}

template<typename closure_t>
class LuaClosure {
public:
    LuaClosure(const std::string &name, closure_t closure):
        m_func([=] (lua_State *L) -> int {
            return lua_specialized_call(L, closure);
        }),
        m_call(name, &m_func) {
    }

    inline operator LuaCall&() {
        return m_call;
    }

private:
    lua_call_t m_func;
    LuaCall m_call;
};

static inline int lua_callback_wrapper(lua_State *L)
{
    void * var = lua_touserdata(L, lua_upvalueindex(1));
    auto call = static_cast<lua_call_t *>(var);
    return (*call)(L);
}

class LuaSupport
{
public:
    LuaSupport();
    ~LuaSupport();

    LuaSupport(const LuaSupport &ls) = delete;

    inline operator lua_State*() {
        return m_L;
    }

    void add_callback(const std::string &name, lua_call_t *call) {
        lua_pushlightuserdata(m_L, call);
        lua_pushcclosure(m_L, lua_callback_wrapper, 1);
        lua_setglobal(m_L, name.c_str());
    }

    void add(LuaCall &call) {
        add_callback(call.name(), call.call());
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
    LuaLibrary(LuaSupport &lua, const std::string &name):
        m_lua(lua),
        m_name(name),
        m_methods(),
        m_library_open([=] (lua_State *L) -> int {
            return library_open(L); })
    {
    }
    ~LuaLibrary() {
    }

    void add_call(LuaCall &call) {
        m_methods.push_back(call);
    }

    void finalize_library() {
        m_lua.add_callback(m_name, &m_library_open);
    }

private:

    int library_open(lua_State *L) {
        lua_newtable(L);
        for (auto it = m_methods.begin();
             it != m_methods.end(); it++) {
            auto call = *it;
            lua_pushlightuserdata(L, call.call());
            lua_pushcclosure(L, lua_callback_wrapper, 1);
            lua_setfield(L, -2, call.name().c_str());
        }
        return 1;
    }

    LuaSupport &m_lua;
    const std::string & m_name;
    std::list<LuaCall> m_methods;
    lua_call_t m_library_open;
    friend int lua_library_open(lua_State *L);
};

};
