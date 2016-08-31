/*
 * Copyright (c) 2013, Dan Sledz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "gtest/gtest.h"

#include "lua/lua_glue.h"

TEST(LuaTest, load)
{
    Lua::LuaSupport lua;

}

TEST(LuaTest, custom_function)
{
    Lua::LuaSupport lua;
    std::string script = R"(
-- script.lua
-- Receives a table, returns the sum of its components.
x = 0
for i = 1, #foo do
    x = x + foo[i]
end
return x)";

    int status;
    status = luaL_loadstring(lua, script.c_str());
    EXPECT_EQ(status, 0);

    lua_newtable(lua);
    for (int i = 0; i < 5; i++) {
        lua_pushnumber(lua, i);
        lua_pushnumber(lua, i * 2);
        lua_rawset(lua, -3);
    }
    lua_setglobal(lua, "foo");

    status = lua_pcall(lua, 0, LUA_MULTRET, 0);
    EXPECT_EQ(status, 0);

    int sum = lua_tonumber(lua, -1);
    EXPECT_EQ(sum, 20);
    lua_pop(lua, -1);
}

int test_external_function(lua_State *L)
{
    int n = lua_gettop(L);
    int value = lua_tonumber(L, -1);
    value *= 2 * n;
    lua_pushnumber(L, value);
    return 1;
}

TEST(LuaTest, external_function)
{
    Lua::LuaSupport lua;
    std::string script = R"(
-- Call a function
x = double(5)
return x)";

    lua.add("double", test_external_function);

    lua.call(script);

    int ret = lua_tonumber(lua, -1);
    EXPECT_EQ(ret, 10);
    lua_pop(lua,-1);
}

TEST(LuaText, external_call_object)
{
    Lua::LuaSupport lua;
    std::string script = R"(
-- Call a function
x = double(5)
return x)";
    std::string name = "double";
    Lua::lua_call_t func = test_external_function;
    Lua::LuaCall call(name, &func);

    lua.add(call);
    lua.call(script);

    int ret = lua_tonumber(lua, -1);
    EXPECT_EQ(ret, 10);
    lua_pop(lua,-1);
}

int test_external_function_failure(lua_State *L)
{
    int n = lua_gettop(L);
    if (n != 1) {
        lua_pushstring(L, "Incorrect number of arguments");
        lua_error(L);
    }
    return 0;
}

TEST(LuaTest, external_function_failure)
{
    Lua::LuaSupport lua;
    std::string script = R"(
-- Error check
x = failure(5, 5)
return x)";

    lua.add("failure", test_external_function_failure);
    EXPECT_THROW(lua.call(script), Lua::LuaException);
}

int test_lua_call(lua_State *L)
{
    int n = lua_gettop(L);
    int value = lua_tonumber(L, -1);
    value *= 2 * n;
    lua_pushnumber(L, value);
    return 1;
}

TEST(LuaTest, virtual_func)
{
    Lua::LuaSupport lua;
    Lua::lua_call_t call = test_lua_call;
    Lua::LuaFunction(lua, "func", &call);
    std::string script = R"(
    return func(5))";

    lua.call(script);

    int ret = lua_tonumber(lua, -1);
    EXPECT_EQ(ret, 10);
    lua_pop(lua,-1);
}

TEST(LuaTest, closure_test)
{
    Lua::LuaSupport lua;
    Lua::LuaClosure<int (int)> closure("test", [] (int v) -> int { return v * 5; } );
    lua.add(closure);

    std::string script = R"(
        return test(5)
    )";

    lua.call(script);

    int ret = lua_tonumber(lua, -1);
    EXPECT_EQ(ret, 25);
    lua_pop(lua,-1);
}

TEST(LuaTest, closure_string_return_string)
{
    Lua::LuaSupport lua;
    Lua::LuaClosure<std::string (const std::string &)> closure(
            "test", [] (const std::string &v) -> std::string { return v + "_end" ; } );
    lua.add(closure);

    std::string script = R"(
        x = test("start")
        return x
    )";

    lua.call(script);

    std::string ret = lua_tostring(lua, -1);
    EXPECT_EQ(ret, "start_end");
    lua_pop(lua,-1);
}

TEST(LuaTest, closure_void_return_string)
{
    Lua::LuaSupport lua;
    Lua::LuaClosure<std::string (void)> closure(
            "test", [] (void) -> std::string { return "end" ; } );
    lua.add(closure);

    std::string script = R"(
        x = test()
        return x
    )";

    lua.call(script);

    std::string ret = lua_tostring(lua, -1);
    EXPECT_EQ(ret, "end");
    lua_pop(lua,-1);
}

TEST(LuaTest, closure_exception)
{
    Lua::LuaSupport lua;
    Lua::LuaClosure<std::string (void)> closure(
            "test", [] (void) -> std::string { return "end" ; } );
    lua.add(closure);

    std::string script = R"(
        x = test("start")
        return x
    )";

    EXPECT_THROW(lua.call(script), Lua::LuaException);
}

TEST(LuaTest, library_test)
{
    Lua::LuaSupport lua;
    Lua::LuaClosure<int (int)> inc("inc", [] (int v) -> int { return v + 1; } );
    Lua::LuaClosure<int (int)> dec("dec", [] (int v) -> int { return v - 1; } );
    Lua::LuaLibrary lib(lua, "ops");

    lib.add_call(inc);
    lib.add_call(dec);

    lib.finalize_library();

    std::string script = R"(
        lib = ops()
        x = 10
        x = lib.inc(x)
        x = lib.inc(x)
        x = lib.dec(x)
        return x
    )";

    lua.call(script);

    int ret = lua_tonumber(lua, -1);
    EXPECT_EQ(ret, 11);
    lua_pop(lua,-1);
}
