#include "core/bits.h"
#include "lua/lua_glue.h"

using namespace Lua;

LuaSupport::LuaSupport()
{
    m_L = luaL_newstate();
    assert(m_L != NULL);
    load_libraries();
}

LuaSupport::~LuaSupport()
{
    lua_close(m_L);
}

void
LuaSupport::load_libraries(void)
{
    luaopen_io(m_L);
    luaopen_base(m_L);
    luaopen_table(m_L);
    luaopen_string(m_L);
    luaopen_math(m_L);
}

