/*
 * Copyright (c) 2013, Dan Sledz
 * All rights reserved.
 *
 */

#include "core/bits.h"
#include "core/exception.h"
#include "emu/clock.h"

using namespace EMU;
using namespace Core;

Crystal::Crystal(void):
    m_clocks(),
    m_task(new FiberTask(std::bind(&Crystal::task_fn, *this), "crystal"))
{
}

Crystal::~Crystal(void)
{
}

Crystal::task_fn(void)
{

}

