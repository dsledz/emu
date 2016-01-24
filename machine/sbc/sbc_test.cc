/*
 * Copyright (c) 2013, Dan Sledz
 * All rights reserved.
 */

#include "gtest/gtest.h"
#include "emu/emu.h"

#include "machine/sbc/sbc.h"

using namespace EMU;

TEST(SBCTest, load)
{
    Arcade::SingleBoardZ80 machine("");
}

TEST(SBCTest, run100)
{

                    Core::log.set_level(Core::LogLevel::Trace);
    Arcade::SingleBoardZ80 machine("");

    unsigned steps = 5000;
    for (unsigned i = 0; i < steps; i++) {
        machine.run();
    }
}

