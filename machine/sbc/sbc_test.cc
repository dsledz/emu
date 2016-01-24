/*
 * Copyright (c) 2013, Dan Sledz
 * All rights reserved.
 */

#include "gtest/gtest.h"
#include "emu/emu.h"

#include "machine/sbc/sbc.h"

using namespace EMU;

TEST(SBCTest, run100)
{
    Core::log.set_level(Core::LogLevel::Trace);

    Arcade::SingleBoardZ80 machine("");

    machine.poweron();

    for (unsigned i = 0;; i++) {
        machine.set_time(Time(usec(i*10)));
        usleep(10);
    }

    machine.poweroff();
}

