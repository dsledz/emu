/*
 * Copyright (c) 2013, Dan Sledz
 * All rights reserved.
 */

#include "gtest/gtest.h"

#include "emu/emu.h"
#include "emu/test.h"

#include "machine/sbc/sbc.h"

using namespace EMU;
using namespace EMUTest;

TEST(SBCTest, run100) {
  Arcade::SingleBoardZ80 machine("");
  EmuTime runtime = sec(10);

  Core::log.set_level(LogLevel::Info);
  machine.poweron();

  machine.m_acia->debug_write('\r');
  while (machine.now() < runtime) {
    machine.run_forward(msec(1));
    auto b = machine.m_acia->debug_read();
    if (b.first) {
      std::cout << (char)b.second;
      std::cout.flush();
    }
  }

  machine.poweroff();
}
