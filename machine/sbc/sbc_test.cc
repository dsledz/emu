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

TEST(SBCTest, test) {
  machine_test<Arcade::SingleBoardZ80>("");
}

TEST(SBCTest, debug) {
  Arcade::SingleBoardZ80 machine;
  EmuTime runtime = sec(10);

  Core::log.set_level(LogLevel::Info);
  machine.poweron();
  machine.run_forward(msec(1));

  machine.m_acia->debug_write('\r');
  while (machine.now() < runtime) {
    auto b = machine.m_acia->debug_read();
    if (b.first) {
      std::cout << (char)b.second;
      std::cout.flush();
    }
    machine.run_forward(msec(1));
  }

  machine.poweroff();
}
