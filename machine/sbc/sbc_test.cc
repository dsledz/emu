/*
 * Copyright (c) 2013, Dan Sledz
 * All rights reserved.
 */

#include "emu/emu.h"
#include "gtest/gtest.h"

#include "machine/sbc/sbc.h"

using namespace EMU;

TEST(SBCTest, run100) {
  // Core::log.set_level(Core::LogLevel::Trace);

  Arcade::SingleBoardZ80 machine("");

  machine.poweron();

  machine.m_acia->debug_write('\r');

  for (unsigned i = 0;; i++) {
    machine.set_time(Time(usec(i * 10)));
    auto b = machine.m_acia->debug_read();
    if (b.first) {
      std::cout << (char)b.second;
      std::cout.flush();
    } else
      usleep(30);
  }

  machine.poweroff();
}
