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
/**
 * Common test routines and functions.
 */

#include "gtest/gtest.h"

#include "emu/emu.h"

namespace EMUTest {

using namespace EMU;

const unsigned TEST_CLOCK = 1000000;

template <typename CpuType, unsigned initial_pc = 0x0000>
class TestMachine : public Machine {
 public:
  TestMachine(void)
      : Machine(),
        bus(),
        cpu(this, "maincpu", TEST_CLOCK, &bus),
        ram(this, "ram", 0x10000),
        pc(initial_pc) {
    bus.add(0x0000, &ram);

    set_line("maincpu", Line::RESET, LineState::Pulse);
  }
  ~TestMachine(void) {}

  void write8(byte_t value) { ram.write8(pc++, value); }

  typename CpuType::bus_type bus;
  CpuType cpu;
  RamDevice ram;
  unsigned pc;
};

#define LOAD1(op) machine.write8(op);
#define LOAD2(op, arg) \
  machine.write8(op);  \
  machine.write8(arg);
#define LOAD3(op, arg1, arg2) \
  machine.write8(op);         \
  machine.write8(arg1);       \
  machine.write8(arg2);
#define LOAD4(op, arg1, arg2, arg3) \
  machine.write8(op);               \
  machine.write8(arg1);             \
  machine.write8(arg2);             \
  machine.write8(arg3);
};
