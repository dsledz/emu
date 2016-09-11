/*
 * Copyright (c) 2016, Dan Sledz
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

#include "emu/emu.h"
using namespace EMU;

class DebuggableDevice : public Debuggable {
 public:
  DebuggableDevice() : Debuggable("test") {
    add_debug_var("A", regA);
    add_debug_var("B", regB);
  }
  ~DebuggableDevice() {}

 public:
  uint8_t regA;
  uint8_t regB;
};

TEST(DebuggerTest, test) {
  DebuggableDevice device;

  device.regA = 60;
  device.regB = 10;

  EXPECT_EQ("60", device.read_register("A"));

  debug_vars_t variables = device.read_registers();
  auto first = variables.front();
  EXPECT_EQ(std::make_pair(std::string("A"), std::string("60")), first);
}

class TestMachine : public Machine {
 public:
  TestMachine(void) : Machine(), ram(this, "ram", 0x10000) {}
  ~TestMachine(void) {}

 private:
  RamDevice ram;
};

TEST(DebuggerTest, test_machine) {
  Debugger debugger;
  TestMachine machine;

  machine.set_debugger(&debugger);

  Debugger *d = machine.get_debugger();

  Debuggable *ram = d->get_debuggable("ram");

  EXPECT_EQ(ram->read_register("foo"), "Unknown");
}
