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

#include "machine/gb/lr35902.h"

using namespace GBMachine;

#define LOAD(code) ram[pc++] = code;
#define LOAD1(code, arg1) \
  ram[pc++] = code;       \
  ram[pc++] = arg1;
#define LOAD2(code, arg1, arg2) \
  ram[pc++] = code;             \
  ram[pc++] = arg1;             \
  ram[pc++] = arg2;

class LR35902CpuTest : public LR35902Cpu
{
public:
 LR35902CpuTest(Machine *machine, const std::string &name, unsigned hertz,
                AddressBus16x8 *bus):
     LR35902Cpu(machine, name, hertz, bus) { }
 virtual ~LR35902CpuTest() { }

 void step_once(void) {
   dispatch();
 }
};

class LR35902Test : public ::testing::Test {
 public:
  LR35902Test(void)
      : machine(),
        bus(),
        cpu(&machine, "test", 1000000, &bus),
        ram(0xE000),
        pc(0x100) {
    Core::log.set_level(LogLevel::Trace);
    bus.add(0x0000, ram);
  }

  Machine machine;
  AddressBus16x8 bus;
  LR35902CpuTest cpu;
  bvec ram;
  addr_t pc;
};

TEST_F(LR35902Test, Constructor) {}

TEST_F(LR35902Test, opcode_0x00) {
  LOAD(0x00);
  cpu.step_once();
}

TEST_F(LR35902Test, opcode_0x01) {
  LOAD2(0x1, 0x12, 0x34);
  cpu.step_once();

  EXPECT_EQ(0x34, cpu.fetch(Register::B));
  EXPECT_EQ(0x12, cpu.fetch(Register::C));
}
