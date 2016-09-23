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

#include "cpu/lib/cpu3.h"
#include "emu/test.h"

using namespace EMU;
using namespace EMUTest;
using namespace CPU3;

enum TestOpcode {
  Unknown = -1,
  TestSTA,
  TestLDA,
  TestSTB,
  TestLDB,
  TestADD,
  TestSUB
};

enum Phase {
  OpRead = 0,
  OperandLow,
  OperandHigh,
  Offset,
  SourceResult,
};

struct TestState {
  bool WriteEnabled;
  Phase CurrentPhase;
  /* Internal Registers */
  reg16_t EA;
  reg8_t ARG;
  reg8_t Opcode;

  /* Register File */
  reg16_t PC;
  reg8_t A;
  reg8_t B;
};

static void WriteFunc(AddressBus16x8 *bus, uint16_t address, uint8_t data) {
  WriteFunc(address, data);
}

static uint8_t ReadFunc(AddressBus16x8 *bus, uint16_t address) {
  return ReadFunc(address);
}

static void OpFunc(struct TestState *state, AddressBus16x8 *bus) {
  switch (state->ARG) {
    case TestSTA:
      state->ARG = ReadFunc(bus, state->PC);
      state->EA = state->ARG;
      state->PC++;
      state->ARG = ReadFunc(bus, state->PC);
      state->EA |= state->ARG << 8;
      state->ARG = state->A;
      state->PC++;
      WriteFunc(bus, state->EA, state->ARG);
      state->PC++;
      break;
    case TestLDA:
      state->ARG = ReadFunc(bus, state->PC);
      state->EA = state->ARG;
      state->PC++;
      state->ARG = ReadFunc(bus, state->PC);
      state->EA = state->ARG << 8;
      state->PC++;
      state->A = ReadFunc(bus, state->EA);
      state->PC++;
      break;
  }
}

class TestCpu : public Cpu<AddressBus16, TestState, uint8_t, OpFunc> {
 public:
  TestCpu(Machine *machine, const std::string &name, ClockDivider divider,
          AddressBus16x8 *bus)
      : Cpu(machine, name, hertz, bus) {}
  virtual ~TestCpu(void) {}
};

TEST(CpuTest, constructor) { TestMachine<TestCpu> machine; }
