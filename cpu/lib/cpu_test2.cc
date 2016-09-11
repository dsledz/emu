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

#include "cpu/lib/cpu2.h"
#include "emu/clock_bus.h"
#include "emu/debugger.h"
#include "emu/test.h"

using namespace EMU;
using namespace EMUTest;
using namespace CPU2;

class TestState : public Debuggable {
 public:
  TestState(void)
      : Debuggable("testcpu"),
        Phase(CpuPhase::Interrupt),
        PC(0x0000),
        data(0),
        opcode(0),
        A(0),
        B(0),
        INT0(LineState::Clear) {
    add_debug_var("A", A);
    add_debug_var("B", B);
    // add_debug_var("PC", PC);
  }

  virtual void reset(void) {}

 public:
  CpuPhase Phase;
  uint16_t PC;
  uint8_t data;
  uint8_t opcode;
  reg8_t A;
  reg8_t B;
  LineState INT0;
  ClockedBus16 *bus;
};

struct TestOpcode {};

class TestCpu : public Cpu<ClockedBus16, TestState, TestOpcode> {
 public:
  TestCpu(Machine *machine, const std::string &name, unsigned hertz,
          TestState *state)
      : Cpu(machine, name, hertz, state) {}
  ~TestCpu() {}

  virtual void execute(void) {
    while (true) {
      switch (m_state->Phase) {
        case CpuPhase::Interrupt: {
          if (m_state->INT0 == LineState::Pulse) {
            m_state->INT0 = LineState::Clear;
            m_state->PC = 0x0000;
          }
          m_state->Phase = CpuPhase::Decode;
        }
        case CpuPhase::Decode: {
          m_state->bus->io_read(m_state->PC, &m_state->opcode);
          m_state->Phase = CpuPhase::Dispatch;
        }
        case CpuPhase::Dispatch: {
          switch (m_state->opcode) {
            case 0:
              m_state->A++;
              break;
            case 1:
              m_state->B++;
              break;
          }
        }
      }
    }
  }
};

template <typename CpuType, unsigned initial_pc = 0x0000>
class TestMachine2 : public Machine {
 public:
  TestMachine2(void)
      : Machine(),
        bus(),
        state(),
        cpu(this, "maincpu", 1000000, &state),
        ram(this, "ram", 0x10000),
        pc(initial_pc) {
    state.bus = &bus;
    bus.add(0x0000, &ram);

    set_line("maincpu", Line::RESET, LineState::Pulse);
  }
  ~TestMachine2(void) {}

  void write8(byte_t value) { ram.write8(pc++, value); }

  typename CpuType::bus_type bus;
  typename CpuType::state_type state;
  CpuType cpu;
  RamDevice ram;
  unsigned pc;
};

TEST(CpuTest, constructor) { TestMachine2<TestCpu> machine; }

TEST(CpuTest, debug_print) {
  TestMachine2<TestCpu> machine;
  TestState *state = machine.cpu.state();

  EXPECT_EQ("0", state->read_register("A"));
  state->write_register("A", std::to_string(5));
  EXPECT_EQ("5", state->read_register("A"));
}
