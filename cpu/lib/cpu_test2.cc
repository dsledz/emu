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

#include "emu/test.h"
#include "emu/clock_bus.h"
#include "emu/debugger.h"
#include "cpu/lib/cpu2.h"

using namespace EMU;
using namespace EMUTest;
using namespace CPU2;

class TestState: public Debuggable {
public:
    TestState(void): Debuggable("testcpu"),
    Phase(CpuPhase::Interrupt),
    PC(0x0000),
    data(0),
    opcode(0),
    A(0),
    B(0),
    INT0(LineState::Clear)
    {
        add_debug_var("A", A);
        add_debug_var("B", B);
        //add_debug_var("PC", PC);
    }

public:
    CpuPhase Phase;
    uint16_t PC;
    uint8_t  data;
    uint8_t  opcode;
    reg8_t   A;
    reg8_t   B;
    LineState INT0;
};

struct TestClass {
    void Interrupt(TestState *state, ClockedBus16 *bus) {
        if (state->INT0 == LineState::Pulse) {
            state->INT0 = LineState::Clear;
            state->PC = 0x0000;
        }
        state->Phase = CpuPhase::Decode;
    }

    void Decode(TestState *state, ClockedBus16 *bus) {
        bus->io_read(state->PC, &state->opcode);
        state->Phase = CpuPhase::Dispatch;
    }

    void Dispatch(TestState *state, ClockedBus16 *bus) {
        switch (state->opcode) {
        case 0:
            state->A++;
            break;
        case 1:
            state->B++;
            break;
        }
    }
};

struct TestOpcode
{
};

class TestCpu: public Cpu<ClockedBus16, TestState, TestOpcode, TestClass> {
public:
    TestCpu(Machine *machine, const std::string &name, unsigned hertz,
            ClockedBus16 *bus):
        Cpu(machine, name, hertz, bus)
    {
    }
    virtual ~TestCpu(void)
    {
    }
};

TEST(CpuTest, constructor)
{
    TestMachine<TestCpu> machine;
}

TEST(CpuTest, debug_print)
{
    TestMachine<TestCpu> machine;
    TestState *state = machine.cpu.state();

    EXPECT_EQ("0", state->read_register("A"));
    state->write_register("A", std::to_string(5));
    EXPECT_EQ("5", state->read_register("A"));
}
