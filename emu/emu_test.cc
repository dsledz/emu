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

#include "emu/emu.h"
using namespace EMU;

class TestDevice: public Device {
private:
    byte_t _v;
public:
    TestDevice(Machine *machine): Device(machine, "testdev"), _v(0) {
    }
    ~TestDevice(void) {
    }
    virtual void save(SaveState &state) {
    }
    virtual void load(LoadState &state) {
    }
    virtual void tick(unsigned cycles) {
    }
    virtual void line(Line line, LineState state) {
    }
};

TEST(MachineTest, time2)
{
    EXPECT_EQ(50000, Time(usec(50)).ns);
    EXPECT_EQ(1000000, Time(msec(1)).ns);
    EXPECT_EQ(2000000000, Time(sec(2)).ns);
}

TEST(MachineTest, time3)
{
    Cycles hertz(18432000);
    Cycles c = Time(usec(200)).to_cycles(hertz);
    EXPECT_EQ(Cycles(3686), c);
}

TEST(MachineTest, ioport)
{
    Machine machine;
    machine.add_ioport("test1");

    IOPort *port = machine.ioport("test1");
    EXPECT_THROW(machine.ioport("test2"), KeyError);

    EXPECT_EQ(0, machine.read_ioport("test1"));

    port->value = 10;
    EXPECT_EQ(10, machine.read_ioport("test1"));
}

class TestMachine: public Machine {
public:
    TestMachine(void):
        Machine(),
        dev(this)
    {
    }
    ~TestMachine(void) {
    }

private:
    TestDevice dev;
};

TEST(MachineTest, constructor)
{
    TestMachine machine;

}

enum NESKey {
    A = 0,
    B = 1,
    Select = 2,
    Start = 3,
    Up = 4,
    Down = 5,
    Left = 6,
    Right = 7,
    Size = 8,
};

TEST(MachineTest, input_map)
{
    Machine machine;
    machine.add_ioport("PORT1");
    machine.add_ioport("PORT2");

    IOPort *port = machine.ioport("PORT1");
    machine.add_input(InputSignal(InputKey::Joy1Btn1, port, NESKey::A, true));
    machine.add_input(InputSignal(InputKey::Joy1Btn2, port, NESKey::B, true));
    machine.add_input(InputSignal(InputKey::Select1, port, NESKey::Select, true));
    machine.add_input(InputSignal(InputKey::Start1, port, NESKey::Start, true));
    machine.add_input(InputSignal(InputKey::Joy1Up, port, NESKey::Up, true));
    machine.add_input(InputSignal(InputKey::Joy1Down, port, NESKey::Down, true));
    machine.add_input(InputSignal(InputKey::Joy1Left, port, NESKey::Left, true));
    machine.add_input(InputSignal(InputKey::Joy1Right, port, NESKey::Right, true));

    EXPECT_THROW(
        machine.add_input(InputSignal(InputKey::Joy1Btn1, port, NESKey::A, true)),
        DuplicateInput);
}

