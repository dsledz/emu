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

#include "emu.h"
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

TEST(MachineTest, time1)
{
    Machine machine;
    TestDevice dev(&machine);
    machine.add_device(&dev);
    bool called = false;
    Time period = machine.quantum();
    Timer_ptr short_timer = machine.add_timer(
        period - Time(msec(2)),
        [&](){ called = !called; },
        period
        );
    bool delayed = false;
    Timer_ptr long_timer = machine.add_timer(
        Time(msec(500)),
        [&](){ delayed = true; },
        time_zero
        );
    EXPECT_EQ(false, called);
    EXPECT_EQ(false, delayed);
    machine.run();
    EXPECT_EQ(true, called);
    EXPECT_EQ(false, delayed);
    machine.run();
    EXPECT_EQ(false, called);
    EXPECT_EQ(false, delayed);
    machine.remove_timer(short_timer);
    machine.run();
    EXPECT_EQ(false, called);
    EXPECT_EQ(false, delayed);
}

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
