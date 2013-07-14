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
    virtual void set_line(InputLine line, LineState state) {
    }
};

TEST(AddressBusTest, single)
{
    AddressBus<16> bus;
    byte_t v = 0;
    bus.add_port(0x100, IOPort(&v));

    EXPECT_EQ(0, bus.read(0x100));
    EXPECT_THROW(bus.read(0x101), BusError);
    EXPECT_THROW(bus.write(0x101, 10), BusError);

    bus.write(0x100, 50);

    EXPECT_EQ(50, v);
    EXPECT_EQ(50, bus.read(0x100));
}

TEST(AddressBusTest, default_read_write)
{
    AddressBus<16> bus;
    bus.add_port(0x100, IOPort());

    EXPECT_EQ(0, bus.read(0x100));
    bus.write(0x100, 10);
    EXPECT_EQ(0, bus.read(0x100));
}

TEST(AddressBusTest, range)
{
    AddressBus<16> bus;
    byte_t v = 0;
    bus.add_port(0x0100, 0xff00, IOPort(&v));

    EXPECT_EQ(0, bus.read(0x0100));
    EXPECT_EQ(0, bus.read(0x0101));
    bus.write(0x0101, 10);
    EXPECT_EQ(10, bus.read(0x0100));

    bus.write(0x0100, 50);
    EXPECT_EQ(50, v);
    EXPECT_EQ(50, bus.read(0x0100));
}

TEST(Radix, simple)
{
    RadixTree<IOPort, 16> tree;
    tree.add(0x100, IOPort());
    struct IOPort &f = tree.find(0x100);
    EXPECT_EQ(0, f.read(0x100));

    tree.add(0x101, IOPort([](addr_t addr) throw() -> byte_t { return 10; },
                          DefaultWrite()));
    struct IOPort &f2 = tree.find(0x101);
    EXPECT_EQ(10, f2.read(0x101));
    EXPECT_EQ(0, tree.find(0x100).read(0x100));
}

TEST(Radix, multiple)
{
    RadixTree<IOPort, 16> tree;

    addr_t a1 = 0x4000;
    addr_t a2 = 0x0800;
    addr_t a3 = 0x00c0;
    addr_t a4 = 0x0001;
    byte_t x4 = 0x4;
    byte_t x8 = 0x8;
    byte_t xC = 0xC;
    byte_t x0 = 0x0;
    tree.add(a1, IOPort(&x4));
    tree.add(a2, IOPort(&x8));
    tree.add(a3, IOPort(&xC));
    tree.add(a4, IOPort(&x0));
    EXPECT_EQ(x4, tree.find(a1).read(a1));
    EXPECT_EQ(x8, tree.find(a2).read(a2));
    EXPECT_EQ(xC, tree.find(a3).read(a3));
    EXPECT_EQ(x0, tree.find(a4).read(a4));
}

TEST(Radix, range)
{
    RadixTree<IOPort, 16> tree;

    byte_t v = 0;
    tree.add(0x4000, 0xff00, IOPort(&v));

    EXPECT_EQ(0, tree.find(0x4000).read(0x4000));

    tree.find(0x4000).write(0x4000, 10);
    EXPECT_EQ(10, tree.find(0x4001).read(0x4001));

    byte_t v1 = 0;
    tree.add(0x1000, 0xff00, IOPort(&v1));

    EXPECT_EQ(0, tree.find(0x1000).read(0x1000));

    byte_t v2 = 0;
    tree.add(0x8000, 0xff00, IOPort(&v2));
    tree.find(0x8000).write(0x8000, 20);
    EXPECT_EQ(20, tree.find(0x8001).read(0x8001));

    EXPECT_THROW(tree.add(0x4001, IOPort()), BusError);
    EXPECT_EQ(10, tree.find(0x4001).read(0x4001));
}

TEST(Radix, range2)
{
#define ADD_PORT(addr) \
    byte_t v ## addr = addr >> 12; \
    tree.add(addr, 0xF000, IOPort(&v##addr));

    RadixTree<IOPort, 16> tree;
    ADD_PORT(0x1000);
    ADD_PORT(0x0000);
    ADD_PORT(0x8000);
    ADD_PORT(0x2000);
    ADD_PORT(0xB000);
    ADD_PORT(0x3000);
    ADD_PORT(0x5000);
    ADD_PORT(0x6000);
    ADD_PORT(0x4000);
    ADD_PORT(0xF000);
    ADD_PORT(0x9000);
    ADD_PORT(0xA000);
    ADD_PORT(0xC000);
    ADD_PORT(0xD000);
    ADD_PORT(0x7000);
    ADD_PORT(0xE000);
#undef ADD_PORT

    for (unsigned i = 0; i < 16; i++) {
        addr_t addr = i << 12;
        EXPECT_EQ(i, tree.find(addr).read(addr));
    }
}

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
