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
#if 0
TEST(Radix, simple)
{
    RadixTree<IOPort, addr_t, 16> tree;
    tree.add(0x100, IOPort());
    struct IOPort &f = tree.find(0x100);
    EXPECT_EQ(0, f.read(0x100));

    tree.add(0x101, IOPort(
        [](offset_t offset) throw() -> byte_t { return 10; },
        DefaultWrite8()));
    struct IOPort &f2 = tree.find(0x101);
    EXPECT_EQ(10, f2.read(0x101));
    EXPECT_EQ(0, tree.find(0x100).read(0x100));
}

TEST(Radix, multiple)
{
    RadixTree<IOPort, addr_t, 16> tree;

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
    RadixTree<IOPort, addr_t, 16> tree;

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

    RadixTree<IOPort, addr_t, 16> tree;
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

#endif

typedef DataBus<uint16_t, 16, uint8_t> Bus16x8;

TEST(BusTest, Bus16by8)
{
    Bus16x8 bus;

    byte_t foo;
    bus.add(Bus16x8::IOPort(0x1000, 0xF000));
    bus.add(Bus16x8::IOPort(0x2000, 0xF000, &foo));

    bus.read(0x1000);
    bus.write(0x1000, 0x00);

    bus.write(0x2000, 0x66);
    EXPECT_EQ(0x66, foo);
    EXPECT_EQ(0x66, bus.read(0x2000));

    EXPECT_THROW(bus.read(0x0000), BusError);
}

typedef DataBus<uint16_t, 16, uint16_t> Bus16x16;

TEST(BusTest, Bus16by16)
{
    Bus16x16 bus;

    uint16_t foo;
    bus.add(Bus16x16::IOPort(0x1000, 0xF000));
    bus.add(Bus16x16::IOPort(0x2000, 0xF000, &foo));

    bus.read(0x1000);
    bus.write(0x1000, 0x0000);

    bus.write(0x2000, 0x6666);
    EXPECT_EQ(0x6666, foo);
    EXPECT_EQ(0x6666, bus.read(0x2000));

    EXPECT_THROW(bus.read(0x0000), BusError);
}

typedef DataBus<uint32_t, 24, uint16_t> Bus24x16;

TEST(BusTest, Bus24x16)
{
    Bus24x16 bus;

    uint16_t foo = 0x2211;
    bus.add(Bus24x16::IOPort(0x1000,   0xFFF000, &foo));
    bus.add(Bus24x16::IOPort(0x100000, 0xFFFFFF, &foo));

    EXPECT_EQ(foo, bus.read(0x100000));
    EXPECT_EQ(bus.read(0x1000), bus.read(0x100000));
}

