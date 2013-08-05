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

#include "m6502.h"

using namespace EMU;
using namespace M6502;

class m6502Machine: public Machine
{
public:
    m6502Machine(void):
        Machine(),
        bus(),
        cpu(this, "cpu", 1000000, &bus),
        ram(0x2000),
        irq_vec(0x0008)
    {
        bus.add(0x0000, 0x1FFF, &ram);
        bus.add(0xFFF8, 0xFFFF, &irq_vec);

        irq_vec.write8(0x0006, 0x0000);
        irq_vec.write8(0x0007, 0x0010);

        /* Our PC starts at 0x0000, but don't want to put code in the zpg */
        ram.write8(0x0000, 0x4C);
        ram.write8(0x0001, 0x00);
        ram.write8(0x0002, 0x10);
    }
    ~m6502Machine(void)
    {
    }

    AddressBus16 bus;
    M6502Cpu cpu;
    Ram ram;
    Ram irq_vec;
};

TEST(M6502Test, constructor)
{
    m6502Machine machine;
}

#define LOAD1(op) \
    machine.ram.write8(pc++, op);
#define LOAD2(op, arg) \
    machine.ram.write8(pc++, op); \
    machine.ram.write8(pc++, arg);
#define LOAD3(op, arg1, arg2) \
    machine.ram.write8(pc++, op); \
    machine.ram.write8(pc++, arg1); \
    machine.ram.write8(pc++, arg2);

TEST(M6502Test, opcode_ea)
{
    m6502Machine machine;
    addr_t pc = 0x1000;

    /* NOP */
    LOAD1(0xea);

    machine.cpu.execute(Time(usec(10)));
}


TEST(M6502Test, immediate)
{
    m6502Machine machine;
    addr_t pc = 0x1000;

    /* LDA # */
    LOAD2(0xA9, 0x10);
    /* ORA # */
    LOAD2(0x09, 0x01);
    /* STA 0+0x10 */
    LOAD2(0x85, 0x10);
    /* LDX # */
    LOAD2(0xA2, 0x50);
    /* STA 0+0x10+X */
    LOAD2(0x95, 0x10);
    /* STA x,ind (0x005F,0x0060) -> 0x1100*/
    LOAD2(0x81, 0x0F)

    machine.cpu.execute(Time(usec(30)));

    EXPECT_EQ(0x11, machine.bus.read(0x0010));
    EXPECT_EQ(0x11, machine.bus.read(0x0060));
    EXPECT_EQ(0x11, machine.bus.read(0x1100));
}
