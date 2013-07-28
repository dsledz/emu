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

#include "z80.h"
#include "rom.h"

using namespace Z80;

#define LOAD(code) \
    ram.write8(pc++, code)
#define LOAD1(code, arg1) \
    ram.write8(pc++, code); \
    ram.write8(pc++, arg1);
#define LOAD2(code, arg1, arg2) \
    ram.write8(pc++, code); \
    ram.write8(pc++, arg1); \
    ram.write8(pc++, arg2);

class Z80Test: public ::testing::Test {
    public:
        Z80Test(void):
            machine(),
            bus(),
            cpu(&machine, "cpu", 1000000, &bus),
            ram(0x2000), pc(0)
        {
            bus.add(0x0000, 0xE000, &ram);
        }

        Machine machine;
        AddressBus16 bus;
        Z80Cpu cpu;
        Ram ram;
        addr_t pc;
};

TEST_F(Z80Test, Constructor)
{
}

/* NOP */
TEST_F(Z80Test, opcode_0x00)
{
    LOAD(0x00);
    cpu.execute(Time(usec(30)));
}

/* LD BC, d16 */
TEST_F(Z80Test, opcode_0x01)
{
    LOAD2(0x01, 0x12, 0x34);
    cpu.execute(Time(usec(10)));

    EXPECT_EQ(0x34, cpu.fetch(Reg::B));
    EXPECT_EQ(0x12, cpu.fetch(Reg::C));
}

class Zexall: public Machine
{
public:
    Zexall(void):
        bus(),
        cpu(this, "cpu", 1000000, &bus),
        rom("zex.bin"),
        _data(0), _req(0), _req_last(0), _ack(0)
    {
        cpu.load_rom(&rom, 0x000);

        bus.add(0xFFFF, &_data);
        bus.add(0xFFFE, 0xFFFF,
                READ_CB(Zexall::req_read, this),
                WRITE_CB(Zexall::req_write, this));
        bus.add(0xFFFD, &_ack);
    }
    ~Zexall(void) {
    }

    byte_t req_read(byte_t vlaue)
    {
        return _req;
    }

    void req_write(offset_t offset, byte_t value)
    {
        if (_req_last != value) {
            _ack++;
            std::cout << _data;
            std::cout.flush();
        }
        _req_last = _req;
        _req = value;
    }

    AddressBus16 bus;
    Z80Cpu cpu;
    Rom rom;
    byte_t _data, _req, _req_last, _ack;
};

TEST(Zexall_test, test)
{
    Zexall zex;

    zex.cpu.execute(Time(sec(10)));
}
