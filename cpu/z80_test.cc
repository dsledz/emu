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
            machine(18432000),
            cpu(&machine, "test", 1),
            ram(0x2000), pc(0) {
            cpu.bus()->add_port(0x0000, &ram);
        }

        Machine machine;
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
    cpu.tick(30);
}

/* LD BC, d16 */
TEST_F(Z80Test, opcode_0x01)
{
    LOAD2(0x01, 0x12, 0x34);
    cpu.tick(30);

    EXPECT_EQ(0x34, cpu.fetch(Reg::B));
    EXPECT_EQ(0x12, cpu.fetch(Reg::C));
}

