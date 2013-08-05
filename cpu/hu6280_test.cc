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

#include "cpu/m6502.h"

using namespace M6502;

#define LOAD1(code) \
    bank0[pc++] = code;
#define LOAD2(code, arg1) \
    bank0[pc++] = code; \
    bank0[pc++] = arg1;
#define LOAD3(code, arg1, arg2) \
    bank0[pc++] = code; \
    bank0[pc++] = arg1; \
    bank0[pc++] = arg2;

class hu6280Test: public ::testing::Test {
public:
    hu6280Test(void):
        machine(),
        bus(),
        cpu(&machine, "cpu", 1000000, &bus),
        bank0(0x2000),
        bankFF(0x2000),
        bankF8(0x2000),
        pc(0)
    {
        bus.add(0xFF << 13, &bankFF);
        bus.add(0xF8 << 13, &bankF8);
        bus.add(0x00 << 13, &bank0);

        bank0[0x1FFE] = 0x00;
        bank0[0x1FFF] = 0xE0;

        cpu.line(Line::RESET, LineState::Pulse);
    }

    Machine machine;
    AddressBus21 bus;
    hu6280Cpu cpu;
    bvec bank0, bankFF, bankF8;
    offset_t pc;
};

TEST_F(hu6280Test, Constructor)
{
}

TEST_F(hu6280Test, opcode_EA)
{
    LOAD1(0xEA);
    cpu.execute(Time(usec(1)));
}

TEST_F(hu6280Test, opcode_A0)
{
    LOAD2(0xA9, 0x10);
    cpu.execute(Time(usec(2)));
}

