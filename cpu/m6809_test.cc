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

#include "cpu/m6809.h"

using namespace EMU;
using namespace EMUTest;
using namespace M6809;

typedef TestMachine<M6809Cpu, 0x0000> M6809Machine;

TEST(M6809Test, constructor)
{
    M6809Machine machine;
}

TEST(M6809Test, opcode_9B)
{
    M6809Machine machine;

    LOAD2(0x9B, 0x01);

    machine.cpu.test_step();

    EXPECT_EQ(1, machine.cpu.state()->D.b.l);
}

TEST(M6809Test, opcode_CB)
{
    M6809Machine machine;

    LOAD2(0xDB, 0x01);
    LOAD2(0xDB, 0x02);

    machine.cpu.test_step();
    EXPECT_EQ(1, machine.cpu.state()->D.b.h);

    machine.cpu.test_step();
    EXPECT_EQ(0xDC, machine.cpu.state()->D.b.h);
}

