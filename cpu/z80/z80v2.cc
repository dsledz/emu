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

#include "cpu/z80/z80v2.h"
#include "cpu/z80/z80_ops.h"

using namespace Z80v2;

#define OPCODE(code, cycles, bytes, name, op) { \
    code, \
    name, \
    bytes, \
    cycles, \
    std::bind(&op, _1, _2), \
}

#define OPCODE1(code, cycles, bytes, name, op, arg1) { \
    code, \
    name, \
    bytes, \
    cycles, \
    std::bind(&op, _1, _2, arg1), \
}

Z80Cpu::Z80Cpu(Machine *machine, const std::string &name, unsigned hertz,
               bus_type *bus):
    Cpu(machine, name, hertz, bus)
{
    Z80Opcode opcodes[] = {
        OPCODE(0x00,  4, 1, "NOP", NOP),
        OPCODE(0x80,  4, 1, "ADD", ADD, ArgRegB),
        OPCODE(0x81,  4, 1, "ADD", ADD, ArgRegC),
        OPCODE(0x82,  4, 1, "ADD", ADD, ArgRegD),
        OPCODE(0x83,  4, 1, "ADD", ADD, ArgRegE),
        OPCODE(0x84,  4, 1, "ADD", ADD, ArgRegH),
        OPCODE(0x85,  4, 1, "ADD", ADD, ArgRegL),
        OPCODE(0x86,  4, 1, "ADD", ADD, ArgRegHL),
        OPCODE(0x87,  4, 1, "ADD", ADD, ArgRegA),
    };

    for (int i = 0; i < sizeof(opcodes)/sizeof(opcodes[0]); i++) {
        _opcodes[opcode[i].code] = opcodes[i];
    }
}
