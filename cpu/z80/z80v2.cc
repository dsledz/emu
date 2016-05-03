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

using namespace Z80v2;

#define OPCODE(opnum, name) \
static inline void func_##opnum(Z80State *state); \
static const char *name_##opnum = name; \
static inline void func_##opnum(Z80State *state)

static inline void pc_read(Z80State *state, reg8_t *reg)
{
    state->bus->io_read(state->PC.d, reg);
    state->PC.d++;
    state->icycles++;
}

static inline void bus_write(Z80State *state, reg16_t addr, reg8_t reg)
{
    state->bus->io_write(addr.d, reg);
}

OPCODE(0x00, "NOP")
{

}

OPCODE(0x01, "LD BC, d16")
{
    reg16_t arg;
    pc_read(state, &arg.b.l);
    pc_read(state, &arg.b.h);
    state->BC = arg;
}

OPCODE(0x02, "LD (BC), A")
{
    bus_write(state, state->BC.d, state->AF.b.h);
}

OPCODE(0x03, "INC BC")
{
    state->BC.d++;
}

static inline void
INC(Z80State *state, reg8_t &dest)
{
    uint16_t result = dest + 1;

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result & 0xff) == 0;
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = bit_isset(dest ^ 1 ^ result, 4);
    state->AF.b.f.X = bit_isset(result, 3);
    state->AF.b.f.V = (dest == 0x7F);
    state->AF.b.f.N = false;

    dest = static_cast<reg8_t>(result);
}

static inline void
DEC(Z80State *state, reg8_t &dest)
{
    uint16_t result = dest - 1;

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result & 0xff) == 0;
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = bit_isset(dest ^ 1 ^ result, 4);
    state->AF.b.f.X = bit_isset(result, 3);
    state->AF.b.f.V = (dest == 0x80);
    state->AF.b.f.N = true;

    dest = (byte_t)result;
}

OPCODE(0x04, "INC B")
{
    INC(state, state->BC.b.h);
}

OPCODE(0x05, "DEC B")
{
    DEC(state, state->BC.b.h);
}

#define OPCODE_DEF(num) \
    { num, name_##num, func_##num }

#define OPCODE16(prefix, num) \
    OPCODE_DEF(0x##num##0), OPCODE_DEF(0x##num##1), OPCODE_DEF(0x##num##2), \
    OPCODE_DEF(0x##num##3), OPCODE_DEF(0x##num##4), OPCODE_DEF(0x##num##5), \
    OPCODE_DEF(0x##num##6), OPCODE_DEF(0x##num##7), OPCODE_DEF(0x##num##8), \
    OPCODE_DEF(0x##num##9), OPCODE_DEF(0x##num##A), OPCODE_DEF(0x##num##B), \
    OPCODE_DEF(0x##num##C), OPCODE_DEF(0x##num##D), OPCODE_DEF(0x##num##E), \
    OPCODE_DEF(0x##num##F)

#if 0
static struct Z80Opcode opcodes[255] = {
    OPCODE16(op_, 0), OPCODE16(op_, 1), OPCODE16(op_, 2), OPCODE16(op_, 3),
    OPCODE16(op_, 4), OPCODE16(op_, 5), OPCODE16(op_, 6), OPCODE16(op_, 7),
    OPCODE16(op_, 8), OPCODE16(op_, 9), OPCODE16(op_, A), OPCODE16(op_, B),
    OPCODE16(op_, C), OPCODE16(op_, D), OPCODE16(op_, E), OPCODE16(op_, F),
}

#else
static struct Z80Opcode opcodes[] = {
    OPCODE_DEF(0x00), OPCODE_DEF(0x01), OPCODE_DEF(0x02), OPCODE_DEF(0x03),
    OPCODE_DEF(0x04), OPCODE_DEF(0x05)
};
#endif


Z80Class::Z80Class(void)
{
}

Z80Class::~Z80Class(void)
{
}

void
Z80Class::Interrupt(Z80State *state, Z80Bus *bus)
{
    state->Phase = CpuPhase::Decode;
}

void
Z80Class::Decode(Z80State *state, Z80Bus *bus)
{
    state->bus = bus;
    reg8_t op;
    pc_read(state, &op);
    state->Op = &opcodes[op];
    state->Phase = CpuPhase::Dispatch;
}

void
Z80Class::Dispatch(Z80State *state, Z80Bus *bus)
{
    state->Op->func(state);
    state->Phase = CpuPhase::Interrupt;
}



