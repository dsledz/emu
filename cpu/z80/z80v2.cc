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
using namespace Z80;

#define OPCODE(opnum, cycles, bytes, name) \
static inline void func_##opnum(Z80State *state); \
static const char *name_##opnum = name; \
static const uint8_t cycles_##opnum = cycles; \
static const uint8_t  bytes_##opnum = bytes; \
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

static inline void bus_read(Z80State *state, reg16_t addr, reg8_t *reg)
{
    state->bus->io_read(addr.d, reg);
}

OPCODE(0x00, 4, 1, "NOP")
{

}

OPCODE(0x01, 10, 1, "LD BC, d16")
{
    pc_read(state, &state->d16.b.l);
    pc_read(state, &state->d16.b.h);
    state->BC = state->d16;
}

OPCODE(0x02, 7, 1, "LD (BC), A")
{
    bus_write(state, state->BC.d, state->AF.b.h);
}

OPCODE(0x03, 6, 1, "INC BC")
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
INC(Z80State *state, reg16_t &dest)
{
    uint16_t result = dest.d + 1;

    dest = (reg16_t)result;
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

static inline void
DEC(Z80State *state, reg16_t &dest)
{
    uint16_t result = dest.d - 1;

    dest = (reg16_t)result;
}

OPCODE(0x04, 4, 1, "INC B")
{
    INC(state, state->BC.b.h);
}

OPCODE(0x05, 4, 1, "DEC B")
{
    DEC(state, state->BC.b.h);
}

OPCODE(0x06, 7, 2, "LD B, d8")
{
    pc_read(state, &state->i8);
    state->BC.b.h = state->i8;
}

OPCODE(0x07, 4, 1, "RLCA")
{
    /* NYI */
}

OPCODE(0x08, 4, 1, "EX AF, AF")
{
    /* NYI */
}

OPCODE(0x09, 11, 1, "ADD HL, BC")
{
    /* NYI */
}

OPCODE(0x0A, 7, 1, "LD A,(BC)")
{
    bus_read(state, state->BC, &state->i8);
    state->AF.b.h = state->i8;
}

OPCODE(0x0B, 6, 1, "DEC BC")
{
    DEC(state, state->BC);
}

OPCODE(0x0C,  4, 1, "INC C")
{
    INC(state, state->BC.b.l);
}

OPCODE(0x0D,  4, 1, "DEC C")
{
    DEC(state, state->BC.b.l);
}

OPCODE(0x0E,  7, 2, "LD C, d8")
{
    pc_read(state, &state->d8);
    state->BC.b.l = state->d8;
}

OPCODE(0x0F,  4, 1, "RRCA")
{
    /* NYI */
}

OPCODE(0x10,  8, 1, "DJNZ d8")
{
    /* NYI */
}

OPCODE(0x11, 10, 3, "LD DE,d16")
{
    pc_read(state, &state->d16.b.l);
    pc_read(state, &state->d16.b.h);
    state->DE = state->d16;
}

OPCODE(0x12,  7, 1, "LD (DE),A")
{
    bus_write(state, state->DE, state->AF.b.h);
}

OPCODE(0x13,  6, 1, "INC DE")
{
    INC(state, state->DE);
}

OPCODE(0x14,  4, 1, "INC D")
{
    INC(state, state->DE.b.h);
}

OPCODE(0x15,  4, 1, "DEC D")
{
    DEC(state, state->DE.b.h);
}

OPCODE(0x16,  7, 2, "LD D, d8")
{
    pc_read(state, &state->d8);
    state->DE.b.h = state->d8;
}

OPCODE(0x17,  4, 1, "RLA")
{
    /* NYI: _rla()); */
}

OPCODE(0x18, 12, 2, "JR r8")
{
    /* NYI:_jr(true, _r8())); */
}

static inline void __attribute__((__used__))
ADD(Z80State *state, reg16_t &dest, reg16_t &arg)
{
    uint16_t result = dest.d + arg.d;

    state->AF.b.f.Y = bit_isset(result, 13);
    state->AF.b.f.H = bit_isset(dest.d ^ arg.d ^ result, 12);
    state->AF.b.f.X = bit_isset(result, 11);
    state->AF.b.f.C = (result > 0xffff) ? 1 : 0;
    state->AF.b.f.N = 0;

    dest = static_cast<reg16_t>(result);
}

OPCODE(0x19, 11, 1, "ADD HL,DE")
{
    /* NYI: _add16(*vrHL, _rDE)); */
}

OPCODE(0x1A,  7, 1, "LD A,(DE)")
{
    bus_read(state, state->DE, &state->d8);
    state->AF.b.h = state->d8;
}

OPCODE(0x1B,  6, 1, "DEC DE")
{
    DEC(state, state->DE);
}

OPCODE(0x1C,  4, 1, "INC E")
{
    INC(state, state->DE.b.l);
}

OPCODE(0x1D,  4, 1, "DEC E")
{
    INC(state, state->DE.b.l);
}

OPCODE(0x1E,  7, 2, "LD E,d8")
{
    pc_read(state, &state->d8);
    state->DE.b.l = state->d8;
}

OPCODE(0x1F,  4, 1, "RRA")
{
    /* NYI: , _rra()); */
}

#if 0
OPCODE(0x20,  7, 2, "JR NZ,r8", _jr(!_flags.Z, _r8()));
OPCODE(0x21, 10, 3, "LD HL,d16", _ld16(*vrHL, _d16()));
OPCODE(0x22, 16, 3, "LD (d16), HL", _ld16i(_d16(), *vrHL));
OPCODE(0x23,  7, 1, "INC HL", _inc16(*vrHL));
OPCODE(0x24,  4, 1, "INC H", _inc(*vrH));
OPCODE(0x25,  4, 1, "DEC H", _dec(*vrH));
OPCODE(0x26,  7, 2, "LD H,d8", _ld(*vrH, _d8()));
OPCODE(0x27,  4, 1, "DAA", _daa());
OPCODE(0x28,  7, 2, "JR Z,r8", _jr(_flags.Z, _r8()));
OPCODE(0x29, 11, 1, "ADD HL,HL", _add16(*vrHL, *vrHL));
OPCODE(0x2A, 16, 3, "LD HL, (d16)", _ld16(*vrHL, _i16()));
OPCODE(0x2B,  6, 1, "DEC HL", _dec16(*vrHL));
OPCODE(0x2C,  4, 1, "INC L", _inc(*vrL));
OPCODE(0x2D,  4, 1, "DEC L", _dec(*vrL));
OPCODE(0x2E,  7, 2, "LD L,d8", _ld(*vrL, _d8()));
OPCODE(0x2F,  4, 2, "CPL", _cpl());
OPCODE(0x30,  7, 2, "JR NC,r8", _jr(!_flags.C, _r8()));
OPCODE(0x31, 10, 3, "LD SP,d16", _ld16(_rSP, _d16()));
OPCODE(0x32, 13, 3, "LD (d16), A", _ldmem(_d16(), _rA));
OPCODE(0x33,  6, 1, "INC SP", _inc16(_rSP));
OPCODE(0x34, 11, 1, "INC (HL)", _inci(_dAddr()));
OPCODE(0x35, 11, 1, "DEC (HL)", _deci(_dAddr()));
OPCODE(0x36, 10, 2, "LD (HL),d8", _ldmem(_dAddr(), _d8()));
OPCODE(0x37,  4, 1, "SCF", _scf());
OPCODE(0x38,  7 ,2, "JR C,r8", _jr(_flags.C, _r8()));
OPCODE(0x39, 11, 1, "ADD HL,SP", _add16(*vrHL, _rSP));
OPCODE(0x3A, 13, 3, "LD A,(d16)", _ld(_rA, _i8(_d16())));
OPCODE(0x3B,  6, 1, "DEC SP", _dec16(_rSP));
OPCODE(0x3C,  4, 1, "INC A", _inc(_rA));
OPCODE(0x3D,  4, 1, "DEC A", _dec(_rA));
OPCODE(0x3E,  7, 2, "LD A,d8", _ld(_rA, _d8()));
OPCODE(0x3F,  4, 1, "CCF", _ccf());
OPCODE(0x40,  4, 1, "LD B,B", _ld(_rB, _rB));
OPCODE(0x41,  4, 1, "LD B,C", _ld(_rB, _rC));
OPCODE(0x42,  4, 1, "LD B,D", _ld(_rB, _rD));
OPCODE(0x43,  4, 1, "LD B,E", _ld(_rB, _rE));
OPCODE(0x44,  4, 1, "LD B,H", _ld(_rB, *vrH));
OPCODE(0x45,  4, 1, "LD B,L", _ld(_rB, *vrL));
OPCODE(0x46,  7, 1, "LD B,(HL)", _ld(_rB, _iAddr()));
OPCODE(0x47,  4, 1, "LD B,A", _ld(_rB, _rA));
OPCODE(0x48,  4, 1, "LD C,B", _ld(_rC, _rB));
OPCODE(0x49,  4, 1, "LD C,C", _ld(_rC, _rC));
OPCODE(0x4A,  4, 1, "LD C,D", _ld(_rC, _rD));
OPCODE(0x4B,  4, 1, "LD C,E", _ld(_rC, _rE));
OPCODE(0x4C,  4, 1, "LD C,H", _ld(_rC, *vrH));
OPCODE(0x4D,  4, 1, "LD C,L", _ld(_rC, *vrL));
OPCODE(0x4E,  7, 1, "LD C,(HL)", _ld(_rC, _iAddr()));
OPCODE(0x4F,  4, 1, "LD C,A", _ld(_rC, _rA));
OPCODE(0x50,  4, 1, "LD D,B", _ld(_rD, _rB));
OPCODE(0x51,  4, 1, "LD D,C", _ld(_rD, _rC));
OPCODE(0x52,  4, 1, "LD D,D", _ld(_rD, _rD));
OPCODE(0x53,  4, 1, "LD D,E", _ld(_rD, _rE));
OPCODE(0x54,  4, 1, "LD D,H", _ld(_rD, *vrH));
OPCODE(0x55,  4, 1, "LD D,L", _ld(_rD, *vrL));
OPCODE(0x56,  7, 1, "LD D,(HL)", _ld(_rD, _iAddr()));
OPCODE(0x57,  4, 1, "LD D,A", _ld(_rD, _rA));
OPCODE(0x58,  4, 1, "LD E,B", _ld(_rE, _rB));
OPCODE(0x59,  4, 1, "LD E,C", _ld(_rE, _rC));
OPCODE(0x5A,  4, 1, "LD E,D", _ld(_rE, _rD));
OPCODE(0x5B,  4, 1, "LD E,E", _ld(_rE, _rE));
OPCODE(0x5C,  4, 1, "LD E,H", _ld(_rE, *vrH));
OPCODE(0x5D,  4, 1, "LD E,L", _ld(_rE, *vrL));
OPCODE(0x5E,  7, 1, "LD E,(HL)", _ld(_rE, _iAddr()));
OPCODE(0x5F,  4, 1, "LD E,A", _ld(_rE, _rA));
OPCODE(0x60,  4, 1, "LD H,B", _ld(*vrH, _rB));
OPCODE(0x61,  4, 1, "LD H,C", _ld(*vrH, _rC));
OPCODE(0x62,  4, 1, "LD H,D", _ld(*vrH, _rD));
OPCODE(0x63,  4, 1, "LD H,E", _ld(*vrH, _rE));
OPCODE(0x64,  4, 1, "LD H,H", _ld(*vrH, *vrH));
OPCODE(0x65,  4, 1, "LD H,L", _ld(*vrH, *vrL));
OPCODE(0x66,  7, 1, "LD H,(HL)", _ld(_rH, _iAddr()));
OPCODE(0x67,  4, 1, "LD H,A", _ld(*vrH, _rA));
OPCODE(0x68,  4, 1, "LD L,B", _ld(*vrL, _rB));
OPCODE(0x69,  4, 1, "LD L,C", _ld(*vrL, _rC));
OPCODE(0x6A,  4, 1, "LD L,D", _ld(*vrL, _rD));
OPCODE(0x6B,  4, 1, "LD L,E", _ld(*vrL, _rE));
OPCODE(0x6C,  4, 1, "LD L,H", _ld(*vrL, *vrH));
OPCODE(0x6D,  4, 1, "LD L,L", _ld(*vrL, *vrL));
OPCODE(0x6E,  7, 1, "LD L,(HL)", _ld(_rL, _iAddr()));
OPCODE(0x6F,  4, 1, "LD L,A", _ld(*vrL, _rA));
OPCODE(0x70,  7, 1, "LD (HL),B", _ldmem(_dAddr(), _rB));
OPCODE(0x71,  7, 1, "LD (HL),C", _ldmem(_dAddr(), _rC));
OPCODE(0x72,  7, 1, "LD (HL),D", _ldmem(_dAddr(), _rD));
OPCODE(0x73,  7, 1, "LD (HL),E", _ldmem(_dAddr(), _rE));
OPCODE(0x74,  7, 1, "LD (HL),H", _ldmem(_dAddr(), _rH));
OPCODE(0x75,  7, 1, "LD (HL),L", _ldmem(_dAddr(), _rL));
OPCODE(0x76,  4, 1, "HALT", _halt());
OPCODE(0x77,  7, 1, "LD (HL),A", _ldmem(_dAddr(), _rA));
OPCODE(0x78,  4, 1, "LD A,B", _ld(_rA, _rB));
OPCODE(0x79,  4, 1, "LD A,C", _ld(_rA, _rC));
OPCODE(0x7A,  4, 1, "LD A,D", _ld(_rA, _rD));
OPCODE(0x7B,  4, 1, "LD A,E", _ld(_rA, _rE));
OPCODE(0x7C,  4, 1, "LD A,H", _ld(_rA, *vrH));
OPCODE(0x7D,  4, 1, "LD A,L", _ld(_rA, *vrL));
OPCODE(0x7E,  7, 1, "LD A,(HL)", _ld(_rA, _iAddr()));
OPCODE(0x7F,  4, 1, "LD A,A", _ld(_rA, _rA));
OPCODE(0x80,  4, 1, "ADD A,B", _add(_rA, _rB));
OPCODE(0x81,  4, 1, "ADD A,C", _add(_rA, _rC));
OPCODE(0x82,  4, 1, "ADD A,D", _add(_rA, _rD));
OPCODE(0x83,  4, 1, "ADD A,E", _add(_rA, _rE));
OPCODE(0x84,  4, 1, "ADD A,H", _add(_rA, *vrH));
OPCODE(0x85,  4, 1, "ADD A,L", _add(_rA, *vrL));
OPCODE(0x86,  7, 1, "ADD A,(HL)", _add(_rA, _iAddr()));
OPCODE(0x87,  4, 1, "ADD A,A", _add(_rA, _rA));
OPCODE(0x88,  4, 1, "ADC A,B", _adc(_rA, _rB));
OPCODE(0x89,  4, 1, "ADC A,C", _adc(_rA, _rC));
OPCODE(0x8A,  4, 1, "ADC A,D", _adc(_rA, _rD));
OPCODE(0x8B,  4, 1, "ADC A,E", _adc(_rA, _rE));
OPCODE(0x8C,  4, 1, "ADC A,H", _adc(_rA, *vrH));
OPCODE(0x8D,  4, 1, "ADC A,L", _adc(_rA, *vrL));
OPCODE(0x8E,  7, 1, "ADC A,(HL)", _adc(_rA, _iAddr()));
OPCODE(0x8F,  4, 1, "ADC A,A", _adc(_rA, _rA));
OPCODE(0x90,  4, 1, "SUB B", _sub(_rA, _rB));
OPCODE(0x91,  4, 1, "SUB C", _sub(_rA, _rC));
OPCODE(0x92,  4, 1, "SUB D", _sub(_rA, _rD));
OPCODE(0x93,  4, 1, "SUB E", _sub(_rA, _rE));
OPCODE(0x94,  4, 1, "SUB H", _sub(_rA, *vrH));
OPCODE(0x95,  4, 1, "SUB L", _sub(_rA, *vrL));
OPCODE(0x96,  7, 1, "SUB (HL)", _sub(_rA, _iAddr()));
OPCODE(0x97,  4, 1, "SUB A", _sub(_rA, _rA));
OPCODE(0x98,  4, 1, "SBC A,B", _sbc(_rA, _rB));
OPCODE(0x99,  4, 1, "SBC A,C", _sbc(_rA, _rC));
OPCODE(0x9A,  4, 1, "SBC A,D", _sbc(_rA, _rD));
OPCODE(0x9B,  4, 1, "SBC A,E", _sbc(_rA, _rE));
OPCODE(0x9C,  4, 1, "SBC A,H", _sbc(_rA, *vrH));
OPCODE(0x9D,  4, 1, "SBC A,L", _sbc(_rA, *vrL));
OPCODE(0x9E,  7, 1, "SBC A,(HL)", _sbc(_rA, _iAddr()));
OPCODE(0x9F,  4, 1, "SBC A,A", _sbc(_rA, _rA));
OPCODE(0xA0,  4, 1, "AND B", _and(_rA, _rB));
OPCODE(0xA1,  4, 1, "AND C", _and(_rA, _rC));
OPCODE(0xA2,  4, 1, "AND D", _and(_rA, _rD));
OPCODE(0xA3,  4, 1, "AND E", _and(_rA, _rE));
OPCODE(0xA4,  4, 1, "AND H", _and(_rA, *vrH));
OPCODE(0xA5,  4, 1, "AND L", _and(_rA, *vrL));
OPCODE(0xA6,  4, 1, "AND (HL)", _and(_rA, _iAddr()));
OPCODE(0xA7,  4, 1, "AND A", _and(_rA, _rA));
OPCODE(0xA8,  4, 1, "XOR B", _xor(_rA, _rB));
OPCODE(0xA9,  4, 1, "XOR C", _xor(_rA, _rC));
OPCODE(0xAA,  4, 1, "XOR D", _xor(_rA, _rD));
OPCODE(0xAB,  4, 1, "XOR E", _xor(_rA, _rE));
OPCODE(0xAC,  4, 1, "XOR H", _xor(_rA, *vrH));
OPCODE(0xAD,  4, 1, "XOR L", _xor(_rA, *vrL));
OPCODE(0xAE,  7, 2, "XOR (HL)", _xor(_rA, _iAddr()));
OPCODE(0xAF,  4, 1, "XOR A", _xor(_rA, _rA));
OPCODE(0xB0,  4, 1, "OR B", _or(_rA, _rB));
OPCODE(0xB1,  4, 1, "OR C", _or(_rA, _rC));
OPCODE(0xB2,  4, 1, "OR D", _or(_rA, _rD));
OPCODE(0xB3,  4, 1, "OR E", _or(_rA, _rE));
OPCODE(0xB4,  4, 1, "OR H", _or(_rA, *vrH));
OPCODE(0xB5,  4, 1, "OR L", _or(_rA, *vrL));
OPCODE(0xB6,  7, 1, "OR (HL)", _or(_rA, _iAddr()));
OPCODE(0xB7,  4, 1, "OR A", _or(_rA, _rA));
OPCODE(0xB8,  4, 1, "CP B", _cp(_rA, _rB));
OPCODE(0xB9,  4, 1, "CP C", _cp(_rA, _rC));
OPCODE(0xBA,  4, 1, "CP D", _cp(_rA, _rD));
OPCODE(0xBB,  4, 1, "CP E", _cp(_rA, _rE));
OPCODE(0xBC,  4, 1, "CP H", _cp(_rA, *vrH));
OPCODE(0xBD,  4, 1, "CP L", _cp(_rA, *vrL));
OPCODE(0xBE,  7, 1, "CP (HL)", _cp(_rA, _iAddr()));
OPCODE(0xBF,  4, 1, "CP A", _cp(_rA, _rA));
OPCODE(0xC0,  5, 1, "RET NZ", _ret(!_flags.Z));
OPCODE(0xC1, 10, 1, "POP BC", _pop(_rB, _rC));
OPCODE(0xC2, 10, 0, "JP NZ", _jp(!_flags.Z, _d16()));
OPCODE(0xC3, 10, 0, "JP a16", _jp(true, _d16()));
OPCODE(0xC4, 10, 3, "CALL NZ,a16", _call(!_flags.Z, _d16()));
OPCODE(0xC5, 11, 1, "PUSH BC", _push(_rB, _rC));
OPCODE(0xC6,  7, 2, "ADD A,d8", _add(_rA, _d8()));
OPCODE(0xC7, 11, 1, "RST 00H", _rst(0x00));
OPCODE(0xC8,  5, 1, "RET Z", _ret(_flags.Z));
OPCODE(0xC9,  6, 1, "RET", _ret(true));
OPCODE(0xCA, 10, 3, "JP Z,a16", _jp(_flags.Z, _d16()));
OPCODE(0xCB,  0, 1, "PREFIX CB", dispatch_cb());
OPCODE(0xCC, 10, 3, "CALL Z,a16", _call(_flags.Z, _d16()));
OPCODE(0xCD, 10, 3, "CALL a16", _call(true, _d16()));
OPCODE(0xCE,  7, 2, "ADC A,d8", _adc(_rA, _d8()));
OPCODE(0xCF, 11, 1, "RST 08H", _rst(0x08));
OPCODE(0xD0,  5, 1, "RET NC", _ret(!_flags.C));
OPCODE(0xD1, 10, 1, "POP DE", _pop(_rD, _rE));
OPCODE(0xD2, 10, 0, "JP NC", _jp(!_flags.C, _d16()));
OPCODE(0xD3, 11, 2, "OUT d8, A", _out(_d8(), _rA));
OPCODE(0xD4, 10, 3, "CALL NC,a16", _call(!_flags.C, _d16()));
OPCODE(0xD5, 11, 1, "PUSH DE", _push(_rD, _rE));
OPCODE(0xD6,  7, 2, "SUB A,d8", _sub(_rA, _d8()));
OPCODE(0xD7, 11, 1, "RST 10H", _rst(0x10));
OPCODE(0xD8,  5, 1, "RET C", _ret(_flags.C));
OPCODE(0xD9,  4, 1, "EXX BC DE HL", _exx());
OPCODE(0xDA, 10, 3, "JP C,a16", _jp(_flags.C, _d16()));
OPCODE(0xDB, 11, 2, "IN A, d8", _in(_rA, _d8()));
OPCODE(0xDC, 10, 3, "CALL C, a16", _call(_flags.C, _d16()));
OPCODE(0xDE,  7, 2, "SBC A,d8", _sbc(_rA, _d8()));
OPCODE(0xDF, 11, 1, "RST 18H", _rst(0x18));
OPCODE(0xE0,  5, 1, "RET PO", _ret(!_flags.V));
OPCODE(0xE1, 10, 1, "POP HL", _pop(*vrH, *vrL));
OPCODE(0xE2, 10, 3, "JP PO,a16", _jp(!_flags.V, _d16()));
OPCODE(0xE3, 19, 1, "EX (SP),HL", _exi(_rSP, *vrH, *vrL));
OPCODE(0xE4, 10, 3, "CALL PO,a16", _call(!_flags.V, _d16()));
OPCODE(0xE5, 16, 1, "PUSH HL", _push(*vrH, *vrL));
OPCODE(0xE6,  7, 2, "AND d8", _and(_rA, _d8()));
OPCODE(0xE7, 16, 1, "RST 20H", _rst(0x20));
OPCODE(0xE8,  5, 1, "RET PE", _ret(_flags.V));
OPCODE(0xE9,  4, 1, "JP HL", _jp(true, *vrHL));
OPCODE(0xEA,  4, 3, "JP PE,a16", _jp(_flags.V, _d16()));
OPCODE(0xEB,  4, 1, "EX DE, HL", _ex(_rDE, _rHL));
OPCODE(0xEC, 10, 3, "CALL PE,a16", _call(_flags.V, _d16()));
OPCODE(0xED,  0, 0, "EXTD", dispatch_ed());
OPCODE(0xEE,  7, 1, "XOR d8", _xor(_rA, _d8()));
OPCODE(0xEF, 11, 1, "RST 28H", _rst(0x28));
OPCODE(0xF0,  5, 1, "RET P", _ret(!_flags.S));
OPCODE(0xF1, 10, 1, "POP AF", _pop(_rA, _rF));
OPCODE(0xF2, 10, 3, "JP P,a16", _jp(!_flags.S, _d16()));
OPCODE(0xF3,  4, 1, "DI", _di());
OPCODE(0xF4, 10, 3, "CALL P,a16", _call(!_flags.S, _d16()));
OPCODE(0xF5, 11, 1, "PUSH AF", _push(_rA, _rF));
OPCODE(0xF6,  7, 2, "OR d8", _or(_rA, _d8()));
OPCODE(0xF7, 11, 1, "RST 30H", _rst(0x30));
OPCODE(0xF8,  5, 1, "RET M", _ret(_flags.S));
OPCODE(0xF9,  6, 1, "LD SP,HL", _ld16(_rSP, *vrHL));
OPCODE(0xFA,  4, 3, "JP M,a16", _jp(_flags.S, _d16()));
OPCODE(0xFB,  4, 1, "EI", _ei());
OPCODE(0xFC, 10, 3, "CALL M,a16", _call(_flags.S, _d16()));
OPCODE(0xFE,  7, 2, "CP d8", _cp(_rA, _d8()));
OPCODE(0xFF, 11, 1, "RST 38H", _rst(0x38));

#endif

#define OPCODE_DEF(num) \
    { num, name_##num, cycles_##num, bytes_##num, func_##num }

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
    OPCODE16(op_, 0), OPCODE16(op_, 1)
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
    state->latch_pc = state->PC;
    pc_read(state, &state->latch_op);
    state->Op = &opcodes[state->latch_op];
    state->Phase = CpuPhase::Dispatch;
}

void
Z80Class::Dispatch(Z80State *state, Z80Bus *bus)
{
    state->Op->func(state);
    Log(state);
    state->Phase = CpuPhase::Interrupt;
}

std::string
Z80Class::Log(Z80State *state)
{
    std::stringstream os;
    const std::string &str = state->Op->name;
    os << Hex(state->latch_pc) << ":" << Hex(state->latch_op) << ":"
       << state->Op->name << " =>";
    const std::string &delimiters = " ,";
    auto lastPos = str.find_first_not_of(delimiters, 0);
    auto pos = str.find_first_of(delimiters, lastPos);
    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        std::string it = str.substr(lastPos, pos - lastPos);
        os << " ";
        if (it == "(IX+d)")
            os << "(IX+" << Hex(state->d8) << ")";
        else if (it == "(IY+d)")
            os << "(IY+" << Hex(state->d8) << ")";
        else if (it == "d16" || it == "a16")
            os << Hex(state->d16);
        else if (it == "(d16)")
            os << "(" << Hex(state->d16) << ")";
        else if (it == "(d8)")
            os << "(" << Hex(state->d8) << ")";
        else if (it == "d8" || it == "r8")
            os << Hex(state->d8);
        else
            os << it;
        lastPos = str.find_first_not_of(delimiters, pos);
        pos = str.find_first_of(delimiters, lastPos);
    }
    return os.str();
}

