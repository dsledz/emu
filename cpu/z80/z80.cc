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

#include "cpu/z80/z80.h"
#include "cpu/z80/z80_ops.h"

using namespace Z80;

#define OPCODE(opnum, cycles, bytes, name, code)     \
  template <typename CpuState>                       \
  static inline void func_##opnum(CpuState *state);  \
  static const char *name_##opnum = name;            \
  static const uint8_t cycles_##opnum = cycles;      \
  static const uint8_t bytes_##opnum = bytes;        \
  template <typename CpuState>                       \
  static inline void func_##opnum(CpuState *state) { \
    code;                                            \
  }

OPCODE(0x00, 4, 1, "NOP", );
OPCODE(0x01, 10, 3, "LD BC,d16", LD16(state, state->BC, D16(state)));
OPCODE(0x02, 7, 1, "LD (BC),A", LDMEM(state, state->BC.d, state->AF.b.h));
OPCODE(0x03, 6, 1, "INC BC", INC16(state, state->BC));
OPCODE(0x04, 4, 1, "INC B", INC(state, state->BC.b.h));
OPCODE(0x05, 4, 1, "DEC B", DEC(state, state->BC.b.h));
OPCODE(0x06, 7, 2, "LD B, d8", LD(state, state->BC.b.h, D8(state)));
OPCODE(0x07, 4, 1, "RLCA", RLCA(state));
OPCODE(0x08, 4, 1, "EX AF,AF'", EX(state, state->AF.d, state->AF2.d));
OPCODE(0x09, 11, 1, "ADD HL,BC", ADD16(state, *state->vHL, state->BC.d));
OPCODE(0x0A, 7, 1, "LD A,(BC)",
       LD(state, state->AF.b.h, state->bus_read(state, state->BC.d)));
OPCODE(0x0B, 6, 1, "DEC BC", DEC16(state, state->BC));
OPCODE(0x0C, 4, 1, "INC C", INC(state, state->BC.b.l));
OPCODE(0x0D, 4, 1, "DEC C", DEC(state, state->BC.b.l));
OPCODE(0x0E, 7, 2, "LD C, d8", LD(state, state->BC.b.l, D8(state)));
OPCODE(0x0F, 4, 1, "RRCA", RRCA(state));
OPCODE(0x10, 8, 1, "DJNZ r8", DJNZ(state, D8(state)));
OPCODE(0x11, 10, 3, "LD DE,d16", LD16(state, state->DE, D16(state)));
OPCODE(0x12, 7, 1, "LD (DE),A", LDMEM(state, state->DE.d, state->AF.b.h));
OPCODE(0x13, 6, 1, "INC DE", INC16(state, state->DE));
OPCODE(0x14, 4, 1, "INC D", INC(state, state->DE.b.h));
OPCODE(0x15, 4, 1, "DEC D", DEC(state, state->DE.b.h));
OPCODE(0x16, 7, 2, "LD D, d8", LD(state, state->DE.b.h, D8(state)));
OPCODE(0x17, 4, 1, "RLA", RLA(state));
OPCODE(0x18, 12, 2, "JR r8", JR(state, true, D8(state)));
OPCODE(0x19, 11, 1, "ADD HL,DE", ADD16(state, *state->vHL, state->DE.d));
OPCODE(0x1A, 7, 1, "LD A,(DE)",
       LD(state, state->AF.b.h, state->bus_read(state, state->DE.d)));
OPCODE(0x1B, 6, 1, "DEC DE", DEC16(state, state->DE));
OPCODE(0x1C, 4, 1, "INC E", INC(state, state->DE.b.l));
OPCODE(0x1D, 4, 1, "DEC E", DEC(state, state->DE.b.l));
OPCODE(0x1E, 7, 2, "LD E, d8", LD(state, state->DE.b.l, D8(state)));
OPCODE(0x1F, 4, 1, "RRA", RRA(state));
OPCODE(0x20, 7, 2, "JR NZ,r8", JR(state, !state->AF.b.f.Z, D8(state)));
OPCODE(0x21, 10, 3, "LD HL,d16", LD16(state, *state->vHL, D16(state)));
OPCODE(0x22, 16, 3, "LD (d16), HL", LD16I(state, D16(state), state->vHL->d));
OPCODE(0x23, 7, 1, "INC HL", INC16(state, *state->vHL));
OPCODE(0x24, 4, 1, "INC H", INC(state, state->vHL->b.h));
OPCODE(0x25, 4, 1, "DEC H", DEC(state, state->vHL->b.h));
OPCODE(0x26, 7, 2, "LD H, d8", LD(state, state->vHL->b.h, D8(state)));
OPCODE(0x27, 4, 1, "DAA", DAA(state));
OPCODE(0x28, 7, 2, "JR Z,r8", JR(state, state->AF.b.f.Z, D8(state)));
OPCODE(0x29, 11, 1, "ADD HL,HL", ADD16(state, *state->vHL, state->vHL->d));
OPCODE(0x2A, 16, 3, "LD HL, (d16)", LD16(state, *state->vHL, I16(state)));
OPCODE(0x2B, 6, 1, "DEC HL", DEC16(state, *state->vHL));
OPCODE(0x2C, 4, 1, "INC L", INC(state, state->vHL->b.l));
OPCODE(0x2D, 4, 1, "DEC L", DEC(state, state->vHL->b.l));
OPCODE(0x2E, 7, 2, "LD L, d8", LD(state, state->vHL->b.l, D8(state)));
OPCODE(0x2F, 4, 2, "CPL", CPL(state));
OPCODE(0x30, 7, 2, "JR NC,r8", JR(state, !state->AF.b.f.C, D8(state)));
OPCODE(0x31, 10, 3, "LD SP,d16", LD16(state, state->SP, D16(state)));
OPCODE(0x32, 13, 3, "LD (d16), A", LDMEM(state, D16(state), state->AF.b.h));
OPCODE(0x33, 6, 1, "INC SP", INC16(state, state->SP));
OPCODE(0x34, 11, 1, "INC (HL)", INCI(state, state->vHL->d));
OPCODE(0x35, 11, 1, "DEC (HL)", DECI(state, state->vHL->d));
OPCODE(0x36, 10, 2, "LD (HL), d8", LDMEM(state, DADDR(state), D8(state)));
OPCODE(0x37, 4, 1, "SCF", SCF(state));
OPCODE(0x38, 7, 2, "JR C,r8", JR(state, state->AF.b.f.C, D8(state)));
OPCODE(0x39, 11, 1, "ADD HL,SP", ADD16(state, *state->vHL, state->SP.d));
OPCODE(0x3A, 13, 3, "LD A,(d16)",
       LD(state, state->AF.b.h, I8(state, D16(state))));
OPCODE(0x3B, 6, 1, "DEC SP", DEC16(state, state->SP));
OPCODE(0x3C, 4, 1, "INC A", INC(state, state->AF.b.h));
OPCODE(0x3D, 4, 1, "DEC A", DEC(state, state->AF.b.h));
OPCODE(0x3E, 7, 2, "LD A, d8", LD(state, state->AF.b.h, D8(state)));
OPCODE(0x3F, 4, 1, "CCF", CCF(state));
OPCODE(0x40, 4, 1, "LD B,B", LD(state, state->BC.b.h, state->BC.b.h));
OPCODE(0x41, 4, 1, "LD B,C", LD(state, state->BC.b.h, state->BC.b.l));
OPCODE(0x42, 4, 1, "LD B,D", LD(state, state->BC.b.h, state->DE.b.h));
OPCODE(0x43, 4, 1, "LD B,E", LD(state, state->BC.b.h, state->DE.b.l));
OPCODE(0x44, 4, 1, "LD B,H", LD(state, state->BC.b.h, state->vHL->b.h));
OPCODE(0x45, 4, 1, "LD B,L", LD(state, state->BC.b.h, state->vHL->b.l));
OPCODE(0x46, 7, 1, "LD B,(HL)", LD(state, state->BC.b.h, IADDR(state)));
OPCODE(0x47, 4, 1, "LD B,A", LD(state, state->BC.b.h, state->AF.b.h));
OPCODE(0x48, 4, 1, "LD C,B", LD(state, state->BC.b.l, state->BC.b.h));
OPCODE(0x49, 4, 1, "LD C,C", LD(state, state->BC.b.l, state->BC.b.l));
OPCODE(0x4A, 4, 1, "LD C,D", LD(state, state->BC.b.l, state->DE.b.h));
OPCODE(0x4B, 4, 1, "LD C,E", LD(state, state->BC.b.l, state->DE.b.l));
OPCODE(0x4C, 4, 1, "LD C,H", LD(state, state->BC.b.l, state->vHL->b.h));
OPCODE(0x4D, 4, 1, "LD C,L", LD(state, state->BC.b.l, state->vHL->b.l));
OPCODE(0x4E, 7, 1, "LD C,(HL)", LD(state, state->BC.b.l, IADDR(state)));
OPCODE(0x4F, 4, 1, "LD C,A", LD(state, state->BC.b.l, state->AF.b.h));
OPCODE(0x50, 4, 1, "LD D,B", LD(state, state->DE.b.h, state->BC.b.h));
OPCODE(0x51, 4, 1, "LD D,C", LD(state, state->DE.b.h, state->BC.b.l));
OPCODE(0x52, 4, 1, "LD D,D", LD(state, state->DE.b.h, state->DE.b.h));
OPCODE(0x53, 4, 1, "LD D,E", LD(state, state->DE.b.h, state->DE.b.l));
OPCODE(0x54, 4, 1, "LD D,H", LD(state, state->DE.b.h, state->vHL->b.h));
OPCODE(0x55, 4, 1, "LD D,L", LD(state, state->DE.b.h, state->vHL->b.l));
OPCODE(0x56, 7, 1, "LD D,(HL)", LD(state, state->DE.b.h, IADDR(state)));
OPCODE(0x57, 4, 1, "LD D,A", LD(state, state->DE.b.h, state->AF.b.h));
OPCODE(0x58, 4, 1, "LD E,B", LD(state, state->DE.b.l, state->BC.b.h));
OPCODE(0x59, 4, 1, "LD E,C", LD(state, state->DE.b.l, state->BC.b.l));
OPCODE(0x5A, 4, 1, "LD E,D", LD(state, state->DE.b.l, state->DE.b.h));
OPCODE(0x5B, 4, 1, "LD E,E", LD(state, state->DE.b.l, state->DE.b.l));
OPCODE(0x5C, 4, 1, "LD E,H", LD(state, state->DE.b.l, state->vHL->b.h));
OPCODE(0x5D, 4, 1, "LD E,L", LD(state, state->DE.b.l, state->vHL->b.l));
OPCODE(0x5E, 7, 1, "LD E,(HL)", LD(state, state->DE.b.l, IADDR(state)));
OPCODE(0x5F, 4, 1, "LD E,A", LD(state, state->DE.b.l, state->AF.b.h));
OPCODE(0x60, 4, 1, "LD H,B", LD(state, state->vHL->b.h, state->BC.b.h));
OPCODE(0x61, 4, 1, "LD H,C", LD(state, state->vHL->b.h, state->BC.b.l));
OPCODE(0x62, 4, 1, "LD H,D", LD(state, state->vHL->b.h, state->DE.b.h));
OPCODE(0x63, 4, 1, "LD H,E", LD(state, state->vHL->b.h, state->DE.b.l));
OPCODE(0x64, 4, 1, "LD H,H", LD(state, state->vHL->b.h, state->vHL->b.h));
OPCODE(0x65, 4, 1, "LD H,L", LD(state, state->vHL->b.h, state->vHL->b.l));
OPCODE(0x66, 7, 1, "LD H,(HL)", LD(state, state->HL.b.h, IADDR(state)));
OPCODE(0x67, 4, 1, "LD H,A", LD(state, state->vHL->b.h, state->AF.b.h));
OPCODE(0x68, 4, 1, "LD L,B", LD(state, state->vHL->b.l, state->BC.b.h));
OPCODE(0x69, 4, 1, "LD L,C", LD(state, state->vHL->b.l, state->BC.b.l));
OPCODE(0x6A, 4, 1, "LD L,D", LD(state, state->vHL->b.l, state->DE.b.h));
OPCODE(0x6B, 4, 1, "LD L,E", LD(state, state->vHL->b.l, state->DE.b.l));
OPCODE(0x6C, 4, 1, "LD L,H", LD(state, state->vHL->b.l, state->vHL->b.h));
OPCODE(0x6D, 4, 1, "LD L,L", LD(state, state->vHL->b.l, state->vHL->b.l));
OPCODE(0x6E, 7, 1, "LD L,(HL)", LD(state, state->HL.b.l, IADDR(state)));
OPCODE(0x6F, 4, 1, "LD L,A", LD(state, state->vHL->b.l, state->AF.b.h));
OPCODE(0x70, 7, 1, "LD (HL),B", LDMEM(state, DADDR(state), state->BC.b.h));
OPCODE(0x71, 7, 1, "LD (HL),C", LDMEM(state, DADDR(state), state->BC.b.l));
OPCODE(0x72, 7, 1, "LD (HL),D", LDMEM(state, DADDR(state), state->DE.b.h));
OPCODE(0x73, 7, 1, "LD (HL),E", LDMEM(state, DADDR(state), state->DE.b.l));
OPCODE(0x74, 7, 1, "LD (HL),H", LDMEM(state, DADDR(state), state->HL.b.h));
OPCODE(0x75, 7, 1, "LD (HL),L", LDMEM(state, DADDR(state), state->HL.b.l));
OPCODE(0x76, 4, 1, "HALT", HALT(state));
OPCODE(0x77, 7, 1, "LD (HL),A", LDMEM(state, DADDR(state), state->AF.b.h));
OPCODE(0x78, 4, 1, "LD A,B", LD(state, state->AF.b.h, state->BC.b.h));
OPCODE(0x79, 4, 1, "LD A,C", LD(state, state->AF.b.h, state->BC.b.l));
OPCODE(0x7A, 4, 1, "LD A,D", LD(state, state->AF.b.h, state->DE.b.h));
OPCODE(0x7B, 4, 1, "LD A,E", LD(state, state->AF.b.h, state->DE.b.l));
OPCODE(0x7C, 4, 1, "LD A,H", LD(state, state->AF.b.h, state->vHL->b.h));
OPCODE(0x7D, 4, 1, "LD A,L", LD(state, state->AF.b.h, state->vHL->b.l));
OPCODE(0x7E, 7, 1, "LD A,(HL)", LD(state, state->AF.b.h, IADDR(state)));
OPCODE(0x7F, 4, 1, "LD A,A", LD(state, state->AF.b.h, state->AF.b.h));
OPCODE(0x80, 4, 1, "ADD A,B", ADD(state, state->AF.b.h, state->BC.b.h));
OPCODE(0x81, 4, 1, "ADD A,C", ADD(state, state->AF.b.h, state->BC.b.l));
OPCODE(0x82, 4, 1, "ADD A,D", ADD(state, state->AF.b.h, state->DE.b.h));
OPCODE(0x83, 4, 1, "ADD A,E", ADD(state, state->AF.b.h, state->DE.b.l));
OPCODE(0x84, 4, 1, "ADD A,H", ADD(state, state->AF.b.h, state->vHL->b.h));
OPCODE(0x85, 4, 1, "ADD A,L", ADD(state, state->AF.b.h, state->vHL->b.l));
OPCODE(0x86, 7, 1, "ADD A,(HL)", ADD(state, state->AF.b.h, IADDR(state)));
OPCODE(0x87, 4, 1, "ADD A,A", ADD(state, state->AF.b.h, state->AF.b.h));
OPCODE(0x88, 4, 1, "ADC A,B", ADC(state, state->AF.b.h, state->BC.b.h));
OPCODE(0x89, 4, 1, "ADC A,C", ADC(state, state->AF.b.h, state->BC.b.l));
OPCODE(0x8A, 4, 1, "ADC A,D", ADC(state, state->AF.b.h, state->DE.b.h));
OPCODE(0x8B, 4, 1, "ADC A,E", ADC(state, state->AF.b.h, state->DE.b.l));
OPCODE(0x8C, 4, 1, "ADC A,H", ADC(state, state->AF.b.h, state->vHL->b.h));
OPCODE(0x8D, 4, 1, "ADC A,L", ADC(state, state->AF.b.h, state->vHL->b.l));
OPCODE(0x8E, 7, 1, "ADC A,(HL)", ADC(state, state->AF.b.h, IADDR(state)));
OPCODE(0x8F, 4, 1, "ADC A,A", ADC(state, state->AF.b.h, state->AF.b.h));
OPCODE(0x90, 4, 1, "SUB B", SUB(state, state->AF.b.h, state->BC.b.h));
OPCODE(0x91, 4, 1, "SUB C", SUB(state, state->AF.b.h, state->BC.b.l));
OPCODE(0x92, 4, 1, "SUB D", SUB(state, state->AF.b.h, state->DE.b.h));
OPCODE(0x93, 4, 1, "SUB E", SUB(state, state->AF.b.h, state->DE.b.l));
OPCODE(0x94, 4, 1, "SUB H", SUB(state, state->AF.b.h, state->vHL->b.h));
OPCODE(0x95, 4, 1, "SUB L", SUB(state, state->AF.b.h, state->vHL->b.l));
OPCODE(0x96, 7, 1, "SUB (HL)", SUB(state, state->AF.b.h, IADDR(state)));
OPCODE(0x97, 4, 1, "SUB A", SUB(state, state->AF.b.h, state->AF.b.h));
OPCODE(0x98, 4, 1, "SBC A,B", SBC(state, state->AF.b.h, state->BC.b.h));
OPCODE(0x99, 4, 1, "SBC A,C", SBC(state, state->AF.b.h, state->BC.b.l));
OPCODE(0x9A, 4, 1, "SBC A,D", SBC(state, state->AF.b.h, state->DE.b.h));
OPCODE(0x9B, 4, 1, "SBC A,E", SBC(state, state->AF.b.h, state->DE.b.l));
OPCODE(0x9C, 4, 1, "SBC A,H", SBC(state, state->AF.b.h, state->vHL->b.h));
OPCODE(0x9D, 4, 1, "SBC A,L", SBC(state, state->AF.b.h, state->vHL->b.l));
OPCODE(0x9E, 7, 1, "SBC A,(HL)", SBC(state, state->AF.b.h, IADDR(state)));
OPCODE(0x9F, 4, 1, "SBC A,A", SBC(state, state->AF.b.h, state->AF.b.h));
OPCODE(0xA0, 4, 1, "AND B", AND(state, state->AF.b.h, state->BC.b.h));
OPCODE(0xA1, 4, 1, "AND C", AND(state, state->AF.b.h, state->BC.b.l));
OPCODE(0xA2, 4, 1, "AND D", AND(state, state->AF.b.h, state->DE.b.h));
OPCODE(0xA3, 4, 1, "AND E", AND(state, state->AF.b.h, state->DE.b.l));
OPCODE(0xA4, 4, 1, "AND H", AND(state, state->AF.b.h, state->vHL->b.h));
OPCODE(0xA5, 4, 1, "AND L", AND(state, state->AF.b.h, state->vHL->b.l));
OPCODE(0xA6, 4, 1, "AND (HL)", AND(state, state->AF.b.h, IADDR(state)));
OPCODE(0xA7, 4, 1, "AND A", AND(state, state->AF.b.h, state->AF.b.h));
OPCODE(0xA8, 4, 1, "XOR B", XOR(state, state->AF.b.h, state->BC.b.h));
OPCODE(0xA9, 4, 1, "XOR C", XOR(state, state->AF.b.h, state->BC.b.l));
OPCODE(0xAA, 4, 1, "XOR D", XOR(state, state->AF.b.h, state->DE.b.h));
OPCODE(0xAB, 4, 1, "XOR E", XOR(state, state->AF.b.h, state->DE.b.l));
OPCODE(0xAC, 4, 1, "XOR H", XOR(state, state->AF.b.h, state->vHL->b.h));
OPCODE(0xAD, 4, 1, "XOR L", XOR(state, state->AF.b.h, state->vHL->b.l));
OPCODE(0xAE, 7, 2, "XOR (HL)", XOR(state, state->AF.b.h, IADDR(state)));
OPCODE(0xAF, 4, 1, "XOR A", XOR(state, state->AF.b.h, state->AF.b.h));
OPCODE(0xB0, 4, 1, "OR B", OR(state, state->AF.b.h, state->BC.b.h));
OPCODE(0xB1, 4, 1, "OR C", OR(state, state->AF.b.h, state->BC.b.l));
OPCODE(0xB2, 4, 1, "OR D", OR(state, state->AF.b.h, state->DE.b.h));
OPCODE(0xB3, 4, 1, "OR E", OR(state, state->AF.b.h, state->DE.b.l));
OPCODE(0xB4, 4, 1, "OR H", OR(state, state->AF.b.h, state->vHL->b.h));
OPCODE(0xB5, 4, 1, "OR L", OR(state, state->AF.b.h, state->vHL->b.l));
OPCODE(0xB6, 7, 1, "OR (HL)", OR(state, state->AF.b.h, IADDR(state)));
OPCODE(0xB7, 4, 1, "OR A", OR(state, state->AF.b.h, state->AF.b.h));
OPCODE(0xB8, 4, 1, "CP B", CP(state, state->AF.b.h, state->BC.b.h));
OPCODE(0xB9, 4, 1, "CP C", CP(state, state->AF.b.h, state->BC.b.l));
OPCODE(0xBA, 4, 1, "CP D", CP(state, state->AF.b.h, state->DE.b.h));
OPCODE(0xBB, 4, 1, "CP E", CP(state, state->AF.b.h, state->DE.b.l));
OPCODE(0xBC, 4, 1, "CP H", CP(state, state->AF.b.h, state->vHL->b.h));
OPCODE(0xBD, 4, 1, "CP L", CP(state, state->AF.b.h, state->vHL->b.l));
OPCODE(0xBE, 7, 1, "CP (HL)", CP(state, state->AF.b.h, IADDR(state)));
OPCODE(0xBF, 4, 1, "CP A", CP(state, state->AF.b.h, state->AF.b.h));
OPCODE(0xC0, 5, 1, "RET NZ", RET(state, !state->AF.b.f.Z));
OPCODE(0xC1, 10, 1, "POP BC", POP(state, state->BC.b.h, state->BC.b.l));
OPCODE(0xC2, 10, 0, "JP NZ,d16", JP(state, !state->AF.b.f.Z, D16(state)));
OPCODE(0xC3, 10, 0, "JP d16", JP(state, true, D16(state)));
OPCODE(0xC4, 10, 3, "CALL NZ,d16", CALL(state, !state->AF.b.f.Z, D16(state)));
OPCODE(0xC5, 11, 1, "PUSH BC", PUSH(state, state->BC.b.h, state->BC.b.l));
OPCODE(0xC6, 7, 2, "ADD A, d8", ADD(state, state->AF.b.h, D8(state)));
OPCODE(0xC7, 11, 1, "RST 00H", RST(state, 0x00));
OPCODE(0xC8, 5, 1, "RET Z", RET(state, state->AF.b.f.Z));
OPCODE(0xC9, 6, 1, "RET", RET(state, true));
OPCODE(0xCA, 10, 3, "JP Z, d16", JP(state, state->AF.b.f.Z, D16(state)));
OPCODE(0xCB, 0, 1, "PREFIX CB", DISPATCH_CB(state));
OPCODE(0xCC, 10, 3, "CALL Z, d16", CALL(state, state->AF.b.f.Z, D16(state)));
OPCODE(0xCD, 10, 3, "CALL d16", CALL(state, true, D16(state)));
OPCODE(0xCE, 7, 2, "ADC A, d8", ADC(state, state->AF.b.h, D8(state)));
OPCODE(0xCF, 11, 1, "RST 08H", RST(state, 0x08));
OPCODE(0xD0, 5, 1, "RET NC", RET(state, !state->AF.b.f.C));
OPCODE(0xD1, 10, 1, "POP DE", POP(state, state->DE.b.h, state->DE.b.l));
OPCODE(0xD2, 10, 0, "JP NC", JP(state, !state->AF.b.f.C, D16(state)));
OPCODE(0xD3, 11, 2, "OUT d8, A", OUT(state, D8(state), state->AF.b.h));
OPCODE(0xD4, 10, 3, "CALL NC, d16", CALL(state, !state->AF.b.f.C, D16(state)));
OPCODE(0xD5, 11, 1, "PUSH DE", PUSH(state, state->DE.b.h, state->DE.b.l));
OPCODE(0xD6, 7, 2, "SUB A, d8", SUB(state, state->AF.b.h, D8(state)));
OPCODE(0xD7, 11, 1, "RST 10H", RST(state, 0x10));
OPCODE(0xD8, 5, 1, "RET C", RET(state, state->AF.b.f.C));
OPCODE(0xD9, 4, 1, "EXX BC DE HL", EXX(state));
OPCODE(0xDA, 10, 3, "JP C, d16", JP(state, state->AF.b.f.C, D16(state)));
OPCODE(0xDB, 11, 2, "IN A, d8", IN(state, state->AF.b.h, D8(state)));
OPCODE(0xDC, 10, 3, "CALL C, d16", CALL(state, state->AF.b.f.C, D16(state)));
OPCODE(0xDD, 0, 0, "EXTD", DISPATCH_ED(state));
OPCODE(0xDE, 7, 2, "SBC A, d8", SBC(state, state->AF.b.h, D8(state)));
OPCODE(0xDF, 11, 1, "RST 18H", RST(state, 0x18));
OPCODE(0xE0, 5, 1, "RET PO", RET(state, !state->AF.b.f.V));
OPCODE(0xE1, 10, 1, "POP HL", POP(state, state->vHL->b.h, state->vHL->b.l));
OPCODE(0xE2, 10, 3, "JP PO, a16", JP(state, !state->AF.b.f.V, D16(state)));
OPCODE(0xE3, 19, 1, "EX (SP),HL",
       EXI(state, state->SP.d, state->vHL->b.h, state->vHL->b.l));
OPCODE(0xE4, 10, 3, "CALL PO, a16", CALL(state, !state->AF.b.f.V, D16(state)));
OPCODE(0xE5, 16, 1, "PUSH HL", PUSH(state, state->vHL->b.h, state->vHL->b.l));
OPCODE(0xE6, 7, 2, "AND d8", AND(state, state->AF.b.h, D8(state)));
OPCODE(0xE7, 16, 1, "RST 20H", RST(state, 0x20));
OPCODE(0xE8, 5, 1, "RET PE", RET(state, state->AF.b.f.V));
OPCODE(0xE9, 4, 1, "JP HL", JP(state, true, state->vHL->d));
OPCODE(0xEA, 4, 3, "JP PE, a16", JP(state, state->AF.b.f.V, D16(state)));
OPCODE(0xEB, 4, 1, "EX DE, HL", EX(state, state->DE.d, state->HL.d));
OPCODE(0xEC, 10, 3, "CALL PE, a16", CALL(state, state->AF.b.f.V, D16(state)));
OPCODE(0xED, 0, 0, "ED", DISPATCH_ED(state));
OPCODE(0xEE, 7, 1, "XOR d8", XOR(state, state->AF.b.h, D8(state)));
OPCODE(0xEF, 11, 1, "RST 28H", RST(state, 0x28));
OPCODE(0xF0, 5, 1, "RET P", RET(state, !state->AF.b.f.S));
OPCODE(0xF1, 10, 1, "POP AF", POP(state, state->AF.b.h, state->AF.b.l));
OPCODE(0xF2, 10, 3, "JP P, a16", JP(state, !state->AF.b.f.S, D16(state)));
OPCODE(0xF3, 4, 1, "DI", DI(state));
OPCODE(0xF4, 10, 3, "CALL P, a16", CALL(state, !state->AF.b.f.S, D16(state)));
OPCODE(0xF5, 11, 1, "PUSH AF", PUSH(state, state->AF.b.h, state->AF.b.l));
OPCODE(0xF6, 7, 2, "OR d8", OR(state, state->AF.b.h, D8(state)));
OPCODE(0xF7, 11, 1, "RST 30H", RST(state, 0x30));
OPCODE(0xF8, 5, 1, "RET M", RET(state, state->AF.b.f.S));
OPCODE(0xF9, 6, 1, "LD SP,HL", LD16(state, state->SP, state->vHL->d));
OPCODE(0xFA, 4, 3, "JP M, a16", JP(state, state->AF.b.f.S, D16(state)));
OPCODE(0xFB, 4, 1, "EI", EI(state));
OPCODE(0xFC, 10, 3, "CALL M, a16", CALL(state, state->AF.b.f.S, D16(state)));
OPCODE(0xFD, 0, 0, "FD", DISPATCH_ED(state));
OPCODE(0xFE, 7, 2, "CP d8", CP(state, state->AF.b.h, D8(state)));
OPCODE(0xFF, 11, 1, "RST 38H", RST(state, 0x38));

#define OPCODE_DEF(prefix, num)                                       \
  [0x##num] = {0x##num, name_0x##prefix##num, cycles_0x##prefix##num, \
               bytes_0x##prefix##num, func_0x##prefix##num}

#define OPCODE16(prefix, num)                                 \
  OPCODE_DEF(prefix, num##0)                                  \
  , OPCODE_DEF(prefix, num##1), OPCODE_DEF(prefix, num##2),   \
      OPCODE_DEF(prefix, num##3), OPCODE_DEF(prefix, num##4), \
      OPCODE_DEF(prefix, num##5), OPCODE_DEF(prefix, num##6), \
      OPCODE_DEF(prefix, num##7), OPCODE_DEF(prefix, num##8), \
      OPCODE_DEF(prefix, num##9), OPCODE_DEF(prefix, num##A), \
      OPCODE_DEF(prefix, num##B), OPCODE_DEF(prefix, num##C), \
      OPCODE_DEF(prefix, num##D), OPCODE_DEF(prefix, num##E), \
      OPCODE_DEF(prefix, num##F)

static Z80Opcode opcodes[256] = {
    OPCODE16(, 0), OPCODE16(, 1), OPCODE16(, 2), OPCODE16(, 3),
    OPCODE16(, 4), OPCODE16(, 5), OPCODE16(, 6), OPCODE16(, 7),
    OPCODE16(, 8), OPCODE16(, 9), OPCODE16(, A), OPCODE16(, B),
    OPCODE16(, C), OPCODE16(, D), OPCODE16(, E), OPCODE16(, F)};

OPCODE(0xED00, 5, 2, "NOP", );
OPCODE(0xED40, 12, 2, "IN B, (C)", IN(state, state->BC.b.h, state->BC.b.l));
OPCODE(0xED41, 12, 2, "OUT (C), B", OUT(state, state->BC.b.l, state->BC.b.h));
OPCODE(0xED42, 15, 2, "SBC HL, BC", SBC16(state, state->HL.d, state->BC.d));
OPCODE(0xED43, 20, 4, "LD (d16), BC", LD16I(state, D16(state), state->BC.d));
OPCODE(0xED44, 8, 2, "NEG", NEG(state, state->AF.b.h));
OPCODE(0xED45, 14, 2, "RET N", RETN(state));
OPCODE(0xED47, 9, 2, "LD I, A", LD(state, state->I, state->AF.b.h));
OPCODE(0xED48, 12, 2, "IN C, (C)", IN(state, state->BC.b.l, state->BC.b.l));
OPCODE(0xED49, 12, 2, "OUT (C), C", OUT(state, state->BC.b.l, state->BC.b.l));
OPCODE(0xED4A, 15, 2, "ADC HL, BC", ADC16(state, state->HL.d, state->BC.d));
OPCODE(0xED4B, 20, 2, "LD BC, (d16)", LD16(state, state->BC, I16(state)));
OPCODE(0xED4C, 8, 2, "NEG", NEG(state, state->AF.b.h));
OPCODE(0xED4D, 14, 2, "RET I", RETI(state));
OPCODE(0xED4F, 9, 2, "LD R, A", LD(state, state->R, state->AF.b.h));
OPCODE(0xED50, 12, 2, "IN D, (C)", IN(state, state->DE.b.h, state->BC.b.l));
OPCODE(0xED51, 12, 2, "OUT (C), D", OUT(state, state->BC.b.l, state->DE.b.h));
OPCODE(0xED52, 15, 2, "SBC HL, DE", SBC16(state, state->HL.d, state->DE.d));
OPCODE(0xED53, 20, 4, "LD (d16), DE", LD16I(state, D16(state), state->DE.d));
OPCODE(0xED54, 8, 2, "NEG", NEG(state, state->AF.b.h));
OPCODE(0xED55, 14, 2, "RET N", RETN(state));
OPCODE(0xED56, 8, 2, "IM 1", IM(state, 1));
OPCODE(0xED58, 12, 2, "IN E, (C)", IN(state, state->DE.b.l, state->BC.b.l));
OPCODE(0xED59, 12, 2, "OUT (C), E", OUT(state, state->BC.b.l, state->DE.b.l));
OPCODE(0xED5A, 15, 2, "ADC HL, DE", ADC16(state, state->HL.d, state->DE.d));
OPCODE(0xED5B, 20, 2, "LD DE, (d16)", LD16(state, state->DE, I16(state)));
OPCODE(0xED5C, 8, 2, "NEG", NEG(state, state->AF.b.h));
OPCODE(0xED5D, 14, 2, "RET N", RETN(state));
OPCODE(0xED5E, 8, 2, "IM 2", IM(state, 2));
OPCODE(0xED5F, 9, 2, "LD A, R", LD(state, state->AF.b.h, state->R));
OPCODE(0xED60, 12, 2, "IN H, (C)", IN(state, state->HL.b.h, state->BC.b.l));
OPCODE(0xED61, 12, 2, "OUT (C), H", OUT(state, state->BC.b.l, state->HL.b.h));
OPCODE(0xED62, 15, 2, "SBC HL, HL", SBC16(state, state->HL.d, state->HL.d));
OPCODE(0xED63, 20, 4, "LD (d16), HL", LD16I(state, D16(state), state->HL.d));
OPCODE(0xED64, 8, 2, "NEG", NEG(state, state->AF.b.h));
OPCODE(0xED65, 14, 2, "RET N", RETN(state));
OPCODE(0xED67, 18, 2, "RRD", RRD(state));
OPCODE(0xED68, 12, 2, "IN L, (C)", IN(state, state->HL.b.l, state->BC.b.l));
OPCODE(0xED69, 12, 2, "OUT (C), L", OUT(state, state->BC.b.l, state->HL.b.l));
OPCODE(0xED6A, 15, 2, "ADC HL, HL", ADC16(state, state->HL.d, state->HL.d));
OPCODE(0xED6B, 20, 2, "LD HL, (d16)", LD16(state, state->HL, I16(state)));
OPCODE(0xED6C, 8, 2, "NEG", NEG(state, state->AF.b.h));
OPCODE(0xED6D, 14, 2, "RET N", RETN(state));
OPCODE(0xED6F, 18, 2, "RLD", RLD(state));
OPCODE(0xED70, 12, 2, "IN (C)", IN(state, state->HL.b.h, state->BC.b.l));
OPCODE(0xED71, 12, 2, "OUT (C), 0", OUT(state, state->BC.b.l, 0));
OPCODE(0xED72, 15, 2, "SBC HL, SP", SBC16(state, state->HL.d, state->SP.d));
OPCODE(0xED73, 20, 4, "LD (d16), SP", LD16I(state, D16(state), state->SP.d));
OPCODE(0xED74, 8, 2, "NEG", NEG(state, state->AF.b.h));
OPCODE(0xED75, 14, 2, "RET N", RETN(state));
OPCODE(0xED78, 12, 2, "IN A, (C)", IN(state, state->AF.b.h, state->BC.b.l));
OPCODE(0xED79, 12, 2, "OUT (C), A", OUT(state, state->BC.b.l, state->AF.b.h));
OPCODE(0xED7A, 15, 2, "ADC HL, SP", ADC16(state, state->HL.d, state->SP.d));
OPCODE(0xED7B, 20, 2, "LD SP, (d16)", LD16(state, state->SP, I16(state)));
OPCODE(0xED7C, 8, 2, "NEG", NEG(state, state->AF.b.h));
OPCODE(0xED7D, 14, 2, "RET N", RETN(state));
OPCODE(0xEDA0, 16, 2, "LDI (HL) (DE) BC", LDI(state));
OPCODE(0xEDA1, 16, 2, "CPI", CPI(state));
OPCODE(0xEDA8, 16, 2, "LDD (HL) (DE) BC", LDD(state));
OPCODE(0xEDA9, 16, 2, "CPD", CPD(state));
OPCODE(0xEDB0, 16, 2, "LDIR", LDIR(state));
OPCODE(0xEDB1, 16, 2, "CPIR", CPIR(state));
OPCODE(0xEDB3, 16, 2, "OTIR", OTIR(state));
OPCODE(0xEDB8, 16, 2, "LDDR", LDDR(state));
OPCODE(0xEDB9, 16, 2, "CPDR", CPDR(state));

static Z80Opcode EDopcodes[256] = {
    OPCODE_DEF(ED, 00), OPCODE_DEF(ED, 40), OPCODE_DEF(ED, 41),
    OPCODE_DEF(ED, 42), OPCODE_DEF(ED, 43), OPCODE_DEF(ED, 44),
    OPCODE_DEF(ED, 45), OPCODE_DEF(ED, 47), OPCODE_DEF(ED, 48),
    OPCODE_DEF(ED, 49), OPCODE_DEF(ED, 4A), OPCODE_DEF(ED, 4B),
    OPCODE_DEF(ED, 4C), OPCODE_DEF(ED, 4D), OPCODE_DEF(ED, 4F),
    OPCODE_DEF(ED, 50), OPCODE_DEF(ED, 51), OPCODE_DEF(ED, 52),
    OPCODE_DEF(ED, 53), OPCODE_DEF(ED, 54), OPCODE_DEF(ED, 55),
    OPCODE_DEF(ED, 56), OPCODE_DEF(ED, 58), OPCODE_DEF(ED, 59),
    OPCODE_DEF(ED, 5A), OPCODE_DEF(ED, 5B), OPCODE_DEF(ED, 5C),
    OPCODE_DEF(ED, 5D), OPCODE_DEF(ED, 5E), OPCODE_DEF(ED, 5F),
    OPCODE_DEF(ED, 60), OPCODE_DEF(ED, 61), OPCODE_DEF(ED, 62),
    OPCODE_DEF(ED, 63), OPCODE_DEF(ED, 64), OPCODE_DEF(ED, 65),
    OPCODE_DEF(ED, 67), OPCODE_DEF(ED, 68), OPCODE_DEF(ED, 69),
    OPCODE_DEF(ED, 6A), OPCODE_DEF(ED, 6B), OPCODE_DEF(ED, 6C),
    OPCODE_DEF(ED, 6D), OPCODE_DEF(ED, 6F), OPCODE_DEF(ED, 70),
    OPCODE_DEF(ED, 71), OPCODE_DEF(ED, 72), OPCODE_DEF(ED, 73),
    OPCODE_DEF(ED, 74), OPCODE_DEF(ED, 75), OPCODE_DEF(ED, 78),
    OPCODE_DEF(ED, 79), OPCODE_DEF(ED, 7A), OPCODE_DEF(ED, 7B),
    OPCODE_DEF(ED, 7C), OPCODE_DEF(ED, 7D), OPCODE_DEF(ED, A0),
    OPCODE_DEF(ED, A1), OPCODE_DEF(ED, A8), OPCODE_DEF(ED, A9),
    OPCODE_DEF(ED, B0), OPCODE_DEF(ED, B1), OPCODE_DEF(ED, B3),
    OPCODE_DEF(ED, B8), OPCODE_DEF(ED, B9),
};

#define OPCODE_SWITCH(op, state) \
  case op: {                     \
    func_##op(state);            \
    break;                       \
  }

#define OPCODE_SWITCH16(prefix, state)                                        \
  OPCODE_SWITCH(prefix##0, state)                                             \
  OPCODE_SWITCH(prefix##1, state) OPCODE_SWITCH(prefix##2, state)             \
      OPCODE_SWITCH(prefix##3, state) OPCODE_SWITCH(prefix##4, state)         \
          OPCODE_SWITCH(prefix##5, state) OPCODE_SWITCH(prefix##6, state)     \
              OPCODE_SWITCH(prefix##7, state) OPCODE_SWITCH(prefix##8, state) \
                  OPCODE_SWITCH(prefix##9, state)                             \
                      OPCODE_SWITCH(prefix##A, state)                         \
                          OPCODE_SWITCH(prefix##B, state)                     \
                              OPCODE_SWITCH(prefix##C, state)                 \
                                  OPCODE_SWITCH(prefix##D, state)             \
                                      OPCODE_SWITCH(prefix##E, state)         \
                                          OPCODE_SWITCH(prefix##F, state)

#define OPCODE_SWITCH_BLOCK(state)                                           \
  switch (state->latch_op) {                                                 \
    OPCODE_SWITCH16(0x0, state)                                              \
    OPCODE_SWITCH16(0x1, state) OPCODE_SWITCH16(0x2, state) OPCODE_SWITCH16( \
        0x3, state) OPCODE_SWITCH16(0x4, state) OPCODE_SWITCH16(0x5, state)  \
        OPCODE_SWITCH16(0x6, state) OPCODE_SWITCH16(0x7, state)              \
            OPCODE_SWITCH16(0x8, state) OPCODE_SWITCH16(0x9, state)          \
                OPCODE_SWITCH16(0xA, state) OPCODE_SWITCH16(0xB, state)      \
                    OPCODE_SWITCH16(0xC, state) OPCODE_SWITCH16(0xD, state)  \
                        OPCODE_SWITCH16(0xE, state)                          \
                            OPCODE_SWITCH16(0xF, state)                      \
  }

Z80Cpu::Z80Cpu(Machine *machine, const std::string &name, unsigned hertz,
               state_type *state)
    : Cpu(machine, name, hertz, state) {}

Z80Cpu::~Z80Cpu(void) {}

void Z80Cpu::interrupt(addr_t addr) {
  DEVICE_DEBUG("Interrupt: ", Hex(addr));
  PUSH(m_state, m_state->PC.b.h, m_state->PC.b.l);
  m_state->PC.d = addr;
  m_state->iff2 = m_state->iff1;
  m_state->iff1 = false;
  add_icycles(20);
}

void Z80Cpu::execute(void) {
  while (true) {
      {
      {
        if (m_state->reset_line == LineState::Pulse) {
          m_state->reset();
          m_state->reset_line = LineState::Clear;
          m_state->halt = false;
        } else if (m_state->nmi_line == LineState::Pulse) {
          interrupt(0x0066);
          m_state->nmi_line = LineState::Clear;
          m_state->halt = false;
        } else if (m_state->int0_line == LineState::Assert && m_state->iff1 &&
                   !m_state->iwait) {
          switch (m_state->imode) {
            case 0:
              throw CpuFault(name(), "Unsupported Interrupt mode 0");
              break;
            case 1:
              interrupt(0x0038);
              break;
            case 2: {
              // XXX: We should do this when we read _data
              m_state->int0_line = LineState::Clear;
              reg16_t irq;
              irq.b.l =
                  m_state->bus_read(m_state, (m_state->I << 8) | m_state->data);
              irq.b.h = m_state->bus_read(
                  m_state, ((m_state->I << 8) | m_state->data) + 1);
              interrupt(irq.d);
            }
          }
          m_state->halt = false;
        }

        if (unlikely(m_state->iwait)) m_state->iwait = false;

        if (unlikely(m_state->yield)) {
          yield();
          m_state->yield = false;
        }
        if (m_state->halt) {
          add_icycles(100);
          continue;
        }
      }
      {
        m_state->latch_pc = m_state->PC;
        m_state->latch_op = pc_read(m_state);
        if (m_state->latch_op == 0xDD) {
          m_state->vHL = &m_state->IX;
          m_state->prefix = Z80Prefix(m_state->latch_op);
          m_state->latch_op = pc_read(m_state);
          m_state->Op = &opcodes[m_state->latch_op];
        } else if (m_state->latch_op == 0xFD) {
          m_state->vHL = &m_state->IY;
          m_state->prefix = Z80Prefix(m_state->latch_op);
          m_state->latch_op = pc_read(m_state);
          m_state->Op = &opcodes[m_state->latch_op];
        } else if (m_state->latch_op == 0xED) {
          m_state->vHL = &m_state->HL;
          m_state->prefix = Z80Prefix(m_state->latch_op);
          m_state->latch_op = pc_read(m_state);
          m_state->Op = &EDopcodes[m_state->latch_op];
        } else {
          m_state->vHL = &m_state->HL;
          m_state->prefix = Z80Prefix::NoPrefix;
          m_state->Op = &opcodes[m_state->latch_op];
        }
      }
      {
        m_state->icycles = m_state->Op->cycles;
#if 0
        if (m_state->prefix != Z80Prefix::EDPrefix) {
          OPCODE_SWITCH_BLOCK(m_state);
        } else
#endif
          m_state->Op->func(m_state);
        IF_LOG(Trace) LOG_TRACE(Log(m_state));
        add_icycles(m_state->icycles);
      }
    }
  }
}

std::string Z80Cpu::Log(Z80State *state) {
  std::stringstream os;
  const std::string &str = state->Op->name;
  os << Hex(state->latch_pc) << ":";
  const std::string &delimiters = " ,";
  auto lastPos = str.find_first_not_of(delimiters, 0);
  auto pos = str.find_first_of(delimiters, lastPos);
  std::stringstream op;
  while (std::string::npos != pos || std::string::npos != lastPos) {
    std::string it = str.substr(lastPos, pos - lastPos);
    op << " ";
    if (it == "(IX+d)")
      op << "(IX+" << Hex(state->d8) << ")";
    else if (it == "(IY+d)")
      op << "(IY+" << Hex(state->d8) << ")";
    else if (it == "d16" || it == "a16")
      op << Hex(state->d16);
    else if (it == "HL") {
      switch (state->prefix) {
        case Z80Prefix::DDPrefix:
          op << "IX";
          break;
        case Z80Prefix::FDPrefix:
          op << "IY";
          break;
        default:
          op << "HL";
          break;
      }
    } else if (it == "L") {
      switch (state->prefix) {
        case Z80Prefix::DDPrefix:
          op << "IXl";
          break;
        case Z80Prefix::FDPrefix:
          op << "IYl";
          break;
        default:
          op << "L";
          break;
      }
    } else if (it == "H") {
      switch (state->prefix) {
        case Z80Prefix::DDPrefix:
          op << "IXh";
          break;
        case Z80Prefix::FDPrefix:
          op << "IYh";
          break;
        default:
          op << "H";
          break;
      }
    } else if (it == "(d16)")
      op << "(" << Hex(state->d16) << ")";
    else if (it == "(d8)")
      op << "(" << Hex(state->d8) << ")";
    else if (it == "d8" || it == "r8")
      op << Hex(state->d8);
    else
      op << it;
    lastPos = str.find_first_not_of(delimiters, pos);
    pos = str.find_first_of(delimiters, lastPos);
  }
  os << std::setfill(' ') << std::left << std::setw(20) << op.str();
  os << "  CPU:";
  os << " PC:" << Hex(state->PC.d);
  os << ",AF:" << Hex(state->AF.d);
  os << ",BC:" << Hex(state->BC.d);
  os << ",DE:" << Hex(state->DE.d);
  os << ",HL:" << Hex(state->HL.d);
  os << ",IX:" << Hex(state->IX.d);
  os << ",IY:" << Hex(state->IY.d);
  os << ",SP:" << Hex(state->SP.d);

  return os.str();
}

void Z80Cpu::line(Line line, LineState state) {
  switch (line) {
    case Line::RESET:
      m_state->reset_line = state;
      break;
    case Line::INT0:
      m_state->int0_line = state;
      break;
    case Line::NMI:
      m_state->nmi_line = state;
      break;
    case Line::WAIT:
      m_state->wait_line = state;
      break;
    default:
      break;
  }
}
