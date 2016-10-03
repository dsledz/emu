/*
 * Copyright (c) 2016, Dan Sledz
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

#include "cpu/z80/lr35902.h"
#include "cpu/z80/z80_ops.h"
#include "cpu/lib/cpu2_macros.h"

using namespace LR35902;

OP(SWAP, byte_t &dest) {
  dest = (dest >> 4) | (dest << 4);

  state->AF.b.f.Z = (dest & 0xff) == 0;
  state->AF.b.f.H = 0;
  state->AF.b.f.N = 1;
  state->AF.b.f.C = 0;
}

OP(SWAP_HL) {
  byte_t value = bus_read(state, state->HL.d);
  SWAP(state, value);
  bus_write(state, state->HL.d, value);
}

#define OPCODE(opnum, cycles, bytes, name, code)     \
  template <typename CpuState>                       \
  static inline void func_##opnum(CpuState *state);  \
  static const char *name_##opnum = name;            \
  static const Cycles cycles_##opnum(cycles);        \
  static const uint8_t bytes_##opnum = bytes;        \
  template <typename CpuState>                       \
  static inline void func_##opnum(CpuState *state) { \
    code;                                            \
  }

OPCODE(0x00, 4, 1, "NOP", );
OPCODE(0x01, 10, 3, "LD BC,d16", LD16(state, state->BC, D16(state)));
OPCODE(0x02, 8, 1, "LD (BC),A", {
  state->WZ.b.l = (state->BC.b.l + 1) & 0xff;
  state->WZ.b.h = state->AF.b.h;
  LDMEM(state, state->BC.d, state->AF.b.h);
});
OPCODE(0x03, 8, 1, "INC BC", INC16(state, state->BC));
OPCODE(0x04, 4, 1, "INC B", INC(state, state->BC.b.h));
OPCODE(0x05, 4, 1, "DEC B", DEC(state, state->BC.b.h));
OPCODE(0x06, 8, 2, "LD B, d8", LD(state, state->BC.b.h, D8(state)));
OPCODE(0x07, 4, 1, "RLCA", RLCA(state));
OPCODE(0x08, 20, 3, "LD (d16), SP", {
  state->WZ.d = D16(state);
  state->WZ.d++;
  LD16I(state, state->d16.d, state->SP.d);
});
OPCODE(0x09, 8, 1, "ADD HL,BC", ADD16(state, *state->vHL, state->BC.d));
OPCODE(0x0A, 8, 1, "LD A,(BC)", {
  LD(state, state->AF.b.h, bus_read(state, state->BC.d));
  state->WZ.d = state->BC.d + 1;
});
OPCODE(0x0B, 8, 1, "DEC BC", DEC16(state, state->BC));
OPCODE(0x0C, 4, 1, "INC C", INC(state, state->BC.b.l));
OPCODE(0x0D, 4, 1, "DEC C", DEC(state, state->BC.b.l));
OPCODE(0x0E, 8, 2, "LD C, d8", LD(state, state->BC.b.l, D8(state)));
OPCODE(0x0F, 4, 1, "RRCA", RRCA(state));
OPCODE(0x10, 4, 2, "STOP", abort());
OPCODE(0x11, 10, 3, "LD DE,d16", LD16(state, state->DE, D16(state)));
OPCODE(0x12, 8, 1, "LD (DE),A", {
  state->WZ.b.l = (state->DE.b.l + 1) & 0xff;
  state->WZ.b.h = state->AF.b.h;
  LDMEM(state, state->DE.d, state->AF.b.h);
});
OPCODE(0x13, 8, 1, "INC DE", INC16(state, state->DE));
OPCODE(0x14, 4, 1, "INC D", INC(state, state->DE.b.h));
OPCODE(0x15, 4, 1, "DEC D", DEC(state, state->DE.b.h));
OPCODE(0x16, 8, 2, "LD D, d8", LD(state, state->DE.b.h, D8(state)));
OPCODE(0x17, 4, 1, "RLA", RLA(state));
OPCODE(0x18, 12, 2, "JR r8", JR(state, true, D8(state)));
OPCODE(0x19, 8, 1, "ADD HL,DE", ADD16(state, *state->vHL, state->DE.d));
OPCODE(0x1A, 8, 1, "LD A,(DE)", {
  LD(state, state->AF.b.h, bus_read(state, state->DE.d));
  state->WZ.d = state->DE.d + 1;
});
OPCODE(0x1B, 8, 1, "DEC DE", DEC16(state, state->DE));
OPCODE(0x1C, 4, 1, "INC E", INC(state, state->DE.b.l));
OPCODE(0x1D, 4, 1, "DEC E", DEC(state, state->DE.b.l));
OPCODE(0x1E, 8, 2, "LD E, d8", LD(state, state->DE.b.l, D8(state)));
OPCODE(0x1F, 4, 1, "RRA", RRA(state));
OPCODE(0x20, 8, 2, "JR NZ,r8", JR(state, !state->AF.b.f.Z, D8(state)));
OPCODE(0x21, 10, 3, "LD HL,d16", LD16(state, *state->vHL, D16(state)));
OPCODE(0x22, 8, 1, "LD (HL+), A", LDMEM(state, ++state->vHL->d, state->AF.b.h));
OPCODE(0x23, 8, 1, "INC HL", INC16(state, *state->vHL));
OPCODE(0x24, 4, 1, "INC H", INC(state, state->vHL->b.h));
OPCODE(0x25, 4, 1, "DEC H", DEC(state, state->vHL->b.h));
OPCODE(0x26, 8, 2, "LD H, d8", LD(state, state->vHL->b.h, D8(state)));
OPCODE(0x27, 4, 1, "DAA", DAA(state));
OPCODE(0x28, 8, 2, "JR Z,r8", JR(state, state->AF.b.f.Z, D8(state)));
OPCODE(0x29, 8, 1, "ADD HL,HL", ADD16(state, *state->vHL, state->vHL->d));
OPCODE(0x2A, 8, 1, "LD A, (HL+)", LD(state, state->AF.b.h, I8(state, ++state->vHL->d)));
OPCODE(0x2B, 6, 1, "DEC HL", DEC16(state, *state->vHL));
OPCODE(0x2C, 4, 1, "INC L", INC(state, state->vHL->b.l));
OPCODE(0x2D, 4, 1, "DEC L", DEC(state, state->vHL->b.l));
OPCODE(0x2E, 8, 2, "LD L, d8", LD(state, state->vHL->b.l, D8(state)));
OPCODE(0x2F, 4, 2, "CPL", CPL(state));
OPCODE(0x30, 8, 2, "JR NC,r8", JR(state, !state->AF.b.f.C, D8(state)));
OPCODE(0x31, 10, 3, "LD SP, d16", LD16(state, state->SP, D16(state)));
OPCODE(0x32, 16, 1, "LD (HL-), A", LDMEM(state, --state->vHL->d, state->AF.b.h));
OPCODE(0x33, 6, 1, "INC SP", INC16(state, state->SP));
OPCODE(0x34, 12, 1, "INC (HL)", INCI(state, DADDR(state)));
OPCODE(0x35, 12, 1, "DEC (HL)", DECI(state, DADDR(state)));
OPCODE(0x36, 8, 2, "LD (HL), d8", LDMEM(state, DADDR(state), D8(state)));
OPCODE(0x37, 4, 1, "SCF", SCF(state));
OPCODE(0x38, 8, 2, "JR C,r8", JR(state, state->AF.b.f.C, D8(state)));
OPCODE(0x39, 8, 1, "ADD HL,SP", ADD16(state, *state->vHL, state->SP.d));
OPCODE(0x3A, 8, 1, "LD A, (HL+)", LD(state, state->AF.b.h, I8(state, --state->vHL->d)));
OPCODE(0x3B, 6, 1, "DEC SP", DEC16(state, state->SP));
OPCODE(0x3C, 4, 1, "INC A", INC(state, state->AF.b.h));
OPCODE(0x3D, 4, 1, "DEC A", DEC(state, state->AF.b.h));
OPCODE(0x3E, 8, 2, "LD A, d8", LD(state, state->AF.b.h, D8(state)));
OPCODE(0x3F, 4, 1, "CCF", CCF(state));
OPCODE(0x40, 4, 1, "LD B,B", LD(state, state->BC.b.h, state->BC.b.h));
OPCODE(0x41, 4, 1, "LD B,C", LD(state, state->BC.b.h, state->BC.b.l));
OPCODE(0x42, 4, 1, "LD B,D", LD(state, state->BC.b.h, state->DE.b.h));
OPCODE(0x43, 4, 1, "LD B,E", LD(state, state->BC.b.h, state->DE.b.l));
OPCODE(0x44, 4, 1, "LD B,H", LD(state, state->BC.b.h, state->vHL->b.h));
OPCODE(0x45, 4, 1, "LD B,L", LD(state, state->BC.b.h, state->vHL->b.l));
OPCODE(0x46, 8, 1, "LD B,(HL)", LD(state, state->BC.b.h, IADDR(state)));
OPCODE(0x47, 4, 1, "LD B,A", LD(state, state->BC.b.h, state->AF.b.h));
OPCODE(0x48, 4, 1, "LD C,B", LD(state, state->BC.b.l, state->BC.b.h));
OPCODE(0x49, 4, 1, "LD C,C", LD(state, state->BC.b.l, state->BC.b.l));
OPCODE(0x4A, 4, 1, "LD C,D", LD(state, state->BC.b.l, state->DE.b.h));
OPCODE(0x4B, 4, 1, "LD C,E", LD(state, state->BC.b.l, state->DE.b.l));
OPCODE(0x4C, 4, 1, "LD C,H", LD(state, state->BC.b.l, state->vHL->b.h));
OPCODE(0x4D, 4, 1, "LD C,L", LD(state, state->BC.b.l, state->vHL->b.l));
OPCODE(0x4E, 8, 1, "LD C,(HL)", LD(state, state->BC.b.l, IADDR(state)));
OPCODE(0x4F, 4, 1, "LD C,A", LD(state, state->BC.b.l, state->AF.b.h));
OPCODE(0x50, 4, 1, "LD D,B", LD(state, state->DE.b.h, state->BC.b.h));
OPCODE(0x51, 4, 1, "LD D,C", LD(state, state->DE.b.h, state->BC.b.l));
OPCODE(0x52, 4, 1, "LD D,D", LD(state, state->DE.b.h, state->DE.b.h));
OPCODE(0x53, 4, 1, "LD D,E", LD(state, state->DE.b.h, state->DE.b.l));
OPCODE(0x54, 4, 1, "LD D,H", LD(state, state->DE.b.h, state->vHL->b.h));
OPCODE(0x55, 4, 1, "LD D,L", LD(state, state->DE.b.h, state->vHL->b.l));
OPCODE(0x56, 8, 1, "LD D,(HL)", LD(state, state->DE.b.h, IADDR(state)));
OPCODE(0x57, 4, 1, "LD D,A", LD(state, state->DE.b.h, state->AF.b.h));
OPCODE(0x58, 4, 1, "LD E,B", LD(state, state->DE.b.l, state->BC.b.h));
OPCODE(0x59, 4, 1, "LD E,C", LD(state, state->DE.b.l, state->BC.b.l));
OPCODE(0x5A, 4, 1, "LD E,D", LD(state, state->DE.b.l, state->DE.b.h));
OPCODE(0x5B, 4, 1, "LD E,E", LD(state, state->DE.b.l, state->DE.b.l));
OPCODE(0x5C, 4, 1, "LD E,H", LD(state, state->DE.b.l, state->vHL->b.h));
OPCODE(0x5D, 4, 1, "LD E,L", LD(state, state->DE.b.l, state->vHL->b.l));
OPCODE(0x5E, 8, 1, "LD E,(HL)", LD(state, state->DE.b.l, IADDR(state)));
OPCODE(0x5F, 4, 1, "LD E,A", LD(state, state->DE.b.l, state->AF.b.h));
OPCODE(0x60, 4, 1, "LD H,B", LD(state, state->vHL->b.h, state->BC.b.h));
OPCODE(0x61, 4, 1, "LD H,C", LD(state, state->vHL->b.h, state->BC.b.l));
OPCODE(0x62, 4, 1, "LD H,D", LD(state, state->vHL->b.h, state->DE.b.h));
OPCODE(0x63, 4, 1, "LD H,E", LD(state, state->vHL->b.h, state->DE.b.l));
OPCODE(0x64, 4, 1, "LD H,H", LD(state, state->vHL->b.h, state->vHL->b.h));
OPCODE(0x65, 4, 1, "LD H,L", LD(state, state->vHL->b.h, state->vHL->b.l));
OPCODE(0x66, 8, 1, "LD H,(HL)", LD(state, state->HL.b.h, IADDR(state)));
OPCODE(0x67, 4, 1, "LD H,A", LD(state, state->vHL->b.h, state->AF.b.h));
OPCODE(0x68, 4, 1, "LD L,B", LD(state, state->vHL->b.l, state->BC.b.h));
OPCODE(0x69, 4, 1, "LD L,C", LD(state, state->vHL->b.l, state->BC.b.l));
OPCODE(0x6A, 4, 1, "LD L,D", LD(state, state->vHL->b.l, state->DE.b.h));
OPCODE(0x6B, 4, 1, "LD L,E", LD(state, state->vHL->b.l, state->DE.b.l));
OPCODE(0x6C, 4, 1, "LD L,H", LD(state, state->vHL->b.l, state->vHL->b.h));
OPCODE(0x6D, 4, 1, "LD L,L", LD(state, state->vHL->b.l, state->vHL->b.l));
OPCODE(0x6E, 8, 1, "LD L,(HL)", LD(state, state->HL.b.l, IADDR(state)));
OPCODE(0x6F, 4, 1, "LD L,A", LD(state, state->vHL->b.l, state->AF.b.h));
OPCODE(0x70, 8, 1, "LD (HL),B", LDMEM(state, DADDR(state), state->BC.b.h));
OPCODE(0x71, 8, 1, "LD (HL),C", LDMEM(state, DADDR(state), state->BC.b.l));
OPCODE(0x72, 8, 1, "LD (HL),D", LDMEM(state, DADDR(state), state->DE.b.h));
OPCODE(0x73, 8, 1, "LD (HL),E", LDMEM(state, DADDR(state), state->DE.b.l));
OPCODE(0x74, 8, 1, "LD (HL),H", LDMEM(state, DADDR(state), state->HL.b.h));
OPCODE(0x75, 8, 1, "LD (HL),L", LDMEM(state, DADDR(state), state->HL.b.l));
OPCODE(0x76, 4, 1, "HALT", HALT(state));
OPCODE(0x77, 8, 1, "LD (HL),A", LDMEM(state, DADDR(state), state->AF.b.h));
OPCODE(0x78, 4, 1, "LD A,B", LD(state, state->AF.b.h, state->BC.b.h));
OPCODE(0x79, 4, 1, "LD A,C", LD(state, state->AF.b.h, state->BC.b.l));
OPCODE(0x7A, 4, 1, "LD A,D", LD(state, state->AF.b.h, state->DE.b.h));
OPCODE(0x7B, 4, 1, "LD A,E", LD(state, state->AF.b.h, state->DE.b.l));
OPCODE(0x7C, 4, 1, "LD A,H", LD(state, state->AF.b.h, state->vHL->b.h));
OPCODE(0x7D, 4, 1, "LD A,L", LD(state, state->AF.b.h, state->vHL->b.l));
OPCODE(0x7E, 8, 1, "LD A,(HL)", LD(state, state->AF.b.h, IADDR(state)));
OPCODE(0x7F, 4, 1, "LD A,A", LD(state, state->AF.b.h, state->AF.b.h));
OPCODE(0x80, 4, 1, "ADD A,B", ADD(state, state->AF.b.h, state->BC.b.h));
OPCODE(0x81, 4, 1, "ADD A,C", ADD(state, state->AF.b.h, state->BC.b.l));
OPCODE(0x82, 4, 1, "ADD A,D", ADD(state, state->AF.b.h, state->DE.b.h));
OPCODE(0x83, 4, 1, "ADD A,E", ADD(state, state->AF.b.h, state->DE.b.l));
OPCODE(0x84, 4, 1, "ADD A,H", ADD(state, state->AF.b.h, state->vHL->b.h));
OPCODE(0x85, 4, 1, "ADD A,L", ADD(state, state->AF.b.h, state->vHL->b.l));
OPCODE(0x86, 8, 1, "ADD A,(HL)", ADD(state, state->AF.b.h, IADDR(state)));
OPCODE(0x87, 4, 1, "ADD A,A", ADD(state, state->AF.b.h, state->AF.b.h));
OPCODE(0x88, 8, 1, "ADC A,B", ADC(state, state->AF.b.h, state->BC.b.h));
OPCODE(0x89, 8, 1, "ADC A,C", ADC(state, state->AF.b.h, state->BC.b.l));
OPCODE(0x8A, 8, 1, "ADC A,D", ADC(state, state->AF.b.h, state->DE.b.h));
OPCODE(0x8B, 8, 1, "ADC A,E", ADC(state, state->AF.b.h, state->DE.b.l));
OPCODE(0x8C, 8, 1, "ADC A,H", ADC(state, state->AF.b.h, state->vHL->b.h));
OPCODE(0x8D, 8, 1, "ADC A,L", ADC(state, state->AF.b.h, state->vHL->b.l));
OPCODE(0x8E, 8, 1, "ADC A,(HL)", ADC(state, state->AF.b.h, IADDR(state)));
OPCODE(0x8F, 8, 1, "ADC A,A", ADC(state, state->AF.b.h, state->AF.b.h));
OPCODE(0x90, 4, 1, "SUB B", SUB(state, state->AF.b.h, state->BC.b.h));
OPCODE(0x91, 4, 1, "SUB C", SUB(state, state->AF.b.h, state->BC.b.l));
OPCODE(0x92, 4, 1, "SUB D", SUB(state, state->AF.b.h, state->DE.b.h));
OPCODE(0x93, 4, 1, "SUB E", SUB(state, state->AF.b.h, state->DE.b.l));
OPCODE(0x94, 4, 1, "SUB H", SUB(state, state->AF.b.h, state->vHL->b.h));
OPCODE(0x95, 4, 1, "SUB L", SUB(state, state->AF.b.h, state->vHL->b.l));
OPCODE(0x96, 8, 1, "SUB (HL)", SUB(state, state->AF.b.h, IADDR(state)));
OPCODE(0x97, 4, 1, "SUB A", SUB(state, state->AF.b.h, state->AF.b.h));
OPCODE(0x98, 8, 1, "SBC A,B", SBC(state, state->AF.b.h, state->BC.b.h));
OPCODE(0x99, 8, 1, "SBC A,C", SBC(state, state->AF.b.h, state->BC.b.l));
OPCODE(0x9A, 8, 1, "SBC A,D", SBC(state, state->AF.b.h, state->DE.b.h));
OPCODE(0x9B, 8, 1, "SBC A,E", SBC(state, state->AF.b.h, state->DE.b.l));
OPCODE(0x9C, 8, 1, "SBC A,H", SBC(state, state->AF.b.h, state->vHL->b.h));
OPCODE(0x9D, 8, 1, "SBC A,L", SBC(state, state->AF.b.h, state->vHL->b.l));
OPCODE(0x9E, 8, 1, "SBC A,(HL)", SBC(state, state->AF.b.h, IADDR(state)));
OPCODE(0x9F, 8, 1, "SBC A,A", SBC(state, state->AF.b.h, state->AF.b.h));
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
OPCODE(0xAE, 8, 2, "XOR (HL)", XOR(state, state->AF.b.h, IADDR(state)));
OPCODE(0xAF, 4, 1, "XOR A", XOR(state, state->AF.b.h, state->AF.b.h));
OPCODE(0xB0, 4, 1, "OR B", OR(state, state->AF.b.h, state->BC.b.h));
OPCODE(0xB1, 4, 1, "OR C", OR(state, state->AF.b.h, state->BC.b.l));
OPCODE(0xB2, 4, 1, "OR D", OR(state, state->AF.b.h, state->DE.b.h));
OPCODE(0xB3, 4, 1, "OR E", OR(state, state->AF.b.h, state->DE.b.l));
OPCODE(0xB4, 4, 1, "OR H", OR(state, state->AF.b.h, state->vHL->b.h));
OPCODE(0xB5, 4, 1, "OR L", OR(state, state->AF.b.h, state->vHL->b.l));
OPCODE(0xB6, 8, 1, "OR (HL)", OR(state, state->AF.b.h, IADDR(state)));
OPCODE(0xB7, 4, 1, "OR A", OR(state, state->AF.b.h, state->AF.b.h));
OPCODE(0xB8, 4, 1, "CP B", CP(state, state->AF.b.h, state->BC.b.h));
OPCODE(0xB9, 4, 1, "CP C", CP(state, state->AF.b.h, state->BC.b.l));
OPCODE(0xBA, 4, 1, "CP D", CP(state, state->AF.b.h, state->DE.b.h));
OPCODE(0xBB, 4, 1, "CP E", CP(state, state->AF.b.h, state->DE.b.l));
OPCODE(0xBC, 4, 1, "CP H", CP(state, state->AF.b.h, state->vHL->b.h));
OPCODE(0xBD, 4, 1, "CP L", CP(state, state->AF.b.h, state->vHL->b.l));
OPCODE(0xBE, 8, 1, "CP (HL)", CP(state, state->AF.b.h, IADDR(state)));
OPCODE(0xBF, 4, 1, "CP A", CP(state, state->AF.b.h, state->AF.b.h));
OPCODE(0xC0, 5, 1, "RET NZ", RET(state, !state->AF.b.f.Z));
OPCODE(0xC1, 10, 1, "POP BC", POP(state, state->BC.b.h, state->BC.b.l));
OPCODE(0xC2, 10, 0, "JP NZ,d16", JP(state, !state->AF.b.f.Z, D16(state)));
OPCODE(0xC3, 10, 0, "JP d16", JP(state, true, D16(state)));
OPCODE(0xC4, 10, 3, "CALL NZ,d16", CALL(state, !state->AF.b.f.Z, D16(state)));
OPCODE(0xC5, 11, 1, "PUSH BC", PUSH(state, state->BC.b.h, state->BC.b.l));
OPCODE(0xC6, 8, 2, "ADD A, d8", ADD(state, state->AF.b.h, D8(state)));
OPCODE(0xC7, 11, 1, "RST 00H", RST(state, 0x00));
OPCODE(0xC8, 5, 1, "RET Z", RET(state, state->AF.b.f.Z));
OPCODE(0xC9, 6, 1, "RET", RET(state, true));
OPCODE(0xCA, 10, 3, "JP Z, d16", JP(state, state->AF.b.f.Z, D16(state)));
OPCODE(0xCB, 4, 1, "INVALID", abort());
OPCODE(0xCC, 10, 3, "CALL Z, d16", CALL(state, state->AF.b.f.Z, D16(state)));
OPCODE(0xCD, 10, 3, "CALL d16", CALL(state, true, D16(state)));
OPCODE(0xCE, 8, 2, "ADC A, d8", ADC(state, state->AF.b.h, D8(state)));
OPCODE(0xCF, 11, 1, "RST 08H", RST(state, 0x08));
OPCODE(0xD0, 5, 1, "RET NC", RET(state, !state->AF.b.f.C));
OPCODE(0xD1, 10, 1, "POP DE", POP(state, state->DE.b.h, state->DE.b.l));
OPCODE(0xD2, 10, 0, "JP NC", JP(state, !state->AF.b.f.C, D16(state)));
OPCODE(0xD3, 4, 1, "INVALID", abort());
OPCODE(0xD4, 10, 3, "CALL NC, d16", CALL(state, !state->AF.b.f.C, D16(state)));
OPCODE(0xD5, 11, 1, "PUSH DE", PUSH(state, state->DE.b.h, state->DE.b.l));
OPCODE(0xD6, 8, 2, "SUB A, d8", SUB(state, state->AF.b.h, D8(state)));
OPCODE(0xD7, 11, 1, "RST 10H", RST(state, 0x10));
OPCODE(0xD8, 5, 1, "RET C", RET(state, state->AF.b.f.C));
OPCODE(0xD9, 16, 1, "RETI", RETI(state));
OPCODE(0xDA, 10, 3, "JP C, d16", JP(state, state->AF.b.f.C, D16(state)));
OPCODE(0xDB, 4, 1, "INVALID", abort());
OPCODE(0xDC, 10, 3, "CALL C, d16", CALL(state, state->AF.b.f.C, D16(state)));
OPCODE(0xDD, 4, 1, "INVALID", abort());
OPCODE(0xDE, 8, 2, "SBC A, d8", SBC(state, state->AF.b.h, D8(state)));
OPCODE(0xDF, 11, 1, "RST 18H", RST(state, 0x18));
OPCODE(0xE0, 12, 2, "LD (d8), A",
       LDMEM(state, pc_read(state) | 0xff00, state->AF.b.h));
OPCODE(0xE1, 10, 1, "POP HL", POP(state, state->vHL->b.h, state->vHL->b.l));
OPCODE(0xE2, 8, 2, "LD (C), A", LDMEM(state, state->BC.b.l | 0xff, state->AF.b.h));
OPCODE(0xE3, 4, 1, "INVALID", abort());
OPCODE(0xE4, 4, 1, "INVALID", abort());
OPCODE(0xE5, 16, 1, "PUSH HL", PUSH(state, state->vHL->b.h, state->vHL->b.l));
OPCODE(0xE6, 8, 2, "AND d8", AND(state, state->AF.b.h, D8(state)));
OPCODE(0xE7, 16, 1, "RST 20H", RST(state, 0x20));
OPCODE(0xE8, 16, 2, "ADD SP, r8", ADD16rel(state, state->SP, D8(state)));
OPCODE(0xE9, 4, 1, "JP HL", JP(state, true, state->vHL->d));
OPCODE(0xEA, 16, 3, "LD (d16), A", LDMEM(state, D16(state), state->AF.b.h));
OPCODE(0xEB, 4, 1, "INVALID", abort());
OPCODE(0xEC, 4, 1, "INVALID", abort());
OPCODE(0xED, 4, 1, "INVALID", abort());
OPCODE(0xEE, 8, 1, "XOR d8", XOR(state, state->AF.b.h, D8(state)));
OPCODE(0xEF, 11, 1, "RST 28H", RST(state, 0x28));
OPCODE(0xF0, 12, 2, "LD A, (d8)",
       LD(state, state->AF.b.h, I8(state, pc_read(state) | 0xff00)));
OPCODE(0xF1, 10, 1, "POP AF", POP(state, state->AF.b.h, state->AF.b.l));
OPCODE(0xF2, 8, 2, "LD A, (C)",
       LD(state, state->AF.b.h, I8(state, state->BC.b.l | 0xff00)));
OPCODE(0xF3, 4, 1, "DI", DI(state));
OPCODE(0xF4, 4, 1, "INVALID", abort());
OPCODE(0xF5, 11, 1, "PUSH AF", PUSH(state, state->AF.b.h, state->AF.b.l));
OPCODE(0xF6, 8, 2, "OR d8", OR(state, state->AF.b.h, D8(state)));
OPCODE(0xF7, 11, 1, "RST 30H", RST(state, 0x30));
OPCODE(0xF8, 12, 2, "LD HL,SP+r8", LD16(state, *state->vHL, state->SP.d + D8(state)));
OPCODE(0xF9, 6, 1, "LD SP,HL", LD16(state, state->SP, state->vHL->d));
OPCODE(0xFA, 16, 3, "LD A, (d16)",
       LD(state, state->AF.b.h, I8(state, D16(state))));
OPCODE(0xFB, 4, 1, "EI", EI(state));
OPCODE(0xFC, 4, 1, "INVALID", abort());
OPCODE(0xFD, 4, 1, "INVALID", abort());
OPCODE(0xFE, 8, 2, "CP d8", CP(state, state->AF.b.h, D8(state)));
OPCODE(0xFF, 11, 1, "RST 38H", RST(state, 0x38));

OPCODE(0xCB00, 8, 2, "RLC B", RLC(state, state->BC.b.h, state->BC.b.h));
OPCODE(0xCB01, 8, 2, "RLC C", RLC(state, state->BC.b.l, state->BC.b.l));
OPCODE(0xCB02, 8, 2, "RLC D", RLC(state, state->DE.b.h, state->DE.b.h));
OPCODE(0xCB03, 8, 2, "RLC E", RLC(state, state->DE.b.l, state->DE.b.l));
OPCODE(0xCB04, 8, 2, "RLC H", RLC(state, state->HL.b.h, state->HL.b.h));
OPCODE(0xCB05, 8, 2, "RLC L", RLC(state, state->HL.b.l, state->HL.b.l));
OPCODE(0xCB06, 15, 2, "RLC (HL)", RLC_HL(state));
OPCODE(0xCB07, 8, 2, "RLC A", RLC(state, state->AF.b.h, state->AF.b.h));
OPCODE(0xCB08, 8, 2, "RRC B", RRC(state, state->BC.b.h, state->BC.b.h));
OPCODE(0xCB09, 8, 2, "RRC C", RRC(state, state->BC.b.l, state->BC.b.l));
OPCODE(0xCB0A, 8, 2, "RRC D", RRC(state, state->DE.b.h, state->DE.b.h));
OPCODE(0xCB0B, 8, 2, "RRC E", RRC(state, state->DE.b.l, state->DE.b.l));
OPCODE(0xCB0C, 8, 2, "RRC H", RRC(state, state->HL.b.h, state->HL.b.h));
OPCODE(0xCB0D, 8, 2, "RRC L", RRC(state, state->HL.b.l, state->HL.b.l));
OPCODE(0xCB0E, 15, 2, "RRC (HL)", RRC_HL(state));
OPCODE(0xCB0F, 8, 2, "RRC A", RRC(state, state->AF.b.h, state->AF.b.h));
OPCODE(0xCB10, 8, 2, "RL B", RL(state, state->BC.b.h, state->BC.b.h));
OPCODE(0xCB11, 8, 2, "RL C", RL(state, state->BC.b.l, state->BC.b.l));
OPCODE(0xCB12, 8, 2, "RL D", RL(state, state->DE.b.h, state->DE.b.h));
OPCODE(0xCB13, 8, 2, "RL E", RL(state, state->DE.b.l, state->DE.b.l));
OPCODE(0xCB14, 8, 2, "RL H", RL(state, state->HL.b.h, state->HL.b.h));
OPCODE(0xCB15, 8, 2, "RL L", RL(state, state->HL.b.l, state->HL.b.l));
OPCODE(0xCB16, 15, 2, "RL (HL)", RL_HL(state));
OPCODE(0xCB17, 8, 2, "RL A", RL(state, state->AF.b.h, state->AF.b.h));
OPCODE(0xCB18, 8, 2, "RR B", RR(state, state->BC.b.h, state->BC.b.h));
OPCODE(0xCB19, 8, 2, "RR C", RR(state, state->BC.b.l, state->BC.b.l));
OPCODE(0xCB1A, 8, 2, "RR D", RR(state, state->DE.b.h, state->DE.b.h));
OPCODE(0xCB1B, 8, 2, "RR E", RR(state, state->DE.b.l, state->DE.b.l));
OPCODE(0xCB1C, 8, 2, "RR H", RR(state, state->HL.b.h, state->HL.b.h));
OPCODE(0xCB1D, 8, 2, "RR L", RR(state, state->HL.b.l, state->HL.b.l));
OPCODE(0xCB1E, 15, 2, "RR (HL)", RR_HL(state));
OPCODE(0xCB1F, 8, 2, "RR A", RR(state, state->AF.b.h, state->AF.b.h));
OPCODE(0xCB20, 8, 2, "SLA B", SLA(state, state->BC.b.h, state->BC.b.h));
OPCODE(0xCB21, 8, 2, "SLA C", SLA(state, state->BC.b.l, state->BC.b.l));
OPCODE(0xCB22, 8, 2, "SLA D", SLA(state, state->DE.b.h, state->DE.b.h));
OPCODE(0xCB23, 8, 2, "SLA E", SLA(state, state->DE.b.l, state->DE.b.l));
OPCODE(0xCB24, 8, 2, "SLA H", SLA(state, state->HL.b.h, state->HL.b.h));
OPCODE(0xCB25, 8, 2, "SLA L", SLA(state, state->HL.b.l, state->HL.b.l));
OPCODE(0xCB26, 15, 2, "SLA (HL)", SLA_HL(state));
OPCODE(0xCB27, 8, 2, "SLA A", SLA(state, state->AF.b.h, state->AF.b.h));
OPCODE(0xCB28, 8, 2, "SRA B", SRA(state, state->BC.b.h, state->BC.b.h));
OPCODE(0xCB29, 8, 2, "SRA C", SRA(state, state->BC.b.l, state->BC.b.l));
OPCODE(0xCB2A, 8, 2, "SRA D", SRA(state, state->DE.b.h, state->DE.b.h));
OPCODE(0xCB2B, 8, 2, "SRA E", SRA(state, state->DE.b.l, state->DE.b.l));
OPCODE(0xCB2C, 8, 2, "SRA H", SRA(state, state->HL.b.h, state->HL.b.h));
OPCODE(0xCB2D, 8, 2, "SRA L", SRA(state, state->HL.b.l, state->HL.b.l));
OPCODE(0xCB2E, 15, 2, "SRA (HL)", SRA_HL(state));
OPCODE(0xCB2F, 8, 2, "SRA A", SRA(state, state->AF.b.h, state->AF.b.h));
OPCODE(0xCB30, 8, 2, "SWAP B", SWAP(state, state->BC.b.h));
OPCODE(0xCB31, 8, 2, "SWAP C", SWAP(state, state->BC.b.l));
OPCODE(0xCB32, 8, 2, "SWAP D", SWAP(state, state->DE.b.h));
OPCODE(0xCB33, 8, 2, "SWAP E", SWAP(state, state->DE.b.l));
OPCODE(0xCB34, 8, 2, "SWAP H", SWAP(state, state->HL.b.h));
OPCODE(0xCB35, 8, 2, "SWAP L", SWAP(state, state->HL.b.l));
OPCODE(0xCB36, 15, 2, "SWAP (HL)", SWAP_HL(state));
OPCODE(0xCB37, 8, 2, "SWAP A", SWAP(state, state->AF.b.h));
OPCODE(0xCB38, 8, 2, "SRL B", SRL(state, state->BC.b.h, state->BC.b.h));
OPCODE(0xCB39, 8, 2, "SRL C", SRL(state, state->BC.b.l, state->BC.b.l));
OPCODE(0xCB3A, 8, 2, "SRL D", SRL(state, state->DE.b.h, state->DE.b.h));
OPCODE(0xCB3B, 8, 2, "SRL E", SRL(state, state->DE.b.l, state->DE.b.l));
OPCODE(0xCB3C, 8, 2, "SRL H", SRL(state, state->HL.b.h, state->HL.b.h));
OPCODE(0xCB3D, 8, 2, "SRL L", SRL(state, state->HL.b.l, state->HL.b.l));
OPCODE(0xCB3E, 15, 2, "SRL (HL)", SRL_HL(state));
OPCODE(0xCB3F, 8, 2, "SRL A", SRL(state, state->AF.b.h, state->AF.b.h));
OPCODE(0xCB40, 8, 2, "BIT 0, B", BIT_TEST(state, state->BC.b.h, 0));
OPCODE(0xCB41, 8, 2, "BIT 0, C", BIT_TEST(state, state->BC.b.l, 0));
OPCODE(0xCB42, 8, 2, "BIT 0, D", BIT_TEST(state, state->DE.b.h, 0));
OPCODE(0xCB43, 8, 2, "BIT 0, E", BIT_TEST(state, state->DE.b.l, 0));
OPCODE(0xCB44, 8, 2, "BIT 0, H", BIT_TEST(state, state->HL.b.h, 0));
OPCODE(0xCB45, 8, 2, "BIT 0, L", BIT_TEST(state, state->HL.b.l, 0));
OPCODE(0xCB46, 12, 2, "BIT 0, (HL)", BIT_TEST_HL(state, 0));
OPCODE(0xCB47, 8, 2, "BIT 0, A", BIT_TEST(state, state->AF.b.h, 0));
OPCODE(0xCB48, 8, 2, "BIT 1, B", BIT_TEST(state, state->BC.b.h, 1));
OPCODE(0xCB49, 8, 2, "BIT 1, C", BIT_TEST(state, state->BC.b.l, 1));
OPCODE(0xCB4A, 8, 2, "BIT 1, D", BIT_TEST(state, state->DE.b.h, 1));
OPCODE(0xCB4B, 8, 2, "BIT 1, E", BIT_TEST(state, state->DE.b.l, 1));
OPCODE(0xCB4C, 8, 2, "BIT 1, H", BIT_TEST(state, state->HL.b.h, 1));
OPCODE(0xCB4D, 8, 2, "BIT 1, L", BIT_TEST(state, state->HL.b.l, 1));
OPCODE(0xCB4E, 12, 2, "BIT 1, (HL)", BIT_TEST_HL(state, 1));
OPCODE(0xCB4F, 8, 2, "BIT 1, A", BIT_TEST(state, state->AF.b.h, 1));
OPCODE(0xCB50, 8, 2, "BIT 2, B", BIT_TEST(state, state->BC.b.h, 2));
OPCODE(0xCB51, 8, 2, "BIT 2, C", BIT_TEST(state, state->BC.b.l, 2));
OPCODE(0xCB52, 8, 2, "BIT 2, D", BIT_TEST(state, state->DE.b.h, 2));
OPCODE(0xCB53, 8, 2, "BIT 2, E", BIT_TEST(state, state->DE.b.l, 2));
OPCODE(0xCB54, 8, 2, "BIT 2, H", BIT_TEST(state, state->HL.b.h, 2));
OPCODE(0xCB55, 8, 2, "BIT 2, L", BIT_TEST(state, state->HL.b.l, 2));
OPCODE(0xCB56, 12, 2, "BIT 2, (HL)", BIT_TEST_HL(state, 2));
OPCODE(0xCB57, 8, 2, "BIT 2, A", BIT_TEST(state, state->AF.b.h, 2));
OPCODE(0xCB58, 8, 2, "BIT 3, B", BIT_TEST(state, state->BC.b.h, 3));
OPCODE(0xCB59, 8, 2, "BIT 3, C", BIT_TEST(state, state->BC.b.l, 3));
OPCODE(0xCB5A, 8, 2, "BIT 3, D", BIT_TEST(state, state->DE.b.h, 3));
OPCODE(0xCB5B, 8, 2, "BIT 3, E", BIT_TEST(state, state->DE.b.l, 3));
OPCODE(0xCB5C, 8, 2, "BIT 3, H", BIT_TEST(state, state->HL.b.h, 3));
OPCODE(0xCB5D, 8, 2, "BIT 3, L", BIT_TEST(state, state->HL.b.l, 3));
OPCODE(0xCB5E, 12, 2, "BIT 3, (HL)", BIT_TEST_HL(state, 3));
OPCODE(0xCB5F, 8, 2, "BIT 3, A", BIT_TEST(state, state->AF.b.h, 3));
OPCODE(0xCB60, 8, 2, "BIT 4, B", BIT_TEST(state, state->BC.b.h, 4));
OPCODE(0xCB61, 8, 2, "BIT 4, C", BIT_TEST(state, state->BC.b.l, 4));
OPCODE(0xCB62, 8, 2, "BIT 4, D", BIT_TEST(state, state->DE.b.h, 4));
OPCODE(0xCB63, 8, 2, "BIT 4, E", BIT_TEST(state, state->DE.b.l, 4));
OPCODE(0xCB64, 8, 2, "BIT 4, H", BIT_TEST(state, state->HL.b.h, 4));
OPCODE(0xCB65, 8, 2, "BIT 4, L", BIT_TEST(state, state->HL.b.l, 4));
OPCODE(0xCB66, 12, 2, "BIT 4, (HL)", BIT_TEST_HL(state, 4));
OPCODE(0xCB67, 8, 2, "BIT 4, A", BIT_TEST(state, state->AF.b.h, 4));
OPCODE(0xCB68, 8, 2, "BIT 5, B", BIT_TEST(state, state->BC.b.h, 5));
OPCODE(0xCB69, 8, 2, "BIT 5, C", BIT_TEST(state, state->BC.b.l, 5));
OPCODE(0xCB6A, 8, 2, "BIT 5, D", BIT_TEST(state, state->DE.b.h, 5));
OPCODE(0xCB6B, 8, 2, "BIT 5, E", BIT_TEST(state, state->DE.b.l, 5));
OPCODE(0xCB6C, 8, 2, "BIT 5, H", BIT_TEST(state, state->HL.b.h, 5));
OPCODE(0xCB6D, 8, 2, "BIT 5, L", BIT_TEST(state, state->HL.b.l, 5));
OPCODE(0xCB6E, 12, 2, "BIT 5, (HL)", BIT_TEST_HL(state, 5));
OPCODE(0xCB6F, 8, 2, "BIT 5, A", BIT_TEST(state, state->AF.b.h, 5));
OPCODE(0xCB70, 8, 2, "BIT 6, B", BIT_TEST(state, state->BC.b.h, 6));
OPCODE(0xCB71, 8, 2, "BIT 6, C", BIT_TEST(state, state->BC.b.l, 6));
OPCODE(0xCB72, 8, 2, "BIT 6, D", BIT_TEST(state, state->DE.b.h, 6));
OPCODE(0xCB73, 8, 2, "BIT 6, E", BIT_TEST(state, state->DE.b.l, 6));
OPCODE(0xCB74, 8, 2, "BIT 6, H", BIT_TEST(state, state->HL.b.h, 6));
OPCODE(0xCB75, 8, 2, "BIT 6, L", BIT_TEST(state, state->HL.b.l, 6));
OPCODE(0xCB76, 12, 2, "BIT 6, (HL)", BIT_TEST_HL(state, 6));
OPCODE(0xCB77, 8, 2, "BIT 6, A", BIT_TEST(state, state->AF.b.h, 6));
OPCODE(0xCB78, 8, 2, "BIT 7, B", BIT_TEST(state, state->BC.b.h, 7));
OPCODE(0xCB79, 8, 2, "BIT 7, C", BIT_TEST(state, state->BC.b.l, 7));
OPCODE(0xCB7A, 8, 2, "BIT 7, D", BIT_TEST(state, state->DE.b.h, 7));
OPCODE(0xCB7B, 8, 2, "BIT 7, E", BIT_TEST(state, state->DE.b.l, 7));
OPCODE(0xCB7C, 8, 2, "BIT 7, H", BIT_TEST(state, state->HL.b.h, 7));
OPCODE(0xCB7D, 8, 2, "BIT 7, L", BIT_TEST(state, state->HL.b.l, 7));
OPCODE(0xCB7E, 12, 2, "BIT 7, (HL)", BIT_TEST_HL(state, 7));
OPCODE(0xCB7F, 8, 2, "BIT 7, A", BIT_TEST(state, state->AF.b.h, 7));
OPCODE(0xCB80, 8, 2, "RES 0, B", BIT_RESET(state, state->BC.b.h, 0));
OPCODE(0xCB81, 8, 2, "RES 0, C", BIT_RESET(state, state->BC.b.l, 0));
OPCODE(0xCB82, 8, 2, "RES 0, D", BIT_RESET(state, state->DE.b.h, 0));
OPCODE(0xCB83, 8, 2, "RES 0, E", BIT_RESET(state, state->DE.b.l, 0));
OPCODE(0xCB84, 8, 2, "RES 0, H", BIT_RESET(state, state->HL.b.h, 0));
OPCODE(0xCB85, 8, 2, "RES 0, L", BIT_RESET(state, state->HL.b.l, 0));
OPCODE(0xCB86, 15, 2, "RES 0, (HL)", BIT_RESET_HL(state, 0));
OPCODE(0xCB87, 8, 2, "RES 0, A", BIT_RESET(state, state->AF.b.h, 0));
OPCODE(0xCB88, 8, 2, "RES 1, B", BIT_RESET(state, state->BC.b.h, 1));
OPCODE(0xCB89, 8, 2, "RES 1, C", BIT_RESET(state, state->BC.b.l, 1));
OPCODE(0xCB8A, 8, 2, "RES 1, D", BIT_RESET(state, state->DE.b.h, 1));
OPCODE(0xCB8B, 8, 2, "RES 1, E", BIT_RESET(state, state->DE.b.l, 1));
OPCODE(0xCB8C, 8, 2, "RES 1, H", BIT_RESET(state, state->HL.b.h, 1));
OPCODE(0xCB8D, 8, 2, "RES 1, L", BIT_RESET(state, state->HL.b.l, 1));
OPCODE(0xCB8E, 15, 2, "RES 1, (HL)", BIT_RESET_HL(state, 1));
OPCODE(0xCB8F, 8, 2, "RES 1, A", BIT_RESET(state, state->AF.b.h, 1));
OPCODE(0xCB90, 8, 2, "RES 2, B", BIT_RESET(state, state->BC.b.h, 2));
OPCODE(0xCB91, 8, 2, "RES 2, C", BIT_RESET(state, state->BC.b.l, 2));
OPCODE(0xCB92, 8, 2, "RES 2, D", BIT_RESET(state, state->DE.b.h, 2));
OPCODE(0xCB93, 8, 2, "RES 2, E", BIT_RESET(state, state->DE.b.l, 2));
OPCODE(0xCB94, 8, 2, "RES 2, H", BIT_RESET(state, state->HL.b.h, 2));
OPCODE(0xCB95, 8, 2, "RES 2, L", BIT_RESET(state, state->HL.b.l, 2));
OPCODE(0xCB96, 15, 2, "RES 2, (HL)", BIT_RESET_HL(state, 2));
OPCODE(0xCB97, 8, 2, "RES 2, A", BIT_RESET(state, state->AF.b.h, 2));
OPCODE(0xCB98, 8, 2, "RES 3, B", BIT_RESET(state, state->BC.b.h, 3));
OPCODE(0xCB99, 8, 2, "RES 3, C", BIT_RESET(state, state->BC.b.l, 3));
OPCODE(0xCB9A, 8, 2, "RES 3, D", BIT_RESET(state, state->DE.b.h, 3));
OPCODE(0xCB9B, 8, 2, "RES 3, E", BIT_RESET(state, state->DE.b.l, 3));
OPCODE(0xCB9C, 8, 2, "RES 3, H", BIT_RESET(state, state->HL.b.h, 3));
OPCODE(0xCB9D, 8, 2, "RES 3, L", BIT_RESET(state, state->HL.b.l, 3));
OPCODE(0xCB9E, 15, 2, "RES 3, (HL)", BIT_RESET_HL(state, 3));
OPCODE(0xCB9F, 8, 2, "RES 3, A", BIT_RESET(state, state->AF.b.h, 3));
OPCODE(0xCBA0, 8, 2, "RES 4, B", BIT_RESET(state, state->BC.b.h, 4));
OPCODE(0xCBA1, 8, 2, "RES 4, C", BIT_RESET(state, state->BC.b.l, 4));
OPCODE(0xCBA2, 8, 2, "RES 4, D", BIT_RESET(state, state->DE.b.h, 4));
OPCODE(0xCBA3, 8, 2, "RES 4, E", BIT_RESET(state, state->DE.b.l, 4));
OPCODE(0xCBA4, 8, 2, "RES 4, H", BIT_RESET(state, state->HL.b.h, 4));
OPCODE(0xCBA5, 8, 2, "RES 4, L", BIT_RESET(state, state->HL.b.l, 4));
OPCODE(0xCBA6, 15, 2, "RES 4, (HL)", BIT_RESET_HL(state, 4));
OPCODE(0xCBA7, 8, 2, "RES 4, A", BIT_RESET(state, state->AF.b.h, 4));
OPCODE(0xCBA8, 8, 2, "RES 5, B", BIT_RESET(state, state->BC.b.h, 5));
OPCODE(0xCBA9, 8, 2, "RES 5, C", BIT_RESET(state, state->BC.b.l, 5));
OPCODE(0xCBAA, 8, 2, "RES 5, D", BIT_RESET(state, state->DE.b.h, 5));
OPCODE(0xCBAB, 8, 2, "RES 5, E", BIT_RESET(state, state->DE.b.l, 5));
OPCODE(0xCBAC, 8, 2, "RES 5, H", BIT_RESET(state, state->HL.b.h, 5));
OPCODE(0xCBAD, 8, 2, "RES 5, L", BIT_RESET(state, state->HL.b.l, 5));
OPCODE(0xCBAE, 15, 2, "RES 5, (HL)", BIT_RESET_HL(state, 5));
OPCODE(0xCBAF, 8, 2, "RES 5, A", BIT_RESET(state, state->AF.b.h, 5));
OPCODE(0xCBB0, 8, 2, "RES 6, B", BIT_RESET(state, state->BC.b.h, 6));
OPCODE(0xCBB1, 8, 2, "RES 6, C", BIT_RESET(state, state->BC.b.l, 6));
OPCODE(0xCBB2, 8, 2, "RES 6, D", BIT_RESET(state, state->DE.b.h, 6));
OPCODE(0xCBB3, 8, 2, "RES 6, E", BIT_RESET(state, state->DE.b.l, 6));
OPCODE(0xCBB4, 8, 2, "RES 6, H", BIT_RESET(state, state->HL.b.h, 6));
OPCODE(0xCBB5, 8, 2, "RES 6, L", BIT_RESET(state, state->HL.b.l, 6));
OPCODE(0xCBB6, 15, 2, "RES 6, (HL)", BIT_RESET_HL(state, 6));
OPCODE(0xCBB7, 8, 2, "RES 6, A", BIT_RESET(state, state->AF.b.h, 6));
OPCODE(0xCBB8, 8, 2, "RES 7, B", BIT_RESET(state, state->BC.b.h, 7));
OPCODE(0xCBB9, 8, 2, "RES 7, C", BIT_RESET(state, state->BC.b.l, 7));
OPCODE(0xCBBA, 8, 2, "RES 7, D", BIT_RESET(state, state->DE.b.h, 7));
OPCODE(0xCBBB, 8, 2, "RES 7, E", BIT_RESET(state, state->DE.b.l, 7));
OPCODE(0xCBBC, 8, 2, "RES 7, H", BIT_RESET(state, state->HL.b.h, 7));
OPCODE(0xCBBD, 8, 2, "RES 7, L", BIT_RESET(state, state->HL.b.l, 7));
OPCODE(0xCBBE, 15, 2, "RES 7, (HL)", BIT_RESET_HL(state, 7));
OPCODE(0xCBBF, 8, 2, "RES 7, A", BIT_RESET(state, state->AF.b.h, 7));
OPCODE(0xCBC0, 8, 2, "SET 0, B", BIT_SET(state, state->BC.b.h, 0));
OPCODE(0xCBC1, 8, 2, "SET 0, C", BIT_SET(state, state->BC.b.l, 0));
OPCODE(0xCBC2, 8, 2, "SET 0, D", BIT_SET(state, state->DE.b.h, 0));
OPCODE(0xCBC3, 8, 2, "SET 0, E", BIT_SET(state, state->DE.b.l, 0));
OPCODE(0xCBC4, 8, 2, "SET 0, H", BIT_SET(state, state->HL.b.h, 0));
OPCODE(0xCBC5, 8, 2, "SET 0, L", BIT_SET(state, state->HL.b.l, 0));
OPCODE(0xCBC6, 15, 2, "SET 0, (HL)", BIT_SET_HL(state, 0));
OPCODE(0xCBC7, 8, 2, "SET 0, A", BIT_SET(state, state->AF.b.h, 0));
OPCODE(0xCBC8, 8, 2, "SET 1, B", BIT_SET(state, state->BC.b.h, 1));
OPCODE(0xCBC9, 8, 2, "SET 1, C", BIT_SET(state, state->BC.b.l, 1));
OPCODE(0xCBCA, 8, 2, "SET 1, D", BIT_SET(state, state->DE.b.h, 1));
OPCODE(0xCBCB, 8, 2, "SET 1, E", BIT_SET(state, state->DE.b.l, 1));
OPCODE(0xCBCC, 8, 2, "SET 1, H", BIT_SET(state, state->HL.b.h, 1));
OPCODE(0xCBCD, 8, 2, "SET 1, L", BIT_SET(state, state->HL.b.l, 1));
OPCODE(0xCBCE, 15, 2, "SET 1, (HL)", BIT_SET_HL(state, 1));
OPCODE(0xCBCF, 8, 2, "SET 1, A", BIT_SET(state, state->AF.b.h, 1));
OPCODE(0xCBD0, 8, 2, "SET 2, B", BIT_SET(state, state->BC.b.h, 2));
OPCODE(0xCBD1, 8, 2, "SET 2, C", BIT_SET(state, state->BC.b.l, 2));
OPCODE(0xCBD2, 8, 2, "SET 2, D", BIT_SET(state, state->DE.b.h, 2));
OPCODE(0xCBD3, 8, 2, "SET 2, E", BIT_SET(state, state->DE.b.l, 2));
OPCODE(0xCBD4, 8, 2, "SET 2, H", BIT_SET(state, state->HL.b.h, 2));
OPCODE(0xCBD5, 8, 2, "SET 2, L", BIT_SET(state, state->HL.b.l, 2));
OPCODE(0xCBD6, 15, 2, "SET 2, (HL)", BIT_SET_HL(state, 2));
OPCODE(0xCBD7, 8, 2, "SET 2, A", BIT_SET(state, state->AF.b.h, 2));
OPCODE(0xCBD8, 8, 2, "SET 3, B", BIT_SET(state, state->BC.b.h, 3));
OPCODE(0xCBD9, 8, 2, "SET 3, C", BIT_SET(state, state->BC.b.l, 3));
OPCODE(0xCBDA, 8, 2, "SET 3, D", BIT_SET(state, state->DE.b.h, 3));
OPCODE(0xCBDB, 8, 2, "SET 3, E", BIT_SET(state, state->DE.b.l, 3));
OPCODE(0xCBDC, 8, 2, "SET 3, H", BIT_SET(state, state->HL.b.h, 3));
OPCODE(0xCBDD, 8, 2, "SET 3, L", BIT_SET(state, state->HL.b.l, 3));
OPCODE(0xCBDE, 15, 2, "SET 3, (HL)", BIT_SET_HL(state, 3));
OPCODE(0xCBDF, 8, 2, "SET 3, A", BIT_SET(state, state->AF.b.h, 3));
OPCODE(0xCBE0, 8, 2, "SET 4, B", BIT_SET(state, state->BC.b.h, 4));
OPCODE(0xCBE1, 8, 2, "SET 4, C", BIT_SET(state, state->BC.b.l, 4));
OPCODE(0xCBE2, 8, 2, "SET 4, D", BIT_SET(state, state->DE.b.h, 4));
OPCODE(0xCBE3, 8, 2, "SET 4, E", BIT_SET(state, state->DE.b.l, 4));
OPCODE(0xCBE4, 8, 2, "SET 4, H", BIT_SET(state, state->HL.b.h, 4));
OPCODE(0xCBE5, 8, 2, "SET 4, L", BIT_SET(state, state->HL.b.l, 4));
OPCODE(0xCBE6, 15, 2, "SET 4, (HL)", BIT_SET_HL(state, 4));
OPCODE(0xCBE7, 8, 2, "SET 4, A", BIT_SET(state, state->AF.b.h, 4));
OPCODE(0xCBE8, 8, 2, "SET 5, B", BIT_SET(state, state->BC.b.h, 5));
OPCODE(0xCBE9, 8, 2, "SET 5, C", BIT_SET(state, state->BC.b.l, 5));
OPCODE(0xCBEA, 8, 2, "SET 5, D", BIT_SET(state, state->DE.b.h, 5));
OPCODE(0xCBEB, 8, 2, "SET 5, E", BIT_SET(state, state->DE.b.l, 5));
OPCODE(0xCBEC, 8, 2, "SET 5, H", BIT_SET(state, state->HL.b.h, 5));
OPCODE(0xCBED, 8, 2, "SET 5, L", BIT_SET(state, state->HL.b.l, 5));
OPCODE(0xCBEE, 15, 2, "SET 5, (HL)", BIT_SET_HL(state, 5));
OPCODE(0xCBEF, 8, 2, "SET 5, A", BIT_SET(state, state->AF.b.h, 5));
OPCODE(0xCBF0, 8, 2, "SET 6, B", BIT_SET(state, state->BC.b.h, 6));
OPCODE(0xCBF1, 8, 2, "SET 6, C", BIT_SET(state, state->BC.b.l, 6));
OPCODE(0xCBF2, 8, 2, "SET 6, D", BIT_SET(state, state->DE.b.h, 6));
OPCODE(0xCBF3, 8, 2, "SET 6, E", BIT_SET(state, state->DE.b.l, 6));
OPCODE(0xCBF4, 8, 2, "SET 6, H", BIT_SET(state, state->HL.b.h, 6));
OPCODE(0xCBF5, 8, 2, "SET 6, L", BIT_SET(state, state->HL.b.l, 6));
OPCODE(0xCBF6, 15, 2, "SET 6, (HL)", BIT_SET_HL(state, 6));
OPCODE(0xCBF7, 8, 2, "SET 6, A", BIT_SET(state, state->AF.b.h, 6));
OPCODE(0xCBF8, 8, 2, "SET 7, B", BIT_SET(state, state->BC.b.h, 7));
OPCODE(0xCBF9, 8, 2, "SET 7, C", BIT_SET(state, state->BC.b.l, 7));
OPCODE(0xCBFA, 8, 2, "SET 7, D", BIT_SET(state, state->DE.b.h, 7));
OPCODE(0xCBFB, 8, 2, "SET 7, E", BIT_SET(state, state->DE.b.l, 7));
OPCODE(0xCBFC, 8, 2, "SET 7, H", BIT_SET(state, state->HL.b.h, 7));
OPCODE(0xCBFD, 8, 2, "SET 7, L", BIT_SET(state, state->HL.b.l, 7));
OPCODE(0xCBFE, 15, 2, "SET 7, (HL)", BIT_SET_HL(state, 7));
OPCODE(0xCBFF, 8, 2, "SET 7, A", BIT_SET(state, state->AF.b.h, 7));

static LR35902Opcode opcodes[256] = {
    OPCODE16(, 0), OPCODE16(, 1), OPCODE16(, 2), OPCODE16(, 3),
    OPCODE16(, 4), OPCODE16(, 5), OPCODE16(, 6), OPCODE16(, 7),
    OPCODE16(, 8), OPCODE16(, 9), OPCODE16(, A), OPCODE16(, B),
    OPCODE16(, C), OPCODE16(, D), OPCODE16(, E), OPCODE16(, F)};

static LR35902Opcode CBopcodes[256] = {
    OPCODE16(CB, 0), OPCODE16(CB, 1), OPCODE16(CB, 2), OPCODE16(CB, 3),
    OPCODE16(CB, 4), OPCODE16(CB, 5), OPCODE16(CB, 6), OPCODE16(CB, 7),
    OPCODE16(CB, 8), OPCODE16(CB, 9), OPCODE16(CB, A), OPCODE16(CB, B),
    OPCODE16(CB, C), OPCODE16(CB, D), OPCODE16(CB, E), OPCODE16(CB, F),
};

LR35902Cpu::LR35902Cpu(Machine *machine, const std::string &name,
                       ClockDivider divider, state_type *state)
    : Cpu(machine, name, divider, state) {
  state->bus->add(0xFF0F, &state->IF);
}

LR35902Cpu::~LR35902Cpu(void) {}

void LR35902Cpu::line(Line line, LineState state) {
  switch (line) {
    case Line::RESET:
      m_state->reset_line = state;
      break;
    case Line::INT0: /* Interrupt::VBlank */
      bit_set(m_state->IF, 0, true);
      break;
    case Line::INT1: /* Interrupt::LCDStat */
      bit_set(m_state->IF, 1, true);
      break;
    case Line::INT2: /* Interrupt::Timeout */
      bit_set(m_state->IF, 2, true);
      break;
    case Line::INT3: /* Interrupt::Serial */
      bit_set(m_state->IF, 3, true);
      break;
    case Line::INT4: /* Interrupt::Joypad */
      bit_set(m_state->IF, 4, true);
      break;
    default:
      break;
  };
}

void LR35902Cpu::interrupt(addr_t addr) {
  DEVICE_TRACE("Interrupt: ", Hex(addr));
  PUSH(m_state, m_state->PC.b.h, m_state->PC.b.l);
  m_state->PC.d = addr;
  m_state->WZ.d = addr;
  m_state->iff2 = m_state->iff1;
  m_state->iff1 = false;
  add_icycles(Cycles(20));
}

addr_t InterruptVector[] = {
    0x40, /* Interrupt::VBlank */
    0x48, /* Interrupt::LCDStat */
    0x50, /* Interrupt::Timeout */
    0x58, /* Interrupt::Serial */
    0x60, /* Interrupt::Joypad */
};

void LR35902Cpu::execute(void) {
  while (true) {
    byte_t iflags = bus_read(m_state, 0xFFFF);
    for (unsigned i = 0; i < 5; i++) {
      if (bit_isset(m_state->IF, i)) {
        if (!bit_isset(iflags, i)) continue;
        interrupt(InterruptVector[i]);
        bit_set(m_state->IF, i, false);
        m_state->halt = false;
        break;
      }
    }

    if (unlikely(m_state->iwait)) m_state->iwait = false;

    if (unlikely(m_state->yield)) {
      yield();
      m_state->yield = false;
    }

    if (m_state->halt) {
      // TODO: Pick a real value
      add_icycles(Cycles(100));
      continue;
    }
    m_state->latch_pc = m_state->PC;
    m_state->latch_op = pc_read(m_state);
    m_state->vHL = &m_state->HL;
    if (m_state->latch_op == 0xCB) {
      m_state->prefix = LR35902Prefix::CBPrefix;
      m_state->vHL = &m_state->HL;
      m_state->latch_op = pc_read(m_state);
      m_state->Op = &CBopcodes[m_state->latch_op];
    } else {
      m_state->vHL = &m_state->HL;
      m_state->prefix = LR35902Prefix::NoPrefix;
      m_state->Op = &opcodes[m_state->latch_op];
    }
    m_state->icycles = m_state->Op->cycles;
#if 1
    if (m_state->prefix == LR35902Prefix::CBPrefix) {
      OPCODE_SWITCH_BLOCK(0xCB, m_state);
    } else if (m_state->prefix == LR35902Prefix::NoPrefix) {
      OPCODE_SWITCH_BLOCK(0x, m_state);
    } else
#endif
      m_state->Op->func(m_state);

    DEVICE_TRACE(Log(m_state));
    add_icycles(m_state->icycles);
  }
}

std::string LR35902Cpu::Log(LR35902State *state) {
  std::stringstream os;
  const std::string &str = state->Op->name;
  os << Hex(state->latch_pc) << ":";
  os << Hex(state->latch_op) << ":";
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
      op << "HL";
    } else if (it == "L") {
      op << "L";
    } else if (it == "H") {
      op << "H";
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
  os << ",SP:" << Hex(state->SP.d);

  return os.str();
}

