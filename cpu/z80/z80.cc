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
#include "cpu/lib/cpu2_macros.h"

using namespace Z80;

OPCODE(0x00, 4, 1, "NOP", );
OPCODE(0x01, 10, 3, "LD BC,d16", LD16(state, state->BC, D16(state)));
OPCODE(0x02, 7, 1, "LD (BC),A", {
  state->WZ.b.l = (state->BC.b.l + 1) & 0xff;
  state->WZ.b.h = state->AF.b.h;
  LDMEM(state, state->BC.d, state->AF.b.h);
});
OPCODE(0x03, 6, 1, "INC BC", INC16(state, state->BC));
OPCODE(0x04, 4, 1, "INC B", INC(state, state->BC.b.h));
OPCODE(0x05, 4, 1, "DEC B", DEC(state, state->BC.b.h));
OPCODE(0x06, 7, 2, "LD B, d8", LD(state, state->BC.b.h, D8(state)));
OPCODE(0x07, 4, 1, "RLCA", RLCA(state));
OPCODE(0x08, 4, 1, "EX AF,AF'", EX(state, state->AF.d, state->AF2.d));
OPCODE(0x09, 11, 1, "ADD HL,BC", ADD16(state, *state->vHL, state->BC.d));
OPCODE(0x0A, 7, 1, "LD A,(BC)", {
  LD(state, state->AF.b.h, bus_read(state, state->BC.d));
  state->WZ.d = state->BC.d + 1;
});
OPCODE(0x0B, 6, 1, "DEC BC", DEC16(state, state->BC));
OPCODE(0x0C, 4, 1, "INC C", INC(state, state->BC.b.l));
OPCODE(0x0D, 4, 1, "DEC C", DEC(state, state->BC.b.l));
OPCODE(0x0E, 7, 2, "LD C, d8", LD(state, state->BC.b.l, D8(state)));
OPCODE(0x0F, 4, 1, "RRCA", RRCA(state));
OPCODE(0x10, 8, 1, "DJNZ r8", DJNZ(state, D8(state)));
OPCODE(0x11, 10, 3, "LD DE,d16", LD16(state, state->DE, D16(state)));
OPCODE(0x12, 7, 1, "LD (DE),A", {
  state->WZ.b.l = (state->DE.b.l + 1) & 0xff;
  state->WZ.b.h = state->AF.b.h;
  LDMEM(state, state->DE.d, state->AF.b.h);
});
OPCODE(0x13, 6, 1, "INC DE", INC16(state, state->DE));
OPCODE(0x14, 4, 1, "INC D", INC(state, state->DE.b.h));
OPCODE(0x15, 4, 1, "DEC D", DEC(state, state->DE.b.h));
OPCODE(0x16, 7, 2, "LD D, d8", LD(state, state->DE.b.h, D8(state)));
OPCODE(0x17, 4, 1, "RLA", RLA(state));
OPCODE(0x18, 12, 2, "JR r8", JR(state, true, D8(state)));
OPCODE(0x19, 11, 1, "ADD HL,DE", ADD16(state, *state->vHL, state->DE.d));
OPCODE(0x1A, 7, 1, "LD A,(DE)", {
  LD(state, state->AF.b.h, bus_read(state, state->DE.d));
  state->WZ.d = state->DE.d + 1;
});
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
OPCODE(0x31, 10, 3, "LD SP, d16", LD16(state, state->SP, D16(state)));
OPCODE(0x32, 13, 3, "LD (d16), A", {
  state->WZ.d = D16(state);
  state->WZ.b.l += 1;
  state->WZ.b.h = state->AF.b.h;
  LDMEM(state, state->d16.d, state->AF.b.h);
});
OPCODE(0x33, 6, 1, "INC SP", INC16(state, state->SP));
OPCODE(0x34, 11, 1, "INC (HL)", INCI(state, DADDR(state)));
OPCODE(0x35, 11, 1, "DEC (HL)", DECI(state, DADDR(state)));
OPCODE(0x36, 10, 2, "LD (HL), d8", LDMEM(state, DADDR(state), D8(state)));
OPCODE(0x37, 4, 1, "SCF", SCF(state));
OPCODE(0x38, 7, 2, "JR C,r8", JR(state, state->AF.b.f.C, D8(state)));
OPCODE(0x39, 11, 1, "ADD HL,SP", ADD16(state, *state->vHL, state->SP.d));
OPCODE(0x3A, 13, 3, "LD A,(d16)", {
  state->WZ.d = D16(state);
  state->WZ.d++;
  LD(state, state->AF.b.h, I8(state, state->d16.d));
});
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
OPCODE(0xD3, 11, 2, "OUT d8, A", {
  state->WZ.b.l = (D8(state) + 1) & 0xff;
  state->WZ.b.h = state->AF.b.h;
  Z80_OUT(state, state->d8, state->AF.b.h);
});
OPCODE(0xD4, 10, 3, "CALL NC, d16", CALL(state, !state->AF.b.f.C, D16(state)));
OPCODE(0xD5, 11, 1, "PUSH DE", PUSH(state, state->DE.b.h, state->DE.b.l));
OPCODE(0xD6, 7, 2, "SUB A, d8", SUB(state, state->AF.b.h, D8(state)));
OPCODE(0xD7, 11, 1, "RST 10H", RST(state, 0x10));
OPCODE(0xD8, 5, 1, "RET C", RET(state, state->AF.b.f.C));
OPCODE(0xD9, 4, 1, "EXX BC DE HL", EXX(state));
OPCODE(0xDA, 10, 3, "JP C, d16", JP(state, state->AF.b.f.C, D16(state)));
OPCODE(0xDB, 11, 2, "IN A, d8", {
  state->WZ.b.h = state->AF.b.h;
  state->WZ.b.l = D8(state) + 1;
  Z80_IN(state, state->AF.b.h, state->d8);
});
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
OPCODE(0xCB30, 8, 2, "SLL B", SLL(state, state->BC.b.h, state->BC.b.h));
OPCODE(0xCB31, 8, 2, "SLL C", SLL(state, state->BC.b.l, state->BC.b.l));
OPCODE(0xCB32, 8, 2, "SLL D", SLL(state, state->DE.b.h, state->DE.b.h));
OPCODE(0xCB33, 8, 2, "SLL E", SLL(state, state->DE.b.l, state->DE.b.l));
OPCODE(0xCB34, 8, 2, "SLL H", SLL(state, state->HL.b.h, state->HL.b.h));
OPCODE(0xCB35, 8, 2, "SLL L", SLL(state, state->HL.b.l, state->HL.b.l));
OPCODE(0xCB36, 15, 2, "SLL (HL)", SLL_HL(state));
OPCODE(0xCB37, 8, 2, "SLL A", SLL(state, state->AF.b.h, state->AF.b.h));
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

OPCODE(0xED00, 5, 2, "NOP", );
OPCODE(0xED01, 5, 2, "NOP", );
OPCODE(0xED02, 5, 2, "NOP", );
OPCODE(0xED03, 5, 2, "NOP", );
OPCODE(0xED04, 5, 2, "NOP", );
OPCODE(0xED05, 5, 2, "NOP", );
OPCODE(0xED06, 5, 2, "NOP", );
OPCODE(0xED07, 5, 2, "NOP", );
OPCODE(0xED08, 5, 2, "NOP", );
OPCODE(0xED09, 5, 2, "NOP", );
OPCODE(0xED0A, 5, 2, "NOP", );
OPCODE(0xED0B, 5, 2, "NOP", );
OPCODE(0xED0C, 5, 2, "NOP", );
OPCODE(0xED0D, 5, 2, "NOP", );
OPCODE(0xED0E, 5, 2, "NOP", );
OPCODE(0xED0F, 5, 2, "NOP", );
OPCODE(0xED10, 5, 2, "NOP", );
OPCODE(0xED11, 5, 2, "NOP", );
OPCODE(0xED12, 5, 2, "NOP", );
OPCODE(0xED13, 5, 2, "NOP", );
OPCODE(0xED14, 5, 2, "NOP", );
OPCODE(0xED15, 5, 2, "NOP", );
OPCODE(0xED16, 5, 2, "NOP", );
OPCODE(0xED17, 5, 2, "NOP", );
OPCODE(0xED18, 5, 2, "NOP", );
OPCODE(0xED19, 5, 2, "NOP", );
OPCODE(0xED1A, 5, 2, "NOP", );
OPCODE(0xED1B, 5, 2, "NOP", );
OPCODE(0xED1C, 5, 2, "NOP", );
OPCODE(0xED1D, 5, 2, "NOP", );
OPCODE(0xED1E, 5, 2, "NOP", );
OPCODE(0xED1F, 5, 2, "NOP", );
OPCODE(0xED20, 5, 2, "NOP", );
OPCODE(0xED21, 5, 2, "NOP", );
OPCODE(0xED22, 5, 2, "NOP", );
OPCODE(0xED23, 5, 2, "NOP", );
OPCODE(0xED24, 5, 2, "NOP", );
OPCODE(0xED25, 5, 2, "NOP", );
OPCODE(0xED26, 5, 2, "NOP", );
OPCODE(0xED27, 5, 2, "NOP", );
OPCODE(0xED28, 5, 2, "NOP", );
OPCODE(0xED29, 5, 2, "NOP", );
OPCODE(0xED2A, 5, 2, "NOP", );
OPCODE(0xED2B, 5, 2, "NOP", );
OPCODE(0xED2C, 5, 2, "NOP", );
OPCODE(0xED2D, 5, 2, "NOP", );
OPCODE(0xED2E, 5, 2, "NOP", );
OPCODE(0xED2F, 5, 2, "NOP", );
OPCODE(0xED30, 5, 2, "NOP", );
OPCODE(0xED31, 5, 2, "NOP", );
OPCODE(0xED32, 5, 2, "NOP", );
OPCODE(0xED33, 5, 2, "NOP", );
OPCODE(0xED34, 5, 2, "NOP", );
OPCODE(0xED35, 5, 2, "NOP", );
OPCODE(0xED36, 5, 2, "NOP", );
OPCODE(0xED37, 5, 2, "NOP", );
OPCODE(0xED38, 5, 2, "NOP", );
OPCODE(0xED39, 5, 2, "NOP", );
OPCODE(0xED3A, 5, 2, "NOP", );
OPCODE(0xED3B, 5, 2, "NOP", );
OPCODE(0xED3C, 5, 2, "NOP", );
OPCODE(0xED3D, 5, 2, "NOP", );
OPCODE(0xED3E, 5, 2, "NOP", );
OPCODE(0xED3F, 5, 2, "NOP", );
OPCODE(0xED40, 12, 2, "IN B, (C)", Z80_IN(state, state->BC.b.h, state->BC.b.l));
OPCODE(0xED41, 12, 2, "OUT (C), B", {
  Z80_OUT(state, state->BC.b.l, state->BC.b.h);
  state->WZ.d = state->BC.d + 1;
});
OPCODE(0xED42, 15, 2, "SBC HL, BC", SBC16(state, state->HL.d, state->BC.d));
OPCODE(0xED43, 20, 4, "LD (d16), BC", {
  state->WZ.d = D16(state);
  state->WZ.d++;
  LD16I(state, state->d16.d, state->BC.d);
});
OPCODE(0xED44, 8, 2, "NEG", NEG(state, state->AF.b.h));
OPCODE(0xED45, 14, 2, "RET N", RETN(state));
OPCODE(0xED46, 5, 2, "NOP", );
OPCODE(0xED47, 9, 2, "LD I, A", LD(state, state->I, state->AF.b.h));
OPCODE(0xED48, 12, 2, "IN C, (C)", Z80_IN(state, state->BC.b.l, state->BC.b.l));
OPCODE(0xED49, 12, 2, "OUT (C), C", Z80_OUT(state, state->BC.b.l, state->BC.b.l));
OPCODE(0xED4A, 15, 2, "ADC HL, BC", ADC16(state, state->HL.d, state->BC.d));
OPCODE(0xED4B, 20, 2, "LD BC, (d16)", LD16(state, state->BC, I16(state)));
OPCODE(0xED4C, 8, 2, "NEG", NEG(state, state->AF.b.h));
OPCODE(0xED4D, 14, 2, "RET I", RETI(state));
OPCODE(0xED4E, 5, 2, "NOP", );
OPCODE(0xED4F, 9, 2, "LD R, A", LD(state, state->R, state->AF.b.h));
OPCODE(0xED50, 12, 2, "IN D, (C)", Z80_IN(state, state->DE.b.h, state->BC.b.l));
OPCODE(0xED51, 12, 2, "OUT (C), D", Z80_OUT(state, state->BC.b.l, state->DE.b.h));
OPCODE(0xED52, 15, 2, "SBC HL, DE", SBC16(state, state->HL.d, state->DE.d));
OPCODE(0xED53, 20, 4, "LD (d16), DE", {
  state->WZ.d = D16(state);
  state->WZ.d++;
  LD16I(state, state->d16.d, state->DE.d);
});
OPCODE(0xED54, 8, 2, "NEG", NEG(state, state->AF.b.h));
OPCODE(0xED55, 14, 2, "RET N", RETN(state));
OPCODE(0xED56, 8, 2, "IM 1", IM(state, 1));
OPCODE(0xED57, 5, 2, "NOP", );
OPCODE(0xED58, 12, 2, "IN E, (C)", Z80_IN(state, state->DE.b.l, state->BC.b.l));
OPCODE(0xED59, 12, 2, "OUT (C), E", Z80_OUT(state, state->BC.b.l, state->DE.b.l));
OPCODE(0xED5A, 15, 2, "ADC HL, DE", ADC16(state, state->HL.d, state->DE.d));
OPCODE(0xED5B, 20, 2, "LD DE, (d16)", LD16(state, state->DE, I16(state)));
OPCODE(0xED5C, 8, 2, "NEG", NEG(state, state->AF.b.h));
OPCODE(0xED5D, 14, 2, "RET N", RETN(state));
OPCODE(0xED5E, 8, 2, "IM 2", IM(state, 2));
OPCODE(0xED5F, 9, 2, "LD A, R", LD(state, state->AF.b.h, state->R));
OPCODE(0xED60, 12, 2, "IN H, (C)", Z80_IN(state, state->HL.b.h, state->BC.b.l));
OPCODE(0xED61, 12, 2, "OUT (C), H", Z80_OUT(state, state->BC.b.l, state->HL.b.h));
OPCODE(0xED62, 15, 2, "SBC HL, HL", SBC16(state, state->HL.d, state->HL.d));
OPCODE(0xED63, 20, 4, "LD (d16), HL", {
  state->WZ.d = D16(state);
  state->WZ.d++;
  LD16I(state, state->d16.d, state->HL.d);
});
OPCODE(0xED64, 8, 2, "NEG", NEG(state, state->AF.b.h));
OPCODE(0xED65, 14, 2, "RET N", RETN(state));
OPCODE(0xED66, 5, 2, "NOP", );
OPCODE(0xED67, 18, 2, "RRD", RRD(state));
OPCODE(0xED68, 12, 2, "IN L, (C)", Z80_IN(state, state->HL.b.l, state->BC.b.l));
OPCODE(0xED69, 12, 2, "OUT (C), L", Z80_OUT(state, state->BC.b.l, state->HL.b.l));
OPCODE(0xED6A, 15, 2, "ADC HL, HL", ADC16(state, state->HL.d, state->HL.d));
OPCODE(0xED6B, 20, 2, "LD HL, (d16)", LD16(state, state->HL, I16(state)));
OPCODE(0xED6C, 8, 2, "NEG", NEG(state, state->AF.b.h));
OPCODE(0xED6D, 14, 2, "RET N", RETN(state));
OPCODE(0xED6E, 5, 2, "NOP", );
OPCODE(0xED6F, 18, 2, "RLD", RLD(state));
OPCODE(0xED70, 12, 2, "IN (C)", Z80_IN(state, state->HL.b.h, state->BC.b.l));
OPCODE(0xED71, 12, 2, "OUT (C), 0", Z80_OUT(state, state->BC.b.l, 0));
OPCODE(0xED72, 15, 2, "SBC HL, SP", SBC16(state, state->HL.d, state->SP.d));
OPCODE(0xED73, 20, 4, "LD (d16), SP", {
  state->WZ.d = D16(state);
  state->WZ.d++;
  LD16I(state, state->d16.d, state->SP.d);
});
OPCODE(0xED74, 8, 2, "NEG", NEG(state, state->AF.b.h));
OPCODE(0xED75, 14, 2, "RET N", RETN(state));
OPCODE(0xED76, 5, 2, "NOP", );
OPCODE(0xED77, 5, 2, "NOP", );
OPCODE(0xED78, 12, 2, "IN A, (C)", {
  Z80_IN(state, state->AF.b.h, state->BC.b.l);
  state->WZ.d = state->BC.d + 1;
});
OPCODE(0xED79, 12, 2, "OUT (C), A", Z80_OUT(state, state->BC.b.l, state->AF.b.h));
OPCODE(0xED7A, 15, 2, "ADC HL, SP", ADC16(state, state->HL.d, state->SP.d));
OPCODE(0xED7B, 20, 2, "LD SP, (d16)", LD16(state, state->SP, I16(state)));
OPCODE(0xED7C, 8, 2, "NEG", NEG(state, state->AF.b.h));
OPCODE(0xED7D, 14, 2, "RET N", RETN(state));
OPCODE(0xED7E, 5, 2, "NOP", );
OPCODE(0xED7F, 5, 2, "NOP", );
OPCODE(0xED80, 5, 2, "NOP", );
OPCODE(0xED81, 5, 2, "NOP", );
OPCODE(0xED82, 5, 2, "NOP", );
OPCODE(0xED83, 5, 2, "NOP", );
OPCODE(0xED84, 5, 2, "NOP", );
OPCODE(0xED85, 5, 2, "NOP", );
OPCODE(0xED86, 5, 2, "NOP", );
OPCODE(0xED87, 5, 2, "NOP", );
OPCODE(0xED88, 5, 2, "NOP", );
OPCODE(0xED89, 5, 2, "NOP", );
OPCODE(0xED8A, 5, 2, "NOP", );
OPCODE(0xED8B, 5, 2, "NOP", );
OPCODE(0xED8C, 5, 2, "NOP", );
OPCODE(0xED8D, 5, 2, "NOP", );
OPCODE(0xED8E, 5, 2, "NOP", );
OPCODE(0xED8F, 5, 2, "NOP", );
OPCODE(0xED90, 5, 2, "NOP", );
OPCODE(0xED91, 5, 2, "NOP", );
OPCODE(0xED92, 5, 2, "NOP", );
OPCODE(0xED93, 5, 2, "NOP", );
OPCODE(0xED94, 5, 2, "NOP", );
OPCODE(0xED95, 5, 2, "NOP", );
OPCODE(0xED96, 5, 2, "NOP", );
OPCODE(0xED97, 5, 2, "NOP", );
OPCODE(0xED98, 5, 2, "NOP", );
OPCODE(0xED99, 5, 2, "NOP", );
OPCODE(0xED9A, 5, 2, "NOP", );
OPCODE(0xED9B, 5, 2, "NOP", );
OPCODE(0xED9C, 5, 2, "NOP", );
OPCODE(0xED9D, 5, 2, "NOP", );
OPCODE(0xED9E, 5, 2, "NOP", );
OPCODE(0xED9F, 5, 2, "NOP", );
OPCODE(0xEDA0, 16, 2, "LDI (HL) (DE) BC", LDI(state));
OPCODE(0xEDA1, 16, 2, "CPI", CPI(state));
OPCODE(0xEDA2, 5, 2, "NOP", );
OPCODE(0xEDA3, 5, 2, "NOP", );
OPCODE(0xEDA4, 5, 2, "NOP", );
OPCODE(0xEDA5, 5, 2, "NOP", );
OPCODE(0xEDA6, 5, 2, "NOP", );
OPCODE(0xEDA7, 5, 2, "NOP", );
OPCODE(0xEDA8, 16, 2, "LDD (HL) (DE) BC", LDD(state));
OPCODE(0xEDA9, 16, 2, "CPD", CPD(state));
OPCODE(0xEDAA, 5, 2, "NOP", );
OPCODE(0xEDAB, 5, 2, "NOP", );
OPCODE(0xEDAC, 5, 2, "NOP", );
OPCODE(0xEDAD, 5, 2, "NOP", );
OPCODE(0xEDAE, 5, 2, "NOP", );
OPCODE(0xEDAF, 5, 2, "NOP", );
OPCODE(0xEDB0, 16, 2, "LDIR", LDIR(state));
OPCODE(0xEDB1, 16, 2, "CPIR", CPIR(state));
OPCODE(0xEDB2, 5, 2, "NOP", );
OPCODE(0xEDB3, 16, 2, "OTIR", OTIR(state));
OPCODE(0xEDB4, 5, 2, "NOP", );
OPCODE(0xEDB5, 5, 2, "NOP", );
OPCODE(0xEDB6, 5, 2, "NOP", );
OPCODE(0xEDB7, 5, 2, "NOP", );
OPCODE(0xEDB8, 16, 2, "LDDR", LDDR(state));
OPCODE(0xEDB9, 16, 2, "CPDR", CPDR(state));
OPCODE(0xEDBA, 5, 2, "NOP", );
OPCODE(0xEDBB, 5, 2, "NOP", );
OPCODE(0xEDBC, 5, 2, "NOP", );
OPCODE(0xEDBD, 5, 2, "NOP", );
OPCODE(0xEDBE, 5, 2, "NOP", );
OPCODE(0xEDBF, 5, 2, "NOP", );
OPCODE(0xEDC0, 5, 2, "NOP", );
OPCODE(0xEDC1, 5, 2, "NOP", );
OPCODE(0xEDC2, 5, 2, "NOP", );
OPCODE(0xEDC3, 5, 2, "NOP", );
OPCODE(0xEDC4, 5, 2, "NOP", );
OPCODE(0xEDC5, 5, 2, "NOP", );
OPCODE(0xEDC6, 5, 2, "NOP", );
OPCODE(0xEDC7, 5, 2, "NOP", );
OPCODE(0xEDC8, 5, 2, "NOP", );
OPCODE(0xEDC9, 5, 2, "NOP", );
OPCODE(0xEDCA, 5, 2, "NOP", );
OPCODE(0xEDCB, 5, 2, "NOP", );
OPCODE(0xEDCC, 5, 2, "NOP", );
OPCODE(0xEDCD, 5, 2, "NOP", );
OPCODE(0xEDCE, 5, 2, "NOP", );
OPCODE(0xEDCF, 5, 2, "NOP", );
OPCODE(0xEDD0, 5, 2, "NOP", );
OPCODE(0xEDD1, 5, 2, "NOP", );
OPCODE(0xEDD2, 5, 2, "NOP", );
OPCODE(0xEDD3, 5, 2, "NOP", );
OPCODE(0xEDD4, 5, 2, "NOP", );
OPCODE(0xEDD5, 5, 2, "NOP", );
OPCODE(0xEDD6, 5, 2, "NOP", );
OPCODE(0xEDD7, 5, 2, "NOP", );
OPCODE(0xEDD8, 5, 2, "NOP", );
OPCODE(0xEDD9, 5, 2, "NOP", );
OPCODE(0xEDDA, 5, 2, "NOP", );
OPCODE(0xEDDB, 5, 2, "NOP", );
OPCODE(0xEDDC, 5, 2, "NOP", );
OPCODE(0xEDDD, 5, 2, "NOP", );
OPCODE(0xEDDE, 5, 2, "NOP", );
OPCODE(0xEDDF, 5, 2, "NOP", );
OPCODE(0xEDE0, 5, 2, "NOP", );
OPCODE(0xEDE1, 5, 2, "NOP", );
OPCODE(0xEDE2, 5, 2, "NOP", );
OPCODE(0xEDE3, 5, 2, "NOP", );
OPCODE(0xEDE4, 5, 2, "NOP", );
OPCODE(0xEDE5, 5, 2, "NOP", );
OPCODE(0xEDE6, 5, 2, "NOP", );
OPCODE(0xEDE7, 5, 2, "NOP", );
OPCODE(0xEDE8, 5, 2, "NOP", );
OPCODE(0xEDE9, 5, 2, "NOP", );
OPCODE(0xEDEA, 5, 2, "NOP", );
OPCODE(0xEDEB, 5, 2, "NOP", );
OPCODE(0xEDEC, 5, 2, "NOP", );
OPCODE(0xEDED, 5, 2, "NOP", );
OPCODE(0xEDEE, 5, 2, "NOP", );
OPCODE(0xEDEF, 5, 2, "NOP", );
OPCODE(0xEDF0, 5, 2, "NOP", );
OPCODE(0xEDF1, 5, 2, "NOP", );
OPCODE(0xEDF2, 5, 2, "NOP", );
OPCODE(0xEDF3, 5, 2, "NOP", );
OPCODE(0xEDF4, 5, 2, "NOP", );
OPCODE(0xEDF5, 5, 2, "NOP", );
OPCODE(0xEDF6, 5, 2, "NOP", );
OPCODE(0xEDF7, 5, 2, "NOP", );
OPCODE(0xEDF8, 5, 2, "NOP", );
OPCODE(0xEDF9, 5, 2, "NOP", );
OPCODE(0xEDFA, 5, 2, "NOP", );
OPCODE(0xEDFB, 5, 2, "NOP", );
OPCODE(0xEDFC, 5, 2, "NOP", );
OPCODE(0xEDFD, 5, 2, "NOP", );
OPCODE(0xEDFE, 5, 2, "NOP", );
OPCODE(0xEDFF, 5, 2, "NOP", );

static Z80Opcode opcodes[256] = {
    OPCODE16(, 0), OPCODE16(, 1), OPCODE16(, 2), OPCODE16(, 3),
    OPCODE16(, 4), OPCODE16(, 5), OPCODE16(, 6), OPCODE16(, 7),
    OPCODE16(, 8), OPCODE16(, 9), OPCODE16(, A), OPCODE16(, B),
    OPCODE16(, C), OPCODE16(, D), OPCODE16(, E), OPCODE16(, F)};

static Z80Opcode CBopcodes[256] = {
    OPCODE16(CB, 0), OPCODE16(CB, 1), OPCODE16(CB, 2), OPCODE16(CB, 3),
    OPCODE16(CB, 4), OPCODE16(CB, 5), OPCODE16(CB, 6), OPCODE16(CB, 7),
    OPCODE16(CB, 8), OPCODE16(CB, 9), OPCODE16(CB, A), OPCODE16(CB, B),
    OPCODE16(CB, C), OPCODE16(CB, D), OPCODE16(CB, E), OPCODE16(CB, F),
};

static Z80Opcode EDopcodes[256] = {
    OPCODE16(ED, 0), OPCODE16(ED, 1), OPCODE16(ED, 2), OPCODE16(ED, 3),
    OPCODE16(ED, 4), OPCODE16(ED, 5), OPCODE16(ED, 6), OPCODE16(ED, 7),
    OPCODE16(ED, 8), OPCODE16(ED, 9), OPCODE16(ED, A), OPCODE16(ED, B),
    OPCODE16(ED, C), OPCODE16(ED, D), OPCODE16(ED, E), OPCODE16(ED, F),
};

Z80Cpu::Z80Cpu(Machine *machine, const std::string &name, ClockDivider divider,
               state_type *state)
    : Cpu(machine, name, divider, state) {}

Z80Cpu::~Z80Cpu(void) {}

void Z80Cpu::interrupt(addr_t addr) {
  DEVICE_TRACE("Interrupt: ", Hex(addr));
  PUSH(m_state, m_state->PC.b.h, m_state->PC.b.l);
  m_state->PC.d = addr;
  m_state->WZ.d = addr;
  m_state->iff2 = m_state->iff1;
  m_state->iff1 = false;
  add_icycles(Cycles(20));
}

void Z80Cpu::execute(void) {
  while (true) {
    if (m_state->reset_line == LineState::Pulse) {
      DEVICE_INFO("Reset");
      reset();
      m_state->reset_line = LineState::Clear;
      m_state->halt = false;
    } else if (m_state->nmi_line == LineState::Pulse) {
      m_state->nmi_line = LineState::Clear;
      interrupt(0x0066);
      m_state->halt = false;
    } else if (m_state->int0_line == LineState::Assert && m_state->iff1 &&
               !m_state->iwait) {
      DEVICE_TRACE("Taking interrupt");
      switch (m_state->imode) {
        case 0:
          throw CpuFault(name(), "Unsupported Interrupt mode 0");
          break;
        case 1:
          m_state->int0_line = LineState::Clear;
          interrupt(0x0038);
          break;
        case 2: {
          // XXX: We should do this when we read _data
          m_state->int0_line = LineState::Clear;
          reg16_t irq;
          irq.b.l = bus_read(m_state, (m_state->I << 8) | m_state->data);
          irq.b.h = bus_read(m_state, ((m_state->I << 8) | m_state->data) + 1);
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
      // TODO: Pick a real value
      add_icycles(Cycles(100));
      continue;
    }
    m_state->latch_pc = m_state->PC;
    m_state->latch_op = pc_read(m_state);
    if (m_state->latch_op == 0xDD) {
      m_state->vHL = &m_state->IX;
      m_state->prefix = Z80Prefix::DDPrefix;
      m_state->latch_op = pc_read(m_state);
      m_state->Op = &opcodes[m_state->latch_op];
    } else if (m_state->latch_op == 0xFD) {
      m_state->vHL = &m_state->IY;
      m_state->prefix = Z80Prefix::FDPrefix;
      m_state->latch_op = pc_read(m_state);
      m_state->Op = &opcodes[m_state->latch_op];
    } else if (m_state->latch_op == 0xED) {
      m_state->vHL = &m_state->HL;
      m_state->prefix = Z80Prefix::EDPrefix;
      m_state->latch_op = pc_read(m_state);
      m_state->Op = &EDopcodes[m_state->latch_op];
    } else if (m_state->latch_op == 0xCB) {
      m_state->prefix = Z80Prefix::CBPrefix;
      m_state->vHL = &m_state->HL;
      m_state->latch_op = pc_read(m_state);
      m_state->Op = &CBopcodes[m_state->latch_op];
    } else {
      m_state->vHL = &m_state->HL;
      m_state->prefix = Z80Prefix::NoPrefix;
      m_state->Op = &opcodes[m_state->latch_op];
    }
    m_state->icycles = m_state->Op->cycles;
#if 1
    if (m_state->prefix == Z80Prefix::CBPrefix) {
      OPCODE_SWITCH_BLOCK(0xCB, m_state);
    } else if (m_state->prefix == Z80Prefix::NoPrefix) {
      OPCODE_SWITCH_BLOCK(0x, m_state);
    } else
#endif
      m_state->Op->func(m_state);
    DEVICE_TRACE(Log(m_state));
    add_icycles(m_state->icycles);
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
