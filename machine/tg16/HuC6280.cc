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

#include "machine/tg16/HuC6280.h"
#include "machine/tg16/HuC6280_jit_ops.h"
#include "machine/tg16/HuC6280_ops.h"

using namespace EMU;
using namespace M6502;
using namespace M65C02;
using namespace HuC6280;
using namespace std::placeholders;

#define OPCODE(code, bytes, cycles, name, addr, op, ...)         \
  {                                                              \
    code, name, bytes, cycles, std::bind(&addr<state_type>, _1), \
        std::bind(&op<state_type>, _1, ##__VA_ARGS__),           \
        std::bind(&addr##_jit, _1, _2, _3),                      \
        std::bind(&op##_jit, _1, _2, ##__VA_ARGS__),             \
  }

HuC6280Cpu::HuC6280Cpu(Machine *machine, const std::string &name,
                       ClockDivider divider, AddressBus21x8 *bus)
    : Cpu(machine, name, divider, bus),
      m_irq_status(0),
      m_irq_disable(0),
      m_timer_status(false),
      m_timer_load(0),
      m_timer_value(0) {
  Opcode opcodes[] = {
      OPCODE(0x00, 1, 7, "BRK", Inherent, BRK),
      OPCODE(0x01, 2, 6, "ORA Xind", XIndirect, ORA),
      OPCODE(0x02, 1, 2, "SXY", Inherent, SXY),
      OPCODE(0x03, 2, 4, "ST0 #", Immediate, ST0),
      OPCODE(0x04, 2, 2, "TSB #", Immediate, TSB),
      OPCODE(0x05, 2, 3, "ORA zpg", ZeroPage, ORA),
      OPCODE(0x06, 2, 5, "ASL zpg", ZeroPage, ASL),
      OPCODE(0x07, 2, 5, "RMB 0", ZeroPage, RMB, 0),
      OPCODE(0x08, 1, 3, "PHP", Inherent, PHP),
      OPCODE(0x09, 2, 2, "ORA #", Immediate, ORA),
      OPCODE(0x0A, 1, 2, "ASL A", Inherent, ASLA),
      OPCODE(0x0B, 1, 1, "NOP", Inherent, NOP),
      OPCODE(0x0C, 2, 2, "TSB A", Inherent, TSB),
      OPCODE(0x0D, 3, 4, "ORA abs", Absolute, ORA),
      OPCODE(0x0E, 3, 5, "ASL abs", Absolute, ASL),
      OPCODE(0x0F, 3, 5, "BBR 0", ZeroPage, BBR, 0),
      OPCODE(0x10, 2, 2, "BPL rel", Relative, BPL),
      OPCODE(0x11, 2, 5, "ORA indY", IndirectY, ORA),
      OPCODE(0x12, 2, 5, "ORA ind", ZeroIndirect, ORA),
      OPCODE(0x13, 2, 4, "ST0 #", Immediate, ST1),
      OPCODE(0x14, 2, 2, "TRB zpg", ZeroPage, TRB),
      OPCODE(0x15, 2, 4, "ORA zpgX", ZeroPageX, ORA),
      OPCODE(0x16, 2, 6, "ASL zpgX", ZeroPageX, ASL),
      OPCODE(0x17, 2, 5, "RMB 1", ZeroPage, RMB, 1),
      OPCODE(0x18, 1, 2, "CLC", Inherent, CLC),
      OPCODE(0x19, 3, 4, "ORA absY", AbsoluteY, ORA),
      OPCODE(0x1A, 1, 2, "INC A", Inherent, INA),
      OPCODE(0x1B, 1, 1, "NOP", Inherent, NOP),
      OPCODE(0x1C, 1, 2, "TRB A", Inherent, TRB),
      OPCODE(0x1D, 3, 4, "ORA absX", AbsoluteX, ORA),
      OPCODE(0x1E, 3, 6, "ASL absX", AbsoluteX, ASL),
      OPCODE(0x1F, 3, 5, "BBR 1", ZeroPage, BBR, 1),
      OPCODE(0x20, 3, 6, "JSR", Absolute, JSR),
      OPCODE(0x21, 2, 6, "AND Xind", XIndirect, AND),
      OPCODE(0x22, 1, 2, "SAX", Inherent, SAX),
      OPCODE(0x23, 2, 4, "ST2 #", Immediate, ST2),
      OPCODE(0x24, 2, 3, "BIT zpg", ZeroPage, BIT),
      OPCODE(0x25, 2, 3, "AND zpg", ZeroPage, AND),
      OPCODE(0x26, 2, 5, "ROL zpg", ZeroPage, ROL),
      OPCODE(0x27, 2, 5, "RMB 2", ZeroPage, RMB, 2),
      OPCODE(0x28, 1, 4, "PLP", Inherent, PLP),
      OPCODE(0x29, 2, 2, "AND #", Immediate, AND),
      OPCODE(0x2A, 1, 2, "ROL A", Inherent, ROLA),
      OPCODE(0x2B, 1, 1, "NOP", Inherent, NOP),
      OPCODE(0x2C, 3, 4, "BIT abs", Absolute, BIT),
      OPCODE(0x2D, 3, 4, "AND abs", Absolute, AND),
      OPCODE(0x2E, 3, 6, "ROL abs", Absolute, ROL),
      OPCODE(0x2F, 3, 5, "BBR 2", ZeroPage, BBR, 2),
      OPCODE(0x30, 2, 2, "BMI rel", Relative, BMI),
      OPCODE(0x31, 2, 5, "AND indY", IndirectY, AND),
      OPCODE(0x32, 2, 5, "AND ind", ZeroIndirect, AND),
      OPCODE(0x33, 1, 1, "NOP", Inherent, NOP),
      OPCODE(0x34, 2, 4, "BIT zpgX", ZeroPageX, BIT),
      OPCODE(0x35, 2, 4, "AND zpgX", ZeroPageX, AND),
      OPCODE(0x36, 2, 6, "ROL zpgX", ZeroPageX, ROL),
      OPCODE(0x37, 2, 5, "RMB 3", ZeroPage, RMB, 3),
      OPCODE(0x38, 1, 2, "SEC", Inherent, SEC),
      OPCODE(0x39, 3, 4, "AND absY", AbsoluteY, AND),
      OPCODE(0x3A, 1, 2, "DEC A", Inherent, DEA),
      OPCODE(0x3B, 1, 1, "NOP", Inherent, NOP),
      OPCODE(0x3C, 3, 4, "BIT absX", AbsoluteX, BIT),
      OPCODE(0x3D, 3, 4, "AND absX", AbsoluteX, AND),
      OPCODE(0x3E, 3, 6, "ROL absX", AbsoluteX, ROL),
      OPCODE(0x3F, 3, 5, "BBR 3", ZeroPage, BBR, 3),
      OPCODE(0x40, 1, 6, "RTI", Inherent, RTI),
      OPCODE(0x41, 2, 6, "EOR Xind", XIndirect, EOR),
      OPCODE(0x42, 1, 2, "SAY", Inherent, SAY),
      OPCODE(0x43, 2, 4, "TMA", Immediate, TMA),
      OPCODE(0x44, 2, 5, "BSR rel", Relative, BSR),
      OPCODE(0x45, 2, 3, "EOR zpg", ZeroPage, EOR),
      OPCODE(0x46, 2, 5, "LSR zpg", ZeroPage, LSR),
      OPCODE(0x47, 2, 5, "RMB 4", ZeroPage, RMB, 4),
      OPCODE(0x48, 1, 3, "PHA", Inherent, PHA),
      OPCODE(0x49, 2, 2, "EOR #", Immediate, EOR),
      OPCODE(0x4A, 1, 2, "LSR A", Inherent, LSRA),
      OPCODE(0x4B, 1, 1, "NOP", Inherent, NOP),
      OPCODE(0x4C, 3, 3, "JMP abs", Absolute, JMP),
      OPCODE(0x4D, 3, 4, "EOR abs", Absolute, EOR),
      OPCODE(0x4E, 3, 6, "LSR abs", Absolute, LSR),
      OPCODE(0x4F, 3, 5, "BBR 4", ZeroPage, BBR, 4),
      OPCODE(0x50, 2, 2, "BVC rel", Relative, BVC),
      OPCODE(0x51, 2, 5, "EOR indY", IndirectY, EOR),
      OPCODE(0x52, 2, 5, "EOR ind", ZeroIndirect, EOR),
      OPCODE(0x53, 2, 4, "TAM", Immediate, TAM),
      OPCODE(0x54, 1, 2, "CSL", Inherent, CSL),
      OPCODE(0x55, 2, 4, "EOR zpgX", ZeroPageX, EOR),
      OPCODE(0x56, 2, 6, "LSR zpgX", ZeroPageX, LSR),
      OPCODE(0x57, 2, 5, "RMB 5", ZeroPage, RMB, 5),
      OPCODE(0x58, 1, 2, "CLI", Inherent, CLI),
      OPCODE(0x59, 3, 4, "EOR absY", AbsoluteY, EOR),
      OPCODE(0x5A, 1, 3, "PHY", Inherent, PHY),
      OPCODE(0x5B, 1, 1, "NOP", Inherent, NOP),
      OPCODE(0x5D, 3, 4, "EOR absX", AbsoluteX, EOR),
      OPCODE(0x5E, 3, 6, "LSR absX", AbsoluteX, LSR),
      OPCODE(0x5F, 3, 5, "BBR 5", ZeroPage, BBR, 5),
      OPCODE(0x60, 1, 6, "RTS", Inherent, RTS),
      OPCODE(0x61, 2, 6, "ADC Xind", XIndirect, ADC),
      OPCODE(0x62, 1, 2, "CLA", Inherent, CLA),
      OPCODE(0x63, 1, 1, "NOP", Inherent, NOP),
      OPCODE(0x64, 2, 5, "STZ zpg", ZeroPage, STZ),
      OPCODE(0x65, 2, 3, "ADC zpg", ZeroPage, ADC),
      OPCODE(0x66, 2, 5, "ROR zpg", ZeroPage, ROR),
      OPCODE(0x67, 2, 5, "RMB 6", ZeroPage, RMB, 6),
      OPCODE(0x68, 1, 4, "PLA", Inherent, PLA),
      OPCODE(0x69, 2, 2, "ADC #", Immediate, ADC),
      OPCODE(0x6A, 1, 2, "ROR A", Inherent, RORA),
      OPCODE(0x6B, 1, 1, "NOP", Inherent, NOP),
      OPCODE(0x6C, 3, 5, "JMP ind", Indirect_Fixed, JMP),
      OPCODE(0x6D, 3, 4, "ADC abs", Absolute, ADC),
      OPCODE(0x6E, 3, 6, "ROR abs", Absolute, ROR),
      OPCODE(0x6F, 3, 5, "BBR 6", ZeroPage, BBR, 6),
      OPCODE(0x70, 2, 2, "BVS rel", Relative, BVS),
      OPCODE(0x71, 2, 5, "ADC indY", IndirectY, ADC),
      OPCODE(0x72, 2, 5, "ADC ind", ZeroIndirect, ADC),
      OPCODE(0x73, 7, 12, "TII", Inherent, TII),
      OPCODE(0x74, 2, 5, "STZ zpgX", ZeroPageX, STZ),
      OPCODE(0x75, 2, 4, "ADC zpgX", ZeroPageX, ADC),
      OPCODE(0x76, 2, 6, "ROR zpgX", ZeroPageX, ROR),
      OPCODE(0x77, 2, 5, "RMB 7", ZeroPage, RMB, 7),
      OPCODE(0x78, 1, 2, "SEI", Inherent, SEI),
      OPCODE(0x79, 3, 4, "ADC absY", AbsoluteY, ADC),
      OPCODE(0x7A, 1, 4, "PLY", Inherent, PLY),
      OPCODE(0x7B, 1, 1, "NOP", Inherent, NOP),
      OPCODE(0x7C, 7, 3, "JMP absX", AbsoluteX, JMP),
      OPCODE(0x7D, 3, 4, "ADC absX", AbsoluteX, ADC),
      OPCODE(0x7E, 3, 6, "ROR absX", AbsoluteX, ROR),
      OPCODE(0x7F, 3, 5, "BBR 7", ZeroPage, BBR, 7),
      OPCODE(0x80, 2, 2, "BRA rel", Relative, BRA),
      OPCODE(0x81, 2, 6, "STA Xind", XIndirect, STA),
      OPCODE(0x82, 1, 2, "CLX", Inherent, CLX),
      OPCODE(0x83, 3, 5, "TST Imm, zpg", Immediate, TST_ZPG),
      OPCODE(0x84, 2, 3, "STY zpg", ZeroPage, STY),
      OPCODE(0x85, 2, 3, "STA zpg", ZeroPage, STA),
      OPCODE(0x86, 2, 3, "STX zpg", ZeroPage, STX),
      OPCODE(0x87, 2, 5, "SMB 0", ZeroPage, SMB, 0),
      OPCODE(0x88, 1, 2, "DEY", Inherent, DEY),
      OPCODE(0x89, 2, 2, "BIT #", Immediate, BITIMM),
      OPCODE(0x8A, 1, 2, "TXA", Inherent, TXA),
      OPCODE(0x8B, 1, 1, "NOP", Inherent, NOP),
      OPCODE(0x8C, 3, 4, "STY abs", Absolute, STY),
      OPCODE(0x8D, 3, 4, "STA abs", Absolute, STA),
      OPCODE(0x8E, 3, 4, "STX abs", Absolute, STX),
      OPCODE(0x8F, 3, 5, "BBS 0", ZeroPage, BBS, 0),
      OPCODE(0x90, 2, 2, "BCC rel", Relative, BCC),
      OPCODE(0x91, 2, 6, "STA indY", IndirectY, STA),
      OPCODE(0x92, 2, 5, "STA ind", ZeroIndirect, STA),
      OPCODE(0x93, 4, 5, "TST imm, abs", Immediate, TST_ABS),
      OPCODE(0x94, 2, 4, "STY zpgX", ZeroPageX, STY),
      OPCODE(0x95, 2, 4, "STA zpgX", ZeroPageX, STA),
      OPCODE(0x96, 2, 4, "STX zpg,Y", ZeroPageY, STX),
      OPCODE(0x97, 2, 5, "SMB 1", ZeroPage, SMB, 1),
      OPCODE(0x98, 1, 2, "TYA", Inherent, TYA),
      OPCODE(0x99, 3, 5, "STA absY", AbsoluteY, STA),
      OPCODE(0x9A, 1, 2, "TXS", Inherent, TXS),
      OPCODE(0x9B, 1, 1, "NOP", Inherent, NOP),
      OPCODE(0x9C, 3, 4, "STZ abs", Absolute, STZ),
      OPCODE(0x9D, 3, 5, "STA absX", AbsoluteX, STA),
      OPCODE(0x9E, 3, 5, "STZ absX", AbsoluteX, STZ),
      OPCODE(0x9F, 3, 5, "BBS 1", ZeroPage, BBS, 1),
      OPCODE(0xA0, 2, 2, "LDY #", Immediate, LDY),
      OPCODE(0xA1, 2, 6, "LDA Xind", XIndirect, LDA),
      OPCODE(0xA2, 2, 2, "LDX #", Immediate, LDX),
      OPCODE(0xA3, 3, 5, "TST Imm, zpgX", Immediate, TST_ZPGX),
      OPCODE(0xA4, 2, 3, "LDY zpg", ZeroPage, LDY),
      OPCODE(0xA5, 2, 3, "LDA zpg", ZeroPage, LDA),
      OPCODE(0xA6, 2, 3, "LDX zpg", ZeroPage, LDX),
      OPCODE(0xA7, 2, 5, "SMB 2", ZeroPage, SMB, 2),
      OPCODE(0xA8, 1, 2, "TAY", Inherent, TAY),
      OPCODE(0xA9, 2, 2, "LDA #", Immediate, LDA),
      OPCODE(0xAA, 1, 2, "TAX", Inherent, TAX),
      OPCODE(0xAB, 1, 1, "NOP", Inherent, NOP),
      OPCODE(0xAC, 3, 4, "LDY abs", Absolute, LDY),
      OPCODE(0xAD, 3, 4, "LDA abs", Absolute, LDA),
      OPCODE(0xAE, 3, 4, "LDX abs", Absolute, LDX),
      OPCODE(0xAF, 3, 5, "BBS 2", ZeroPage, BBS, 2),
      OPCODE(0xB0, 2, 2, "BCS rel", Relative, BCS),
      OPCODE(0xB1, 2, 5, "LDA indY", IndirectY, LDA),
      OPCODE(0xB2, 2, 5, "LDA ind", ZeroIndirect, LDA),
      OPCODE(0xB3, 4, 5, "TST imm, absX", Immediate, TST_ABSX),
      OPCODE(0xB4, 2, 4, "LDY zpgX", ZeroPageX, LDY),
      OPCODE(0xB5, 2, 4, "LDA zpgX", ZeroPageX, LDA),
      OPCODE(0xB6, 2, 4, "LDX zpg,Y", ZeroPageY, LDX),
      OPCODE(0xB7, 2, 5, "SMB 3", ZeroPage, SMB, 3),
      OPCODE(0xB8, 1, 2, "CLV", Inherent, CLV),
      OPCODE(0xB9, 3, 4, "LDA absY", AbsoluteY, LDA),
      OPCODE(0xBA, 1, 2, "TSX", Inherent, TSX),
      OPCODE(0xBB, 1, 1, "NOP", Inherent, NOP),
      OPCODE(0xBC, 3, 4, "LDY absX", AbsoluteX, LDY),
      OPCODE(0xBD, 3, 4, "LDA absX", AbsoluteX, LDA),
      OPCODE(0xBE, 3, 4, "LDX absY", AbsoluteY, LDX),
      OPCODE(0xBF, 3, 5, "BBS 3", ZeroPage, BBS, 3),
      OPCODE(0xC0, 2, 2, "CPY #", Immediate, CPY),
      OPCODE(0xC1, 2, 6, "CMP Xind", XIndirect, CMP),
      OPCODE(0xC2, 1, 2, "CLY", Inherent, CLY),
      OPCODE(0xC3, 1, 8, "TDD", Inherent, TDD),
      OPCODE(0xC4, 2, 3, "CPY zpg", ZeroPage, CPY),
      OPCODE(0xC5, 2, 3, "CMP zpg", ZeroPage, CMP),
      OPCODE(0xC6, 2, 5, "DEC zpg", ZeroPage, DEC),
      OPCODE(0xC7, 2, 5, "SMB 4", ZeroPage, SMB, 4),
      OPCODE(0xC8, 1, 2, "INY", Inherent, INY),
      OPCODE(0xC9, 2, 2, "CMP #", Immediate, CMP),
      OPCODE(0xCA, 1, 2, "DEX", Inherent, DEX),
      OPCODE(0xCB, 1, 1, "NOP", Inherent, NOP),
      OPCODE(0xCC, 3, 4, "CPY abs", Absolute, CPY),
      OPCODE(0xCD, 3, 4, "CMP abs", Absolute, CMP),
      OPCODE(0xCE, 3, 3, "DEC abs", Absolute, DEC),
      OPCODE(0xCF, 3, 5, "BBS 4", ZeroPage, BBS, 4),
      OPCODE(0xD0, 2, 2, "BNE rel", Relative, BNE),
      OPCODE(0xD1, 2, 5, "CMP indY", IndirectY, CMP),
      OPCODE(0xD2, 2, 5, "CMP ind", ZeroIndirect, CMP),
      OPCODE(0xD3, 1, 8, "TIN", Inherent, TIN),
      OPCODE(0xD4, 1, 2, "CSH", Inherent, CSH),
      OPCODE(0xD5, 2, 4, "CMP zpgX", ZeroPageX, CMP),
      OPCODE(0xD6, 2, 6, "DEC zpgX", ZeroPageX, DEC),
      OPCODE(0xD7, 2, 5, "SMB 5", ZeroPage, SMB, 5),
      OPCODE(0xD8, 1, 2, "CLD", Inherent, CLD),
      OPCODE(0xD9, 3, 4, "CMP absY", AbsoluteY, CMP),
      OPCODE(0xDA, 1, 4, "PHX", Inherent, PHX),
      OPCODE(0xDB, 1, 1, "NOP", Inherent, NOP),
      OPCODE(0xDD, 3, 4, "CMP absX", AbsoluteX, CMP),
      OPCODE(0xDE, 3, 7, "DEC absX", AbsoluteX, DEC),
      OPCODE(0xDF, 3, 5, "BBS 5", ZeroPage, BBS, 5),
      OPCODE(0xE0, 2, 2, "CPX #", Immediate, CPX),
      OPCODE(0xE1, 2, 6, "SBC Xind", XIndirect, SBC),
      OPCODE(0xE2, 2, 2, "NOP", Immediate, NOP),
      OPCODE(0xE3, 1, 8, "TIA", Inherent, TIA),
      OPCODE(0xE4, 2, 3, "CPX zpg", ZeroPage, CPX),
      OPCODE(0xE5, 2, 3, "SBC zpg", ZeroPage, SBC),
      OPCODE(0xE6, 2, 5, "INC zpg", ZeroPage, INC),
      OPCODE(0xE7, 2, 5, "SMB 6", ZeroPage, SMB, 6),
      OPCODE(0xE8, 1, 2, "INX", Inherent, INX),
      OPCODE(0xE9, 2, 2, "SBC imm", Immediate, SBC),
      OPCODE(0xEA, 1, 2, "NOP", Inherent, NOP),
      OPCODE(0xEB, 2, 2, "SBC", Immediate, NOP),
      OPCODE(0xEC, 3, 4, "CPX abs", Absolute, CPX),
      OPCODE(0xED, 3, 4, "SBC abs", Absolute, SBC),
      OPCODE(0xEE, 3, 6, "INC abs", Absolute, INC),
      OPCODE(0xEF, 3, 5, "BBS 6", ZeroPage, BBS, 6),
      OPCODE(0xF0, 2, 2, "BEQ rel", Relative, BEQ),
      OPCODE(0xF1, 2, 5, "SBC indY", IndirectY, SBC),
      OPCODE(0xF2, 2, 5, "SBC ind", ZeroIndirect, SBC),
      OPCODE(0xF3, 1, 8, "TAI", Inherent, TAI),
      OPCODE(0xF5, 2, 4, "SBC zpgX", ZeroPageX, SBC),
      OPCODE(0xF6, 2, 6, "INC zpgX", ZeroPageX, INC),
      OPCODE(0xF7, 2, 5, "SMB 7", ZeroPage, SMB, 7),
      OPCODE(0xF8, 1, 2, "SED", Inherent, SED),
      OPCODE(0xF9, 3, 4, "SBC absY", AbsoluteY, SBC),
      OPCODE(0xFA, 1, 4, "PLX", Inherent, PLX),
      OPCODE(0xFB, 1, 1, "NOP", Inherent, NOP),
      OPCODE(0xFD, 3, 4, "SBC absX", AbsoluteX, SBC),
      OPCODE(0xFE, 3, 7, "INC absX", AbsoluteX, INC),
      OPCODE(0xFF, 3, 5, "BBS 7", ZeroPage, BBS, 7),
  };

  for (int i = 0; i < sizeof(opcodes) / sizeof(opcodes[0]); i++) {
    m_opcodes[opcodes[i].code] = opcodes[i];
  }

  add_debug_var("A", m_state.A);
  add_debug_var("SP", m_state.SP);
  add_debug_var("X", m_state.X);
  add_debug_var("Y", m_state.Y);
  add_debug_var("SR", m_state.SR);
  add_debug_var("PC", m_state.PC);
  add_debug_var("EA", m_state.EA);
  add_debug_var("ARG", m_state.ARG);

  m_state.reset();
}

HuC6280Cpu::~HuC6280Cpu(void) {}

void HuC6280Cpu::reset(void) {
  m_state.reset();

  m_state.mmu_map[0] = 0xFF;
  m_state.mmu_map[1] = 0xF8;
  m_state.mmu_map[2] = 0x00;
  m_state.mmu_map[3] = 0x00;
  m_state.mmu_map[4] = 0x00;
  m_state.mmu_map[5] = 0x00;
  m_state.mmu_map[6] = 0x00;
  m_state.mmu_map[7] = 0x00;

  m_state.PC.b.l = bus_read(0xFFFE);
  m_state.PC.b.h = bus_read(0xFFFF);
}

bool HuC6280Cpu::Interrupt(void) {
  if (m_timer_status && m_timer_value < 0) {
    bit_set(m_irq_status, 2, true);
    while (m_timer_value <= 0) m_timer_value += m_timer_load;
  }

  if (m_reset_line == LineState::Pulse) {
    DEVICE_DEBUG("Reset triggered");
    reset();
    m_reset_line = LineState::Clear;
    return true;
  } else if (m_nmi_line == LineState::Pulse) {
    DEVICE_DEBUG("NMI triggered");
    NMI(&m_state, 0xFFFC);
    m_nmi_line = LineState::Clear;
    return true;
  } else if (m_state.F.I == 0) {
    uint16_t addr = 0x0000;
    if (bit_isset(m_irq_status, 0) && !bit_isset(m_irq_disable, 0))
      addr = 0xFFF6;
    else if (bit_isset(m_irq_status, 1) && !bit_isset(m_irq_disable, 1))
      addr = 0xFFF8;
    else if (bit_isset(m_irq_status, 2) && !bit_isset(m_irq_disable, 2))
      addr = 0xFFFA;
    if (addr != 0x0000) {
      DEVICE_DEBUG("Interrupt");
      IRQ(&m_state, addr);
      return true;
    }
  }
  return false;
}

void HuC6280Cpu::execute(void) {
  while (true) {
    if (Interrupt()) continue;
    uint16_t pc = m_state.PC.d;
    unsigned cycles = dispatch(pc);
    m_timer_value -= cycles;
  }
}

void HuC6280Cpu::line(Line line, LineState state) {
  switch (line) {
    case Line::RESET:
      reset();
      break;
    case Line::INT2:
      bit_set(m_irq_status, 0, state == LineState::Assert);
      break;
    case Line::INT1:
      bit_set(m_irq_status, 1, state == LineState::Assert);
      break;
    case Line::INT0:
      bit_set(m_irq_status, 2, state == LineState::Assert);
      break;
    default:
      break;
  }
  if (line != Line::RESET) Task::yield();
}

byte_t HuC6280Cpu::timer_read(offset_t offset) {
  return (m_timer_value >> 10) & 0x7F;
}

void HuC6280Cpu::timer_write(offset_t offset, byte_t value) {
  switch (offset & 0x01) {
    case 0:
      m_timer_value = m_timer_load = ((value & 0x7F) + 1) << 10;
      break;
    case 1:
      m_timer_status = bit_isset(value, 0);
      break;
  }
}

byte_t HuC6280Cpu::irq_read(offset_t offset) {
  switch (offset & 0x03) {
    case 2:
      return m_irq_disable;
    case 3:
      return m_irq_status;
  }
  return 0;
}

void HuC6280Cpu::irq_write(offset_t offset, byte_t value) {
  switch (offset & 0x03) {
    case 2:
      m_irq_disable = value;
      break;
    case 3:
      bit_set(m_irq_status, 2, false);
      break;
  }
}

void HuC6280Cpu::log_op(HuC6280State *state, const Opcode *opcode, uint16_t pc, const uint8_t *instr) {
  std::stringstream os;
  const std::string &str = opcode->name;
  os << Hex(pc) << ":";
  os << Hex(opcode->code) << ":";
  const std::string &delimiters = " ,";
  auto lastPos = str.find_first_not_of(delimiters, 0);
  auto pos = str.find_first_of(delimiters, lastPos);
  std::stringstream op;
  while (std::string::npos != pos || std::string::npos != lastPos) {
    std::string it = str.substr(lastPos, pos - lastPos);
    op << " ";
    if (it == "zpg") {
      op << "ZPG+" << Hex(state->EA.b.l);
    } else if (it == "abs") {
      op << Hex(state->EA);
    } else if (it == "zpgX") {
      op << "(ZPG+X)";
    } else if (it == "rel") {
      op << Hex(state->ARG);
    } else if (it == "zpgY") {
      op << "(ZPG+Y)";
    } else if (it == "ind") {
      op << "(ZPG+" << Hex(state->EA.b.l) << ")";
    } else if (it == "Xind") {
      op << "(" << Hex(state->EA.b.l) << ",X)";
    } else if (it == "indY") {
      op << "(" << Hex(state->EA.b.l) << "),Y";
    } else if (it == "absX") {
      op << Hex(state->EA) << "+X";
    } else if (it == "absY") {
      op << Hex(state->EA) << "+Y";
    } else if (it == "#") {
      op << Hex(state->ARG);
    } else
      op << it;
    lastPos = str.find_first_not_of(delimiters, pos);
    pos = str.find_first_of(delimiters, lastPos);
  }
  os << std::setfill(' ') << std::left << std::setw(20) << op.str();
  os << "  CPU:";
  os << " PC: " << Hex(m_state.PC);
  os << " A: " << Hex(m_state.A);
  os << " X: " << Hex(m_state.X);
  os << " Y: " << Hex(m_state.Y);
  os << " F: " << Hex(m_state.SR);
  os << " S: " << Hex(m_state.SP);
  os << " EA: " << Hex(m_state.EA);
  os << " ARG: " << Hex(m_state.ARG);

  DEVICE_TRACE(os.str());
}
