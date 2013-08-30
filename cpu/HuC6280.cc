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

#include "HuC6280.h"

HuC6280Cpu::HuC6280Cpu(Machine *machine, const std::string &name,
                       unsigned clock, bus_type *bus):
    M65C02Cpu(machine, name, clock, bus)
{

}

HuC6280Cpu::~HuC6280Cpu(void)
{
}

#define OPCODE(op, name, addr, oper)  { op, name, \
    std::bind(&M6502Cpu::addr, this), \
    std::bind(&M6502Cpu::oper, this) }

#define OPCODE1(op, name, addr, oper, arg1)  { op, name, \
    std::bind(&HuC6280Cpu::addr, this), \
    std::bind(&HuC6280Cpu::oper, this, arg1) }

void
HuC6280Cpu::build_table(void)
{
    M6502Opcode opcodes[] = {
        OPCODE(0x00, "BRK", Inherent, BRK),
        OPCODE(0x01, "ORA X,ind", XIndirect, ORA),
        OPCODE(0x02, "SXY", Inherent, SXY),
        OPCODE(0x03, "ST0 #", Immediate, ST0),
        OPCODE(0x04, "TSB #", Immediate, TSB),
        OPCODE(0x05, "ORA zpg", ZeroPage, ORA),
        OPCODE(0x06, "ASL zpg", ZeroPage, ASL),
        OPCODE1(0x07, "RMB 0 zpg", ZeroPage, RMB, 0),
        OPCODE(0x08, "PHP", Inherent, PHP),
        OPCODE(0x09, "ORA #", Immediate, ORA),
        OPCODE(0x0A, "ASL A", Inherent, ASLA),
        OPCODE(0x0C, "TSB A", Inherent, TSBA),
        OPCODE(0x0D, "ORA abs", Absolute, ORA),
        OPCODE(0x0E, "ASL abs", Absolute, ASL),
        OPCODE1(0x0F, "BBR 0 zpg", ZeroPage, BBR, 0),

        OPCODE(0x10, "BPL", Relative, BPL),
        OPCODE(0x11, "ORA ind,Y", IndirectY, ORA),
        OPCODE(0x12, "ORA ind", ZeroPageIndirect, ORA),
        OPCODE(0x13, "ST1 #", Immediate, ST1),
        OPCODE(0x14, "TRB zpg", ZeroPage, TRB),
        OPCODE(0x15, "ORA zpg,X", ZeroPageX, ORA),
        OPCODE(0x16, "ASL zpg,X", ZeroPageX, ASL),
        OPCODE1(0x17, "RMB 1 zpg", ZeroPage, RMB, 1),
        OPCODE(0x18, "CLC", Inherent, CLC),
        OPCODE(0x19, "ORA abs,Y", AbsoluteY, ORA),
        OPCODE(0x1A, "INA", Inherent, INA),
        OPCODE(0x1C, "TRB A", Inherent, TRBA),
        OPCODE(0x1D, "ORA abs,X", AbsoluteX, ORA),
        OPCODE(0x1E, "ASL abs,X", AbsoluteX, ASL),
        OPCODE1(0x1F, "BBR 1 zpg", ZeroPage, BBR, 1),

        OPCODE(0x20, "JSR", Absolute, JSR),
        OPCODE(0x21, "AND X,ind", XIndirect, AND),
        OPCODE(0x22, "SAX", Inherent, SAX),
        OPCODE(0x23, "ST2 #", Immediate, ST2
        OPCODE(0x24, "BIT zpg", ZeroPage, BIT),
        OPCODE(0x25, "AND zpg", ZeroPage, AND),
        OPCODE(0x26, "ROL zpg", ZeroPage, ROL),
        OPCODE1(0x27, "RMB 2 zpg", ZeroPage, RMB, 2),
        OPCODE(0x28, "PLP", Inherent, PLP),
        OPCODE(0x29, "AND #", Immediate, AND),
        OPCODE(0x2A, "ROL A", Inherent, ROLA),
        OPCODE(0x2C, "BIT abs", Absolute, BIT),
        OPCODE(0x2D, "AND abs", Absolute, AND),
        OPCODE(0x2E, "ROL abs", Absolute, ROL),
        OPCODE1(0x2F, "BBR 2 zpg", ZeroPage, BBR, 2),

        OPCODE(0x30, "BMI", Relative, BMI),
        OPCODE(0x31, "AND ind,Y", IndirectY, AND),
        OPCODE(0x32, "AND ind", Indirect, AND),
        OPCODE(0x34, "BIT zpg,X", ZeroPageIndirect, BIT),
        OPCODE(0x35, "AND zpg,X", ZeroPageX, AND),
        OPCODE(0x36, "ROL zpg,X", ZeroPageX, ROL),
        OPCODE1(0x37, "RMB 3 zpg", ZeroPage, RMB, 3),
        OPCODE(0x38, "SEC", Inherent, SEC),
        OPCODE(0x39, "AND abs,Y", AbsoluteY, AND),
        OPCODE(0x3A, "DEA", Inherent, DEA),
        OPCODE(0x3D, "AND abs,X", AbsoluteX, AND),
        OPCODE(0x3E, "ROL abs,X", AbsoluteX, ROL),
        OPCODE1(0x3F, "BBR 3 zpg", ZeroPage, BBR, 3),

        OPCODE(0x40, "RTI", Inherent, RTI),
        OPCODE(0x41, "EOR X,ind", XIndirect, EOR),
        OPCODE(0x42, "SAY", Inherent, SAY),
        OPCODE(0x43, "TMA #", Immediate, TMA),
        OPCODE(0x44, "BSR", Relative, BSR),
        OPCODE(0x45, "EOR zpg", ZeroPage, EOR),
        OPCODE(0x46, "LSR zpg", ZeroPage, LSR),
        OPCODE1(0x47, "RMB 4 zpg", ZeroPage, RMB, 4),
        OPCODE(0x48, "PHA", Inherent, PHA),
        OPCODE(0x49, "EOR #", Immediate, EOR),
        OPCODE(0x4A, "LSR A", Inherent, LSRA),
        OPCODE(0x4C, "JMP abs", Absolute, JMP),
        OPCODE(0x4D, "EOR abs", Absolute, EOR),
        OPCODE(0x4E, "LSR abs", Absolute, LSR),
        OPCODE1(0x4F, "BBR 4 zpg", ZeroPage, BBR, 4),

        OPCODE(0x50, "BVC", Relative, BVC),
        OPCODE(0x51, "EOR ind,Y", IndirectY, EOR),
        OPCODE(0x52, "EOR (zpg)", ZeroPageIndirect, EOR),
        OPCODE(0x53, "TAM #", Immediate, TAM),
        OPCODE(0x54, "CSL", Inherent, CSL),
        OPCODE(0x55, "EOR zpg,X", ZeroPageX, EOR),
        OPCODE(0x56, "LSR zpg,X", ZeroPageX, LSR),
        OPCODE1(0x57, "RMB 5 zpg", ZeroPage, RMB, 5),
        OPCODE(0x58, "CLI", Inherent, CLI),
        OPCODE(0x59, "EOR abs,Y", AbsoluteY, EOR),
        OPCODE(0x5A, "PHY", Inherent, PHY),
        OPCODE(0x5D, "EOR abs,X", AbsoluteX, EOR),
        OPCODE(0x5E, "LSR abs,X", AbsoluteX, LSR),
        OPCODE1(0x5F, "BBR 5 zpg", ZeroPage, BBR, 5),

        OPCODE(0x60, "RTS", Inherent, RTS),
        OPCODE(0x61, "ADC X,ind", XIndirect, ADC),
        OPCODE(0x62, "CLA", Inherent, CLA),
        OPCODE(0x64, "STZ zpg", ZeroPage, STZ),
        OPCODE(0x65, "ADC zpg", ZeroPage, ADC),
        OPCODE(0x66, "ROR zpg", ZeroPage, ROR),
        OPCODE1(0x67, "RMB 6 zpg", ZeroPage, RMB, 6),
        OPCODE(0x68, "PLA", Inherent, PLA),
        OPCODE(0x69, "ADC #", Immediate, ADC),
        OPCODE(0x6A, "ROR A", Inherent, RORA),
        OPCODE(0x6C, "JMP ind", Indirect, JMP),
        OPCODE(0x6D, "ADC abs", Absolute, ADC),
        OPCODE(0x6E, "ROR abs", Absolute, ROR),
        OPCODE1(0x6F, "BBR 6 zpg", ZeroPage, BBR, 6),

        OPCODE(0x70, "BVS", Relative, BVS),
        OPCODE(0x71, "ADC ind,Y", IndirectY, ADC),
        OPCODE(0x72, "ADC (zpg)", ZeroPageIndirect, ADC),
        OPCODE(0x73, "TII", Inherent, TII),
        OPCODE(0x74, "STZ zpg,X", ZeroPageX, STZ),
        OPCODE(0x75, "ADC zpg,X", ZeroPageX, ADC),
        OPCODE(0x76, "ROR zpg,X", ZeroPageX, ROR),
        OPCODE1(0x77, "RMB 7 zpg", ZeroPage, RMB, 7),
        OPCODE(0x78, "SEI", Inherent, SEI),
        OPCODE(0x79, "ADC abs,Y", AbsoluteY, ADC),
        OPCODE(0x7A, "PLY", Inherent, PLY),
        OPCODE(0x7C, "JMP (abs,X)", IndirectX, JMP),
        OPCODE(0x7D, "ADC abs,X", AbsoluteX, ADC),
        OPCODE(0x7E, "ROR abs,X", AbsoluteX, ROR),
        OPCODE1(0x7F, "BBR 7 zpg", ZeroPage, BBR, 7),

        OPCODE(0x81, "STA X,ind", XIndirect, STA),
        OPCODE(0x84, "STY zpg", ZeroPage, STY),
        OPCODE(0x85, "STA zpg", ZeroPage, STA),
        OPCODE(0x86, "STX zpg", ZeroPage, STX),
        OPCODE(0x88, "DEY", Inherent, DEY),
        OPCODE(0x8A, "TXA", Inherent, TXA),
        OPCODE(0x8C, "STY abs", Absolute, STY),
        OPCODE(0x8D, "STA abs", Absolute, STA),
        OPCODE(0x8E, "STX abs", Absolute, STX),
        OPCODE(0x90, "BCC", Relative, BCC),
        OPCODE(0x91, "STA ind,Y", IndirectY, STA),
        OPCODE(0x94, "STY zpg,X", ZeroPageX, STY),
        OPCODE(0x95, "STA zpg,X", ZeroPageX, STA),
        OPCODE(0x96, "STX zpg,Y", ZeroPageY, STX),
        OPCODE(0x98, "TYA", Inherent, TYA),
        OPCODE(0x99, "STA abs,Y", AbsoluteY, STA),
        OPCODE(0x9A, "TXS", Inherent, TXS),
        OPCODE(0x9D, "STA abs,X", AbsoluteX, STA),
        OPCODE(0xA0, "LDY #", Immediate, LDY),
        OPCODE(0xA1, "LDA X,ind", XIndirect, LDA),
        OPCODE(0xA2, "LDX #", Immediate, LDX),
        OPCODE(0xA4, "LDY zpg", ZeroPage, LDY),
        OPCODE(0xA5, "LDA zpg", ZeroPage, LDA),
        OPCODE(0xA6, "LDX zpg", ZeroPage, LDX),
        OPCODE(0xA8, "TAY", Inherent, TAY),
        OPCODE(0xA9, "LDA #", Immediate, LDA),
        OPCODE(0xAA, "TAX", Inherent, TAX),
        OPCODE(0xAC, "LDY abs", Absolute, LDY),
        OPCODE(0xAD, "LDA abs", Absolute, LDA),
        OPCODE(0xAE, "LDX abs", Absolute, LDX),
        OPCODE(0xB0, "BCS", Relative, BCS),
        OPCODE(0xB1, "LDA ind,Y", IndirectY, LDA),
        OPCODE(0xB4, "LDY zpg,X", ZeroPageX, LDY),
        OPCODE(0xB5, "LDA zpg,X", ZeroPageX, LDA),
        OPCODE(0xB6, "LDX zpg,Y", ZeroPageY, LDX),
        OPCODE(0xB8, "CLV", Inherent, CLV),
        OPCODE(0xB9, "LDA abs,Y", AbsoluteY, LDA),
        OPCODE(0xBA, "TSX", Inherent, TSX),
        OPCODE(0xBC, "LDY abs,X", AbsoluteX, LDY),
        OPCODE(0xBD, "LDA abs,X", AbsoluteX, LDA),
        OPCODE(0xBE, "LDX abs,Y", AbsoluteY, LDX),
        OPCODE(0xC0, "CPY #", Immediate, CPY),
        OPCODE(0xC1, "CMP X,ind", XIndirect, CMP),
        OPCODE(0xC4, "CPY zpg", ZeroPage, CPY),
        OPCODE(0xC5, "CMP zpg", ZeroPage, CMP),
        OPCODE(0xC6, "DEC zpg", ZeroPage, DEC),
        OPCODE(0xC8, "INY", Inherent, INY),
        OPCODE(0xC9, "CMP #", Immediate, CMP),
        OPCODE(0xCA, "DEX", Inherent, DEX),
        OPCODE(0xCC, "CPY abs", Absolute, CPY),
        OPCODE(0xCD, "CMP abs", Absolute, CMP),
        OPCODE(0xCE, "DEC abs", Absolute, DEC),
        OPCODE(0xD0, "BNE", Relative, BNE),
        OPCODE(0xD1, "CMP ind,Y", IndirectY, CMP),
        OPCODE(0xD5, "CMP zpg,X", ZeroPageX, CMP),
        OPCODE(0xD6, "DEC zpg,X", ZeroPageX, DEC),
        OPCODE(0xD8, "CLD", Inherent, CLD),
        OPCODE(0xD9, "CMP abs,Y", AbsoluteY, CMP),
        OPCODE(0xDD, "CMP abs,X", AbsoluteX, CMP),
        OPCODE(0xDE, "DEC abs,X", AbsoluteX, DEC),
        OPCODE(0xE0, "CPX #", Immediate, CPX),
        OPCODE(0xE1, "SBC X,ind", XIndirect, SBC),
        OPCODE(0xE4, "CPX zpg", ZeroPage, CPX),
        OPCODE(0xE5, "SBC zpg", ZeroPage, SBC),
        OPCODE(0xE6, "INC zpg", ZeroPage, INC),
        OPCODE(0xE8, "INX", Inherent, INX),
        OPCODE(0xE9, "SBC imm", Immediate, SBC),
        OPCODE(0xEA, "NOP", Inherent, NOP),
        OPCODE(0xEB, "SBC", Immediate, NOP), /* XXX: undocumented */
        OPCODE(0xEC, "CPX abs", Absolute, CPX),
        OPCODE(0xED, "SBC abs", Absolute, SBC),
        OPCODE(0xEE, "INC abs", Absolute, INC),
        OPCODE(0xF0, "BEQ", Relative, BEQ),
        OPCODE(0xF1, "SBC ind,Y", IndirectY, SBC),
        OPCODE(0xF5, "SBC zpg,X", ZeroPageX, SBC),
        OPCODE(0xF6, "INC zpg,X", ZeroPageX, INC),
        OPCODE(0xF8, "SED", Inherent, SED),
        OPCODE(0xF9, "SBC abs,Y", AbsoluteY, SBC),
        OPCODE(0xFD, "SBC abs,X", AbsoluteX, SBC),
        OPCODE(0xFE, "INC abs,X", AbsoluteX, INC),
    };

    for (int i = 0; i < sizeof(opcodes)/sizeof(opcodes[0]); i++) {
        _opcodes[opcodes[i].code] = opcodes[i];
    }

}

