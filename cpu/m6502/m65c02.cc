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

#include "cpu/m6502/m65c02.h"
#include "cpu/m6502/m6502_ops.h"
#include "cpu/m6502/m6502_jit_ops.h"

using namespace EMU;
using namespace M6502v2;
using namespace M65C02v2;
using namespace std::placeholders;

#define OPCODE(code, bytes, cycles, name, addr, op, ...) { \
    code, \
    name, \
    bytes, \
    cycles, \
    std::bind(&addr, _1), \
    std::bind(&op, _1, ##__VA_ARGS__), \
    std::bind(&addr ## _jit, _1, _2, _3), \
    std::bind(&op ## _jit, _1, _2, ##__VA_ARGS__), \
}

M65c02Cpu::M65c02Cpu(Machine *machine, const std::string &name, unsigned hertz,
    AddressBus16 *bus):
    M6502Cpu(machine, name, hertz, bus)
{
    Opcode opcodes[] = {
        OPCODE(0x00, 1, 7, "BRK", Inherent, BRK),
        OPCODE(0x01, 2, 6, "ORA X,ind", XIndirect, ORA),
        OPCODE(0x04, 2, 2, "TSB #", Immediate, TSB),
        OPCODE(0x05, 2, 3, "ORA zpg", ZeroPage, ORA),
        OPCODE(0x06, 2, 5, "ASL zpg", ZeroPage, ASL),
        OPCODE(0x07, 2, 5, "RMB 0", ZeroPage, RMB, 0),
        OPCODE(0x08, 1, 3, "PHP", Inherent, PHP),
        OPCODE(0x09, 2, 2, "ORA #", Immediate, ORA),
        OPCODE(0x0A, 1, 2, "ASL A", Inherent, ASLA),
        OPCODE(0x0C, 2, 2, "TSB A", Inherent, TSB),
        OPCODE(0x0D, 3, 4, "ORA abs", Absolute, ORA),
        OPCODE(0x0E, 3, 5, "ASL abs", Absolute, ASL),
        OPCODE(0x0F, 3, 5, "BBR 0", ZeroPage, BBR, 0),
        OPCODE(0x10, 2, 2, "BPL", Relative, BPL),
        OPCODE(0x11, 2, 5, "ORA ind,Y", IndirectY, ORA),
        OPCODE(0x12, 2, 5, "ORA ind", ZeroIndirect, ORA),
        OPCODE(0x15, 2, 4, "ORA zpg,X", ZeroPageX, ORA),
        OPCODE(0x16, 2, 6, "ASL zpg,X", ZeroPageX, ASL),
        OPCODE(0x17, 2, 5, "RMB 1", ZeroPage, RMB, 1),
        OPCODE(0x18, 1, 2, "CLC", Inherent, CLC),
        OPCODE(0x19, 3, 4, "ORA abs,Y", AbsoluteY, ORA),
        OPCODE(0x1A, 1, 2, "INC A", Inherent, INA),
        OPCODE(0x1C, 1, 2, "TRB A", Inherent, TRB),
        OPCODE(0x1D, 3, 4, "ORA abs,X", AbsoluteX, ORA),
        OPCODE(0x1E, 3, 7, "ASL abs,X", AbsoluteX, ASL),
        OPCODE(0x1F, 3, 5, "BBR 1", ZeroPage, BBR, 1),
        OPCODE(0x20, 3, 6, "JSR", Absolute, JSR),
        OPCODE(0x21, 2, 6, "AND X,ind", XIndirect, AND),
        OPCODE(0x24, 2, 3, "BIT zpg", ZeroPage, BIT),
        OPCODE(0x25, 2, 3, "AND zpg", ZeroPage, AND),
        OPCODE(0x26, 2, 5, "ROL zpg", ZeroPage, ROL),
        OPCODE(0x27, 2, 5, "RMB 2", ZeroPage, RMB, 2),
        OPCODE(0x28, 1, 4, "PLP", Inherent, PLP),
        OPCODE(0x29, 2, 2, "AND #", Immediate, AND),
        OPCODE(0x2A, 1, 2, "ROL A", Inherent, ROLA),
        OPCODE(0x2C, 3, 4, "BIT abs", Absolute, BIT),
        OPCODE(0x2D, 3, 4, "AND abs", Absolute, AND),
        OPCODE(0x2E, 3, 6, "ROL abs", Absolute, ROL),
        OPCODE(0x2F, 3, 5, "BBR 2", ZeroPage, BBR, 2),
        OPCODE(0x30, 2, 2, "BMI", Relative, BMI),
        OPCODE(0x31, 2, 5, "AND ind,Y", IndirectY, AND),
        OPCODE(0x32, 2, 5, "AND ind", ZeroIndirect, AND),
        OPCODE(0x34, 2, 4, "BIT zpg,X", ZeroPageX, BIT),
        OPCODE(0x35, 2, 4, "AND zpg,X", ZeroPageX, AND),
        OPCODE(0x36, 2, 6, "ROL zpg,X", ZeroPageX, ROL),
        OPCODE(0x37, 2, 5, "RMB 3", ZeroPage, RMB, 3),
        OPCODE(0x38, 1, 2, "SEC", Inherent, SEC),
        OPCODE(0x39, 3, 4, "AND abs,Y", AbsoluteY, AND),
        OPCODE(0x3A, 1, 2, "DEC A", Inherent, DEA),
        OPCODE(0x3C, 3, 4, "BIT abs,X", AbsoluteX, BIT),
        OPCODE(0x3D, 3, 4, "AND abs,X", AbsoluteX, AND),
        OPCODE(0x3E, 3, 7, "ROL abs,X", AbsoluteX, ROL),
        OPCODE(0x3F, 3, 5, "BBR 3", ZeroPage, BBR, 3),
        OPCODE(0x40, 1, 6, "RTI", Inherent, RTI),
        OPCODE(0x41, 2, 6, "EOR X,ind", XIndirect, EOR),
        OPCODE(0x45, 2, 3, "EOR zpg", ZeroPage, EOR),
        OPCODE(0x46, 2, 5, "LSR zpg", ZeroPage, LSR),
        OPCODE(0x47, 2, 5, "RMB 4", ZeroPage, RMB, 4),
        OPCODE(0x48, 1, 3, "PHA", Inherent, PHA),
        OPCODE(0x49, 2, 2, "EOR #", Immediate, EOR),
        OPCODE(0x4A, 1, 2, "LSR A", Inherent, LSRA),
        OPCODE(0x4C, 3, 3, "JMP abs", Absolute, JMP),
        OPCODE(0x4D, 3, 4, "EOR abs", Absolute, EOR),
        OPCODE(0x4E, 3, 6, "LSR abs", Absolute, LSR),
        OPCODE(0x4F, 3, 5, "BBR 4", ZeroPage, BBR, 4),
        OPCODE(0x50, 2, 2, "BVC", Relative, BVC),
        OPCODE(0x51, 2, 5, "EOR ind,Y", IndirectY, EOR),
        OPCODE(0x52, 2, 5, "EOR ind", ZeroIndirect, EOR),
        OPCODE(0x55, 2, 4, "EOR zpg,X", ZeroPageX, EOR),
        OPCODE(0x56, 2, 6, "LSR zpg,X", ZeroPageX, LSR),
        OPCODE(0x57, 2, 5, "RMB 5", ZeroPage, RMB, 5),
        OPCODE(0x58, 1, 2, "CLI", Inherent, CLI),
        OPCODE(0x59, 3, 4, "EOR abs,Y", AbsoluteY, EOR),
        OPCODE(0x5A, 1, 3, "PHY", Inherent, PHY),
        OPCODE(0x5D, 3, 4, "EOR abs,X", AbsoluteX, EOR),
        OPCODE(0x5E, 3, 7, "LSR abs,X", AbsoluteX, LSR),
        OPCODE(0x5F, 3, 5, "BBR 5", ZeroPage, BBR, 5),
        OPCODE(0x60, 1, 6, "RTS", Inherent, RTS),
        OPCODE(0x61, 2, 6, "ADC X,ind", XIndirect, ADC),
        OPCODE(0x64, 2, 5, "STZ zpg", ZeroPage, STZ),
        OPCODE(0x65, 2, 3, "ADC zpg", ZeroPage, ADC),
        OPCODE(0x66, 2, 5, "ROR zpg", ZeroPage, ROR),
        OPCODE(0x67, 2, 5, "RMB 6", ZeroPage, RMB, 6),
        OPCODE(0x68, 1, 4, "PLA", Inherent, PLA),
        OPCODE(0x69, 2, 2, "ADC #", Immediate, ADC),
        OPCODE(0x6A, 1, 2, "ROR A", Inherent, RORA),
        OPCODE(0x6C, 3, 5, "JMP ind", Indirect, JMP),
        OPCODE(0x6D, 3, 4, "ADC abs", Absolute, ADC),
        OPCODE(0x6E, 3, 6, "ROR abs", Absolute, ROR),
        OPCODE(0x6F, 3, 5, "BBR 6", ZeroPage, BBR, 6),
        OPCODE(0x70, 2, 2, "BVS", Relative, BVS),
        OPCODE(0x71, 2, 5, "ADC ind,Y", IndirectY, ADC),
        OPCODE(0x72, 2, 5, "ADC ind", ZeroIndirect, ADC),
        OPCODE(0x74, 2, 5, "STZ zpg,X", ZeroPageX, STZ),
        OPCODE(0x75, 2, 4, "ADC zpg,X", ZeroPageX, ADC),
        OPCODE(0x76, 2, 6, "ROR zpg,X", ZeroPageX, ROR),
        OPCODE(0x77, 2, 5, "RMB 7", ZeroPage, RMB, 7),
        OPCODE(0x78, 1, 2, "SEI", Inherent, SEI),
        OPCODE(0x79, 3, 4, "ADC abs,Y", AbsoluteY, ADC),
        OPCODE(0x7A, 1, 4, "PLY", Inherent, PLY),
        OPCODE(0x7D, 3, 4, "ADC abs,X", AbsoluteX, ADC),
        OPCODE(0x7E, 3, 7, "ROR abs,X", AbsoluteX, ROR),
        OPCODE(0x7F, 3, 5, "BBR 7", ZeroPage, BBR, 7),
        OPCODE(0x80, 2, 2, "BRA", Relative, BRA),
        OPCODE(0x81, 2, 6, "STA X,ind", XIndirect, STA),
        OPCODE(0x84, 2, 3, "STY zpg", ZeroPage, STY),
        OPCODE(0x85, 2, 3, "STA zpg", ZeroPage, STA),
        OPCODE(0x86, 2, 3, "STX zpg", ZeroPage, STX),
        OPCODE(0x87, 2, 5, "SMB 0", ZeroPage, SMB, 0),
        OPCODE(0x88, 1, 2, "DEY", Inherent, DEY),
        OPCODE(0x89, 2, 2, "BIT #", Immediate, BITIMM),
        OPCODE(0x8A, 1, 2, "TXA", Inherent, TXA),
        OPCODE(0x8C, 3, 4, "STY abs", Absolute, STY),
        OPCODE(0x8D, 3, 4, "STA abs", Absolute, STA),
        OPCODE(0x8E, 3, 4, "STX abs", Absolute, STX),
        OPCODE(0x8F, 3, 5, "BBS 0", ZeroPage, BBS, 0),
        OPCODE(0x90, 2, 2, "BCC", Relative, BCC),
        OPCODE(0x91, 2, 6, "STA ind,Y", IndirectY, STA),
        OPCODE(0x92, 2, 5, "STA ind", ZeroIndirect, STA),
        OPCODE(0x94, 2, 4, "STY zpg,X", ZeroPageX, STY),
        OPCODE(0x95, 2, 4, "STA zpg,X", ZeroPageX, STA),
        OPCODE(0x96, 2, 4, "STX zpg,Y", ZeroPageY, STX),
        OPCODE(0x97, 2, 5, "SMB 1", ZeroPage, SMB, 1),
        OPCODE(0x98, 1, 2, "TYA", Inherent, TYA),
        OPCODE(0x99, 3, 5, "STA abs,Y", AbsoluteY, STA),
        OPCODE(0x9A, 1, 2, "TXS", Inherent, TXS),
        OPCODE(0x9C, 3, 4, "STZ abs", Absolute, STZ),
        OPCODE(0x9D, 3, 5, "STA abs,X", AbsoluteX, STA),
        OPCODE(0x9E, 3, 5, "STZ abs,X", AbsoluteX, STZ),
        OPCODE(0x9F, 3, 5, "BBS 1", ZeroPage, BBS, 1),
        OPCODE(0xA0, 2, 2, "LDY #", Immediate, LDY),
        OPCODE(0xA1, 2, 6, "LDA X,ind", XIndirect, LDA),
        OPCODE(0xA2, 2, 2, "LDX #", Immediate, LDX),
        OPCODE(0xA4, 2, 3, "LDY zpg", ZeroPage, LDY),
        OPCODE(0xA5, 2, 3, "LDA zpg", ZeroPage, LDA),
        OPCODE(0xA6, 2, 3, "LDX zpg", ZeroPage, LDX),
        OPCODE(0xA7, 2, 5, "SMB 2", ZeroPage, SMB, 2),
        OPCODE(0xA8, 1, 2, "TAY", Inherent, TAY),
        OPCODE(0xA9, 2, 2, "LDA #", Immediate, LDA),
        OPCODE(0xAA, 1, 2, "TAX", Inherent, TAX),
        OPCODE(0xAC, 3, 4, "LDY abs", Absolute, LDY),
        OPCODE(0xAD, 3, 4, "LDA abs", Absolute, LDA),
        OPCODE(0xAE, 3, 4, "LDX abs", Absolute, LDX),
        OPCODE(0xAF, 3, 5, "BBS 2", ZeroPage, BBS, 2),
        OPCODE(0xB0, 2, 2, "BCS", Relative, BCS),
        OPCODE(0xB1, 2, 5, "LDA ind,Y", IndirectY, LDA),
        OPCODE(0xB2, 2, 5, "LDA ind", ZeroIndirect, LDA),
        OPCODE(0xB4, 2, 4, "LDY zpg,X", ZeroPageX, LDY),
        OPCODE(0xB5, 2, 4, "LDA zpg,X", ZeroPageX, LDA),
        OPCODE(0xB6, 2, 4, "LDX zpg,Y", ZeroPageY, LDX),
        OPCODE(0xB7, 2, 5, "SMB 3", ZeroPage, SMB, 3),
        OPCODE(0xB8, 1, 2, "CLV", Inherent, CLV),
        OPCODE(0xB9, 3, 4, "LDA abs,Y", AbsoluteY, LDA),
        OPCODE(0xBA, 1, 2, "TSX", Inherent, TSX),
        OPCODE(0xBC, 3, 4, "LDY abs,X", AbsoluteX, LDY),
        OPCODE(0xBD, 3, 4, "LDA abs,X", AbsoluteX, LDA),
        OPCODE(0xBE, 3, 4, "LDX abs,Y", AbsoluteY, LDX),
        OPCODE(0xBF, 3, 5, "BBS 3", ZeroPage, BBS, 3),
        OPCODE(0xC0, 2, 2, "CPY #", Immediate, CPY),
        OPCODE(0xC1, 2, 6, "CMP X,ind", XIndirect, CMP),
        OPCODE(0xC4, 2, 3, "CPY zpg", ZeroPage, CPY),
        OPCODE(0xC5, 2, 3, "CMP zpg", ZeroPage, CMP),
        OPCODE(0xC6, 2, 5, "DEC zpg", ZeroPage, DEC),
        OPCODE(0xC7, 2, 5, "SMB 4", ZeroPage, SMB, 4),
        OPCODE(0xC8, 1, 2, "INY", Inherent, INY),
        OPCODE(0xC9, 2, 2, "CMP #", Immediate, CMP),
        OPCODE(0xCA, 1, 2, "DEX", Inherent, DEX),
        OPCODE(0xCC, 3, 4, "CPY abs", Absolute, CPY),
        OPCODE(0xCD, 3, 4, "CMP abs", Absolute, CMP),
        OPCODE(0xCE, 3, 3, "DEC abs", Absolute, DEC),
        OPCODE(0xCF, 3, 5, "BBS 4", ZeroPage, BBS, 4),
        OPCODE(0xD0, 2, 2, "BNE", Relative, BNE),
        OPCODE(0xD1, 2, 5, "CMP ind,Y", IndirectY, CMP),
        OPCODE(0xD2, 2, 5, "CMP ind", ZeroIndirect, CMP),
        OPCODE(0xD5, 2, 4, "CMP zpg,X", ZeroPageX, CMP),
        OPCODE(0xD6, 2, 6, "DEC zpg,X", ZeroPageX, DEC),
        OPCODE(0xD7, 2, 5, "SMB 5", ZeroPage, SMB, 5),
        OPCODE(0xD8, 1, 2, "CLD", Inherent, CLD),
        OPCODE(0xD9, 3, 4, "CMP abs,Y", AbsoluteY, CMP),
        OPCODE(0xDA, 1, 4, "PHX", Inherent, PHX),
        OPCODE(0xDD, 3, 4, "CMP abs,X", AbsoluteX, CMP),
        OPCODE(0xDE, 3, 7, "DEC abs,X", AbsoluteX, DEC),
        OPCODE(0xDF, 3, 5, "BBS 5", ZeroPage, BBS, 5),
        OPCODE(0xE0, 2, 2, "CPX #", Immediate, CPX),
        OPCODE(0xE1, 2, 6, "SBC X,ind", XIndirect, SBC),
        OPCODE(0xE4, 2, 3, "CPX zpg", ZeroPage, CPX),
        OPCODE(0xE5, 2, 3, "SBC zpg", ZeroPage, SBC),
        OPCODE(0xE6, 2, 5, "INC zpg", ZeroPage, INC),
        OPCODE(0xE7, 2, 5, "SMB 6", ZeroPage, SMB, 6),
        OPCODE(0xE8, 1, 2, "INX", Inherent, INX),
        OPCODE(0xE9, 2, 2, "SBC imm", Immediate, SBC),
        OPCODE(0xEA, 1, 2, "NOP", Inherent, NOP),
        OPCODE(0xEB, 2, 2, "SBC", Immediate, NOP), /* XXX: undocumented */
        OPCODE(0xEC, 3, 4, "CPX abs", Absolute, CPX),
        OPCODE(0xED, 3, 4, "SBC abs", Absolute, SBC),
        OPCODE(0xEE, 3, 6, "INC abs", Absolute, INC),
        OPCODE(0xEF, 3, 5, "BBS 6", ZeroPage, BBS, 6),
        OPCODE(0xF0, 2, 2, "BEQ", Relative, BEQ),
        OPCODE(0xF1, 2, 5, "SBC ind,Y", IndirectY, SBC),
        OPCODE(0xF2, 2, 5, "SBC ind", ZeroIndirect, SBC),
        OPCODE(0xF5, 2, 4, "SBC zpg,X", ZeroPageX, SBC),
        OPCODE(0xF6, 2, 6, "INC zpg,X", ZeroPageX, INC),
        OPCODE(0xF7, 2, 5, "SMB 7", ZeroPage, SMB, 7),
        OPCODE(0xF8, 1, 2, "SED", Inherent, SED),
        OPCODE(0xF9, 3, 4, "SBC abs,Y", AbsoluteY, SBC),
        OPCODE(0xFA, 1, 4, "PLX", Inherent, PLX),
        OPCODE(0xFD, 3, 4, "SBC abs,X", AbsoluteX, SBC),
        OPCODE(0xFE, 3, 7, "INC abs,X", AbsoluteX, INC),
        OPCODE(0xFF, 3, 5, "BBS 7", ZeroPage, BBS, 7),
    };

    for (int i = 0; i < sizeof(opcodes)/sizeof(opcodes[0]); i++) {
        m_opcodes[opcodes[i].code] = opcodes[i];
    }
}
