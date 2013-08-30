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

#include "cpu/m6502v2.h"

using namespace EMU;
using namespace M6502v2;

#define OPCODE(op, name, addr, oper)  { op, name, \
    std::bind(&M6502Cpu::addr, this), \
    std::bind(&M6502Cpu::oper, this) }

M6502Cpu::M6502Cpu(Machine *machine, const std::string &name,
                            unsigned hertz, AddressBus16 *bus):
    Cpu(machine, name, hertz, bus),
    _nmi_line(LineState::Clear),
    _irq_line(LineState::Clear),
    state()
{
    M6502Opcode opcodes[] = {
        OPCODE(0x00, "BRK", Inherent, BRK),
        OPCODE(0x01, "ORA X,ind", XIndirect, ORA),
        OPCODE(0x05, "ORA zpg", ZeroPage, ORA),
        OPCODE(0x06, "ASL zpg", ZeroPage, ASL),
        OPCODE(0x08, "PHP", Inherent, PHP),
        OPCODE(0x09, "ORA #", Immediate, ORA),
        OPCODE(0x0A, "ASL A", Inherent, ASLA),
        OPCODE(0x0D, "ORA abs", Absolute, ORA),
        OPCODE(0x0E, "ASL abs", Absolute, ASL),
        OPCODE(0x10, "BPL", Relative, BPL),
        OPCODE(0x11, "ORA ind,Y", IndirectY, ORA),
        OPCODE(0x15, "ORA zpg,X", ZeroPageX, ORA),
        OPCODE(0x16, "ASL zpg,X", ZeroPageX, ASL),
        OPCODE(0x18, "CLC", Inherent, CLC),
        OPCODE(0x19, "ORA abs,Y", AbsoluteY, ORA),
        OPCODE(0x1D, "ORA abs,X", AbsoluteX, ORA),
        OPCODE(0x1E, "ASL abs,X", AbsoluteX, ASL),
        OPCODE(0x20, "JSR", Absolute, JSR),
        OPCODE(0x21, "AND X,ind", XIndirect, AND),
        OPCODE(0x24, "BIT zpg", ZeroPage, BIT),
        OPCODE(0x25, "AND zpg", ZeroPage, AND),
        OPCODE(0x26, "ROL zpg", ZeroPage, ROL),
        OPCODE(0x28, "PLP", Inherent, PLP),
        OPCODE(0x29, "AND #", Immediate, AND),
        OPCODE(0x2A, "ROL A", Inherent, ROLA),
        OPCODE(0x2C, "BIT abs", Absolute, BIT),
        OPCODE(0x2D, "AND abs", Absolute, AND),
        OPCODE(0x2E, "ROL abs", Absolute, ROL),
        OPCODE(0x30, "BMI", Relative, BMI),
        OPCODE(0x31, "AND ind,Y", IndirectY, AND),
        OPCODE(0x35, "AND zpg,X", ZeroPageX, AND),
        OPCODE(0x36, "ROL zpg,X", ZeroPageX, ROL),
        OPCODE(0x38, "SEC", Inherent, SEC),
        OPCODE(0x39, "AND abs,Y", AbsoluteY, AND),
        OPCODE(0x3D, "AND abs,X", AbsoluteX, AND),
        OPCODE(0x3E, "ROL abs,X", AbsoluteX, ROL),
        OPCODE(0x40, "RTI", Inherent, RTI),
        OPCODE(0x41, "EOR X,ind", XIndirect, EOR),
        OPCODE(0x45, "EOR zpg", ZeroPage, EOR),
        OPCODE(0x46, "LSR zpg", ZeroPage, LSR),
        OPCODE(0x48, "PHA", Inherent, PHA),
        OPCODE(0x49, "EOR #", Immediate, EOR),
        OPCODE(0x4A, "LSR A", Inherent, LSRA),
        OPCODE(0x4C, "JMP abs", Absolute, JMP),
        OPCODE(0x4D, "EOR abs", Absolute, EOR),
        OPCODE(0x4E, "LSR abs", Absolute, LSR),
        OPCODE(0x50, "BVC", Relative, BVC),
        OPCODE(0x51, "EOR ind,Y", IndirectY, EOR),
        OPCODE(0x55, "EOR zpg,X", ZeroPageX, EOR),
        OPCODE(0x56, "LSR zpg,X", ZeroPageX, LSR),
        OPCODE(0x58, "CLI", Inherent, CLI),
        OPCODE(0x59, "EOR abs,Y", AbsoluteY, EOR),
        OPCODE(0x5D, "EOR abs,X", AbsoluteX, EOR),
        OPCODE(0x5E, "LSR abs,X", AbsoluteX, LSR),
        OPCODE(0x60, "RTS", Inherent, RTS),
        OPCODE(0x61, "ADC X,ind", XIndirect, ADC),
        OPCODE(0x65, "ADC zpg", ZeroPage, ADC),
        OPCODE(0x66, "ROR zpg", ZeroPage, ROR),
        OPCODE(0x68, "PLA", Inherent, PLA),
        OPCODE(0x69, "ADC #", Immediate, ADC),
        OPCODE(0x6A, "ROR A", Inherent, RORA),
        OPCODE(0x6C, "JMP ind", Indirect, JMP),
        OPCODE(0x6D, "ADC abs", Absolute, ADC),
        OPCODE(0x6E, "ROR abs", Absolute, ROR),
        OPCODE(0x70, "BVS", Relative, BVS),
        OPCODE(0x71, "ADC ind,Y", IndirectY, ADC),
        OPCODE(0x75, "ADC zpg,X", ZeroPageX, ADC),
        OPCODE(0x76, "ROR zpg,X", ZeroPageX, ROR),
        OPCODE(0x78, "SEI", Inherent, SEI),
        OPCODE(0x79, "ADC abs,Y", AbsoluteY, ADC),
        OPCODE(0x7D, "ADC abs,X", AbsoluteX, ADC),
        OPCODE(0x7E, "ROR abs,X", AbsoluteX, ROR),
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

M6502Cpu::~M6502Cpu(void)
{
}

void
M6502Cpu::line(Line line, LineState state)
{
    switch (line) {
    case Line::RESET:
        /* XXX: We should treat this as a line */
        _reset();
        break;
    case Line::INT0:
        _irq_line = state;
        break;
    case Line::NMI:
        _nmi_line = state;
        break;
    default:
        break;
    }
}

void
M6502Cpu::_reset(void)
{
    state.PC.d = 0;
    state.A = 0;
    state.X = 0;
    state.Y = 0;
    state.SR = 0;
    state.SP = 0xff;
    state.PC.b.l = bus_read(0xFFFC);
    state.PC.b.h = bus_read(0xFFFD);
}

Cycles
M6502Cpu::step(void)
{
    set_icycles(0);

    /* Interrupts */
    if (_nmi_line == LineState::Pulse) {
        DBG("NMI triggered");
        state.F.B = 0;
        push(state.PC.b.h);
        push(state.PC.b.l);
        push(state.SR);
        state.PC.b.l = bus_read(0xFFFA);
        state.PC.b.h = bus_read(0xFFFB);
        _nmi_line = LineState::Clear;
        return get_icycles();
    } else if (state.F.I == 0 && _irq_line == LineState::Assert) {
        DBG("IRQ triggered");
        state.F.B = 0;
        state.F.I = 1;
        push(state.PC.b.h);
        push(state.PC.b.l);
        push(state.SR);
        state.PC.b.l = bus_read(0xFFFE);
        state.PC.b.h = bus_read(0xFFFF);
        return get_icycles();
    }

    return dispatch();
}

void
M6502Cpu::log_op(uint16_t pc, const M6502Opcode *op)
{
    std::stringstream os;
    os << std::setw(8) << _name << ":"
       << Hex(pc) << ":" << Hex(op->code) << ":"
       << op->name << " = >";
    const std::string &str = op->name;
    const std::string &delimiters = " ";
    auto lastPos = str.find_first_not_of(delimiters, 0);
    auto pos = str.find_first_of(delimiters, lastPos);
    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        std::string it = str.substr(lastPos, pos - lastPos);
        os << " ";
        if (it == "abs" || it == "abs,X" || it == "abs,Y")
            os << Hex(state.EA);
        else if (it == "ind")
            os << " " << Hex(state.EA);
        else if (it == "LDX")
            os << Hex(state.X);
        else if (it == "LDY")
            os << Hex(state.Y);
        else if (it == "LDA")
            os << Hex(state.A);
        else if (it == "STX")
            os << Hex(state.X);
        else if (it == "STY")
            os << Hex(state.Y);
        else if (it == "STA")
            os << Hex(state.A);
        else if (it == "#")
            os << Hex(state.ARG);
        else if (it == "BEQ")
            os << it << " " << Hex(state.EA);
        else if (it == "zpg" || it == "zpg,X" || it == "zpg,Y" ||
                 it == "ind,Y" || it == "X,ind")
            os << Hex(state.EA);
        else if (it == "(zpg)")
            os << Hex(state.EA);
        else
            os << it;
        lastPos = str.find_first_not_of(delimiters, pos);
        pos = str.find_first_of(delimiters, lastPos);
    }
    TRACE(os.str());
}

std::string
M6502Cpu::dasm(addr_type addr)
{
    return "";
}

Cycles
M6502Cpu::dispatch(void)
{
    uint16_t pc = state.PC.d;
    uint8_t code = pc_read();

    auto it = _opcodes.find(code);
    if (it == _opcodes.end()) {
        std::cout << "Unknown opcode: " << Hex(code) << std::endl;
        throw CpuOpcodeFault(_name, code, pc);
    }

    M6502Opcode *op = &it->second;

    op->addr_mode();
    op->operation();

    IF_LOG(Trace)
        log_op(pc, op);

    return get_icycles();
}
