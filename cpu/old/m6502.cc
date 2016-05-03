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

#include "cpu/m6502.h"

#include <sstream>

using namespace EMU;
using namespace M6502;

M6502Cpu::M6502Cpu(Machine *machine, const std::string &name, unsigned hertz,
                   AddressBus16 *bus):
    ClockedDevice(machine, name, hertz),
    _rPC(), _rA(), _rX(), _rY(), _rSR(), _rSP(), _zpg(0),
    _nmi_line(LineState::Clear),
    _irq_line(LineState::Clear),
    _reset_line(LineState::Clear),
    _bus(bus),
    _icycles(0)
{

}

M6502Cpu::~M6502Cpu(void)
{
}

void
M6502Cpu::execute(void)
{
    while (true) {
        add_icycles(dispatch());
    }
}

void
M6502Cpu::line(Line line, LineState state)
{
    switch (line) {
    case Line::RESET:
        _reset_line = state;
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
M6502Cpu::reset(void)
{
    _rPC = 0;
    _rA = 0;
    _rX = 0;
    _rY = 0;
    _rSR = 0;
    _rSP = 0xff;
    _rPC.b.l = bus_read(0xFFFC);
    _rPC.b.h = bus_read(0xFFFD);
}

void
M6502Cpu::log_op(byte_t op)
{
    std::stringstream os;
    os << std::setw(8) << name() << ":"
       << Hex(_op_pc) << ":" << Hex(op) << ":"
       << _op_name << " = >";
    const std::string &str = _op_name;
    const std::string &delimiters = " ";
    auto lastPos = str.find_first_not_of(delimiters, 0);
    auto pos = str.find_first_of(delimiters, lastPos);
    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        std::string it = str.substr(lastPos, pos - lastPos);
        os << " ";
        if (it == "abs" || it == "abs,X" || it == "abs,Y")
            os << Hex(_rEA);
        else if (it == "ind")
            os << Hex(_op_ind) << " " << Hex(_rEA);
        else if (it == "LDX")
            os << Hex(_rX);
        else if (it == "LDY")
            os << Hex(_rY);
        else if (it == "LDA")
            os << Hex(_rA);
        else if (it == "STX")
            os << Hex(_rX);
        else if (it == "STY")
            os << Hex(_rY);
        else if (it == "STA")
            os << Hex(_rA);
        else if (it == "#")
            os << Hex(_rTmp);
        else if (it == "BEQ")
            os << "BEQ " << Hex(_rEA);
        else if (it == "zpg" || it == "zpg,X" || it == "zpg,Y" ||
                 it == "ind,Y" || it == "X,ind")
            os << Hex(_rEA);
        else if (it == "(zpg)")
            os << Hex(_rEA);
        else
            os << it;
        lastPos = str.find_first_not_of(delimiters, pos);
        pos = str.find_first_of(delimiters, lastPos);
    }
    DEVICE_TRACE(os.str());
}

#define OPCODE(op, name, func) \
    case op: { \
        IF_LOG(Trace) \
            start_op(name); \
        func; \
        break; \
    }

Cycles
M6502Cpu::dispatch(void)
{
    _icycles = Cycles(0);
    if (_reset_line == LineState::Pulse) {
        DEVICE_DEBUG("Reset triggered");
        reset();
        _reset_line = LineState::Clear;
        return _icycles;
    } else if (_nmi_line == LineState::Pulse) {
        DEVICE_DEBUG("NMI triggered");
        _rF.B = 0;
        op_interrupt(0xFFFA);
        _nmi_line = LineState::Clear;
        return _icycles;
    } else if (_rF.I == 0 && _irq_line == LineState::Assert) {
        DEVICE_DEBUG("IRQ triggered");
        _rF.B = 0;
        _rF.I = 1;
        op_interrupt(0xFFFE);
        return _icycles;
    }
    _op_pc = _rPC.d;
    byte_t op = pc_read(); /* pc_read */

    switch (op) {
        OPCODE(0x00, "BRK", _rF.B = 1; _rPC.d++; op_interrupt(0xFFFE));
        OPCODE(0x01, "ORA X,ind", XInd(); op_ora());
        OPCODE(0x05, "ORA zpg", Zpg(); op_ora());
        OPCODE(0x06, "ASL zpg", Zpg(); op_asl(););
        OPCODE(0x08, "PHP", _rF.B = 1; push(_rSR));
        OPCODE(0x09, "ORA #", Imm(); op_ora());
        OPCODE(0x0A, "ASL A", Acc(); op_asl());
        OPCODE(0x0D, "ORA abs", Abs(); op_ora());
        OPCODE(0x0E, "ASL abs", Abs(); op_asl());
        OPCODE(0x10, "BPL", Rel(); op_branch(!_rF.N));
        OPCODE(0x11, "ORA ind,Y", IndY(); op_ora());
        OPCODE(0x15, "ORA zpg,X", ZpgX(); op_ora());
        OPCODE(0x16, "ASL zpg,X", ZpgX(); op_asl());
        OPCODE(0x18, "CLC", _rF.C = 0);
        OPCODE(0x19, "ORA abs,Y", Abs(_rY); op_ora());
        OPCODE(0x1D, "ORA abs,X", Abs(_rX); op_ora());
        OPCODE(0x1E, "ASL abs,X", Abs(_rX); op_asl());
        OPCODE(0x20, "JSR", Abs(); op_jsr());
        OPCODE(0x21, "AND X,ind", XInd(); op_and());
        OPCODE(0x24, "BIT zpg", Zpg(); op_bit());
        OPCODE(0x25, "AND zpg", Zpg(); op_and());
        OPCODE(0x26, "ROL zpg", Zpg(); op_rol());
        OPCODE(0x28, "PLP", op_plp());
        OPCODE(0x29, "AND #", Imm(); op_and());
        OPCODE(0x2A, "ROL A", Acc(); op_rol());
        OPCODE(0x2C, "BIT abs", Abs(); op_bit());
        OPCODE(0x2D, "AND abs", Abs(); op_and());
        OPCODE(0x2E, "ROL abs", Abs(); op_rol());
        OPCODE(0x30, "BMI", Rel(); op_branch(_rF.N));
        OPCODE(0x31, "AND ind,Y", IndY(); op_and());
        OPCODE(0x35, "AND zpg,X", ZpgX(); op_and());
        OPCODE(0x36, "ROL zpg,X", ZpgX(); op_rol());
        OPCODE(0x38, "SEC", _rF.C = 1);
        OPCODE(0x39, "AND abs,Y", Abs(_rY); op_and());
        OPCODE(0x3D, "AND abs,X", Abs(_rX); op_and());
        OPCODE(0x3E, "ROL abs,X", Abs(_rX); op_rol());
        OPCODE(0x40, "RTI", op_rti());
        OPCODE(0x41, "EOR X,ind", XInd(); op_eor());
        OPCODE(0x45, "EOR zpg", Zpg(); op_eor());
        OPCODE(0x46, "LSR zpg", Zpg(); op_lsr());
        OPCODE(0x48, "PHA", push(_rA));
        OPCODE(0x49, "EOR #", Imm(); op_eor());
        OPCODE(0x4A, "LSR A", Acc(); op_lsr(););
        OPCODE(0x4C, "JMP abs", Abs(); op_jmp());
        OPCODE(0x4D, "EOR abs", Abs(); op_eor());
        OPCODE(0x4E, "LSR abs", Abs(); op_lsr());
        OPCODE(0x50, "BVC", Rel(); op_branch(!_rF.V));
        OPCODE(0x51, "EOR ind,Y", IndY(); op_eor());
        OPCODE(0x55, "EOR zpg,X", ZpgX(); op_eor());
        OPCODE(0x56, "LSR zpg,X", ZpgX(); op_lsr());
        OPCODE(0x58, "CLI", _rF.I = 0);
        OPCODE(0x59, "EOR abs,Y", Abs(_rY); op_eor());
        OPCODE(0x5D, "EOR abs,X", Abs(_rX); op_eor());
        OPCODE(0x5E, "LSR abs,X", Abs(_rX); op_lsr());
        OPCODE(0x60, "RTS", _rPC.b.l=pop(); _rPC.b.h=pop(); _rPC.d++);
        OPCODE(0x61, "ADC X,ind", XInd(); op_adc());
        OPCODE(0x65, "ADC zpg", Zpg(); op_adc());
        OPCODE(0x66, "ROR zpg", Zpg(); op_ror());
        OPCODE(0x68, "PLA", _rA=pop(); set_sz(_rA));
        OPCODE(0x69, "ADC #", Imm(); op_adc());
        OPCODE(0x6A, "ROR Acc", Acc(); op_ror());
        OPCODE(0x6C, "JMP ind", Ind(); op_jmp());
        OPCODE(0x6D, "ADC abs", Abs(); op_adc());
        OPCODE(0x6E, "ROR abs", Abs(); op_ror());
        OPCODE(0x70, "BVS", Rel(); op_branch(_rF.V));
        OPCODE(0x71, "ADC ind,Y", IndY(); op_adc());
        OPCODE(0x75, "ADC zpg,X", ZpgX(); op_adc());
        OPCODE(0x76, "ROR zpg,X", ZpgX(); op_ror());
        OPCODE(0x78, "SEI", _rF.I = 1);
        OPCODE(0x79, "ADC abs,Y", Abs(_rY); op_adc());
        OPCODE(0x7D, "ADC abs,X", Abs(_rX); op_adc());
        OPCODE(0x7E, "ROR abs,X", Abs(_rX); op_ror());
        OPCODE(0x81, "STA X,ind", XInd(); store(_rA));
        OPCODE(0x84, "STY zpg", Zpg(); store(_rY));
        OPCODE(0x85, "STA zpg", Zpg(); store(_rA));
        OPCODE(0x86, "STX zpg", Zpg(); store(_rX));
        OPCODE(0x88, "DEY", op_dey());
        OPCODE(0x8A, "TXA", _rA = _rX; set_sz(_rA));
        OPCODE(0x8C, "STY abs", Abs(); store(_rY));
        OPCODE(0x8D, "STA abs", Abs(); store(_rA));
        OPCODE(0x8E, "STX abs", Abs(); store(_rX));
        OPCODE(0x90, "BCC", Rel(); op_branch(!_rF.C));
        OPCODE(0x91, "STA ind,Y", IndY(); store(_rA));
        OPCODE(0x94, "STY zpg,X", ZpgX(); store(_rY));
        OPCODE(0x95, "STA zpg,X", ZpgX(); store(_rA));
        OPCODE(0x96, "STX zpg,Y", ZpgY(); store(_rX));
        OPCODE(0x98, "TYA", _rA = _rY; set_sz(_rA));
        OPCODE(0x99, "STA abs,Y", Abs(_rY); store(_rA));
        OPCODE(0x9A, "TXS", _rSP = _rX);
        OPCODE(0x9D, "STA abs,X", Abs(_rX); store(_rA));
        OPCODE(0xA0, "LDY #", Imm(); op_ldy());
        OPCODE(0xA1, "LDA X,ind", XInd(); op_lda());
        OPCODE(0xA2, "LDX #", Imm(); op_ldx());
        OPCODE(0xA4, "LDY zpg", Zpg(); op_ldy());
        OPCODE(0xA5, "LDA zpg", Zpg(); op_lda());
        OPCODE(0xA6, "LDX zpg", Zpg(); op_ldx());
        OPCODE(0xA8, "TAY", _rY = _rA; set_sz(_rY));
        OPCODE(0xA9, "LDA #", Imm(); op_lda());
        OPCODE(0xAA, "TAX", _rX = _rA; set_sz(_rX));
        OPCODE(0xAC, "LDY abs", Abs(); op_ldy());
        OPCODE(0xAD, "LDA abs", Abs(); op_lda());
        OPCODE(0xAE, "LDX abs", Abs(); op_ldx());
        OPCODE(0xB0, "BCS", Rel(); op_branch(_rF.C));
        OPCODE(0xB1, "LDA ind,Y", IndY(); op_lda());
        OPCODE(0xB4, "LDY zpg,X", ZpgX(); op_ldy());
        OPCODE(0xB5, "LDA zpg,X", ZpgX(); op_lda());
        OPCODE(0xB6, "LDX zpg,Y", ZpgY(); op_ldx());
        OPCODE(0xB8, "CLV", _rF.V = 0);
        OPCODE(0xB9, "LDA abs,Y", Abs(_rY); op_lda());
        OPCODE(0xBA, "TSX", _rX = _rSP; set_sz(_rX));
        OPCODE(0xBC, "LDY abs,X", Abs(_rX); op_ldy());
        OPCODE(0xBD, "LDA abs,X", Abs(_rX); op_lda());
        OPCODE(0xBE, "LDX abs,Y", Abs(_rY); op_ldx());
        OPCODE(0xC0, "CPY #", Imm(); op_cmp(_rY));
        OPCODE(0xC1, "CMP X,ind", XInd(); op_cmp(_rA));
        OPCODE(0xC4, "CPY zpg", Zpg(); op_cmp(_rY));
        OPCODE(0xC5, "CMP zpg", Zpg(); op_cmp(_rA));
        OPCODE(0xC6, "DEC zpg", Zpg(); op_dec());
        OPCODE(0xC8, "INY", op_iny());
        OPCODE(0xC9, "CMP #", Imm(); op_cmp(_rA););
        OPCODE(0xCA, "DEX", op_dex());
        OPCODE(0xCC, "CPY abs", Abs(); op_cmp(_rY));
        OPCODE(0xCD, "CMP abs", Abs(); op_cmp(_rA));
        OPCODE(0xCE, "DEC abs", Abs(); op_dec());
        OPCODE(0xD0, "BNE", Rel(); op_branch(!_rF.Z));
        OPCODE(0xD1, "CMP ind,Y", IndY(); op_cmp(_rA));
        OPCODE(0xD5, "CMP zpg,X", ZpgX(); op_cmp(_rA));
        OPCODE(0xD6, "DEC zpg,X", ZpgX(); op_dec());
        OPCODE(0xD8, "CLD", _rF.D = 0);
        OPCODE(0xD9, "CMP abs,Y", Abs(_rY); op_cmp(_rA));
        OPCODE(0xDD, "CMP abs,X", Abs(_rX); op_cmp(_rA));
        OPCODE(0xDE, "DEC abs,X", Abs(_rX); op_dec());
        OPCODE(0xE0, "CPX #", Imm(); op_cmp(_rX));
        OPCODE(0xE1, "SBC X,ind", XInd(); op_sbc());
        OPCODE(0xE4, "CPX zpg", Zpg(); op_cmp(_rX));
        OPCODE(0xE5, "SBC zpg", Zpg(); op_sbc());
        OPCODE(0xE6, "INC zpg", Zpg(); op_inc());
        OPCODE(0xE8, "INX", op_inx());
        OPCODE(0xE9, "SBC imm", Imm(); op_sbc());
        OPCODE(0xEA, "NOP", );
        OPCODE(0xEC, "CPX abs", Abs(); op_cmp(_rX));
        OPCODE(0xED, "SBC abs", Abs(); op_sbc());
        OPCODE(0xEE, "INC abs", Abs(); op_inc());
        OPCODE(0xF0, "BEQ", Rel(); op_branch(_rF.Z));
        OPCODE(0xF1, "SBC ind,Y", IndY(); op_sbc());
        OPCODE(0xF5, "SBC zpg,X", ZpgX(); op_sbc());
        OPCODE(0xF6, "INC zpg,X", ZpgX(); op_inc());
        OPCODE(0xF8, "SED", _rF.D = 0);
        OPCODE(0xF9, "SBC abs,Y", Abs(_rY); op_sbc());
        OPCODE(0xFD, "SBC abs,X", Abs(_rX); op_sbc());
        OPCODE(0xFE, "INC abs,X", Abs(_rX); op_inc());
    default:
        std::cout << "Unknown opcode: " << Hex(op) << std::endl;
        throw CpuOpcodeFault(name(), op, _op_pc);
    }

    IF_LOG(Trace)
        log_op(op);

    return _icycles;
}


