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

using namespace EMU;
using namespace M6502;

m65c02Cpu::m65c02Cpu(Machine *machine, const std::string &name, unsigned hertz,
                     AddressBus16x8 *bus)
    : M6502Cpu(machine, name, hertz, bus) {}

m65c02Cpu::~m65c02Cpu(void) {}

void m65c02Cpu::reset(void) {}

void m65c02Cpu::execute(void) {
  while (true) {
    add_icycles(dispatch());
  }
}

void m65c02Cpu::line(Line line, LineState state) {
  switch (line) {
    case Line::RESET:
      reset();
      break;
    default:
      break;
  }
}

#define OPCODE(op, name, func) \
  case op: {                   \
    IF_LOG(Trace)              \
    start_op(name);            \
    func;                      \
    break;                     \
  }

Cycles m65c02Cpu::dispatch(void) {
  _icycles = Cycles(0);
  /* XXX: Interrupts */
  _op_pc = _rPC.d;
  byte_t op = pc_read();

  switch (op) {
    OPCODE(0x00, "BRK", _rF.B = 1; _rPC.d++; op_interrupt(0xFFFE));
    OPCODE(0x01, "ORA X,ind", XInd(); op_ora());
    OPCODE(0x04, "TSB #", Imm(); op_tsb());
    OPCODE(0x05, "ORA zpg", Zpg(); op_ora());
    OPCODE(0x06, "ASL zpg", Zpg(); op_asl(););
    OPCODE(0x08, "PHP", _rF.B = 1; push(_rSR));
    OPCODE(0x09, "ORA #", Imm(); op_ora());
    OPCODE(0x0A, "ASL A", Acc(); op_asl());
    OPCODE(0x0C, "TSB A", Acc(); op_tsb());
    OPCODE(0x0D, "ORA abs", Abs(); op_ora());
    OPCODE(0x0E, "ASL abs", Abs(); op_asl());
    OPCODE(0x10, "BPL", Rel(); op_branch(!_rF.N));
    OPCODE(0x11, "ORA ind,Y", IndY(); op_ora());
    OPCODE(0x12, "ORA ind", ZpgInd(); op_ora());
    OPCODE(0x15, "ORA zpg,X", ZpgX(); op_ora());
    OPCODE(0x16, "ASL zpg,X", ZpgX(); op_asl());
    OPCODE(0x18, "CLC", _rF.C = 0);
    OPCODE(0x19, "ORA abs,Y", Abs(_rY); op_ora());
    OPCODE(0x1A, "Inc A", Acc(); op_ina());
    OPCODE(0x1C, "TRB A", Acc(); op_trb());
    OPCODE(0x1D, "ORA abs,X", Abs(_rX); op_ora());
    OPCODE(0x1E, "ASL abs,X", Abs(_rX); op_asl());
    OPCODE(0x20, "JSR", Abs(); _rPC.d--; push(_rPC.b.h); push(_rPC.b.l);
           op_jmp());
    OPCODE(0x21, "AND X,ind", XInd(); op_and());
    OPCODE(0x24, "BIT zpg", Zpg(); op_bit());
    OPCODE(0x25, "AND zpg", Zpg(); op_and());
    OPCODE(0x26, "ROL zpg", Zpg(); op_rol());
    OPCODE(0x28, "PLP", _rSR = pop(); _rF.B = 1; _rF.E = 1);
    OPCODE(0x29, "AND #", Imm(); op_and());
    OPCODE(0x2A, "ROL A", Acc(); op_rol());
    OPCODE(0x2C, "BIT abs", Abs(); op_bit());
    OPCODE(0x2D, "AND abs", Abs(); op_and());
    OPCODE(0x2E, "ROL abs", Abs(); op_rol());
    OPCODE(0x30, "BMI", Rel(); op_branch(_rF.N));
    OPCODE(0x31, "AND ind,Y", IndY(); op_and());
    OPCODE(0x32, "AND ind", ZpgInd(); op_and());
    OPCODE(0x34, "BIT zpg,X", ZpgX(); op_bit());
    OPCODE(0x35, "AND zpg,X", ZpgX(); op_and());
    OPCODE(0x36, "ROL zpg,X", ZpgX(); op_rol());
    OPCODE(0x38, "SEC", _rF.C = 1);
    OPCODE(0x39, "AND abs,Y", Abs(_rY); op_and());
    OPCODE(0x3A, "DEC A", op_dea());
    OPCODE(0x3C, "BIT abs,X", Abs(_rX); op_bit());
    OPCODE(0x3D, "AND abs,X", Abs(_rX); op_and());
    OPCODE(0x3E, "ROL abs,X", Abs(_rX); op_rol());
    OPCODE(0x40, "RTI", _rSR = pop(); _rF.E = 1; _rPC.b.l = pop();
           _rPC.b.h = pop());
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
    OPCODE(0x52, "EOR (d)", ZpgInd(); op_eor());
    OPCODE(0x55, "EOR zpg,X", ZpgX(); op_eor());
    OPCODE(0x56, "LSR zpg,X", ZpgX(); op_lsr());
    OPCODE(0x58, "CLI", _rF.I = 0);
    OPCODE(0x59, "EOR abs,Y", Abs(_rY); op_eor());
    OPCODE(0x5A, "PHY", op_phy());
    OPCODE(0x5D, "EOR abs,X", Abs(_rX); op_eor());
    OPCODE(0x5E, "LSR abs,X", Abs(_rX); op_lsr());
    OPCODE(0x60, "RTS", _rPC.b.l = pop(); _rPC.b.h = pop(); _rPC.d++);
    OPCODE(0x61, "ADC X,ind", XInd(); op_adc());
    OPCODE(0x64, "STZ zpg", Zpg(); store(0));
    OPCODE(0x65, "ADC zpg", Zpg(); op_adc());
    OPCODE(0x66, "ROR zpg", Zpg(); op_ror());
    OPCODE(0x68, "PLA", _rA = pop(); set_sz(_rA));
    OPCODE(0x69, "ADC #", Imm(); op_adc());
    OPCODE(0x6A, "ROR Acc", Acc(); op_ror());
    OPCODE(0x6C, "JMP ind", Ind(); op_jmp());
    OPCODE(0x6D, "ADC abs", Abs(); op_adc());
    OPCODE(0x6E, "ROR abs", Abs(); op_ror());
    OPCODE(0x70, "BVS", Rel(); op_branch(_rF.V));
    OPCODE(0x71, "ADC ind,Y", IndY(); op_adc());
    OPCODE(0x72, "ADC (d)", ZpgInd(); op_adc());
    OPCODE(0x74, "STZ zpg,X", ZpgX(); store(0));
    OPCODE(0x75, "ADC zpg,X", ZpgX(); op_adc());
    OPCODE(0x76, "ROR zpg,X", ZpgX(); op_ror());
    OPCODE(0x78, "SEI", _rF.I = 1);
    OPCODE(0x79, "ADC abs,Y", Abs(_rY); op_adc());
    OPCODE(0x7A, "PLY", op_ply());
    OPCODE(0x7C, "JMP (abs,X)", Ind(_rX); op_jmp());
    OPCODE(0x7D, "ADC abs,X", Abs(_rX); op_adc());
    OPCODE(0x7E, "ROR abs,X", Abs(_rX); op_ror());
    OPCODE(0x80, "BRA r", Rel(); op_branch(true));
    OPCODE(0x81, "STA X,ind", XInd(); store(_rA));
    OPCODE(0x84, "STY zpg", Zpg(); store(_rY));
    OPCODE(0x85, "STA zpg", Zpg(); store(_rA));
    OPCODE(0x86, "STX zpg", Zpg(); store(_rX));
    OPCODE(0x88, "DEY", op_dey());
    OPCODE(0x89, "BIT #", Imm(); op_bitimm());
    OPCODE(0x8A, "TXA", _rA = _rX; set_sz(_rA));
    OPCODE(0x8C, "STY abs", Abs(); store(_rY));
    OPCODE(0x8D, "STA abs", Abs(); store(_rA));
    OPCODE(0x8E, "STX abs", Abs(); store(_rX));
    OPCODE(0x90, "BCC", Rel(); op_branch(!_rF.C));
    OPCODE(0x91, "STA ind,Y", IndY(); store(_rA));
    OPCODE(0x92, "STA (d)", ZpgInd(); store(_rA));
    OPCODE(0x94, "STY zpg,X", ZpgX(); store(_rY));
    OPCODE(0x95, "STA zpg,X", ZpgX(); store(_rA));
    OPCODE(0x96, "STX zpg,Y", ZpgY(); store(_rX));
    OPCODE(0x98, "TYA", _rA = _rY; set_sz(_rA));
    OPCODE(0x99, "STA abs,Y", Abs(_rY); store(_rA));
    OPCODE(0x9A, "TXS", _rSP = _rX);
    OPCODE(0x9C, "STZ abs", Abs(); store(0));
    OPCODE(0x9D, "STA abs,X", Abs(_rX); store(_rA));
    OPCODE(0x9E, "STZ abs,X", Abs(_rX); store(0));
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
    OPCODE(0xB2, "LDA (d)", ZpgInd(); op_lda());
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
    OPCODE(0xD2, "CMP (d)", ZpgInd(); op_cmp(_rA));
    OPCODE(0xD5, "CMP zpg,X", ZpgX(); op_cmp(_rA));
    OPCODE(0xD6, "DEC zpg,X", ZpgX(); op_dec());
    OPCODE(0xD8, "CLD", _rF.D = 0);
    OPCODE(0xD9, "CMP abs,Y", Abs(_rY); op_cmp(_rA));
    OPCODE(0xDA, "PHX", op_phx());
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
    OPCODE(0xF2, "SBC (d)", ZpgInd(); op_sbc());
    OPCODE(0xF1, "SBC ind,Y", IndY(); op_sbc());
    OPCODE(0xF5, "SBC zpg,X", ZpgX(); op_sbc());
    OPCODE(0xF6, "INC zpg,X", ZpgX(); op_inc());
    OPCODE(0xF8, "SED", _rF.D = 0);
    OPCODE(0xF9, "SBC abs,Y", Abs(_rY); op_sbc());
    OPCODE(0xFA, "PLX", op_plx());
    OPCODE(0xFD, "SBC abs,X", Abs(_rX); op_sbc());
    OPCODE(0xFE, "INC abs,X", Abs(_rX); op_inc());
    default:
      if ((op & 0x03) == 0x02) {
        /* 1 cycle(s), 2 byte(s) */
      } else if ((op & 0x03) == 0x03) {
        /* 2 cycle(s), 1 byte(s) */
        _add_icycles(1);
      } else if (op == 0x44) {
        /* 3 cycle(s), 2 byte(s) */
        _add_icycles(2);
      } else if ((op & 0x5f) == 0x54) {
        /* 4 cycle(s), 2 bytes(s) */
        _add_icycles(3);
      } else if (op == 0x5C) {
        /* 8 cycle(s), 3 byte(s) */
        _add_icycles(7);
      } else if ((op & 0xDF) == 0xDC) {
        /* 4 cycle(s), 3 byte(s) */
        _add_icycles(3);
      } else {
        std::cout << "Unknown opcode: " << Hex(op) << std::endl;
        throw CpuOpcodeFault(name(), op, _op_pc);
      }
  }

  IF_LOG(Trace)
  log_op(op);

  return _icycles;
}
