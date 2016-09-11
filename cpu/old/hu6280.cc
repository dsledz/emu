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

/**
 * Hudson Soft huc6280 Cpu.
 *
 * TODO:
 * 1. Delay interrupts one cycle after CLI
 * 2. Verify timing.
 * 3. Add dummy cycle for RMW ops.
 * 4. Map public interrupt names to CPU specific ones.
 */
#include "cpu/m6502.h"

using namespace EMU;
using namespace M6502;

hu6280Cpu::hu6280Cpu(Machine *machine, const std::string &name, unsigned hertz,
                     AddressBus21 *bus)
    : m65c02Cpu(machine, name, hertz, &_mmu),
      _mmu_map(),
      _clock_div(1),
      _irq_status(0),
      _irq_disable(0),
      _timer_status(false),
      _timer_load(0),
      _timer_value(0) {
  _data_bus = bus;
  _mmu.add(0x0000, 0xFFFF, READ_CB(hu6280Cpu::mmu_read, this),
           WRITE_CB(hu6280Cpu::mmu_write, this));

  /* Zero page is actually offset by 8KB */
  _zpg = 0x20;
}

hu6280Cpu::~hu6280Cpu(void) {}

void hu6280Cpu::reset(void) {
  _rPC = 0;
  _rA = 0;
  _rX = 0;
  _rY = 0;
  _rSR = 0;
  _rSP = 0xff;
  _rPC.b.l = bus_read(0xFFFE);
  _rPC.b.h = bus_read(0xFFFF);

  _mmu_map[0] = 0xFF;
  _mmu_map[1] = 0xF8;
  _mmu_map[2] = 0x00;
  _mmu_map[3] = 0x00;
  _mmu_map[4] = 0x00;
  _mmu_map[5] = 0x00;
  _mmu_map[6] = 0x00;
  _mmu_map[7] = 0x00;
}

byte_t hu6280Cpu::mmu_read(offset_t offset) {
  const int bank = (offset & 0xE000) >> 13;
  offset = (offset & 0x1FFF) | (static_cast<offset_t>(_mmu_map[bank]) << 13);
  return _data_bus->read(offset);
}

void hu6280Cpu::mmu_write(offset_t offset, byte_t value) {
  const int bank = (offset & 0xE000) >> 13;
  offset = (offset & 0x1FFF) | (static_cast<offset_t>(_mmu_map[bank]) << 13);
  _data_bus->write(offset, value);
}

void hu6280Cpu::execute(void) {
  while (true) {
    Cycles used = dispatch();
    add_icycles(used);
    _timer_value -= (int)used.v;
  }
}

void hu6280Cpu::line(Line line, LineState state) {
  switch (line) {
    case Line::RESET:
      reset();
      break;
    case Line::INT2:
      bit_set(_irq_status, 0, state == LineState::Assert);
      break;
    case Line::INT1:
      bit_set(_irq_status, 1, state == LineState::Assert);
      break;
    case Line::INT0:
      bit_set(_irq_status, 2, state == LineState::Assert);
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

bool hu6280Cpu::interrupt(void) {
  if (_nmi_line == LineState::Pulse) {
    DEVICE_DEBUG("NMI triggered");
    op_irq(0xFFFC);
    _nmi_line = LineState::Clear;
    return true;
  } else if (_rF.I == 0) {
    uint16_t addr = 0x0000;
    if (bit_isset(_irq_status, 0) && !bit_isset(_irq_disable, 0))
      addr = 0xFFF6;
    else if (bit_isset(_irq_status, 1) && !bit_isset(_irq_disable, 1))
      addr = 0xFFF8;
    else if (bit_isset(_irq_status, 2) && !bit_isset(_irq_disable, 2))
      addr = 0xFFFA;
    if (addr != 0x0000) {
      DEVICE_DEBUG("Interrupt");
      op_irq(addr);
      return true;
    }
  }
  return false;
}

Cycles hu6280Cpu::dispatch(void) {
  _icycles = Cycles(0);

  if (_timer_status && _timer_value < 0) {
    DEVICE_INFO("Timer Triggered");
    bit_set(_irq_status, 2, true);
    while (_timer_value <= 0) _timer_value += _timer_load;
  }

  if (interrupt()) return _icycles;

  _op_pc = _rPC.d;
  byte_t op = pc_read();

  switch (op) {
    OPCODE(0x00, "BRK", _rPC.d++; op_irq(0xFFF6));
    OPCODE(0x01, "ORA X,ind", XInd(); op_ora());
    OPCODE(0x02, "SXY", op_sxy());
    OPCODE(0x03, "ST0 #", op_st0());
    OPCODE(0x04, "TSB #", Imm(); op_tsb());
    OPCODE(0x05, "ORA zpg", Zpg(); op_ora());
    OPCODE(0x06, "ASL zpg", Zpg(); op_asl());
    OPCODE(0x07, "RMB0", Zpg(); op_rmb(0));
    OPCODE(0x08, "PHP", _rF.B = 1; push(_rSR));
    OPCODE(0x09, "ORA #", Imm(); op_ora());
    OPCODE(0x0A, "ASL A", Acc(); op_asl());
    OPCODE(0x0C, "TSB A", Acc(); op_tsb());
    OPCODE(0x0D, "ORA abs", Abs(); op_ora());
    OPCODE(0x0E, "ASL abs", Abs(); op_asl());
    OPCODE(0x0F, "BBR0", op_bbr(0));
    OPCODE(0x10, "BPL", Rel(); op_branch(!_rF.N));
    OPCODE(0x11, "ORA ind,Y", IndY(); op_ora());
    OPCODE(0x12, "ORA ind", ZpgInd(); op_ora());
    OPCODE(0x13, "ST1 #", op_st1());
    OPCODE(0x14, "TRB zpg", Zpg(); op_trb());
    OPCODE(0x15, "ORA zpg,X", ZpgX(); op_ora());
    OPCODE(0x16, "ASL zpg,X", ZpgX(); op_asl());
    OPCODE(0x17, "RMB1", Zpg(); op_rmb(1));
    OPCODE(0x18, "CLC", _rF.C = 0);
    OPCODE(0x19, "ORA abs,Y", Abs(_rY); op_ora());
    OPCODE(0x1A, "Inc A", Acc(); op_ina());
    OPCODE(0x1C, "TRB A", Acc(); op_trb());
    OPCODE(0x1D, "ORA abs,X", Abs(_rX); op_ora());
    OPCODE(0x1E, "ASL abs,X", Abs(_rX); op_asl());
    OPCODE(0x1F, "BBR1", op_bbr(1));
    OPCODE(0x20, "JSR abs", Abs(); op_jsr());
    OPCODE(0x21, "AND X,ind", XInd(); op_and());
    OPCODE(0x22, "SAX", op_sax());
    OPCODE(0x23, "ST2 #", op_st2());
    OPCODE(0x24, "BIT zpg", Zpg(); op_bit());
    OPCODE(0x25, "AND zpg", Zpg(); op_and());
    OPCODE(0x26, "ROL zpg", Zpg(); op_rol());
    OPCODE(0x27, "RMB2", Zpg(); op_rmb(2));
    OPCODE(0x28, "PLP", op_plp());
    OPCODE(0x29, "AND #", Imm(); op_and());
    OPCODE(0x2A, "ROL A", Acc(); op_rol());
    OPCODE(0x2C, "BIT abs", Abs(); op_bit());
    OPCODE(0x2D, "AND abs", Abs(); op_and());
    OPCODE(0x2E, "ROL abs", Abs(); op_rol());
    OPCODE(0x2F, "BBR2", op_bbr(2));
    OPCODE(0x30, "BMI", Rel(); op_branch(_rF.N));
    OPCODE(0x31, "AND ind,Y", IndY(); op_and());
    OPCODE(0x32, "AND ind", ZpgInd(); op_and());
    OPCODE(0x34, "BIT zpg,X", ZpgX(); op_bit());
    OPCODE(0x35, "AND zpg,X", ZpgX(); op_and());
    OPCODE(0x36, "ROL zpg,X", ZpgX(); op_rol());
    OPCODE(0x37, "RMB3", Zpg(); op_rmb(3));
    OPCODE(0x38, "SEC", _rF.C = 1);
    OPCODE(0x39, "AND abs,Y", Abs(_rY); op_and());
    OPCODE(0x3A, "DEC A", op_dea());
    OPCODE(0x3C, "BIT abs,X", Abs(_rX); op_bit());
    OPCODE(0x3D, "AND abs,X", Abs(_rX); op_and());
    OPCODE(0x3E, "ROL abs,X", Abs(_rX); op_rol());
    OPCODE(0x3F, "BBR3", op_bbr(3));
    OPCODE(0x40, "RTI", op_rti());
    OPCODE(0x41, "EOR X,ind", XInd(); op_eor());
    OPCODE(0x42, "SAY", op_say());
    OPCODE(0x43, "TMA #", Imm(); op_tma());
    OPCODE(0x44, "BSR", op_bsr());
    OPCODE(0x45, "EOR zpg", Zpg(); op_eor());
    OPCODE(0x46, "LSR zpg", Zpg(); op_lsr());
    OPCODE(0x47, "RMB4", Zpg(); op_rmb(4));
    OPCODE(0x48, "PHA", push(_rA));
    OPCODE(0x49, "EOR #", Imm(); op_eor());
    OPCODE(0x4A, "LSR A", Acc(); op_lsr(););
    OPCODE(0x4C, "JMP abs", Abs(); op_jmp());
    OPCODE(0x4D, "EOR abs", Abs(); op_eor());
    OPCODE(0x4E, "LSR abs", Abs(); op_lsr());
    OPCODE(0x4F, "BBR4", op_bbr(4));
    OPCODE(0x50, "BVC", Rel(); op_branch(!_rF.V));
    OPCODE(0x51, "EOR ind,Y", IndY(); op_eor());
    OPCODE(0x52, "EOR (zpg)", ZpgInd(); op_eor());
    OPCODE(0x53, "TAM #", Imm(); op_tam());
    OPCODE(0x54, "CSL", op_csl());
    OPCODE(0x55, "EOR zpg,X", ZpgX(); op_eor());
    OPCODE(0x56, "LSR zpg,X", ZpgX(); op_lsr());
    OPCODE(0x57, "RMB5", Zpg(); op_rmb(5));
    OPCODE(0x58, "CLI", _rF.I = 0);
    OPCODE(0x59, "EOR abs,Y", Abs(_rY); op_eor());
    OPCODE(0x5A, "PHY", op_phy());
    OPCODE(0x5D, "EOR abs,X", Abs(_rX); op_eor());
    OPCODE(0x5E, "LSR abs,X", Abs(_rX); op_lsr());
    OPCODE(0x5F, "BBR5", op_bbr(5));
    OPCODE(0x60, "RTS", _rPC.b.l = pop(); _rPC.b.h = pop(); _rPC.d++);
    OPCODE(0x61, "ADC X,ind", XInd(); op_adc());
    OPCODE(0x62, "CLA", op_cla());
    OPCODE(0x64, "STZ zpg", Zpg(); store(0));
    OPCODE(0x65, "ADC zpg", Zpg(); op_adc());
    OPCODE(0x66, "ROR zpg", Zpg(); op_ror());
    OPCODE(0x67, "RMB6", Zpg(); op_rmb(6));
    OPCODE(0x68, "PLA", _rA = pop(); set_sz(_rA));
    OPCODE(0x69, "ADC #", Imm(); op_adc());
    OPCODE(0x6A, "ROR Acc", Acc(); op_ror());
    OPCODE(0x6C, "JMP ind", Ind(); op_jmp());
    OPCODE(0x6D, "ADC abs", Abs(); op_adc());
    OPCODE(0x6E, "ROR abs", Abs(); op_ror());
    OPCODE(0x6F, "BBR6", op_bbr(6));
    OPCODE(0x70, "BVS", Rel(); op_branch(_rF.V));
    OPCODE(0x71, "ADC ind,Y", IndY(); op_adc());
    OPCODE(0x72, "ADC (zpg)", ZpgInd(); op_adc());
    OPCODE(0x73, "TII", op_tii());
    OPCODE(0x74, "STZ zpg,X", ZpgX(); store(0));
    OPCODE(0x75, "ADC zpg,X", ZpgX(); op_adc());
    OPCODE(0x76, "ROR zpg,X", ZpgX(); op_ror());
    OPCODE(0x77, "RMB7", Zpg(); op_rmb(7));
    OPCODE(0x78, "SEI", _rF.I = 1);
    OPCODE(0x79, "ADC abs,Y", Abs(_rY); op_adc());
    OPCODE(0x7A, "PLY", op_ply());
    OPCODE(0x7C, "JMP (abs,X)", Ind(_rX); op_jmp());
    OPCODE(0x7D, "ADC abs,X", Abs(_rX); op_adc());
    OPCODE(0x7E, "ROR abs,X", Abs(_rX); op_ror());
    OPCODE(0x7F, "BBR7", op_bbr(7));
    OPCODE(0x80, "BRA r", Rel(); op_branch(true));
    OPCODE(0x81, "STA X,ind", XInd(); store(_rA));
    OPCODE(0x82, "CLX", op_clx());
    OPCODE(0x83, "TST zpg", byte_t val = pc_read(); Zpg(); op_tst(val));
    OPCODE(0x84, "STY zpg", Zpg(); store(_rY));
    OPCODE(0x85, "STA zpg", Zpg(); store(_rA));
    OPCODE(0x86, "STX zpg", Zpg(); store(_rX));
    OPCODE(0x87, "SMB0", Zpg(); op_smb(0));
    OPCODE(0x88, "DEY", op_dey());
    OPCODE(0x89, "BIT #", Imm(); op_bitimm());
    OPCODE(0x8A, "TXA", _rA = _rX; set_sz(_rA));
    OPCODE(0x8C, "STY abs", Abs(); store(_rY));
    OPCODE(0x8D, "STA abs", Abs(); store(_rA));
    OPCODE(0x8E, "STX abs", Abs(); store(_rX));
    OPCODE(0x8F, "BBS0", op_bbs(0));
    OPCODE(0x90, "BCC", Rel(); op_branch(!_rF.C));
    OPCODE(0x91, "STA ind,Y", IndY(); store(_rA));
    OPCODE(0x92, "STA (zpg)", ZpgInd(); store(_rA));
    OPCODE(0x93, "TST abs", byte_t val = pc_read(); Abs(); op_tst(val));
    OPCODE(0x94, "STY zpg,X", ZpgX(); store(_rY));
    OPCODE(0x95, "STA zpg,X", ZpgX(); store(_rA));
    OPCODE(0x96, "STX zpg,Y", ZpgY(); store(_rX));
    OPCODE(0x97, "SMB1", Zpg(); op_smb(1));
    OPCODE(0x98, "TYA", _rA = _rY; set_sz(_rA));
    OPCODE(0x99, "STA abs,Y", Abs(_rY); store(_rA));
    OPCODE(0x9A, "TXS", _rSP = _rX);
    OPCODE(0x9C, "STZ abs", Abs(); store(0));
    OPCODE(0x9D, "STA abs,X", Abs(_rX); store(_rA));
    OPCODE(0x9E, "STZ abs,X", Abs(_rX); store(0));
    OPCODE(0x9F, "BBS1", op_bbs(1));
    OPCODE(0xA0, "LDY #", Imm(); op_ldy());
    OPCODE(0xA1, "LDA X,ind", XInd(); op_lda());
    OPCODE(0xA2, "LDX #", Imm(); op_ldx());
    OPCODE(0xA3, "TST zpg,X", byte_t val = pc_read(); ZpgX(); op_tst(val));
    OPCODE(0xA4, "LDY zpg", Zpg(); op_ldy());
    OPCODE(0xA5, "LDA zpg", Zpg(); op_lda());
    OPCODE(0xA6, "LDX zpg", Zpg(); op_ldx());
    OPCODE(0xA7, "SMB2", Zpg(); op_smb(2));
    OPCODE(0xA8, "TAY", _rY = _rA; set_sz(_rY));
    OPCODE(0xA9, "LDA #", Imm(); op_lda());
    OPCODE(0xAA, "TAX", _rX = _rA; set_sz(_rX));
    OPCODE(0xAC, "LDY abs", Abs(); op_ldy());
    OPCODE(0xAD, "LDA abs", Abs(); op_lda());
    OPCODE(0xAE, "LDX abs", Abs(); op_ldx());
    OPCODE(0xAF, "BBS2", op_bbs(2));
    OPCODE(0xB0, "BCS", Rel(); op_branch(_rF.C));
    OPCODE(0xB1, "LDA ind,Y", IndY(); op_lda());
    OPCODE(0xB2, "LDA (zpg)", ZpgInd(); op_lda());
    OPCODE(0xB3, "TST abs,X", byte_t val = pc_read(); Abs(_rX); op_tst(val));
    OPCODE(0xB4, "LDY zpg,X", ZpgX(); op_ldy());
    OPCODE(0xB5, "LDA zpg,X", ZpgX(); op_lda());
    OPCODE(0xB6, "LDX zpg,Y", ZpgY(); op_ldx());
    OPCODE(0xB7, "SMB3", Zpg(); op_smb(3));
    OPCODE(0xB8, "CLV", _rF.V = 0);
    OPCODE(0xB9, "LDA abs,Y", Abs(_rY); op_lda());
    OPCODE(0xBA, "TSX", _rX = _rSP; set_sz(_rX));
    OPCODE(0xBC, "LDY abs,X", Abs(_rX); op_ldy());
    OPCODE(0xBD, "LDA abs,X", Abs(_rX); op_lda());
    OPCODE(0xBE, "LDX abs,Y", Abs(_rY); op_ldx());
    OPCODE(0xBF, "BBS3", op_bbs(3));
    OPCODE(0xC0, "CPY #", Imm(); op_cmp(_rY));
    OPCODE(0xC1, "CMP X,ind", XInd(); op_cmp(_rA));
    OPCODE(0xC2, "CLY", op_cly());
    OPCODE(0xC3, "TDD", op_tdd());
    OPCODE(0xC4, "CPY zpg", Zpg(); op_cmp(_rY));
    OPCODE(0xC5, "CMP zpg", Zpg(); op_cmp(_rA));
    OPCODE(0xC6, "DEC zpg", Zpg(); op_dec());
    OPCODE(0xC7, "SMB4", Zpg(); op_smb(4));
    OPCODE(0xC8, "INY", op_iny());
    OPCODE(0xC9, "CMP #", Imm(); op_cmp(_rA););
    OPCODE(0xCA, "DEX", op_dex());
    OPCODE(0xCC, "CPY abs", Abs(); op_cmp(_rY));
    OPCODE(0xCD, "CMP abs", Abs(); op_cmp(_rA));
    OPCODE(0xCE, "DEC abs", Abs(); op_dec());
    OPCODE(0xCF, "BBS4", op_bbs(4));
    OPCODE(0xD0, "BNE", Rel(); op_branch(!_rF.Z));
    OPCODE(0xD1, "CMP ind,Y", IndY(); op_cmp(_rA));
    OPCODE(0xD2, "CMP (zpg)", ZpgInd(); op_cmp(_rA));
    OPCODE(0xD3, "TIN", op_tin());
    OPCODE(0xD4, "CSL", op_csh());
    OPCODE(0xD5, "CMP zpg,X", ZpgX(); op_cmp(_rA));
    OPCODE(0xD6, "DEC zpg,X", ZpgX(); op_dec());
    OPCODE(0xD7, "SMB5", Zpg(); op_smb(5));
    OPCODE(0xD8, "CLD", _rF.D = 0);
    OPCODE(0xD9, "CMP abs,Y", Abs(_rY); op_cmp(_rA));
    OPCODE(0xDA, "PHX", op_phx());
    OPCODE(0xDD, "CMP abs,X", Abs(_rX); op_cmp(_rA));
    OPCODE(0xDE, "DEC abs,X", Abs(_rX); op_dec());
    OPCODE(0xDF, "BBS5", op_bbs(5));
    OPCODE(0xE0, "CPX #", Imm(); op_cmp(_rX));
    OPCODE(0xE1, "SBC X,ind", XInd(); op_sbc());
    OPCODE(0xE3, "TIA", op_tia());
    OPCODE(0xE4, "CPX zpg", Zpg(); op_cmp(_rX));
    OPCODE(0xE5, "SBC zpg", Zpg(); op_sbc());
    OPCODE(0xE6, "INC zpg", Zpg(); op_inc());
    OPCODE(0xE7, "SMB6", Zpg(); op_smb(6));
    OPCODE(0xE8, "INX", op_inx());
    OPCODE(0xE9, "SBC #", Imm(); op_sbc());
    OPCODE(0xEA, "NOP", );
    OPCODE(0xEC, "CPX abs", Abs(); op_cmp(_rX));
    OPCODE(0xED, "SBC abs", Abs(); op_sbc());
    OPCODE(0xEE, "INC abs", Abs(); op_inc());
    OPCODE(0xEF, "BBS6", op_bbs(6));
    OPCODE(0xF0, "BEQ", Rel(); op_branch(_rF.Z));
    OPCODE(0xF1, "SBC ind,Y", IndY(); op_sbc());
    OPCODE(0xF2, "SBC (zpg)", ZpgInd(); op_sbc());
    OPCODE(0xF3, "TAI", op_tai());
    OPCODE(0xF5, "SBC zpg,X", ZpgX(); op_sbc());
    OPCODE(0xF6, "INC zpg,X", ZpgX(); op_inc());
    OPCODE(0xF7, "SMB6", Zpg(); op_smb(7));
    OPCODE(0xF8, "SED", _rF.D = 1);
    OPCODE(0xF9, "SBC abs,Y", Abs(_rY); op_sbc());
    OPCODE(0xFA, "PLX", op_plx());
    OPCODE(0xFD, "SBC abs,X", Abs(_rX); op_sbc());
    OPCODE(0xFE, "INC abs,X", Abs(_rX); op_inc());
    OPCODE(0xFF, "BBS7", op_bbs(7));
    default:
      throw CpuOpcodeFault(name(), op, _op_pc);
#if 0
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
#endif
  }

  IF_LOG(Trace)
  log_op(op);

  return _icycles;
}

byte_t hu6280Cpu::irq_read(offset_t offset) {
  DEVICE_DEBUG("IRQ status");
  switch (offset & 0x03) {
    case 2:
      return _irq_disable;
    case 3:
      return _irq_status;
  }
  return 0;
}

void hu6280Cpu::irq_write(offset_t offset, byte_t value) {
  DEVICE_DEBUG("Updating IRQ status");
  switch (offset & 0x03) {
    case 2:
      _irq_disable = value;
      break;
    case 3:
      bit_set(_irq_status, 2, false);
      break;
  }
}

byte_t hu6280Cpu::timer_read(offset_t offset) {
  return (_timer_value >> 10) & 0x7F;
}

void hu6280Cpu::timer_write(offset_t offset, byte_t value) {
  switch (offset & 0x1) {
    case 0:
      _timer_value = _timer_load = ((value & 0x7F) + 1) << 10;
      break;
    case 1:
      _timer_status = bit_isset(value, 0);
      break;
  }
}
