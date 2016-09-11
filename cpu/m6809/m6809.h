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
 * M6809 Cpu Core
 */
#pragma once

#include "cpu/lib/cpu.h"
#include "emu/emu.h"

namespace M6809 {

using namespace EMU;

enum M6809Irq {
  None = 0,
  SWI3,
  SWI2,
  FIRQ,
  IRQ,
  SWI,
  NMI,
  Reset,
  Size,
  First = SWI3,
  Last = Reset
};

enum class M6809Reg {
  D = 0,
  X = 1,
  Y = 2,
  U = 3,
  S = 4,
  PC = 5,
  A = 8,
  B = 9,
  CC = 10,
  DP = 11,
};

struct M6809State {
  M6809State(void) = default;

  void set(const M6809Reg reg, reg16_t value) {
    switch (reg) {
      case M6809Reg::D:
        D = value;
        break;
      case M6809Reg::X:
        X = value;
        break;
      case M6809Reg::Y:
        Y = value;
        break;
      case M6809Reg::U:
        U = value;
        break;
      case M6809Reg::S:
        S = value;
        break;
      case M6809Reg::PC:
        PC = value;
        break;
      default:
        throw DeviceFault("M6809", "Unknown Register");
    }
  }
  void set(const M6809Reg reg, reg8_t value) {
    switch (reg) {
      case M6809Reg::A:
        D.b.l = value;
        break;
      case M6809Reg::B:
        D.b.l = value;
        break;
      case M6809Reg::CC:
        CC = value;
        break;
      case M6809Reg::DP:
        DP = value;
        break;
      default:
        throw DeviceFault("M6809", "Unknown Register");
    }
  }

  void set_z(int result) { F.Z = (result == 0); }

  void set_c(int result) { F.C = (result & 0x100) != 0; }

  void set_n(int result) { F.N = (result & 0x80) != 0; }

  void set_h(int a, int b, int r) { F.H = ((a ^ b ^ r) & 0x10) != 0; }

  void set_v(int a, int b, int r) {
    F.V = ((a ^ b ^ r ^ (r >> 1)) & 0x80) != 0;
  }

  reg16_t D;
  reg16_t X;
  reg16_t Y;
  reg16_t U;
  reg16_t S;
  reg16_t PC;
  reg8_t DP;
  union {
    struct {
      byte_t C : 1;
      byte_t V : 1;
      byte_t Z : 1;
      byte_t N : 1;
      byte_t I : 1;
      byte_t H : 1;
      byte_t F : 1;
      byte_t E : 1;
    } F;
    reg8_t CC;
  };

  /* Internal Registers */
  reg16_t EA; /* Effective Address */
  reg8_t RD;  /* Read Data */
  reg8_t WD;  /* Write Data */

  /* Interrupt Vectors */
  LineState Irq[M6809Irq::Size];
};

#define _rA m_state.D.b.l
#define _rB m_state.D.b.h
#define _rD m_state.D.d
#define _rX m_state.X.d
#define _rXl m_state.X.b.l
#define _rXh m_state.X.b.h
#define _rY m_state.Y.d
#define _rYl m_state.Y.b.l
#define _rYh m_state.Y.b.h
#define _rU m_state.U.d
#define _rUl m_state.U.b.l
#define _rUh m_state.U.b.h
#define _rS m_state.S.d
#define _rSl m_state.S.b.l
#define _rSh m_state.S.b.h
#define _rPC m_state.PC.d
#define _rPCl m_state.PC.b.l
#define _rPCh m_state.PC.b.h
#define _rDP m_state.DP
#define _rCC m_state.CC
#define _rF m_state.F

#define _rEA m_state.EA.d
#define _rRD m_state.RD
#define _rWD m_state.WD

#define ADDR(addr) void addr(void)
#define OP(name) void name(void)

struct M6809Opcode {
  uint16_t code;
  const char *name;
  std::function<void(void)> addr_mode;
  std::function<void(void)> operation;
};

class M6809Cpu : public CPU::Cpu<AddressBus16x8, M6809State, uint16_t> {
 public:
  M6809Cpu(Machine *machine, const std::string &name, unsigned hertz,
           AddressBus16x8 *bus);
  ~M6809Cpu(void);
  M6809Cpu(const M6809Cpu &cpu) = delete;

  virtual void line(Line line, LineState state);

  virtual std::string dasm(addr_type addr);

 private:
  byte_t pc_read(void) {
    byte_t tmp = bus_read(m_state.PC.d++);
    return tmp;
  }

  uint16_t bus_read16(uint16_t addr) {
    uint16_t tmp = bus_read(addr) | (bus_read(addr + 1) << 8);
    return tmp;
  }

  byte_t data_read(void) {
    m_state.RD = bus_read(m_state.EA.d);
    return m_state.RD;
  }

  void data_write(byte_t data) {
    m_state.WD = data;
    bus_write(m_state.EA.d, m_state.WD);
  }

  void _reset(void);

  void build_table(void);

  void log_op(uint16_t pc, const M6809Opcode *op);

 private:
  /* Addressing Modes */
  ADDR(Immediate) {
    _rEA = _rPC;
    _rPC++;
  }

  ADDR(Direct) {
    m_state.EA.b.l = pc_read();
    m_state.EA.b.h = _rDP;
  }

  ADDR(Extended) { throw DeviceFault(name()); }

  ADDR(Relative) { throw DeviceFault(name()); }

  ADDR(Indexed) {
    byte_t idx = 0; /* XXX: Read index */
    if (bit_isset(idx, 7)) {
      switch (idx & 0x0F) {
        case 0: /* ,R+ */
          break;
        case 1: /* ,R++ */
          break;
        case 2: /* ,-R */
          break;
        case 3: /* ,--R */
          break;
        case 4: /* ,R */
          break;
        case 5: /* (+/- B),R */
          break;
        case 6: /* (+/- A),R */
          break;
        case 8: /* (+/- 7 bit offset),R + one byte */
          break;
        case 9: /* (+/- 15 bit offset),R + two bytes */
          break;
        case 11: /* (+/- D),R */
          break;
        case 12: /* (+/- 7 bit offset),PC + one byte */
          break;
        case 13: /* (+/- 15 bit offset),PC */
          break;
        case 15: /* [address] + one byte*/
          break;
        default:
          throw DeviceFault(name(), "Invalid Instruction");
      }

    } else {
      /* XXX: (+/- 4 bit offset),R  */
    }
  }

  ADDR(Inherent) {}

  /* Instructions */
  OP(ABX) { throw DeviceFault(name()); }

  OP(ADCA) {
    byte_t arg = data_read();
    int result = _rA + arg + _rF.C;
    /* XXX: set flags */
    _rA = result;
  }

  OP(ADCB) {
    byte_t arg = data_read();
    int result = _rB + arg + _rF.C;
    /* XXX: set flags */
    _rB = result;
  }

  OP(ADDA) {
    byte_t arg = data_read();
    int result = _rA + arg;
    /* XXX: Set flags */
    _rA = result;
  }

  OP(ADDB) {
    byte_t arg = data_read();
    int result = _rB + arg;
    /* XXX: Set flags */
    _rB = result;
  }

  OP(ADDD) {
    byte_t arg = data_read();
    int result = _rD + arg;
    /* XXX: set flags */
    _rD = result;
  }

  OP(ANDA) {
    byte_t arg = data_read();
    int result = _rA & arg;
    /* XXX: set flags */
    _rA = result;
  }

  OP(ANDB) {
    byte_t arg = data_read();
    int result = _rB & arg;
    /* XXX: set flags */
    _rB = result;
  }

  OP(ANDCC) { throw DeviceFault(name()); }

  OP(ASL) { throw DeviceFault(name()); }

  OP(ASLA) {
    int result = _rA << 1;
    /* XXX: set flags */
    _rA = result;
  }

  OP(ASLB) {
    int result = _rB << 1;
    /* XXX: set flags */
    _rB = result;
  }

  OP(ASR) { throw DeviceFault(name()); }

  OP(ASRA) {
    int result = _rA >> 1;
    /* XXX: set flags */
    _rA = result;
  }

  OP(ASRB) {
    int result = _rB >> 1;
    /* XXX: set flags */
    _rB = result;
  }

  OP(BITA) {
    byte_t arg = data_read();
    int result a_unused = _rA & arg;
    /* XXX: set flags */
  }

  OP(BITB) {
    byte_t arg = data_read();
    int result a_unused = _rB & arg;
    /* XXX: set flags */
  }

  OP(BRA) { branch(_rEA); }

  OP(BRN) {}

  OP(BCC) {
    if (_rF.C == 0) branch(_rEA);
  }

  OP(BCS) {
    if (_rF.C != 0) branch(_rEA);
  }

  OP(BEQ) {
    if (_rF.Z == 1) branch(_rEA);
  }

  OP(BGE) {
    if (_rF.V ^ _rF.N) branch(_rEA);
  }

  OP(BGT) {
    if ((_rF.Z | (_rF.N ^ _rF.V)) == 0) branch(_rEA);
  }

  OP(BHI) {
    if ((_rF.C | _rF.Z) == 0) branch(_rEA);
  }

  OP(BHS) {
    if (_rF.C == 0) branch(_rEA);
  }

  OP(BLE) {
    if ((_rF.Z | (_rF.N & _rF.V)) != 0) branch(_rEA);
  }

  OP(BLO) {
    if (_rF.C != 0) branch(_rEA);
  }

  OP(BLS) {
    if ((_rF.C | _rF.Z) != 0) branch(_rEA);
  }

  OP(BLT) {
    if ((_rF.N ^ _rF.V) != 0) branch(_rEA);
  }

  OP(BMI) {
    if (_rF.N != 0) branch(_rEA);
  }

  OP(BNE) {
    if (_rF.Z == 0) branch(_rEA);
  }

  OP(BPL) {
    if (_rF.N == 0) branch(_rEA);
  }

  OP(BSR) {
    bus_write(_rS--, _rPCl);
    bus_write(_rS--, _rPCh);
    branch(_rEA);
  }

  OP(BVC) {
    if (_rF.V == 0) branch(_rEA);
  }

  OP(BVS) {
    if (_rF.V != 0) branch(_rEA);
  }

  OP(CLR) { throw DeviceFault(name()); }

  OP(CLRA) {
    int result = 0;
    /* XXX: set flags */
    _rA = result;
  }

  OP(CLRB) {
    int result = 0;
    /* XXX: set flags */
    _rB = result;
  }

  OP(CMPA) {
    byte_t arg = data_read();
    int result a_unused = (_rA == arg);
    /* XXX: set flags */
  }

  OP(CMPB) {
    byte_t arg = data_read();
    int result a_unused = (_rB == arg);
    /* XXX: set flags */
  }

  OP(CMPX) { throw DeviceFault(name()); }

  OP(COM) { throw DeviceFault(name()); }

  OP(COMA) {
    int result = ~_rA;
    /* XXX: set flags */
    _rA = result;
  }

  OP(COMB) {
    int result = ~_rB;
    /* XXX: set flags */
    _rB = result;
  }

  OP(CWAI) { throw DeviceFault(name()); }

  OP(DAA) { throw DeviceFault(name()); }

  OP(DEC) { throw DeviceFault(name()); }

  OP(DECA) {
    int result = _rA - 1;
    /* XXX: set flags */
    _rA = result;
  }

  OP(DECB) {
    int result = _rB - 1;
    /* XXX: set flags */
    _rB = result;
  }

  OP(EORA) {
    byte_t arg = data_read();
    int result = _rA ^ arg;
    /* XXX: set flags */
    _rA = result;
  }

  OP(EORB) {
    byte_t arg = data_read();
    int result = _rB ^ arg;
    /* XXX: set flags */
    _rB = result;
  }

  OP(EXG) { throw DeviceFault(name()); }

  OP(INC) { throw DeviceFault(name()); }

  OP(INCA) {
    int result = _rA + 1;
    /* XXX: set flags */
    _rA = result;
  }

  OP(INCB) {
    int result = _rB + 1;
    /* XXX: set flags */
    _rB = result;
  }

  OP(JMP) { throw DeviceFault(name()); }

  OP(JSR) { throw DeviceFault(name()); }

  OP(LBRA) { throw DeviceFault(name()); }

  OP(LBSR) { throw DeviceFault(name()); }

  OP(LDA) {
    byte_t arg = data_read();
    int result = arg;
    /* XXX: set flags */
    _rA = result;
  }

  OP(LDB) {
    byte_t arg = data_read();
    int result = arg;
    /* XXX: set flags */
    _rB = result;
  }

  OP(LDD) { throw DeviceFault(name()); }

  OP(LDX) { throw DeviceFault(name()); }

  OP(LDU) { throw DeviceFault(name()); }

  OP(LEAX) { throw DeviceFault(name()); }

  OP(LEAY) { throw DeviceFault(name()); }

  OP(LEAS) { throw DeviceFault(name()); }

  OP(LEAU) { throw DeviceFault(name()); }

  OP(LSL) { throw DeviceFault(name()); }

  OP(LSLA) {
    int result = _rA << 1;
    /* XXX: set flags */
    _rA = result;
  }

  OP(LSLB) {
    int result = _rB << 1;
    /* XXX: set flags */
    _rB = result;
  }

  OP(LSR) { throw DeviceFault(name()); }

  OP(LSRA) {
    int result = _rA >> 1;
    /* XXX: set flags */
    _rA = result;
  }

  OP(LSRB) {
    int result = _rB >> 1;
    /* XXX: set flags */
    _rB = result;
  }

  OP(MUL) {
    int result = _rA * _rB;
    /* XXX: set flags */
    _rD = result;
  }

  OP(NEG) { throw DeviceFault(name()); }

  OP(NEGA) {
    int result = _rA & 0x80;
    _rA = result;
  }

  OP(NEGB) {
    int result = _rB & 0x80;
    _rB = result;
  }

  OP(NOP) {}

  OP(ORA) {
    byte_t arg = data_read();
    int result = _rA | arg;
    /* XXX: set flags */
    _rA = result;
  }

  OP(ORB) {
    byte_t arg = data_read();
    int result = _rB | arg;
    /* XXX: set flags */
    _rB = result;
  }

  OP(ORCC) { throw DeviceFault(name()); }

  OP(PSHS) {
    byte_t arg = data_read();
    push(_rS, arg);
  }

  OP(PULS) {
    byte_t arg = data_read();
    pull(_rS, arg);
  }

  OP(PSHU) {
    byte_t arg = data_read();
    push(_rU, arg);
  }

  OP(PULU) {
    byte_t arg = data_read();
    pull(_rU, arg);
  }

  OP(RESET) { throw DeviceFault(name()); }

  OP(ROL) { throw DeviceFault(name()); }

  OP(ROLA) {
    int result = _rA << 1;
    /* XXX: set flags */
    _rA = result;
  }

  OP(ROLB) {
    int result = _rB << 1;
    /* XXX: set flags */
    _rB = result;
  }

  OP(ROR) { throw DeviceFault(name()); }

  OP(RORA) {
    int result = _rA >> 1;
    /* XXX: set flags */
    _rA = result;
  }

  OP(RORB) {
    int result = _rB >> 1;
    /* XXX: set flags */
    _rB = result;
  }

  OP(RTI) {
    _rCC = bus_read(--_rS);
    if (_rF.E == 0) {
      pull(_rS, 0x80);
      /* XXX: 6 cycles */
    } else {
      pull(_rS, 0xFE);
      /* XXX: 15 cycles */
    }
  }

  OP(RTS) { throw DeviceFault(name()); }

  OP(SBCA) {
    byte_t arg = data_read();
    int result = _rA - arg - _rF.C;
    /* XXX: set flags */
    _rA = result;
  }

  OP(SBCB) {
    byte_t arg = data_read();
    int result = _rA - arg - _rF.C;
    /* XXX: set flags */
    _rB = result;
  }

  OP(SEX) { throw DeviceFault(name()); }

  OP(STA) { data_write(_rA); }

  OP(STB) { data_write(_rB); }

  OP(STD) { throw DeviceFault(name()); }

  OP(STX) { throw DeviceFault(name()); }

  OP(STU) { throw DeviceFault(name()); }

  OP(SUBA) {
    byte_t arg = data_read();
    int result = _rA - arg;
    _rA = result;
  }

  OP(SUBB) {
    byte_t arg = data_read();
    int result = _rB - arg;
    _rB = result;
  }

  OP(SUBD) { throw DeviceFault(name()); }

  OP(SWI) { throw DeviceFault(name()); }

  OP(SWI2) { throw DeviceFault(name()); }

  OP(SWI3) { throw DeviceFault(name()); }

  OP(SYNC) { throw DeviceFault(name()); }

  OP(TST) { throw DeviceFault(name()); }

  OP(TSTA) {
    int result a_unused = (_rA == 0);
    /* XXX: set flags */
  }

  OP(TSTB) {
    int result a_unused = (_rB == 0);
    /* XXX: set flags */
  }

  OP(TFR) { throw DeviceFault(name()); }

 private:
  void branch(uint16_t addr) { _rPC = addr; }

  void push(uint16_t &stack, uint8_t arg) {
    if (bit_isset(arg, 7)) {
      bus_write(stack--, _rPCl);
      bus_write(stack--, _rPCh);
    }
    if (bit_isset(arg, 6)) {
      bus_write(stack--, _rUl);
      bus_write(stack--, _rUh);
    }
    if (bit_isset(arg, 5)) {
      bus_write(stack--, _rYl);
      bus_write(stack--, _rYh);
    }
    if (bit_isset(arg, 4)) {
      bus_write(stack--, _rXl);
      bus_write(stack--, _rXh);
    }
    if (bit_isset(arg, 3)) {
      bus_write(stack--, _rDP);
    }
    if (bit_isset(arg, 2)) {
      bus_write(stack--, _rB);
    }
    if (bit_isset(arg, 1)) {
      bus_write(stack--, _rA);
    }
    if (bit_isset(arg, 0)) {
      bus_write(stack--, _rCC);
    }
  }

  void pull(uint16_t &stack, uint8_t arg) {
    if (bit_isset(arg, 0)) {
      _rCC = bus_read(--stack);
    }
    if (bit_isset(arg, 1)) {
      _rA = bus_read(--stack);
    }
    if (bit_isset(arg, 2)) {
      _rB = bus_read(--stack);
    }
    if (bit_isset(arg, 3)) {
      _rDP = bus_read(--stack);
    }
    if (bit_isset(arg, 4)) {
      _rXh = bus_read(--stack);
      _rXl = bus_read(--stack);
    }
    if (bit_isset(arg, 5)) {
      _rYh = bus_read(--stack);
      _rYl = bus_read(--stack);
    }
    if (bit_isset(arg, 6)) {
      _rUh = bus_read(--stack);
      _rUl = bus_read(--stack);
    }
    if (bit_isset(arg, 7)) {
      _rPCh = bus_read(--stack);
      _rPCl = bus_read(--stack);
    }
  }

  void dispatch(void);
  void check_interrupt(void);
  void take_interrupt(uint16_t addr);

  std::unordered_map<uint16_t, M6809Opcode> _opcodes;

  M6809Opcode _op;
};
};
