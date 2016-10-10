#pragma once

#include "emu/emu.h"

#define ADDR(addr)             \
  template <class _state_type> \
  static inline void addr(_state_type *state)
#define OP(op, ...)            \
  template <class _state_type> \
  static inline void op(_state_type *state, ##__VA_ARGS__)

namespace M6502 {
template <class _state_type>
static inline void set_sz(_state_type *state, byte_t result) {
  state->F.N = bit_isset(result, 7);
  state->F.Z = (result == 0);
}

template <class _state_type>
static inline uint8_t pc_read(_state_type *state) {
  return state->bus_read(state->PC.d++);
}

template <class _state_type>
static inline void push(_state_type *state, uint8_t value) {
  state->bus_write(((uint16_t)state->ZPG << 8) + 0x0100 + state->SP, value);
  state->SP--;
}

template <class _state_type>
static inline uint8_t pop(_state_type *state) {
  state->SP++;
  return state->bus_read(((uint16_t)state->ZPG << 8) + 0x0100 + state->SP);
}

template <class _state_type>
static inline uint8_t fetch(_state_type *state) {
  state->ARG = state->bus_read(state->EA.d);
  return state->ARG;
}

template <class _state_type>
static inline void store(_state_type *state, byte_t value) {
  state->bus_write(state->EA.d, value);
}

ADDR(Inherent) {}
ADDR(Absolute) {
  state->EA.b.l = pc_read(state);
  state->EA.b.h = pc_read(state);
}
ADDR(AbsoluteX) {
  state->EA.b.l = pc_read(state);
  state->EA.b.h = pc_read(state);
  state->EA.d += state->X;
  if (state->EA.b.l < state->X) state->icycles += 1;
}
ADDR(AbsoluteY) {
  state->EA.b.l = pc_read(state);
  state->EA.b.h = pc_read(state);
  state->EA.d += state->Y;
  if (state->EA.b.l < state->Y) state->icycles += 1;
}
ADDR(Indirect) {
  reg16_t addr;
  addr.b.l = pc_read(state);
  addr.b.h = pc_read(state);
  state->EA.b.l = state->bus_read(addr.d);
  addr.b.l++;
  state->EA.b.h = state->bus_read(addr.d);
}
ADDR(IndirectY) {
  reg16_t addr;
  addr.b.l = pc_read(state);
  addr.b.h = state->ZPG;
  state->EA.b.l = state->bus_read(addr.d);
  addr.b.l++;
  state->EA.b.h = state->bus_read(addr.d);
  state->EA.d += state->Y;
  if (state->EA.b.l < state->Y) state->icycles += 1;
}
ADDR(XIndirect) {
  reg16_t addr;
  addr.b.l = pc_read(state) + state->X;
  addr.b.h = state->ZPG;
  state->EA.b.l = state->bus_read(addr.d);
  addr.b.l++;
  state->EA.b.h = state->bus_read(addr.d);
}
ADDR(ZeroPageIndirect) {
  reg16_t addr;
  addr.b.l = pc_read(state);
  addr.b.h = state->ZPG;
  state->EA.b.l = state->bus_read(addr.d);
  addr.b.l++;
  state->EA.b.h = state->bus_read(addr.d);
}
ADDR(ZeroPage) {
  state->EA.b.l = pc_read(state);
  state->EA.b.h = state->ZPG;
}
ADDR(ZeroPageX) {
  state->EA.b.l = pc_read(state) + state->X;
  state->EA.b.h = state->ZPG;
}
ADDR(ZeroPageY) {
  state->EA.b.l = pc_read(state) + state->Y;
  state->EA.b.h = state->ZPG;
}
ADDR(Immediate) {
  state->EA = state->PC;
  state->PC.d++;
}
ADDR(Relative) {
  state->ARG = pc_read(state);
  state->EA = state->PC;
  state->EA.d += (char)state->ARG;
}

OP(ADC) {
  int result;
  fetch(state);
  if (state->F.D) {
    uint8_t l = (state->A & 0x0F) + (state->ARG & 0x0F) + state->F.C;
    if (l > 9) l += 6;
    uint8_t h = (state->A >> 4) + (state->ARG >> 4) + (l >> 4);
    result = (h << 4) | (l & 0x0F);
  } else {
    result = state->A + state->ARG + state->F.C;
  }
  set_sz(state, result);
  state->F.C = bit_isset(result, 8);
  state->F.V =
      bit_isset((state->A ^ state->ARG ^ 0x80) & (state->ARG ^ result), 7);
  state->A = result;
}

OP(AND) {
  fetch(state);
  int result = state->A & state->ARG;
  set_sz(state, result);
  state->A = result;
}

OP(ASL) {
  fetch(state);
  int result = state->ARG << 1;
  state->F.C = bit_isset(state->ARG, 7);
  set_sz(state, result);
  store(state, result);
}

OP(ASLA) {
  state->ARG = state->A;
  int result = state->ARG << 1;
  state->F.C = bit_isset(state->ARG, 7);
  set_sz(state, result);
  state->A = result;
}

OP(BCC) {
  if (state->F.C == 0) {
    state->PC = state->EA;
    state->icycles += 2;
  }
}
OP(BCS) {
  if (state->F.C != 0) {
    state->PC = state->EA;
    state->icycles += 2;
  }
}
OP(BEQ) {
  if (state->F.Z != 0) {
    state->PC = state->EA;
    state->icycles += 2;
  }
}
OP(BIT) {
  fetch(state);
  int result = state->ARG;
  state->F.N = bit_isset(result, 7);
  state->F.V = bit_isset(result, 6);
  state->F.Z = (state->A & result) == 0;
}
OP(BMI) {
  if (state->F.N != 0) {
    state->PC = state->EA;
    state->icycles += 2;
  }
}
OP(BNE) {
  if (state->F.Z == 0) {
    state->PC = state->EA;
    state->icycles += 2;
  }
}
OP(BPL) {
  if (state->F.N == 0) {
    state->PC = state->EA;
    state->icycles += 2;
  }
}
OP(BRK) {
  state->F.B = 1;
  state->PC.d++;
  push(state, state->PC.b.h);
  push(state, state->PC.b.l);
  push(state, state->SR);
  state->PC.b.l = state->bus_read(0xFFFE);
  state->PC.b.h = state->bus_read(0xFFFF);
}
OP(BVC) {
  if (state->F.V == 0) {
    state->PC = state->EA;
    state->icycles += 2;
  }
}
OP(BVS) {
  if (state->F.V != 0) {
    state->PC = state->EA;
    state->icycles += 2;
  }
}
OP(CLC) { state->F.C = 0; }
OP(CLD) { state->F.D = 0; }
OP(CLI) { state->F.I = 0; }
OP(CLV) { state->F.V = 0; }
OP(CMP) {
  fetch(state);
  int result = state->A - state->ARG;
  set_sz(state, result);
  state->F.C = state->ARG <= state->A;
}
OP(CPX) {
  fetch(state);
  int result = state->X - state->ARG;
  set_sz(state, result);
  state->F.C = state->ARG <= state->X;
}
OP(CPY) {
  fetch(state);
  int result = state->Y - state->ARG;
  set_sz(state, result);
  state->F.C = state->ARG <= state->Y;
}
OP(DEC) {
  fetch(state);
  int result = state->ARG - 1;
  set_sz(state, result);
  store(state, result);
}
OP(DEA) {
  int result = state->A - 1;
  set_sz(state, result);
  state->A = result;
}
OP(DEX) {
  byte_t result = state->X - 1;
  set_sz(state, result);
  state->X = result;
}
OP(DEY) {
  int result = state->Y - 1;
  set_sz(state, result);
  state->Y = result;
}
OP(EOR) {
  fetch(state);
  int result = state->A ^ state->ARG;
  set_sz(state, result);
  state->A = result;
}
OP(INC) {
  fetch(state);
  int result = state->ARG + 1;
  set_sz(state, result);
  store(state, result);
}
OP(INX) {
  byte_t result = state->X + 1;
  set_sz(state, result);
  state->X = result;
}
OP(INY) {
  int result = state->Y + 1;
  set_sz(state, result);
  state->Y = result;
}
OP(JMP) { state->PC = state->EA; }
OP(JSR) {
  state->PC.d--;
  push(state, state->PC.b.h);
  push(state, state->PC.b.l);
  state->PC = state->EA;
}
OP(LDA) {
  int result = fetch(state);
  set_sz(state, result);
  state->A = result;
}
OP(LDX) {
  int result = fetch(state);
  set_sz(state, result);
  state->X = result;
}
OP(LDY) {
  int result = fetch(state);
  set_sz(state, result);
  state->Y = result;
}
OP(LSR) {
  fetch(state);
  int result = state->ARG >> 1;
  state->F.C = bit_isset(state->ARG, 0);
  set_sz(state, result);
  store(state, result);
}
OP(LSRA) {
  state->ARG = state->A;
  int result = state->ARG >> 1;
  state->F.C = bit_isset(state->ARG, 0);
  set_sz(state, result);
  state->A = result;
}
OP(NOP) {}
OP(ORA) {
  fetch(state);
  int result = state->A | state->ARG;
  set_sz(state, result);
  state->A = result;
}
OP(PHA) { push(state, state->A); }
OP(PHP) { push(state, state->SR); }
OP(PLA) {
  state->A = pop(state);
  set_sz(state, state->A);
}
OP(PLP) {
  state->SR = pop(state);
  state->F.B = 1;
  state->F.E = 1;
}
OP(ROL) {
  fetch(state);
  int result = state->ARG << 1 | state->F.C;
  state->F.C = bit_isset(state->ARG, 7);
  set_sz(state, result);
  store(state, result);
}
OP(ROLA) {
  state->ARG = state->A;
  int result = state->ARG << 1 | state->F.C;
  state->F.C = bit_isset(state->ARG, 7);
  set_sz(state, result);
  state->A = result;
}
OP(ROR) {
  fetch(state);
  int result = state->ARG >> 1 | (state->F.C << 7);
  state->F.C = bit_isset(state->ARG, 0);
  set_sz(state, result);
  store(state, result);
}
OP(RORA) {
  state->ARG = state->A;
  int result = state->ARG >> 1 | (state->F.C << 7);
  state->F.C = bit_isset(state->ARG, 0);
  set_sz(state, result);
  state->A = result;
}
OP(RTI) {
  state->SR = pop(state);
  state->F.E = 1;
  state->PC.b.l = pop(state);
  state->PC.b.h = pop(state);
}
OP(RTS) {
  state->PC.b.l = pop(state);
  state->PC.b.h = pop(state);
  state->PC.d++;
}
OP(SBC) {
  fetch(state);
  uint32_t result = state->A + (~state->ARG) + state->F.C;
  set_sz(state, result);
  state->F.C = result < 0x100;
  state->F.V = bit_isset((result ^ state->A) & (state->A ^ state->ARG), 7);
  state->A = result;
}
OP(SEC) { state->F.C = 1; }
OP(SED) { state->F.D = 1; }
OP(SEI) { state->F.I = 1; }
OP(STA) { store(state, state->A); }
OP(STX) { store(state, state->X); }
OP(STY) { store(state, state->Y); }
OP(TAX) {
  state->X = state->A;
  set_sz(state, state->X);
}
OP(TAY) {
  state->Y = state->A;
  set_sz(state, state->Y);
}
OP(TSX) {
  state->X = state->SP;
  set_sz(state, state->X);
}
OP(TXA) {
  state->A = state->X;
  set_sz(state, state->A);
}
OP(TYA) {
  state->A = state->Y;
  set_sz(state, state->A);
}
OP(TXS) { state->SP = state->X; }

OP(NMI, uint16_t addr) {
  state->F.B = 0;
  push(state, state->PC.b.h);
  push(state, state->PC.b.l);
  push(state, state->SR);
  state->PC.b.l = state->bus_read(addr);
  state->PC.b.h = state->bus_read(++addr);
}

OP(IRQ, uint16_t addr) {
  push(state, state->PC.b.h);
  push(state, state->PC.b.l);
  push(state, state->SR);
  state->F.B = 0;
  state->F.I = 1;
  state->PC.b.l = state->bus_read(addr);
  state->PC.b.h = state->bus_read(++addr);
}
};

/* 65C02 Ops */
namespace M65C02 {
using namespace M6502;

ADDR(ZeroIndirect) {
  reg16_t addr;
  addr.b.l = pc_read(state);
  addr.b.h = state->ZPG;
  state->EA.b.l = state->bus_read(addr.d);
  addr.b.l++;
  state->EA.b.h = state->bus_read(addr.d);
}

OP(TSB) {
  fetch(state);
  int result = state->A | state->ARG;
  state->F.Z = (state->A & state->ARG) == 0;
  store(state, result);
}

OP(TRB) {
  fetch(state);
  int result = ~state->A & state->ARG;
  state->F.Z = (state->A & state->ARG) == 0;
  store(state, result);
}

OP(BITIMM) {
  fetch(state);
  state->F.Z = (state->A & state->ARG) == 0;
}

OP(INA) {
  int result = state->A + 1;
  set_sz(state, result);
  state->A = result;
}

OP(PHX) { push(state, state->X); }

OP(PLX) {
  int result = pop(state);
  set_sz(state, result);
  state->X = result;
}

OP(PHY) { push(state, state->Y); }

OP(PLY) {
  int result = pop(state);
  set_sz(state, result);
  state->Y = result;
}

OP(SMB, uint8_t bit) {
  ZeroPage(state);
  fetch(state);
  int result = state->ARG | (1 << bit);
  store(state, result);
}

OP(RMB, uint8_t bit) {
  ZeroPage(state);
  fetch(state);
  int result = state->ARG & ~(1 << bit);
  store(state, result);
}

OP(BBR, uint8_t bit) {
  ZeroPage(state);
  byte_t arg = fetch(state);
  Relative(state);
  if (!bit_isset(arg, bit)) {
    state->PC = state->EA;
    state->icycles += 2;
  }
}

OP(BBS, uint8_t bit) {
  ZeroPage(state);
  byte_t arg = fetch(state);
  Relative(state);
  if (bit_isset(arg, bit)) {
    state->PC = state->EA;
    state->icycles += 2;
  }
}

OP(STZ) { store(state, 0); }

OP(BRA) {
  state->PC = state->EA;
  state->icycles += 2;
}
};
