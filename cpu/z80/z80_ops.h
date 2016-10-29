#pragma once

#include "cpu/z80/z80.h"

#define OP(op, ...)              \
  template <typename state_type> \
  static inline void a_used op(state_type *state, ##__VA_ARGS__)

namespace Z80 {

static inline reg8_t a_used CALC_PARITY(reg8_t reg);
static inline reg8_t a_used CALC_PARITY(reg16_t reg);
static inline reg8_t a_used CALC_OVERFLOW(reg8_t arg1, reg8_t arg2, reg8_t result);
static inline reg8_t a_used CALC_OVERFLOW(reg8_t arg1, reg8_t arg2, reg16_t result);

OP(PUSH, byte_t high, byte_t low);
OP(POP, byte_t &high, byte_t &low);
OP(RL, byte_t &dest, byte_t value);
OP(RLC, byte_t &dest, byte_t value);
OP(RR, byte_t &dest, byte_t value);
OP(RRC, byte_t &dest, byte_t value);
OP(SLA, byte_t &dest, byte_t value);
OP(SLL, byte_t &dest, byte_t value);
OP(SRL, byte_t &dest, byte_t value);
OP(SRA, byte_t &dest, byte_t value);
OP(SUB, byte_t &dest, byte_t arg);

OP(ADC, byte_t &dest, byte_t arg) {
  uint16_t result = (uint16_t)dest + arg + state->AF.b.f.C;

  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.H = bit_isset(dest ^ arg ^ result, 4);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = CALC_OVERFLOW(dest, arg, result);
  state->AF.b.f.N = 0;
  state->AF.b.f.C = bit_isset(result, 8);

  dest = (byte_t)result;
}

OP(ADC16, uint16_t &wdest, uint16_t arg) {
  uint32_t result = wdest + arg + state->AF.b.f.C;

  state->AF.b.f.S = bit_isset(result, 15);
  state->AF.b.f.Z = (result & 0xffff) == 0;
  state->AF.b.f.Y = bit_isset(result, 13);
  state->AF.b.f.H = bit_isset(wdest ^ arg ^ result, 12);
  state->AF.b.f.X = bit_isset(result, 11);
  state->AF.b.f.V = bit_isset((wdest ^ arg ^ 0x8000) & (arg ^ result), 15);
  state->AF.b.f.N = 0;
  state->AF.b.f.C = bit_isset(result, 16);

  wdest = result;
}

OP(ADD, byte_t &dest, byte_t arg) {
  uint16_t result = (uint16_t)dest + arg;

  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.H = bit_isset(dest ^ arg ^ result, 4);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = CALC_OVERFLOW(dest, arg, result);
  state->AF.b.f.N = 0;
  state->AF.b.f.C = bit_isset(result, 8);

  dest = (byte_t)result;
}

OP(ADD16rel, reg16_t &dest, uint8_t arg) {
  uint32_t result = dest.d + (char)arg;

  state->AF.b.f.Y = bit_isset(result, 13);
  state->AF.b.f.H = bit_isset(dest.d ^ arg ^ result, 12);
  state->AF.b.f.X = bit_isset(result, 11);
  state->AF.b.f.C = (result > 0xffff) ? 1 : 0;
  state->AF.b.f.N = 0;

  dest = static_cast<reg16_t>(result);
}

OP(ADD16, reg16_t &dest, uint16_t arg) {
  uint32_t result = dest.d + arg;

  state->AF.b.f.Y = bit_isset(result, 13);
  state->AF.b.f.H = bit_isset(dest.d ^ arg ^ result, 12);
  state->AF.b.f.X = bit_isset(result, 11);
  state->AF.b.f.C = (result > 0xffff) ? 1 : 0;
  state->AF.b.f.N = 0;

  dest = static_cast<reg16_t>(result);
}

OP(AND, reg8_t &dest, reg8_t arg) {
  byte_t result = dest & arg;

  state->AF.b.l = (result & 0xA4) | 0x10;
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.V = CALC_PARITY(result);

  dest = result;
}

OP(BIT_TEST_HL, const int bit) {
  byte_t value = bus_read(state, state->HL.d);
  byte_t h = state->WZ.b.h;

  state->AF.b.f.S = bit_isset(value, 7);
  state->AF.b.f.Z = !bit_isset(value, bit);
  state->AF.b.f.Y = bit_isset(h, 5);
  state->AF.b.f.H = 1;
  state->AF.b.f.X = bit_isset(h, 3);
  state->AF.b.f.V = !bit_isset(value, bit);
  state->AF.b.f.N = 0;
}


OP(BIT_TEST_HL, byte_t value, byte_t h, const int bit) {
  state->AF.b.f.S = bit_isset(value, 7);
  state->AF.b.f.Z = !bit_isset(value, bit);
  state->AF.b.f.Y = bit_isset(h, 5);
  state->AF.b.f.H = 1;
  state->AF.b.f.X = bit_isset(h, 3);
  state->AF.b.f.V = !bit_isset(value, bit);
  state->AF.b.f.N = 0;
}

OP(BIT_TEST, byte_t value, const int bit) {
  state->AF.b.f.S = bit_isset(value, 7);
  state->AF.b.f.Z = !bit_isset(value, bit);
  state->AF.b.f.Y = bit_isset(value, 5);
  state->AF.b.f.H = 1;
  state->AF.b.f.X = bit_isset(value, 3);
  state->AF.b.f.V = !bit_isset(value, bit);
  state->AF.b.f.N = 0;
}

OP(BIT_RESET, byte_t &dest, const int bit) {
  dest &= ~(1 << bit);
}

OP(BIT_RESET_HL, const int bit) {
  byte_t value = bus_read(state, state->HL.d);
  value &= ~(1 << bit);

  bus_write(state, state->HL.d, value);
}

OP(BIT_SET, byte_t &dest, const int bit) {
  dest |= (1 << bit);
}

OP(BIT_SET_HL, const int bit) {
  byte_t value = bus_read(state, state->HL.d);
  value |= (1 << bit);

  bus_write(state, state->HL.d, value);
}

static inline reg8_t a_used CALC_PARITY(reg8_t reg) {
  return ((0x6996 >> ((reg ^ (reg >> 4)) & 0xf)) & 1) == 0;
}

static inline reg8_t a_used CALC_PARITY(reg16_t arg) {
  reg8_t reg = arg.b.l;
  return ((0x6996 >> ((reg ^ (reg >> 4)) & 0xf)) & 1) == 0;
}

static inline reg8_t a_used CALC_OVERFLOW(reg8_t arg1, reg8_t arg2,
                                          reg8_t result) {
  return bit_isset((arg1 ^ arg2) & (arg1 ^ result), 7);
}

static inline reg8_t a_used CALC_OVERFLOW(reg8_t arg1, reg8_t arg2,
                                          reg16_t result) {
  return bit_isset((arg1 ^ arg2) & (arg1 ^ result.b.l), 7);
}

OP(CALL, bool jump, uint16_t addr) {
  if (jump) {
    PUSH(state, state->PC.b.h, state->PC.b.l);

    state->PC.d = addr;
    add_icycles(state, 12);
  }
}

OP(CP, byte_t dest, byte_t arg) {
  uint16_t result = dest - arg;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(arg, 5);
  state->AF.b.f.H = bit_isset(arg ^ dest ^ result, 4);
  state->AF.b.f.X = bit_isset(arg, 3);
  state->AF.b.f.V = CALC_OVERFLOW(dest, arg, result);
  state->AF.b.f.N = true;
  state->AF.b.f.C = bit_isset(dest ^ arg ^ result, 8);
}

OP(CPD) {
  byte_t dest = state->AF.b.h;
  byte_t arg = bus_read(state, state->HL.d);
  uint16_t result = dest - arg;
  state->WZ.d--;
  state->HL.d--;
  state->BC.d--;

  state->AF.b.f.H = bit_isset(arg ^ dest ^ result, 4);
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(result - state->AF.b.f.H, 1);
  state->AF.b.f.X = bit_isset(result - state->AF.b.f.H, 3);
  state->AF.b.f.V = (state->BC.d != 0);
  state->AF.b.f.N = true;
}

OP(CPDR) {
  CPD(state);
  if (state->BC.d != 0 && !state->AF.b.f.Z) {
    state->PC.d -= 2;
    add_icycles(state, 5);
  }
}

OP(CPI) {
  byte_t dest = state->AF.b.h;
  byte_t arg = bus_read(state, state->HL.d);
  uint16_t result = dest - arg;
  state->WZ.d++;
  state->HL.d++;
  state->BC.d--;

  state->AF.b.f.H = bit_isset(arg ^ dest ^ result, 4);
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(result - state->AF.b.f.H, 1);
  state->AF.b.f.X = bit_isset(result - state->AF.b.f.H, 3);
  state->AF.b.f.V = (state->BC.d != 0);
  state->AF.b.f.N = true;
}

OP(CPIR) {
  CPI(state);
  if (state->BC.d != 0 && !state->AF.b.f.Z) {
    state->PC.d -= 2;
    add_icycles(state, 5);
  }
}

OP(CPL) {
  byte_t &dest = state->AF.b.h;
  byte_t result = ~dest;

  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.H = 1;
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.N = 1;

  dest = result;
}

OP(CCF) {
  state->AF.b.f.Y = bit_isset(state->AF.b.h, 5);
  state->AF.b.f.H = state->AF.b.f.C;
  state->AF.b.f.X = bit_isset(state->AF.b.h, 3);
  state->AF.b.f.N = 0;
  state->AF.b.f.C = !state->AF.b.f.C;
}

OP(DAA) {
  byte_t &dest = state->AF.b.h;
  uint16_t arg = 0;
  uint16_t result = dest;

  if (!state->AF.b.f.N) {
    if (state->AF.b.f.H || (dest & 0x0f) > 9) arg += 0x06;
    if (state->AF.b.f.C || dest > 0x99) arg += 0x60;
    result += arg;
  } else {
    if (state->AF.b.f.H) arg += 0x6;
    if (state->AF.b.f.C) arg += 0x60;
    result -= arg;
  }

  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.H = 0;
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = CALC_PARITY(result);
  state->AF.b.f.C = bit_isset(dest ^ arg ^ result, 8);

  dest = (byte_t)result;
}

OP(DEC, reg8_t &dest) {
  uint16_t result = dest - 1;

  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.H = bit_isset(dest ^ 0x1 ^ result, 4);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = (dest == 0x80);
  state->AF.b.f.N = true;

  dest = (byte_t)result;
}

OP(DECI, addr_t addr) {
  byte_t dest = bus_read(state, addr);

  DEC(state, dest);

  bus_write(state, addr, dest);
}

OP(DEC16, reg16_t &dest) {
  uint16_t result = dest.d - 1;

  dest = (reg16_t)result;
}

OP(DI) {
  state->iff1 = state->iff2 = false;
}

static inline byte_t a_used FETCH(Z80State *state, Z80Arg reg) {
  switch (reg) {
    case Z80Arg::ArgRegA:
      return state->AF.b.h;
    case Z80Arg::ArgRegB:
      return state->BC.b.h;
    case Z80Arg::ArgRegC:
      return state->BC.b.l;
    case Z80Arg::ArgRegD:
      return state->DE.b.h;
    case Z80Arg::ArgRegE:
      return state->DE.b.l;
    case Z80Arg::ArgRegH:
      return state->HL.b.h;
    case Z80Arg::ArgRegL:
      return state->HL.b.l;
    case Z80Arg::ArgRegHL:
      return I8(state, DADDR(state));
  }
}

OP(STORE, Z80Arg reg, addr_t addr, byte_t value) {
  switch (reg) {
    case Z80Arg::ArgRegA:
      state->AF.b.h = value;
      break;
    case Z80Arg::ArgRegB:
      state->BC.b.h = value;
      break;
    case Z80Arg::ArgRegC:
      state->BC.b.l = value;
      break;
    case Z80Arg::ArgRegD:
      state->DE.b.h = value;
      break;
    case Z80Arg::ArgRegE:
      state->DE.b.l = value;
      break;
    case Z80Arg::ArgRegH:
      state->HL.b.h = value;
      break;
    case Z80Arg::ArgRegL:
      state->HL.b.l = value;
      break;
    case Z80Arg::ArgRegHL:
      bus_write(state, addr, value);
      break;
  }
}

OP(DISPATCH_CB) {
  byte_t op;
  byte_t value;
  addr_t addr;
  switch (state->prefix) {
    case Z80Prefix::DDPrefix:
      addr = DIX(state);
      op = pc_read(state);
      value = I8(state, addr);
      add_icycles(state, 8);
      break;
    case Z80Prefix::FDPrefix:
      addr = DIY(state);
      op = pc_read(state);
      value = I8(state, addr);
      add_icycles(state, 8);
      break;
    default:
      abort();
  }
  int bit = (op & 0x38) >> 3;
  Z80Arg reg = Z80Arg(op & 0x07);
  byte_t dest = 0;
  if ((op & 0xC0) == 0x00) {
    if ((op & 0xF8) == 0x00) {
      RLC(state, dest, value);
    } else if ((op & 0xF8) == 0x08) {
      RRC(state, dest, value);
    } else if ((op & 0xF8) == 0x10) {
      RL(state, dest, value);
    } else if ((op & 0xF8) == 0x18) {
      RR(state, dest, value);
    } else if ((op & 0xF8) == 0x20) {
      SLA(state, dest, value);
    } else if ((op & 0xF8) == 0x28) {
      SRA(state, dest, value);
    } else if ((op & 0xF8) == 0x30) {
      SLL(state, dest, value);
    } else if ((op & 0xF8) == 0x38) {
      SRL(state, dest, value);
    }
    (reg == Z80Arg::ArgRegHL) ? add_icycles(state, 15) : add_icycles(state, 8);
    STORE(state, reg, addr, dest);
  } else if ((op & 0xC0) == 0x40) {
    if (reg == Z80Arg::ArgRegHL)
      BIT_TEST_HL(state, value, state->WZ.b.h, bit);
    else
      BIT_TEST(state, value, bit);
    (reg == Z80Arg::ArgRegHL) ? add_icycles(state, 12) : add_icycles(state, 8);
  } else if ((op & 0xC0) == 0x80) {
    BIT_RESET(state, value, bit);
    (reg == Z80Arg::ArgRegHL) ? add_icycles(state, 15) : add_icycles(state, 8);
    STORE(state, reg, addr, value);
  } else if ((op & 0xC0) == 0xC0) {
    BIT_SET(state, value, bit);
    (reg == Z80Arg::ArgRegHL) ? add_icycles(state, 15) : add_icycles(state, 8);
    STORE(state, reg, addr, value);
  }
}

OP(DISPATCH_DD) {
  throw CpuFault("z80", "Invalid 0xDD");
}

OP(DISPATCH_ED) {
  throw CpuFault("z80", "Invalid 0xED");
}

OP(DISPATCH_FD) {
  throw CpuFault("z80", "Invalid 0xFD");
}

OP(DJNZ, byte_t arg) {
  if (--state->BC.b.h != 0) {
    state->PC.d += (char)arg;
    state->WZ = state->PC;
    add_icycles(state, 5);
  }
}

OP(EI) {
  state->iff1 = state->iff2 = true;
  state->iwait = true;
}

OP(EX, uint16_t &lhs, uint16_t &rhs) {
  lhs ^= rhs;
  rhs ^= lhs;
  lhs ^= rhs;
}

OP(EXI, addr_t addr, byte_t &rh, byte_t &rl) {
  byte_t ll = bus_read(state, addr);
  byte_t lh = bus_read(state, addr + 1);
  bus_write(state, addr, rl);
  bus_write(state, addr + 1, rh);
  rh = lh;
  rl = ll;
}

OP(EXX) {
  EX(state, state->BC.d, state->BC2.d);
  EX(state, state->DE.d, state->DE2.d);
  EX(state, state->HL.d, state->HL2.d);
}

OP(HALT) {
  state->halt = true;
}

OP(IM, byte_t arg) {
  state->imode = arg;
}

OP(Z80_IN, byte_t &orig, byte_t port) {
  orig = io_read(state, port);
  state->yield = 1;
}

OP(INC, reg8_t &dest) {
  uint16_t result = dest + 1;

  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.H = bit_isset(dest ^ 1 ^ result, 4);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = (dest == 0x7F);
  state->AF.b.f.N = 0;

  dest = static_cast<reg8_t>(result);
}

OP(INCI, addr_t addr) {
  byte_t dest = bus_read(state, addr);

  INC(state, dest);

  bus_write(state, addr, dest);
}

OP(INC16, reg16_t &dest) {
  uint16_t result = dest.d + 1;

  dest = (reg16_t)result;
}

OP(JR, bool jump, byte_t arg) {
  if (jump) {
    state->PC.d += (char)arg;
    state->WZ = state->PC;
    add_icycles(state, 4);
  }
}

OP(JP, bool jump, uint16_t arg) {
  state->WZ.d = arg;
  if (jump) {
    state->PC.d = arg;
    add_icycles(state, 4);
  }
}

OP(LD, byte_t &dest, byte_t arg) {
  byte_t result = arg;

  dest = result;
}

OP(LD16, reg16_t &wdest, uint16_t arg) {
  wdest.d = arg;
}

OP(LD16I, addr_t addr, uint16_t arg) {
  bus_write(state, addr, arg & 0xff);
  bus_write(state, addr + 1, arg >> 8);
}

OP(LDI) {
  byte_t value = bus_read(state, state->HL.d++);
  bus_write(state, state->DE.d++, value);
  value += state->AF.b.h;
  state->BC.d--;
  state->AF.b.f.Y = bit_isset(value, 1);
  state->AF.b.f.X = bit_isset(value, 3);
  state->AF.b.f.H = 0;
  state->AF.b.f.N = 0;
  state->AF.b.f.V = (state->BC.d != 0);
}

OP(LDIR) {
  LDI(state);
  if (state->BC.d != 0) {
    state->WZ.d = state->PC.d + 1;
    state->PC.d -= 2;
    add_icycles(state, 5);
  }
}

OP(LDD) {
  byte_t value = bus_read(state, state->HL.d--);
  bus_write(state, state->DE.d--, value);
  value += state->AF.b.h;
  state->BC.d--;
  state->AF.b.f.Y = bit_isset(value, 1);
  state->AF.b.f.X = bit_isset(value, 3);
  state->AF.b.f.H = 0;
  state->AF.b.f.N = 0;
  state->AF.b.f.V = (state->BC.d != 0);
}

OP(LDDR) {
  LDD(state);
  if (state->BC.d != 0) {
    state->WZ.d = state->PC.d + 1;
    state->PC.d -= 2;
    add_icycles(state, 5);
  }
}

OP(LDMEM, addr_t addr, byte_t arg) {
  bus_write(state, addr, arg);
}

OP(NEG, byte_t &dest) {
  byte_t arg = dest;
  dest = 0;

  SUB(state, dest, arg);
}

OP(OR, reg8_t &dest, reg8_t arg) {
  byte_t result = dest | arg;

  state->AF.b.l = 0;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = CALC_PARITY(result);

  dest = result;
}

OP(Z80_OUT, byte_t port, byte_t value) {
  io_write(state, port, value);
  state->yield = 1;
}

OP(OTIR) {
  /* XXX: NOP */
}

OP(PUSH, byte_t high, byte_t low) {
  bus_write(state, --state->SP.d, high);
  bus_write(state, --state->SP.d, low);
}

OP(POP, byte_t &high, byte_t &low) {
  low = bus_read(state, state->SP.d++);
  high = bus_read(state, state->SP.d++);
}

OP(RET, bool jump) {
  if (jump) {
    POP(state, state->PC.b.h, state->PC.b.l);
    state->WZ = state->PC;
    add_icycles(state, 16);
  }
}

OP(RETI) {
  POP(state, state->PC.b.h, state->PC.b.l);
  state->WZ = state->PC;
  state->iff1 = state->iff2 = true;
}

OP(RETN) {
  POP(state, state->PC.b.h, state->PC.b.l);
  state->iff1 = state->iff2;
}

OP(RL, byte_t &dest, byte_t value) {
  byte_t result = (value << 1) | state->AF.b.f.C;

  state->AF.b.l = 0;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result == 0);
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = CALC_PARITY(result);
  state->AF.b.f.C = bit_isset(value, 7);

  dest = result;
}

OP(RL_HL) {
  byte_t value = bus_read(state, state->HL.d);
  RL(state, value, value);

  bus_write(state, state->HL.d, value);
}

OP(RLA) {
  bool S = state->AF.b.f.S;
  bool Z = state->AF.b.f.Z;
  bool V = state->AF.b.f.V;
  RL(state, state->AF.b.h, state->AF.b.h);
  state->AF.b.f.S = S;
  state->AF.b.f.Z = Z;
  state->AF.b.f.V = V;
}

OP(RLC, byte_t &dest, byte_t value) {
  byte_t result = (value << 1) | ((value & 0x80) >> 7);

  state->AF.b.l = 0;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result == 0);
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = CALC_PARITY(result);
  state->AF.b.f.C = (value & 0x80) != 0;

  dest = result;
}

OP(RLC_HL) {
  byte_t value = bus_read(state, state->HL.d);

  RLC(state, value, value);

  bus_write(state, state->HL.d, value);
}

OP(RLCA) {
  bool S = state->AF.b.f.S;
  bool Z = state->AF.b.f.Z;
  bool V = state->AF.b.f.V;
  RLC(state, state->AF.b.h, state->AF.b.h);
  state->AF.b.f.S = S;
  state->AF.b.f.Z = Z;
  state->AF.b.f.V = V;
}

OP(RLD) {
  byte_t value = bus_read(state, state->HL.d);

  bus_write(state, state->HL.d,
                   (state->AF.b.h & 0x0F) | ((value & 0x0F) << 4));
  state->AF.b.h = (state->AF.b.h & 0xF0) | ((value & 0xF0) >> 4);
  state->WZ.d = state->HL.d + 1;

  state->AF.b.l = state->AF.b.f.C;
  state->AF.b.f.S = bit_isset(state->AF.b.h, 7);
  state->AF.b.f.Z = state->AF.b.h == 0;
  state->AF.b.f.Y = bit_isset(state->AF.b.h, 5);
  state->AF.b.f.H = 0;
  state->AF.b.f.X = bit_isset(state->AF.b.h, 3);
  state->AF.b.f.V = CALC_PARITY(state->AF.b.h);
  state->AF.b.f.N = 0;
}

OP(RR, byte_t &dest, byte_t value) {
  byte_t result = (value >> 1) | (state->AF.b.f.C ? 0x80 : 0x00);

  state->AF.b.l = value & 0x01;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result == 0);
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = CALC_PARITY(result);

  dest = result;
}

OP(RR_HL) {
  byte_t value = bus_read(state, state->HL.d);

  RR(state, value, value);
  bus_write(state, state->HL.d, value);
}

OP(RRA) {
  bool S = state->AF.b.f.S;
  bool Z = state->AF.b.f.Z;
  bool V = state->AF.b.f.V;
  RR(state, state->AF.b.h, state->AF.b.h);
  state->AF.b.f.S = S;
  state->AF.b.f.Z = Z;
  state->AF.b.f.V = V;
}

OP(RRC, byte_t &dest, byte_t value) {
  byte_t result = (value >> 1) | (value & 0x01 ? 0x80 : 0x00);

  state->AF.b.l = 0;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result == 0);
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = CALC_PARITY(result);
  state->AF.b.f.C = (value & 0x01) != 0;

  dest = result;
}

OP(RRC_HL) {
  byte_t value = bus_read(state, state->HL.d);
  RRC(state, value, value);

  bus_write(state, state->HL.d, value);
}

OP(RRCA) {
  bool S = state->AF.b.f.S;
  bool Z = state->AF.b.f.Z;
  bool V = state->AF.b.f.V;
  RRC(state, state->AF.b.h, state->AF.b.h);
  state->AF.b.f.S = S;
  state->AF.b.f.Z = Z;
  state->AF.b.f.V = V;
}

OP(RRD) {
  byte_t value = bus_read(state, state->HL.d);

  bus_write(state, state->HL.d,
                   (state->AF.b.h & 0x0F) << 4 | ((value & 0xF0) >> 4));
  state->AF.b.h = (state->AF.b.h & 0xF0) | (value & 0x0F);
  state->WZ.d = state->HL.d + 1;

  state->AF.b.l = state->AF.b.f.C;
  state->AF.b.f.S = bit_isset(state->AF.b.h, 7);
  state->AF.b.f.Z = state->AF.b.h == 0;
  state->AF.b.f.Y = bit_isset(state->AF.b.h, 5);
  state->AF.b.f.X = bit_isset(state->AF.b.h, 3);
  state->AF.b.f.V = CALC_PARITY(state->AF.b.h);
}

OP(RST, byte_t arg) {
  PUSH(state, state->PC.b.h, state->PC.b.l);
  state->PC.d = arg;
  state->WZ = state->PC;
}

OP(SLA, byte_t &dest, byte_t value) {
  byte_t result = (value << 1);

  state->AF.b.l = 0;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result == 0);
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = CALC_PARITY(result);
  state->AF.b.f.C = bit_isset(value, 7);

  dest = result;
}

OP(SLA_HL) {
  byte_t value = bus_read(state, state->HL.d);

  SLA(state, value, value);

  bus_write(state, state->HL.d, value);
}

OP(SRA, byte_t &dest, byte_t value) {
  byte_t result = (value >> 1) | (value & 0x80);

  state->AF.b.l = 0;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result == 0);
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = CALC_PARITY(result);
  state->AF.b.f.C = bit_isset(value, 0);

  dest = result;
}

OP(SRA_HL) {
  byte_t value = bus_read(state, state->HL.d);

  SRA(state, value, value);

  bus_write(state, state->HL.d, value);
}

OP(SRL, byte_t &dest, byte_t value) {
  byte_t result = (value >> 1);

  state->AF.b.l = 0;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result == 0);
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = CALC_PARITY(result);
  state->AF.b.f.C = bit_isset(value, 0);

  dest = result;
}

OP(SRL_HL) {
  byte_t value = bus_read(state, state->HL.d);

  SRL(state, value, value);

  bus_write(state, state->HL.d, value);
}

OP(SLL, byte_t &dest, byte_t value) {
  byte_t result = (value << 1) | 0x01;

  state->AF.b.l = 0;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result == 0);
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = CALC_PARITY(result);
  state->AF.b.f.C = bit_isset(value, 7);

  dest = result;
}

OP(SLL_HL) {
  byte_t value = bus_read(state, state->HL.d);

  SLL(state, value, value);

  bus_write(state, state->HL.d, value);
}


OP(SBC, byte_t &dest, byte_t arg) {
  uint16_t result = dest - arg - state->AF.b.f.C;

  state->AF.b.l = 0x02;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.H = bit_isset(dest ^ arg ^ result, 4);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = CALC_OVERFLOW(dest, arg, result);
  state->AF.b.f.C = bit_isset(dest ^ arg ^ result, 8);

  dest = (byte_t)result;
}

OP(SBC16, uint16_t &wdest, uint16_t arg) {
  uint32_t result = wdest - arg - state->AF.b.f.C;

  state->AF.b.l = 0x02;
  state->AF.b.f.S = bit_isset(result, 15);
  state->AF.b.f.Z = (result & 0xffff) == 0;
  state->AF.b.f.Y = bit_isset(result, 13);
  state->AF.b.f.H = bit_isset(wdest ^ arg ^ result, 12);
  state->AF.b.f.X = bit_isset(result, 11);
  state->AF.b.f.V = bit_isset((wdest ^ arg) & (wdest ^ result), 15);
  state->AF.b.f.C = bit_isset(result, 16);

  wdest = result;
}

OP(SCF) {

  state->AF.b.f.Y = bit_isset(state->AF.b.h, 5);
  state->AF.b.f.H = 0;
  state->AF.b.f.X = bit_isset(state->AF.b.h, 3);
  state->AF.b.f.N = 0;
  state->AF.b.f.C = 1;
}

OP(SUB, byte_t &dest, byte_t arg) {
  uint16_t result = dest - arg;

  state->AF.b.l = 0x02;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.H = bit_isset(dest ^ arg ^ result, 4);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = CALC_OVERFLOW(dest, arg, result);
  state->AF.b.f.C = bit_isset(dest ^ arg ^ result, 8);

  dest = (byte_t)result;
}

OP(XOR, reg8_t &dest, byte_t arg) {
  uint8_t result = dest ^ arg;

  state->AF.b.l = 0;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = CALC_PARITY(result);

  dest = result;
}
};
