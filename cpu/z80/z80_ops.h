#pragma once

#include "cpu/z80/z80.h"

#define OP(op, ...) void op(Z80State *state, ##__VA_ARGS__)

namespace Z80 {

static inline int __attribute__((__used__)) CALC_PARITY(reg8_t reg);
static inline void __attribute__((__used__))
PUSH(Z80State *state, byte_t high, byte_t low);
static inline void __attribute__((__used__))
POP(Z80State *state, byte_t &high, byte_t &low);
static inline void __attribute__((__used__))
RL(Z80State *state, byte_t &dest, byte_t value);
static inline void __attribute__((__used__))
RLC(Z80State *state, byte_t &dest, byte_t value);
static inline void __attribute__((__used__))
RR(Z80State *state, byte_t &dest, byte_t value);
static inline void __attribute__((__used__))
RRC(Z80State *state, byte_t &dest, byte_t value);
static inline void __attribute__((__used__))
SLA(Z80State *state, byte_t &dest, byte_t value);
static inline void __attribute__((__used__))
SLL(Z80State *state, byte_t &dest, byte_t value);
static inline void __attribute__((__used__))
SRL(Z80State *state, byte_t &dest, byte_t value);
static inline void __attribute__((__used__))
SRA(Z80State *state, byte_t &dest, byte_t value);
static inline void __attribute__((__used__))
SUB(Z80State *state, byte_t &dest, byte_t arg);

static inline void __attribute__((__used__))
ADC(Z80State *state, byte_t &dest, byte_t arg) {
  uint16_t result = dest + arg + state->AF.b.f.C;

  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.H = bit_isset(dest ^ arg ^ result, 4);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = bit_isset((dest ^ arg ^ 0x80) & (arg ^ result), 7);
  state->AF.b.f.N = 0;
  state->AF.b.f.C = bit_isset(result, 8);

  dest = (byte_t)result;
}

static inline void __attribute__((__used__))
ADC16(Z80State *state, uint16_t &wdest, uint16_t arg) {
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

static inline void __attribute__((__used__))
ADD(Z80State *state, byte_t &dest, byte_t arg) {
  uint16_t result = dest + arg;

  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.H = bit_isset(dest ^ arg ^ result, 4);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = bit_isset(dest ^ arg ^ result, 7);
  state->AF.b.f.N = 0;
  state->AF.b.f.C = bit_isset(result, 8);

  dest = result;
}

static inline void __attribute__((__used__))
ADD16(Z80State *state, reg16_t &dest, uint16_t arg) {
  uint32_t result = dest.d + arg;

  state->AF.b.f.Y = bit_isset(result, 13);
  state->AF.b.f.H = bit_isset(dest.d ^ arg ^ result, 12);
  state->AF.b.f.X = bit_isset(result, 11);
  state->AF.b.f.C = (result > 0xffff) ? 1 : 0;
  state->AF.b.f.N = 0;

  dest = static_cast<reg16_t>(result);
}

static inline void __attribute__((__used__))
AND(Z80State *state, reg8_t &dest, reg8_t arg) {
  byte_t result = dest & arg;

  state->AF.b.l = (result & 0xA4) | 0x10;
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.V = CALC_PARITY(result);

  dest = result;
}

static inline void __attribute__((__used__))
BIT_TEST(Z80State *state, byte_t value, int bit) {

  state->AF.b.l = (value & (0x24 | 1 << bit));
  state->AF.b.f.Z = !bit_isset(value, bit);
  state->AF.b.f.H = 1;
  state->AF.b.f.V = !bit_isset(value, bit);
}

static inline void __attribute__((__used__))
BIT_RESET(Z80State *state, byte_t &dest, byte_t value, int bit) {
  byte_t result = value & ~(1 << bit);

  dest = result;
}

static inline void __attribute__((__used__))
BIT_SET(Z80State *state, byte_t &dest, byte_t value, int bit) {
  byte_t result = value | (1 << bit);

  dest = result;
}

static inline int __attribute__((__used__)) CALC_PARITY(reg8_t reg) {
  return (0x6996 >> (reg >> 4) ^ (reg & 0xf)) & 1;
}

static inline void __attribute__((__used__))
CALL(Z80State *state, bool jump, uint16_t addr) {
  if (jump) {
    PUSH(state, state->PC.b.h, state->PC.b.l);

    state->PC.d = addr;
    ADD_ICYCLES(state, 12);
  }
}

static inline void __attribute__((__used__))
CP(Z80State *state, byte_t dest, byte_t arg) {
  uint16_t result = dest - arg;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(arg, 5);
  state->AF.b.f.H = bit_isset(arg ^ dest ^ result, 4);
  state->AF.b.f.X = bit_isset(arg, 3);
  state->AF.b.f.V = bit_isset((result ^ dest) & (dest ^ arg), 7);
  state->AF.b.f.N = true;
  state->AF.b.f.C = bit_isset(dest ^ arg ^ result, 8);
}

static inline void __attribute__((__used__)) CPD(Z80State *state) {
  byte_t dest = state->AF.b.h;
  byte_t arg = state->bus_read(state, state->HL.d);
  uint16_t result = dest - arg;
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

static inline void __attribute__((__used__)) CPDR(Z80State *state) {
  CPD(state);
  if (state->BC.d != 0 && !state->AF.b.f.Z) {
    state->PC.d -= 2;
    ADD_ICYCLES(state, 5);
  }
}

static inline void __attribute__((__used__)) CPI(Z80State *state) {
  byte_t dest = state->AF.b.h;
  byte_t arg = state->bus_read(state, state->HL.d);
  uint16_t result = dest - arg;
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

void CPIR(Z80State *state) {
  CPI(state);
  if (state->BC.d != 0 && !state->AF.b.f.Z) {
    state->PC.d -= 2;
    ADD_ICYCLES(state, 5);
  }
}

static inline void __attribute__((__used__)) CPL(Z80State *state) {
  byte_t &dest = state->AF.b.h;
  byte_t result = ~dest;

  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.H = 1;
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.N = 1;

  dest = result;
}

static inline void __attribute__((__used__)) CCF(Z80State *state) {
  state->AF.b.f.Y = bit_isset(state->AF.b.h, 5);
  state->AF.b.f.H = state->AF.b.f.C;
  state->AF.b.f.X = bit_isset(state->AF.b.h, 3);
  state->AF.b.f.N = 0;
  state->AF.b.f.C = !state->AF.b.f.C;
}

static inline void __attribute__((__used__)) DAA(Z80State *state) {
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

static inline void DEC(Z80State *state, reg8_t &dest) {
  uint16_t result = dest - 1;

  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.H = bit_isset(dest ^ 1 ^ result, 4);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = (dest == 0x80);
  state->AF.b.f.N = true;

  dest = (byte_t)result;
}

static inline void __attribute__((__used__))
DECI(Z80State *state, addr_t addr) {
  byte_t dest = state->bus_read(state, addr);

  DEC(state, dest);

  state->bus_write(state, addr, dest);
}

static inline void DEC16(Z80State *state, reg16_t &dest) {
  uint16_t result = dest.d - 1;

  dest = (reg16_t)result;
}

static inline void __attribute__((__used__)) DI(Z80State *state) {
  state->iff1 = state->iff2 = false;
}

static inline byte_t __attribute__((__used__))
FETCH(Z80State *state, Z80Arg reg) {
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

static inline void __attribute__((__used__))
STORE(Z80State *state, Z80Arg reg, addr_t addr, byte_t value) {
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
      state->bus_write(state, addr, value);
      break;
  }
}

static inline void __attribute__((__used__)) DISPATCH_CB(Z80State *state) {
  byte_t op;
  byte_t value;
  addr_t addr;
  switch (state->prefix) {
    case Z80Prefix::DDPrefix:
      addr = DIX(state);
      op = pc_read(state);
      value = I8(state, addr);
      ADD_ICYCLES(state, 8);
      break;
    case Z80Prefix::FDPrefix:
      addr = DIY(state);
      op = pc_read(state);
      value = I8(state, addr);
      ADD_ICYCLES(state, 8);
      break;
    default:
      addr = state->HL.d;
      op = pc_read(state);
      value = FETCH(state, Z80Arg(op & 0x07));
      break;
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
    (reg == Z80Arg::ArgRegHL) ? ADD_ICYCLES(state, 15) : ADD_ICYCLES(state, 8);
    STORE(state, reg, addr, dest);
  } else if ((op & 0xC0) == 0x40) {
    BIT_TEST(state, value, bit);
    (reg == Z80Arg::ArgRegHL) ? ADD_ICYCLES(state, 12) : ADD_ICYCLES(state, 8);
  } else if ((op & 0xC0) == 0x80) {
    BIT_RESET(state, dest, value, bit);
    (reg == Z80Arg::ArgRegHL) ? ADD_ICYCLES(state, 15) : ADD_ICYCLES(state, 8);
    STORE(state, reg, addr, dest);
  } else if ((op & 0xC0) == 0xC0) {
    BIT_SET(state, dest, value, bit);
    (reg == Z80Arg::ArgRegHL) ? ADD_ICYCLES(state, 15) : ADD_ICYCLES(state, 8);
    STORE(state, reg, addr, dest);
  }
}

static inline void __attribute__((__used__)) DISPATCH_DD(Z80State *state) {
  throw CpuFault("z80", "Invalid 0xDD");
}

static inline void __attribute__((__used__)) DISPATCH_ED(Z80State *state) {
  throw CpuFault("z80", "Invalid 0xED");
}

static inline void __attribute__((__used__)) DISPATCH_FD(Z80State *state) {
  throw CpuFault("z80", "Invalid 0xFD");
}

static inline void __attribute__((__used__)) DJNZ(Z80State *state, byte_t arg) {
  if (--state->BC.b.h != 0) {
    state->PC.d += (char)arg;
    ADD_ICYCLES(state, 5);
  }
}

static inline void __attribute__((__used__)) EI(Z80State *state) {
  state->iff1 = state->iff2 = true;
  state->iwait = true;
}

static inline void __attribute__((__used__))
EX(Z80State *state, uint16_t &lhs, uint16_t &rhs) {
  lhs ^= rhs;
  rhs ^= lhs;
  lhs ^= rhs;
}

static inline void __attribute__((__used__))
EXI(Z80State *state, addr_t addr, byte_t &rh, byte_t &rl) {
  byte_t ll = state->bus_read(state, addr);
  byte_t lh = state->bus_read(state, addr + 1);
  state->bus_write(state, addr, rl);
  state->bus_write(state, addr + 1, rh);
  rh = lh;
  rl = ll;
}

static inline void __attribute__((__used__)) EXX(Z80State *state) {
  EX(state, state->BC.d, state->BC2.d);
  EX(state, state->DE.d, state->DE2.d);
  EX(state, state->HL.d, state->HL2.d);
}

static inline void __attribute__((__used__)) HALT(Z80State *state) {
  state->halt = true;
}

static inline void __attribute__((__used__)) IM(Z80State *state, byte_t arg) {
  state->imode = arg;
}

static inline void __attribute__((__used__))
IN(Z80State *state, byte_t &orig, byte_t port) {
  orig = state->io_read(state, port);
  state->yield = 1;
}

static inline void __attribute__((__used__))
INC(Z80State *state, reg8_t &dest) {
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

static inline void __attribute__((__used__))
INCI(Z80State *state, addr_t addr) {
  byte_t dest = state->bus_read(state, addr);

  INC(state, dest);

  state->bus_write(state, addr, dest);
}

static inline void INC16(Z80State *state, reg16_t &dest) {
  uint16_t result = dest.d + 1;

  dest = (reg16_t)result;
}

static inline void __attribute__((__used__))
JR(Z80State *state, bool jump, byte_t arg) {
  if (jump) {
    state->PC.d += (char)arg;
    ADD_ICYCLES(state, 4);
  }
}

static inline void __attribute__((__used__))
JP(Z80State *state, bool jump, uint16_t arg) {
  if (jump) {
    state->PC.d = arg;
    ADD_ICYCLES(state, 4);
  }
}

static inline void __attribute__((__used__))
LD(Z80State *state, byte_t &dest, byte_t arg) {
  byte_t result = arg;

  dest = result;
}

static inline void __attribute__((__used__))
LD16(Z80State *state, reg16_t &wdest, uint16_t arg) {
  wdest.d = arg;
}

static inline void __attribute__((__used__))
LD16I(Z80State *state, addr_t addr, uint16_t arg) {
  state->bus_write(state, addr, arg & 0xff);
  state->bus_write(state, addr + 1, arg >> 8);
}

static inline void __attribute__((__used__)) LDI(Z80State *state) {
  byte_t value = state->bus_read(state, state->HL.d++);
  state->bus_write(state, state->DE.d++, value);
  value += state->AF.b.h;
  state->BC.d--;
  state->AF.b.f.Y = bit_isset(value, 1);
  state->AF.b.f.X = bit_isset(value, 3);
  state->AF.b.f.H = 0;
  state->AF.b.f.N = 0;
  state->AF.b.f.V = (state->BC.d != 0);
}

static inline void __attribute__((__used__)) LDIR(Z80State *state) {
  LDI(state);
  if (state->BC.d != 0) {
    state->PC.d -= 2;
    ADD_ICYCLES(state, 5);
  }
}

static inline void __attribute__((__used__)) LDD(Z80State *state) {
  byte_t value = state->bus_read(state, state->HL.d--);
  state->bus_write(state, state->DE.d--, value);
  value += state->AF.b.h;
  state->BC.d--;
  state->AF.b.f.Y = bit_isset(value, 1);
  state->AF.b.f.X = bit_isset(value, 3);
  state->AF.b.f.H = 0;
  state->AF.b.f.N = 0;
  state->AF.b.f.V = (state->BC.d != 0);
}

static inline void __attribute__((__used__)) LDDR(Z80State *state) {
  LDD(state);
  if (state->BC.d != 0) {
    state->PC.d -= 2;
    ADD_ICYCLES(state, 5);
  }
}

static inline void __attribute__((__used__))
LDMEM(Z80State *state, addr_t addr, byte_t arg) {
  state->bus_write(state, addr, arg);
}

static inline void __attribute__((__used__))
NEG(Z80State *state, byte_t &dest) {
  byte_t arg = dest;
  dest = 0;

  SUB(state, dest, arg);
}

static inline void __attribute__((__used__))
OR(Z80State *state, reg8_t &dest, reg8_t arg) {
  byte_t result = dest | arg;

  state->AF.b.l = 0;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = CALC_PARITY(result);

  dest = result;
}

static inline void __attribute__((__used__))
OUT(Z80State *state, byte_t port, byte_t value) {
  state->io_write(state, port, value);
  state->yield = 1;
}

static inline void __attribute__((__used__)) OTIR(Z80State *state) {
  /* XXX: NOP */
}

static inline void __attribute__((__used__))
PUSH(Z80State *state, byte_t high, byte_t low) {
  state->bus_write(state, --state->SP.d, high);
  state->bus_write(state, --state->SP.d, low);
}

static inline void __attribute__((__used__))
POP(Z80State *state, byte_t &high, byte_t &low) {
  low = state->bus_read(state, state->SP.d++);
  high = state->bus_read(state, state->SP.d++);
}

static inline void __attribute__((__used__)) RET(Z80State *state, bool jump) {
  if (jump) {
    POP(state, state->PC.b.h, state->PC.b.l);
    ADD_ICYCLES(state, 16);
  }
}

static inline void __attribute__((__used__)) RETI(Z80State *state) {
  POP(state, state->PC.b.h, state->PC.b.l);
  state->iff1 = state->iff2 = true;
}

static inline void __attribute__((__used__)) RETN(Z80State *state) {
  POP(state, state->PC.b.h, state->PC.b.l);
  state->iff1 = state->iff2;
}

static inline void __attribute__((__used__))
RL(Z80State *state, byte_t &dest, byte_t value) {
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

static inline void __attribute__((__used__)) RLA(Z80State *state) {
  bool S = state->AF.b.f.S;
  bool Z = state->AF.b.f.Z;
  bool V = state->AF.b.f.V;
  RL(state, state->AF.b.h, state->AF.b.h);
  state->AF.b.f.S = S;
  state->AF.b.f.Z = Z;
  state->AF.b.f.V = V;
}

static inline void __attribute__((__used__))
RLC(Z80State *state, byte_t &dest, byte_t value) {
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

static inline void __attribute__((__used__)) RLCA(Z80State *state) {
  bool S = state->AF.b.f.S;
  bool Z = state->AF.b.f.Z;
  bool V = state->AF.b.f.V;
  RLC(state, state->AF.b.h, state->AF.b.h);
  state->AF.b.f.S = S;
  state->AF.b.f.Z = Z;
  state->AF.b.f.V = V;
}

static inline void __attribute__((__used__)) RLD(Z80State *state) {
  byte_t value = state->bus_read(state, state->HL.d);

  state->bus_write(state, state->HL.d,
                   (state->AF.b.h & 0x0F) | ((value & 0x0F) << 4));
  state->AF.b.h = (state->AF.b.h & 0xF0) | ((value & 0xF0) >> 4);

  state->AF.b.l = state->AF.b.f.C;
  state->AF.b.f.S = bit_isset(state->AF.b.h, 7);
  state->AF.b.f.Z = state->AF.b.h == 0;
  state->AF.b.f.Y = bit_isset(state->AF.b.h, 5);
  state->AF.b.f.H = 0;
  state->AF.b.f.X = bit_isset(state->AF.b.h, 3);
  state->AF.b.f.V = CALC_PARITY(state->AF.b.h);
  state->AF.b.f.N = 0;
}

static inline void __attribute__((__used__))
RR(Z80State *state, byte_t &dest, byte_t value) {
  byte_t result = (value >> 1) | (state->AF.b.f.C ? 0x80 : 0x00);

  state->AF.b.l = value & 0x01;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result == 0);
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = CALC_PARITY(result);

  dest = result;
}

static inline void __attribute__((__used__)) RRA(Z80State *state) {
  bool S = state->AF.b.f.S;
  bool Z = state->AF.b.f.Z;
  bool V = state->AF.b.f.V;
  RR(state, state->AF.b.h, state->AF.b.h);
  state->AF.b.f.S = S;
  state->AF.b.f.Z = Z;
  state->AF.b.f.V = V;
}

static inline void __attribute__((__used__))
RRC(Z80State *state, byte_t &dest, byte_t value) {
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

static inline void __attribute__((__used__)) RRCA(Z80State *state) {
  bool S = state->AF.b.f.S;
  bool Z = state->AF.b.f.Z;
  bool V = state->AF.b.f.V;
  RRC(state, state->AF.b.h, state->AF.b.h);
  state->AF.b.f.S = S;
  state->AF.b.f.Z = Z;
  state->AF.b.f.V = V;
}

static inline void __attribute__((__used__)) RRD(Z80State *state) {
  byte_t value = state->bus_read(state, state->HL.d);

  state->bus_write(state, state->HL.d,
                   (state->AF.b.h & 0x0F) << 4 | ((value & 0xF0) >> 4));
  state->AF.b.h = (state->AF.b.h & 0xF0) | (value & 0x0F);

  state->AF.b.l = state->AF.b.f.C;
  state->AF.b.f.S = bit_isset(state->AF.b.h, 7);
  state->AF.b.f.Z = state->AF.b.h == 0;
  state->AF.b.f.Y = bit_isset(state->AF.b.h, 5);
  state->AF.b.f.X = bit_isset(state->AF.b.h, 3);
  state->AF.b.f.V = CALC_PARITY(state->AF.b.h);
}

static inline void __attribute__((__used__)) RST(Z80State *state, byte_t arg) {
  PUSH(state, state->PC.b.h, state->PC.b.l);
  state->PC.d = arg;
}

static inline void __attribute__((__used__))
SLA(Z80State *state, byte_t &dest, byte_t value) {
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

static inline void __attribute__((__used__))
SRA(Z80State *state, byte_t &dest, byte_t value) {
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

static inline void __attribute__((__used__))
SRL(Z80State *state, byte_t &dest, byte_t value) {
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

static inline void __attribute__((__used__))
SLL(Z80State *state, byte_t &dest, byte_t value) {
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

static inline void __attribute__((__used__))
SBC(Z80State *state, byte_t &dest, byte_t arg) {
  uint16_t result = dest - arg - state->AF.b.f.C;

  state->AF.b.l = 0x02;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.H = bit_isset(dest ^ arg ^ result, 4);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = bit_isset((result ^ dest) & (dest ^ arg), 7);
  state->AF.b.f.C = bit_isset(dest ^ arg ^ result, 8);

  dest = result;
}

static inline void __attribute__((__used__))
SBC16(Z80State *state, uint16_t &wdest, uint16_t arg) {
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

static inline void __attribute__((__used__)) SCF(Z80State *state) {

  state->AF.b.l &= 0xC4;
  state->AF.b.f.Y = bit_isset(state->AF.b.h, 5);
  state->AF.b.f.X = bit_isset(state->AF.b.h, 3);
  state->AF.b.f.C = 1;
}

static inline void __attribute__((__used__))
SUB(Z80State *state, byte_t &dest, byte_t arg) {
  uint16_t result = dest - arg;

  state->AF.b.l = 0x2;
  state->AF.b.f.S = bit_isset(result, 7);
  state->AF.b.f.Z = (result & 0xff) == 0;
  state->AF.b.f.Y = bit_isset(result, 5);
  state->AF.b.f.H = bit_isset(dest ^ arg ^ result, 4);
  state->AF.b.f.X = bit_isset(result, 3);
  state->AF.b.f.V = bit_isset((result ^ dest) & (dest ^ arg), 7);
  state->AF.b.f.C = bit_isset(dest ^ arg ^ result, 8);

  dest = (byte_t)result;
}

static inline void __attribute__((__used__))
XOR(Z80State *state, reg8_t &dest, byte_t arg) {
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
