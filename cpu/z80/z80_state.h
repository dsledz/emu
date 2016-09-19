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
#pragma once

#include "core/bits.h"
#include "cpu/lib/cpu2.h"
#include "emu/emu.h"

namespace Z80 {

enum Z80Arg {
  ArgRegB = 0,
  ArgRegC = 1,
  ArgRegD = 2,
  ArgRegE = 3,
  ArgRegH = 4,
  ArgRegL = 5,
  ArgRegHL = 6,
  ArgRegA = 7
};

enum Z80Arg16 { RegBC = 0, RegDE = 1, RegHL = 2, RegIX = 3, RegIY = 4 };

typedef AddressBus16x8 Z80Bus;
typedef std::unique_ptr<Z80Bus> Z80Bus_ptr;
typedef AddressBus8x8 Z80IOBus;
typedef std::unique_ptr<Z80IOBus> Z80IOBus_ptr;

struct Z80State;

enum Z80Prefix {
  NoPrefix = 0x00,
  DDPrefix = 0xDD,
  EDPrefix = 0xED,
  FDPrefix = 0xFD,
};

/* Operation specific state */
struct Z80Op {
  Z80Op(void) : name("NONE") {}

  void reset(void) {
    prefix = Z80Prefix::NoPrefix;
    d8 = i8 = 0;
    pc = opcode = d16 = i16 = yield = 0;
  }

  Z80Prefix prefix;
  uint16_t pc;
  byte_t opcode;
  byte_t d8;
  byte_t i8;
  uint16_t d16;
  uint16_t i16;
  int yield;

  std::string name;
};

struct Z80State;

typedef CPU2::CpuOpcode<Z80State> Z80Opcode;

struct Z80State {
  Z80State(void) = default;
  Z80State(Z80Bus *bus, Z80IOBus *io) : bus(bus), io(io) {}

  union {
    struct {
      union {
        struct {
          byte_t C : 1; /**< carry flag */
          byte_t N : 1; /**< add/substract */
          byte_t V : 1; /**< parity/overflow */
          byte_t X : 1;
          byte_t H : 1; /**< half carry */
          byte_t Y : 1;
          byte_t Z : 1; /**< zero flag */
          byte_t S : 1; /**< sign flag */
        } f;
        reg8_t l;
      };
      reg8_t h;
    } b;
    uint16_t d;
  } AF;
  reg16_t BC;
  reg16_t DE;
  reg16_t HL;
  reg16_t IX;
  reg16_t IY;
  reg16_t SP;
  reg16_t PC;

  reg8_t I;
  reg8_t R;

  reg16_t AF2;
  reg16_t BC2;
  reg16_t DE2;
  reg16_t HL2;

  const Z80Opcode *Op;

  Z80Prefix prefix;
  reg8_t d8;
  reg8_t i8;
  reg16_t d16;
  reg16_t i16;
  reg16_t latch_pc;
  reg8_t latch_op;
  reg16_t *vHL;

  bool halt;
  bool yield;
  bool iff1;
  bool iff2;
  bool iwait;
  int imode;
  byte_t data;

  LineState nmi_line;
  LineState int0_line;
  LineState reset_line;
  LineState wait_line;

  inline void bus_write(Z80State *state, reg16_t addr, reg8_t reg) {
    state->bus->write(addr.d, reg);
  }

  inline reg8_t bus_read(Z80State *state, reg16_t addr) {
    return state->bus->read(addr.d);
  }

  inline void io_write(Z80State *state, reg8_t port, reg8_t reg) {
    state->io->write(port, reg);
  }

  inline reg8_t io_read(Z80State *state, reg8_t port) {
    return state->io->read(port);
  }

  void reset() {
    AF.d = BC.d = DE.d = HL.d = SP.d = IX.d = IY.d = 0;
    AF2.d = BC2.d = DE2.d = HL2.d = 0;
  }

  uint8_t icycles;
  Z80Bus *bus;
  Z80IOBus *io;
} __attribute__((packed));

static inline reg8_t pc_read(Z80State *state) {
  reg8_t result = state->bus->read(state->PC.d);
  state->PC.d++;
  state->icycles++;
  return result;
}

static inline void ADD_ICYCLES(Z80State *state, unsigned cycles) {
  state->icycles += cycles;
}

static inline uint16_t D16(Z80State *state) {
  state->d16.d = pc_read(state) | (pc_read(state) << 8);
  return state->d16.d;
}

static inline byte_t D8(Z80State *state) {
  state->d8 = pc_read(state);
  return state->d8;
}

static inline byte_t I8(Z80State *state, addr_t addr) {
  state->i8 = state->bus_read(state, addr);
  return state->i8;
}

static inline uint16_t DIX(Z80State *state) {
  state->d8 = pc_read(state);
  return state->IX.d + state->d8;
}

static inline uint16_t DIY(Z80State *state) {
  state->d8 = pc_read(state);
  return state->IY.d + state->d8;
}

static inline addr_t DADDR(Z80State *state) {
  switch (state->prefix) {
    case Z80Prefix::DDPrefix:
      return DIX(state);
    case Z80Prefix::FDPrefix:
      return DIY(state);
    default:
      state->d16 = state->HL.d;
      return state->d16.d;
  }
}

static inline byte_t IIX(Z80State *state) { return I8(state, DIX(state)); }

static inline byte_t IIY(Z80State *state) { return I8(state, DIY(state)); }

static inline byte_t IHL(Z80State *state) {
  state->i8 = state->bus_read(state, state->HL.d);
  return state->i8;
}

static inline byte_t IADDR(Z80State *state) { return I8(state, DADDR(state)); }

static inline uint16_t I16(Z80State *state) {
  state->d16 = pc_read(state) | (pc_read(state) << 8);
  state->i16 = (state->bus_read(state, state->d16)) |
               (state->bus_read(state, state->d16.d + 1) << 8);
  return state->i16.d;
}
};
