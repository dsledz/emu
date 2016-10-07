/*
 * Copyright (c) 2016, Dan Sledz
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

namespace LR35902 {

struct LR35902State;

enum LR35902Arg {
  ArgRegB = 0,
  ArgRegC = 1,
  ArgRegD = 2,
  ArgRegE = 3,
  ArgRegH = 4,
  ArgRegL = 5,
  ArgRegHL = 6,
  ArgRegA = 7
};

enum LR35902Prefix {
  NoPrefix = 0x00,
  CBPrefix = 0xCB,
};

typedef AddressBus16x8 LR35902Bus;
typedef std::unique_ptr<LR35902Bus> LR35902Bus_ptr;

typedef CPU2::CpuOpcode<LR35902State> LR35902Opcode;

struct LR35902State {
  LR35902State(void) = default;
  LR35902State(LR35902Bus *bus) : bus(bus) {}

  typedef LR35902Bus bus_type;

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
  reg16_t WZ;

  reg8_t IF;
  reg8_t IE;
  reg8_t I;
  reg8_t R;

  const LR35902Opcode *Op;

  LR35902Prefix prefix;
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
  byte_t data;

  LineState reset_line;

  void reset(void) {
    AF.d = 0x01B0;
    BC.d = 0x0013;
    DE.d = 0x00D8;
    HL.d = 0x014D;
    SP.d = 0xFFFE;
    PC.d = 0x0000;
    IF = 0x00;
    IE = 0x1f;
    iff1 = iff2 = iwait = false;
  }

  Cycles icycles;
  LR35902Bus *bus;
} __attribute__((packed));

static inline reg8_t pc_read(LR35902State *state) {
  reg8_t result = CPU2::bus_read(state, state->PC.d);
  state->PC.d++;
  state->icycles+=1;
  return result;
}

static inline uint16_t D16(LR35902State *state) {
  state->d16.d = pc_read(state) | (pc_read(state) << 8);
  state->WZ = state->d16;
  return state->d16.d;
}

static inline byte_t D8(LR35902State *state) {
  state->d8 = pc_read(state);
  return state->d8;
}

static inline byte_t I8(LR35902State *state, addr_t addr) {
  state->i8 = CPU2::bus_read(state, addr);
  return state->i8;
}

static inline addr_t DADDR(LR35902State *state) {
  state->d16 = state->HL.d;
  return state->d16.d;
}

static inline byte_t IADDR(LR35902State *state) {
  return I8(state, DADDR(state));
}

static inline uint16_t I16(LR35902State *state) {
  state->d16 = pc_read(state) | (pc_read(state) << 8);
  state->WZ.d = state->d16.d + 1;
  state->i16 = (CPU2::bus_read(state, state->d16.d)) |
               (CPU2::bus_read(state, state->d16.d + 1) << 8);
  return state->i16.d;
}

};

