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

#include "cpu/m6502/m6502.h"
#include "cpu/m6502/m65c02.h"
#include "emu/emu.h"

namespace M6502v2 {

#ifdef WIN32
#pragma pack(push, 1)
#endif
struct HuC6280State {
  HuC6280State(void) { reset(); }

  inline uint8_t bus_read(uint16_t addr) {
    offset_t offset = addr;
    const int bank = (offset & 0xE000) >> 13;
    offset = (offset & 0x1FFF) | ((offset_t)mmu_map[bank] << 13);
    return bus->read(offset);
  }
  inline void bus_write(uint16_t addr, uint8_t value) {
    offset_t offset = addr;
    const int bank = (offset & 0xE000) >> 13;
    offset = (offset & 0x1FFF) | ((offset_t)mmu_map[bank] << 13);
    bus->write(offset, value);
  }
  uint8_t get_flags(uint16_t flags) {
    F.C = bit_isset(flags, Flags::CF);
    F.Z = bit_isset(flags, Flags::ZF);
    F.V = bit_isset(flags, Flags::OF);
    F.N = bit_isset(flags, Flags::SF);

    return SR;
  }
  void reset(void) {
    A = 0;
    SP = 0xff;
    X = 0;
    Y = 0;
    SR = 0;
    ZPG = 0x20;
    PC = 0;
    NativeFlags = 0;
    EA = 0;
    ARG = 0;
    clock_divider = 1;
  }

  reg8_t A;
  reg8_t SP;
  reg8_t X;
  reg8_t Y;
  union {
    byte_t SR;
    struct {
      byte_t C : 1;
      byte_t Z : 1;
      byte_t I : 1;
      byte_t D : 1;
      byte_t B : 1; /* XXX: Break flag */
      byte_t E : 1;
      byte_t V : 1;
      byte_t N : 1;
    } F;
  };
  byte_t ZPG;
  reg16_t PC;

  reg16_t NativeFlags;

  reg16_t EA; /* %r8 */
  reg8_t ARG; /* %rdx */

  byte_t mmu_map[8];
  int clock_divider;
  AddressBus21x8 *bus;
  uint8_t icycles;
}
#ifdef WIN32
;
#pragma pack(pop)
#else
__attribute__((packed));
#endif

class HuC6280Cpu
    : public Cpu<AddressBus21x8, M6502Traits, HuC6280State> {
 public:
  HuC6280Cpu(Machine *machine, const std::string &name, ClockDivider divider,
             AddressBus21x8 *bus);
  virtual ~HuC6280Cpu(void);

  virtual void reset(void);
  virtual bool Interrupt(void);
  virtual void line(Line line, LineState state);
  virtual void execute(void);

  HuC6280State *get_state(void) { return &m_state; }

  virtual void log_op(HuC6280State *state, const Opcode *op, uint16_t pc,
                      const uint8_t *instr);

  byte_t irq_read(offset_t offset);
  void irq_write(offset_t offset, byte_t value);

  byte_t timer_read(offset_t offset);
  void timer_write(offset_t offset, byte_t value);

 private:
  void step(void);

  uint8_t m_irq_status;
  uint8_t m_irq_disable;
  bool m_timer_status;
  int m_timer_load;
  int m_timer_value;
  LineState m_nmi_line;
  LineState m_irq_line;
  LineState m_reset_line;
};
};
