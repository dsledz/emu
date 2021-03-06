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
#include "emu/emu.h"

namespace M6502 {

struct M65c02State : public M6502State {};

class M65c02Cpu
    : public Cpu<AddressBus16x8, M6502Traits, M65c02State> {
 public:
  M65c02Cpu(Machine *machine, const std::string &name, ClockDivider divider,
            bus_type *bus);
  ~M65c02Cpu(void);
  M65c02Cpu(const M65c02Cpu &cpu) = delete;

  virtual void reset(void);
  virtual void line(Line line, LineState state);
  virtual void execute(void);

  M65c02State *get_state(void) { return &m_state; }

  virtual void log_op(M65c02State *state, const Opcode *op, uint16_t pc,
                      const uint8_t *instr);
 private:
  virtual bool Interrupt(void);

  LineState m_nmi_line;
  LineState m_irq_line;
  LineState m_reset_line;
};
};
