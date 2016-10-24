/**
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

#include "emu/emu.h"

#include "machine/c64/c64.h"
#include "machine/c64/m6510.h"

using namespace EMU;
using namespace C64Machine;

enum CIARegisters {
  PRA = 0,
  PRB = 1,
  DDRA = 2,
  DDRB = 3,
  TALO = 4,
  TAHI = 5,
  TBLO = 6,
  TBHI = 7,
  TOD_10THS = 8,
  TOD_SEC = 9,
  TOD_MIN = 10,
  TOD_HR = 11,
  SDR = 12,
  ICR = 13,
  CRA = 14,
  CRB = 15
};

C64CIA::C64CIA(C64 *c64, const std::string &name, Line irq_line)
    : ClockedDevice(c64, c64->clock(), name, ClockDivider(8)),
      m_c64(c64),
      m_irq_line(irq_line) {
  m_c64->bus()->add(0xdc00, 0xdcff, READ_CB(C64CIA::cia_read, this),
                    WRITE_CB(C64CIA::cia_write, this));
}

C64CIA::~C64CIA(void) {}

void
C64CIA::execute(void) {
  add_icycles(Cycles(100));
}

byte_t
C64CIA::cia_read(offset_t offset) {
  byte_t result = 0;
  offset &= 0x0f;
  switch (offset) {
    case CIARegisters::PRA:
      result = m_c64->keyboard()->read_columns() & m_regs[CIARegisters::DDRA];
      result |= 0 & ~m_regs[CIARegisters::DDRA];
      break;
    case CIARegisters::PRB:
      result = m_c64->keyboard()->read_rows() & m_regs[CIARegisters::DDRB];
      result |= 0 & ~m_regs[CIARegisters::DDRB];
      break;
    case CIARegisters::DDRA:
    case CIARegisters::DDRB:
    case CIARegisters::TALO:
    case CIARegisters::TAHI:
    case CIARegisters::TBLO:
    case CIARegisters::TBHI:
    case CIARegisters::TOD_10THS:
    case CIARegisters::TOD_SEC:
    case CIARegisters::TOD_MIN:
    case CIARegisters::TOD_HR:
    case CIARegisters::SDR:
    case CIARegisters::ICR:
    case CIARegisters::CRA:
    case CIARegisters::CRB:
      result = m_regs[offset];
  }

  DEVICE_INFO("cia_read(", Hex((byte_t)offset), ") -> ", Hex(result));
  return result;
}

void
C64CIA::cia_write(offset_t offset, byte_t value) {
  offset &= 0x0f;
  m_regs[offset] = value;
  DEVICE_INFO("cia_write(", Hex((byte_t)offset), ", ", Hex(value), ")");
}

