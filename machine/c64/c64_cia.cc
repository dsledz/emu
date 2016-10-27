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
}

C64CIA::~C64CIA(void) {}

void
C64CIA::execute(void) {
  if (m_timerA.running) {
    if (m_timerA.timer < 100) {
      DEVICE_INFO("Triggering interrupt");
      m_regs[CIARegisters::ICR] |= 0x01;
      m_timerA.timer = m_timerA.latch.d;
      machine()->set_line("cpu", Line::INT0, LineState::Assert);
    } else
      m_timerA.timer -= 100;
  }
  add_icycles(Cycles(100));
}

byte_t
C64CIA::cia_read(offset_t offset) {
  byte_t result = 0;
  offset &= 0x0f;
  switch (offset) {
    case CIARegisters::PRA:
      result = m_c64->keyboard()->read_columns();
      break;
    case CIARegisters::PRB:
      result = m_c64->keyboard()->read_rows();
      break;
    case CIARegisters::DDRA:
    case CIARegisters::DDRB:
    case CIARegisters::TALO:
      result = m_timerA.latch.b.l;
      break;
    case CIARegisters::TAHI:
      result = m_timerA.latch.b.h;
      break;
    case CIARegisters::TBLO:
      result = m_timerB.latch.b.l;
      break;
    case CIARegisters::TBHI:
      result = m_timerB.latch.b.h;
      break;
    case CIARegisters::TOD_10THS:
    case CIARegisters::TOD_SEC:
    case CIARegisters::TOD_MIN:
    case CIARegisters::TOD_HR:
    case CIARegisters::SDR:
    case CIARegisters::ICR:
    case CIARegisters::CRB:
      result = m_regs[offset];
      break;
    case CIARegisters::CRA:
      break;
  }

  DEVICE_INFO("cia_read(", Hex((byte_t)offset), ") -> ", Hex(result));
  return result;
}

void C64CIA::cia_write(offset_t offset, byte_t value) {
  offset &= 0x0f;
  switch (offset) {
    case CIARegisters::DDRA:
      m_c64->keyboard()->write_rows(value);
      break;
    case CIARegisters::DDRB:
      m_c64->keyboard()->write_columns(value);
      break;
    case CIARegisters::TALO:
      m_timerA.latch.b.l = value;
      break;
    case CIARegisters::TAHI:
      m_timerA.latch.b.h = value;
      break;
    case CIARegisters::ICR:
      if (bit_isset(value, 7)) {
        m_regs[CIARegisters::ICR] &= ~(value & 0x1f);
        machine()->set_line("cpu", Line::INT0, LineState::Clear);
      } else
        m_regs[CIARegisters::ICR] |= (value & 0x1f);
      break;
    case CIARegisters::CRA:
      if (bit_isset(value, 4)) {
        m_timerA.timer = m_timerA.latch.d;
      }
      m_timerA.running = !bit_isset(value, 0);
    break;
  default:
      m_regs[offset] = value;
    break;
  }
}
