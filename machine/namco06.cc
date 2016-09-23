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

#include "machine/namco06.h"
#include "emu/emu.h"

using namespace EMU;
using namespace Device;

Namco06::Namco06(Machine *machine, Device *parent)
    : ClockedDevice(machine, machine->clock(), "06xx", ClockDivider(1)),
      m_reset_line(LineState::Clear),
      m_parent(parent),
      m_children(),
      m_control(0x00) {
  reset();
}

Namco06::~Namco06(void) {}

void Namco06::reset(void) { m_control = 0x00; }

void Namco06::add_child(int pos, IODevice *child) { m_children[pos] = child; }

byte_t Namco06::read_child(addr_t addr) {
  byte_t result = 0xff;
  byte_t control = m_control;
  if ((control & 0x10) != 0x10) return 0;
  for (int dev = 0; dev < 4; dev++)
    if (bit_isset(control, dev) && m_children[dev] != NULL)
      result &= m_children[dev]->read8(0);
  return result;
}

void Namco06::write_child(addr_t addr, byte_t value) {
  if ((m_control & 0x10) != 0x00) return;
  for (int dev = 0; dev < 4; dev++)
    if (bit_isset(m_control, dev) && m_children[dev] != NULL)
      m_children[dev]->write8(0, value);
}

byte_t Namco06::read_control(addr_t addr) { return m_control; }

void Namco06::write_control(addr_t addr, byte_t value) { m_control = value; }

void Namco06::execute(void) {
  while (true) {
    if (m_reset_line == LineState::Pulse) {
      reset();
      m_reset_line = LineState::Clear;
    }
    add_icycles(Cycles(20));
    if ((m_control & 0x0F) != 0)
      machine()->set_line(m_parent, Line::NMI, LineState::Pulse);
  }
}

void Namco06::line(Line line, LineState state) {
  switch (line) {
    case Line::RESET:
      m_reset_line = state;
      break;
    default:
      DEVICE_DEBUG("Unrecognized signal");
      break;
  }
}
