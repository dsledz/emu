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

#include "emu/emu.h"

#include "machine/arcade/namco51.h"

using namespace EMU;
using namespace Device;

Namco51::Namco51(Machine *machine)
    : IODevice(machine, "namco51", 4),
      m_mode(Mode::Unknown),
      m_remap(false),
      m_credits(0),
      m_coinage_bytes(0),
      m_read_count(0),
      m_last_coin(0),
      m_last_joy(0) {}

Namco51::~Namco51(void) {}

void Namco51::write8(offset_t offset, uint8_t value) {
  if (m_coinage_bytes) {
    DEVICE_DEBUG("51xx coinage write");
    m_coinage_bytes--;
    return;
  }

  switch (Command(value)) {
    case Command::Coinage:
      DEVICE_DEBUG("Coinage bytes");
      m_coinage_bytes = 4;
      break;
    case Command::Credit:
      DEVICE_DEBUG("51xx Credit Mode");
      m_mode = Mode::Credit;
      m_read_count = 0;
      break;
    case Command::DisableRemap:
      DEVICE_DEBUG("51xx Disable Remap");
      m_remap = false;
      break;
    case Command::EnableRemap:
      DEVICE_DEBUG("51xx Enable Remap");
      m_remap = true;
      break;
    case Command::SwitchMode:
      DEVICE_DEBUG("51xx Switch Mode");
      m_mode = Mode::Switch;
      m_read_count = 0;
      break;
    default:
      break;
  }
}

uint8_t Namco51::read8(offset_t offset) {
  if (m_mode == Mode::Switch) {
    switch (m_read_count++ % 3) {
      case 0: /* buttons & coins */
        DEVICE_DEBUG("51xx Buttons");
        return (read_port("IN0") | (read_port("IN1") << 4));
      case 1: /* joysticks */
        DEVICE_DEBUG("51xx Joystick");
        return (read_port("IN2") | (read_port("IN3") << 4));
      case 2: /* unusued */
        DEVICE_DEBUG("51xx Unused");
        return 0x00;
    }

  } else if (m_mode == Mode::Credit) {
    switch (m_read_count++ % 3) {
      case 0: {
        /* Credits */
        /* Read in the new keys */
        /* d = XXCC21XX C = coin, 1 = start1, 2 = start2 */
        int in = (~read_port("IN0") & 0xC) | (~read_port("IN1") << 4);
        /* See if we got credits. */
        if (bit_toggle(in, m_last_coin, 5)) m_credits++;
        if (bit_toggle(in, m_last_coin, 4)) m_credits++;

        /* XXX: This could miss credits if the keys are pressed at
         * the same time.
         */
        if (bit_toggle(in, m_last_coin, 3) && m_credits > 2) m_credits -= 2;

        if (bit_toggle(in, m_last_coin, 2) && m_credits > 1) m_credits -= 1;

        m_last_coin = in;
        /* Credits in BCD */
        return (m_credits % 10) | (m_credits / 10) << 4;
      }
      case 1: {
        if (m_remap) throw DeviceFault("namco51", "remap");

        int joy = read_port("IN2") & 0x0f;
        int in = ~read_port("IN0") & 0x01;
        int toggle = in ^ m_last_joy;

        /* One button press = one fire */
        m_last_joy = (m_last_joy & 0x02) | in;
        joy |= ((toggle & in) ^ 0x01) << 4;
        joy |= (in ^ 0x01) << 5;

        return joy;
      }
      case 2: {
        if (m_remap) throw DeviceFault("namco51", "remap");

        int joy = read_port("IN3") & 0x0f;
        int in = ~read_port("IN0") & 0x02;
        int toggle = in ^ m_last_joy;
        m_last_joy = (m_last_joy & 0x01) | in;
        joy |= ((toggle & in) ^ 0x02) << 3;
        joy |= (in ^ 0x02) << 4;

        return joy;
      }
    }
  }

  DEVICE_DEBUG("51xx Unknown READ");

  return 0;
}

uint8_t *Namco51::direct(offset_t offset) { throw DeviceFault(name()); }

uint8_t Namco51::read_port(const std::string &port) {
  return machine()->read_ioport(port);
}
