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

#include "machine/nes/nes.h"

using namespace NESMachine;

enum NESKey {
  A = 0,
  B = 1,
  Select = 2,
  Start = 3,
  Up = 4,
  Down = 5,
  Left = 6,
  Right = 7,
  Size = 8,
};

NESJoypad1::NESJoypad1(NES *nes)
    : InputDevice(nes, "joypad1"), m_nes(nes), m_shift(0) {
  nes->add_ioport("JOYPAD1");
  IOPort *port = nes->ioport("JOYPAD1");

  nes->add_input(InputSignal(InputKey::Joy1Btn1, port, NESKey::A, true));
  nes->add_input(InputSignal(InputKey::Joy1Btn2, port, NESKey::B, true));
  nes->add_input(InputSignal(InputKey::Select1, port, NESKey::Select, true));
  nes->add_input(InputSignal(InputKey::Start1, port, NESKey::Start, true));
  nes->add_input(InputSignal(InputKey::Joy1Up, port, NESKey::Up, true));
  nes->add_input(InputSignal(InputKey::Joy1Down, port, NESKey::Down, true));
  nes->add_input(InputSignal(InputKey::Joy1Left, port, NESKey::Left, true));
  nes->add_input(InputSignal(InputKey::Joy1Right, port, NESKey::Right, true));
}

NESJoypad1::~NESJoypad1(void)
{
}

byte_t
NESJoypad1::latch_read(void)
{
  byte_t result =0;
  byte_t keys = m_nes->read_ioport("JOYPAD1");
  if (m_shift < 8) result = bit_isset(keys, m_shift);
  m_shift++;
  return result;
}

NESJoypad2::NESJoypad2(NES *nes)
    : InputDevice(nes, "joypad2"),
      m_nes(nes),
      m_shift(0)
{
  nes->add_ioport("JOYPAD2");
  IOPort *port = nes->ioport("JOYPAD2");

  nes->add_input(InputSignal(InputKey::Joy2Btn1, port, NESKey::A, true));
  nes->add_input(InputSignal(InputKey::Joy2Btn2, port, NESKey::B, true));
  nes->add_input(InputSignal(InputKey::Select2, port, NESKey::Select, true));
  nes->add_input(InputSignal(InputKey::Start2, port, NESKey::Start, true));
  nes->add_input(InputSignal(InputKey::Joy2Up, port, NESKey::Up, true));
  nes->add_input(InputSignal(InputKey::Joy2Down, port, NESKey::Down, true));
  nes->add_input(InputSignal(InputKey::Joy2Left, port, NESKey::Left, true));
  nes->add_input(InputSignal(InputKey::Joy2Right, port, NESKey::Right, true));
}

NESJoypad2::~NESJoypad2(void)
{
}

byte_t
NESJoypad2::latch_read(void)
{
  byte_t result =0;
  byte_t keys = m_nes->read_ioport("JOYPAD2");
  if (m_shift < 8) result = bit_isset(keys, m_shift);
  m_shift++;
  return result;
}

