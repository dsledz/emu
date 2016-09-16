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

#include "emu/emu.h"

#include "machine/gb/lr35902.h"

using namespace EMU;

namespace GBMachine {

class GBGraphics;
class GBMBC;

enum GBInterrupt {
  VBlank = 0,
  LCDStat = 1,
  Timeout = 2,
  Serial = 3,
  Joypad = 4,
};

/* XXX: Should we split this up more? */
enum GBReg {
  KEYS = 0xFF00,
  SB = 0xFF01,
  SC = 0xFF02,
  DIV = 0xFF04,
  TIMA = 0xFF05,
  TMA = 0xFF06,
  TAC = 0xFF07,
  IF = 0xFF0F,
  DMA = 0xFF46,
  DMG_RESET = 0xFF50,
  IE = 0xFFFF,
};

class Gameboy : public Machine {
 public:
  Gameboy(void);
  virtual ~Gameboy(void);

  AddressBus16x8 *bus(void) { return m_bus.get(); }

  virtual void load_rom(const std::string &rom);

 private:
  AddressBus16x8_ptr m_bus;

  std::unique_ptr<LR35902Cpu> m_cpu;
  std::unique_ptr<GBGraphics> m_gfx;
  std::unique_ptr<RamDevice> m_ram;
  std::unique_ptr<GBMBC> m_mbc;
  std::unique_ptr<RamDevice> m_hiram;

  Device_ptr m_timer;
  Device_ptr m_serial;
  Device_ptr m_joypad;
};
};
