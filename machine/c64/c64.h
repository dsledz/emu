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
#pragma once

#include "emu/emu.h"
#include "emu/gfx.h"

using namespace EMU;

typedef IOBus<uint16_t, uint8_t, 16, 8> C64Bus;
typedef std::unique_ptr<C64Bus> C64Bus_ptr;

namespace C64Machine {

class C64;

class VIC2: public ScreenDevice {
 public:
  VIC2(C64 *c64);
  ~VIC2(void);

  void load_rom(void);

 private:
  void init_palette(void);

  virtual void do_hend(void);
  virtual void do_hdraw(void);
  virtual void do_vdraw(void);
  virtual void do_vblank(void);
  virtual void do_vnext(void);

  std::function<void(void)> m_vblank_cb;

  C64 *m_c64;
  RamDevice *m_ram;
  ColorMap<16, RGBColor> m_palette;
  C64Bus m_bus;
};

typedef std::unique_ptr<VIC2> VIC2_ptr;

class M6510Cpu;
typedef std::unique_ptr<M6510Cpu> C64Cpu_ptr;

class C64: public Machine {
 public:
  C64(void);
  virtual ~C64(void);

  virtual void load_rom(const std::string &rom);

  uint8_t direction_port_read(offset_t offset);
  void direction_port_write(offset_t offset, uint8_t value);
  uint8_t io_port_read(offset_t offset);
  void io_port_write(offset_t offset, uint8_t value);

  RomSet *romset(void) { return &m_romset; }
  RamDevice *ram(void) { return &m_ram; }

 private:

  RamDevice m_ram;

  RomSet m_romset;

  uint8_t m_direction_port;
  uint8_t m_io_port;
  VIC2_ptr m_vic;
  C64Cpu_ptr m_cpu;
  C64Bus_ptr m_bus;
};

};

