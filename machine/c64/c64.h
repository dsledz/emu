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

enum VICReg {
  M0X = 0x00,
  M0Y = 0x01,
  M1X = 0x02,
  M1Y = 0x03,
  M2X = 0x04,
  M2Y = 0x05,
  M3X = 0x06,
  M3Y = 0x07,
  M4X = 0x08,
  M4Y = 0x09,
  M5X = 0x0A,
  M5Y = 0x0B,
  M6X = 0x0C,
  M6Y = 0x0D,
  M7X = 0x0E,
  M7Y = 0x0F,
  MSBX = 0x10,
  CR1 = 0x11,  // RST8, ECM, BMM, DEN, RSEL, YSCROLL(3)
  RASTER = 0x12,
  LPX = 0x13, // Light pen X
  LPY = 0x14, // Light pen Y
  ME = 0x15, // Sprite enabled
  CR2 = 0x16, // -, -, RES, MCM, CSEL, XSCROLL(3)
  MYE = 0x17,
  MP = 0x18, // VM13, VM12, VM11, VM10, CB13, CB12, CB11, -
  IR = 0x19, // IRQ, -, -, -, ILP, IMMC, IMBC, IRST
  IE = 0x1A, // IRQ, -, -, -, ELP, EMMC, EMBC, ERST
  MDP = 0x1B, // Sprite data priority
  MMC = 0x1C, // Sprite Multicolour
  MXE = 0x1D, // Sprite X expansion
  MM = 0x1E, // Sprite-sprite collision
  MD = 0x1F, // Sprite date collision
  BORDER = 0x20, // Border colour
  B0C = 0x21, // Background colour
  B1C = 0x22,
  B2C = 0x23,
  B3C = 0x24,
  MM0 = 0x25, // Sprite multicolour
  MM1 = 0x26,
  M0C = 0x27, // Sprite colour
  M1C = 0x28,
  M2C = 0x29,
  M3C = 0x2A,
  M4C = 0x2B,
  M5C = 0x2C,
  M6C = 0x2D,
  M7C = 0x2E
};

class C64;

class C64Keyboard : public InputDevice {
 public:
  C64Keyboard(C64 *c64);
  ~C64Keyboard(void);

  byte_t read_rows(void);
  void write_rows(byte_t value);
  byte_t read_columns(void);
  void write_columns(byte_t value);

 private:
  C64 *m_c64;
  byte_t m_mask_row;
  byte_t m_mask_col;
  byte_t m_matrix[64];
};

typedef std::unique_ptr<C64Keyboard> C64Keyboard_ptr;

class C64CIA: public ClockedDevice {
 public:
  C64CIA(C64 *c64, const std::string &name, Line irq_line);
  ~C64CIA(void);

  virtual void execute(void);

 private:

  byte_t cia_read(offset_t offset);
  void cia_write(offset_t offset, byte_t value);

  C64 *m_c64;
  Line m_irq_line;
  // Real time clock
  EmuTime m_clock;
  byte_t m_regs[16];
};

typedef std::unique_ptr<C64CIA> C64CIA_ptr;

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
  C64Bus *bus(void) { return m_bus.get(); }
  C64Keyboard *keyboard(void) { return m_keyboard.get(); }

 private:

  RamDevice m_ram;

  RomSet m_romset;

  uint8_t m_direction_port;
  uint8_t m_io_port;
  VIC2_ptr m_vic;
  C64Cpu_ptr m_cpu;
  C64Bus_ptr m_bus;
  C64CIA_ptr m_cia1;
  C64CIA_ptr m_cia2;
  C64Keyboard_ptr m_keyboard;
};

};

