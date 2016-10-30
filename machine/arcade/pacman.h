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
#include "emu/gfx.h"
#include "emu/screen.h"

#include "cpu/z80/z80.h"

using namespace EMU;
using namespace Z80;

namespace Arcade {

class PacmanGfx : public ScreenDevice {
 public:
  PacmanGfx(Machine *machine, const std::string &name, ClockDivider divider,
            AddressBus16x8 *bus);
  ~PacmanGfx(void);

  RamDevice &vram() { return m_vram; }
  RamDevice &cram() { return m_cram; }
  byte_t spr_read(offset_t offset) {
    offset &= 0x0F;
    if (offset & 0x01) {
      return m_spr[offset >> 1].color;
    } else {
      return m_spr[offset >> 1].flags;
    }
  }
  void spr_write(offset_t offset, byte_t value) {
    offset &= 0x0F;
    if (offset & 0x01) {
      m_spr[offset >> 1].color = value;
    } else {
      m_spr[offset >> 1].flags = value;
    }
  }
  byte_t spr_coord_read(offset_t offset) {
    offset &= 0x0F;
    if (offset & 0x01) {
      return m_spr[offset >> 1].x;
    } else {
      return m_spr[offset >> 1].y;
    }
  }
  void spr_coord_write(offset_t offset, byte_t value) {
    offset &= 0x0F;
    if (offset & 0x01) {
      m_spr[offset >> 1].x = value;
    } else {
      m_spr[offset >> 1].y = value;
    }
  }

  void init(RomSet *romset);

  void set_vblank_cb(std::function<void(void)> func) { m_vblank_cb = func; }

 private:
  virtual void do_vdraw(void);
  virtual void do_vblank(void);

  std::function<void(void)> m_vblank_cb;

  RamDevice m_vram;
  RamDevice m_cram;
  AddressBus16x8 *_bus;

  void init_sprite(GfxObject<16, 16> *obj, byte_t *b);
  void init_tile(GfxObject<8, 8> *obj, byte_t *b);

  void draw_bg(FrameBuffer *screen);
  void draw_sprites(FrameBuffer *screen);

  /* Graphics */
  struct Sprite {
    union {
      struct {
        byte_t xflip : 1;
        byte_t yflip : 1;
        byte_t idx : 6;
      };
      byte_t flags;
    };
    byte_t color;
    byte_t x;
    byte_t y;
  };

  Sprite m_spr[8];

  ColorMap<32, RGBColor> m_colors;
  GfxObject<8, 8> m_tiles[256];
  ColorPalette<4> m_palettes[128];
  GfxObject<16, 16> m_sprites[64];
};

typedef std::unique_ptr<PacmanGfx> PacmanGfx_ptr;

class Pacman : public Machine {
 public:
  Pacman(void);
  virtual ~Pacman(void);

  virtual void load_rom(const std::string &rom);

 private:
  byte_t ram_read(offset_t offset) { return m_ram.read8(offset); }
  void ram_write(offset_t offset, byte_t value) {
    if (offset == 0x2f7 && value == 0xff) {
      std::cout << "Sound write " << Hex(value) << std::endl;
    }
    m_ram.write8(offset, value);
  }

  byte_t in0_read(offset_t offset) { return read_ioport("IN0"); }
  void latch_write(offset_t offset, byte_t value);

  void sound_write(offset_t offset, byte_t value) {}

  byte_t in1_read(offset_t offset) { return read_ioport("IN1"); }

  byte_t dsw1_read(offset_t offset) { return read_ioport("DSW1"); }
  void dsw1_write(offset_t offset, byte_t value) {}

  byte_t dsw2_read(offset_t offset) { return read_ioport("DSW2"); }
  void watchdog_write(offset_t offset, byte_t value) {}

  byte_t io_read(offset_t offset);
  void io_write(offset_t offset, byte_t value);

  void init_bus(void);
  void init_switches(void);
  void init_controls(void);

  unsigned m_hertz;

  RomSet m_romset;
  Z80State m_cpu_state;
  Z80Cpu_ptr m_cpu;
  Z80Bus_ptr m_bus;
  Z80IOBus_ptr m_iobus;
  RamDevice m_ram;
  PacmanGfx_ptr m_gfx;

  /* latches */
  bool m_irq_mask;
};
};
