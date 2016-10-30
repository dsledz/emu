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
#include "emu/gfx.h"

#include "cpu/z80/z80.h"
#include "machine/arcade/namco06.h"
#include "machine/arcade/namco51.h"

using namespace EMU;
using namespace Z80;
using namespace Device;

namespace Arcade {

class GalagaGfx : public ScreenDevice {
 public:
  GalagaGfx(Machine *machine, const std::string &name,
            ClockDivider divider,
            AddressBus16x8 *bus);
  ~GalagaGfx(void);

  RamDevice *vram(void) { return &m_vram; }

  void init(RomSet *romset);

 protected:
  virtual void do_vdraw(void);

 private:
  void init_sprite(GfxObject<16, 16> *obj, byte_t *b);
  void init_tile(GfxObject<8, 8> *obj, byte_t *b);

  void draw_bg(FrameBuffer *screen);
  void draw_sprites(FrameBuffer *screen);

  RamDevice m_vram;
  AddressBus16x8 *m_bus;

  /* Graphic Data */
  ColorMap<32, RGBColor> m_colors;
  GfxObject<8, 8> m_tiles[128];
  ColorPalette<4> m_tile_palette[64];
  GfxObject<16, 16> m_sprites[128];
  ColorPalette<4> m_sprite_palette[64];
};

typedef std::unique_ptr<GalagaGfx> GalagaGfx_ptr;

class Galaga : public Machine {
 public:
  Galaga(void);
  virtual ~Galaga(void);

  virtual void load_rom(const std::string &rom);

 private:
  byte_t dips_read(offset_t offset);
  void latch_write(offset_t offset, byte_t value);

  void init_bus(AddressBus16x8 *bus);
  void init_switches(void);
  void init_controls(void);

  void init_gfx(RomSet *romset);
  void init_sprite(GfxObject<16, 16> *obj, byte_t *b);
  void init_tile(GfxObject<8, 8> *obj, byte_t *b);

  void draw_bg(void);
  void draw_sprites(void);
  void draw_screen(void);

  RomSet m_romset;
  RamDevice m_ram1, m_ram2, m_ram3;

  Z80State m_main_cpu_state;
  Z80Cpu_ptr m_main_cpu;
  Z80Bus_ptr m_bus1;

  Z80State m_sub_cpu_state;
  Z80Cpu_ptr m_sub_cpu;
  Z80Bus_ptr m_bus2;

  Z80State m_snd_cpu_state;
  Z80Cpu_ptr m_snd_cpu;
  Z80Bus_ptr m_bus3;

  Z80IOBus_ptr m_iobus;
  Namco51_ptr m_namco51;
  Namco06_ptr m_namco06;
  GalagaGfx_ptr m_gfx;

  /* Interrupt lines */
  bool m_main_irq;
  bool m_sub_irq;
  bool m_snd_nmi;
};
};
