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
#include "machine/nes/n2a03.h"

using namespace EMU;
using namespace M6502;

namespace NESMachine {

enum NameTableMirroring {
  SingleScreenBLK0 = 0,
  SingleScreenBLK1 = 1,
  TwoScreenVMirroring = 2,
  TwoScreenHMirroring = 3
};

struct iNesHeader {
  char magic[4];
  byte_t rom_banks;
  byte_t vrom_banks;
  byte_t hmirroring : 1;
  byte_t battery : 1;
  byte_t trainer : 1;
  byte_t four_screen : 1;
  byte_t mapper_low : 4;
  byte_t m_reserved : 4;
  byte_t mapper_high : 4;
  byte_t ram_banks;
  byte_t pal_flag;
  byte_t reserved[6];
};

enum NameTable {
  NT0 = 0,
  NT1 = 1,
  NT2 = 2,
  NT3 = 3,
};

class NES;

class NESJoypad1: public InputDevice {
public:
  NESJoypad1(NES *nes);
  ~NESJoypad1(void);

  byte_t latch_read(void);
  void latch_reset(void) { m_shift = 0; }

private:
  NES *m_nes;
  int m_shift;
};

class NESJoypad2: public InputDevice {
public:
  NESJoypad2(NES *nes);
  ~NESJoypad2(void);

  byte_t latch_read(void);
  void latch_reset(void) { m_shift = 0; }

private:
  NES *m_nes;
  int m_shift;
};

class NESMapper : public Device {
 public:
  NESMapper(NES *nes, const iNesHeader *header, bvec &rom);

  virtual ~NESMapper(void);

  virtual void reset(void) = 0;

  /* Convert a prg bank into an offset */
  size_t prg_bank(int bank);
  size_t prg_bank8k(int bank);

  /* Convert a chr bank into an offset */
  size_t chr_bank(int bank);
  size_t chr_bank1k(int bank);

 protected:
  NES *m_nes;
  iNesHeader m_header;
  bvec m_rom;
};

class NESPPU : public ScreenDevice {
 public:
  NESPPU(NES *machine, const std::string &name, ClockDivider divider);
  virtual ~NESPPU(void);

  virtual void reset(void);

  int draw_bg(void);
  int draw_sprite(int color, int x, int y);

  void set_mirroring(NameTableMirroring mirroring);

  Cycles step(void);

 private:
  virtual void do_hend(void);
  virtual void do_hdraw(void);
  virtual void do_vblank(void);

  void init_palette(void);

  void cache_bg_tile(void);
  int get_obj_pixel(int obj, int x, int y);

  byte_t ppu_read(offset_t offset);
  void ppu_write(offset_t offset, byte_t value);

  byte_t ppu_pal_read(offset_t offset);
  void ppu_pal_write(offset_t offset, byte_t value);

  std::vector<RGBColor> m_color_table;
  std::vector<RGBColor> m_palette;
  bvec m_palette_bytes;

  union {
    struct {
      byte_t htable : 1;
      byte_t vtable : 1;
      byte_t vram_step : 1;
      byte_t sprite_table : 1;
      byte_t bg_table : 1;
      byte_t sprite_size : 1;
      byte_t m_reserved0 : 1;
      byte_t nmi_enabled : 1;
    };
    byte_t value;
  } m_reg1;

  union {
    struct {
      byte_t monochrome_mode : 1;
      byte_t bg_clip : 1;
      byte_t spr_clip : 1;
      byte_t bg_visible : 1;
      byte_t spr_visible : 1;
      byte_t color_emph : 3;
    };
    byte_t value;
  } m_reg2;

  union {
    struct {
      byte_t m_reserved1 : 5;
      byte_t lost_sprite : 1;
      byte_t sprite0_hit : 1;
      byte_t vblank : 1;
    };
    byte_t value;
  } m_status;
  byte_t m_sram_addr;
  byte_t m_vram_read;
  union {
    struct {
      addr_t coarse_x : 5;
      addr_t coarse_y : 5;
      addr_t nt_hselect : 1;
      addr_t nt_vselect : 1;
      addr_t fine_y : 3;
      addr_t m_unused : 1;
    };
    addr_t d;
  } m_v;
  union {
    struct {
      addr_t coarse_x : 5;
      addr_t coarse_y : 5;
      addr_t nt_hselect : 1;
      addr_t nt_vselect : 1;
      addr_t fine_y : 3;
      addr_t m_unused : 1;
    };
    addr_t d;
  } m_t;
  byte_t m_x;
  byte_t m_tx;
  byte_t m_latch;
  bool m_flip_flop;
  bool m_vram_locked;

  std::vector<int> m_sprites;

  byte_t m_bgp0;  /* Plane 0 of background */
  byte_t m_bgp1;  /* Plane 1 of background */
  byte_t m_bgpal; /* background palette */

  bvec m_sram; /* Sprite memory */
  bvec m_blk0; /* Name Table 0 */
  bvec m_blk1; /* Name Table 1 */

  IOPort *m_mirror;

  AddressBus16x8 *m_cpu_bus;
  AddressBus16x8 *m_ppu_bus;
  AddressBus8x8 *m_sprite_bus;
};

typedef std::unique_ptr<NESMapper> mapper_ptr;
mapper_ptr load_cartridge(NES *nes, const std::string &rom);

class NES : public Machine {
 public:
  NES(void);
  virtual ~NES(void);

  virtual void load_rom(const std::string &rom);

  uint8_t latch_read(offset_t offset);
  void latch_write(offset_t offset, uint8_t value);

  AddressBus16x8 *cpu_bus(void) { return &m_cpu_bus; }

  AddressBus16x8 *ppu_bus(void) { return &m_ppu_bus; }

  AddressBus8x8 *sprite_bus(void) { return &m_sprite_bus; }

  NESPPU *ppu(void) { return m_ppu.get(); }

 private:
  static const std::vector<std::string> ports;

  std::unique_ptr<N2a03Cpu> m_cpu;
  std::unique_ptr<NESPPU> m_ppu;
  NESJoypad1 m_joypad1;
  NESJoypad2 m_joypad2;
  mapper_ptr m_mapper;
  RamDevice m_ram;

  AddressBus16x8 m_cpu_bus;
  AddressBus16x8 m_ppu_bus;
  AddressBus8x8 m_sprite_bus;

};
};
