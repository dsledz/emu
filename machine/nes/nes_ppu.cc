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

using namespace EMU;
using namespace NESMachine;

NESPPU::NESPPU(NES *machine, const std::string &name, ClockDivider divider)
    : ScreenDevice(machine, name, divider, 340, 261, 256, 0, 260, 20),
      m_color_table(0x40),
      m_palette(0x20),
      m_palette_bytes(0x20),
      m_reg1({}),
      m_reg2({}),
      m_status({}),
      m_sram_addr(0),
      m_v(),
      m_t(),
      m_x(),
      m_tx(),
      m_latch(0),
      m_flip_flop(false),
      m_vram_locked(false),
      m_sram(256),
      m_blk0(0x0400),
      m_blk1(0x0400) {
  m_mirror = machine->ioport("MIRRORING");

  m_cpu_bus = machine->cpu_bus();
  m_ppu_bus = machine->ppu_bus();
  m_sprite_bus = machine->sprite_bus();

  m_cpu_bus->add(0x2000, 0x3FFF, READ_CB(NESPPU::ppu_read, this),
                 WRITE_CB(NESPPU::ppu_write, this));

  m_ppu_bus->add(0x2000, 0x3FFF, READ_CB(NESPPU::ppu_bus_read, this),
                 WRITE_CB(NESPPU::ppu_bus_write, this));

  machine->sprite_bus()->add(0x00, m_sram);

  reset();
}

NESPPU::~NESPPU(void) {}

void NESPPU::reset(void) {
  m_reg1.value = 0;
  m_reg2.value = 0;
  m_status.value = 0;
  m_v.d = 0;
  m_t.d = 0;
  m_x = 0;
  m_tx = 0;
  m_sram_addr = 0;
  m_latch = 0;
  m_flip_flop = false;
  m_vram_locked = false;
  m_hpos = 0;
  m_vpos = 0;
  std::fill(m_blk0.begin(), m_blk0.end(), 0);
  std::fill(m_blk1.begin(), m_blk1.end(), 0);
  std::fill(m_palette_bytes.begin(), m_palette_bytes.end(), 0);
  std::fill(m_sram.begin(), m_sram.end(), 0);

  init_palette();
}

void NESPPU::cache_bg_tile(void) {
  int obj = ppu_nt_read(0x2000 | (m_v.d & 0xFFF)) + (m_reg1.bg_table * 256);
  byte_t b = ppu_nt_read(0x23C0 | (m_v.d & 0x0C00) | ((m_v.d >> 4) & 0x38) |
                         ((m_v.d >> 2) & 0x07));
  /* XXX: calculate */
  int bit = 0;
  if (m_v.coarse_x & 0x02) bit += 2;
  if (m_v.coarse_y & 0x02) bit += 4;

  offset_t offset = obj * 16 + m_v.fine_y;
  m_bgp0 = m_ppu_bus->read(offset);
  m_bgp1 = m_ppu_bus->read(offset + 8);
  m_bgpal = (bit_isset(b, bit) << 2) | (bit_isset(b, bit + 1) << 3);
}

int NESPPU::draw_bg(void) {
  int pen = bit_isset(m_bgp0, 7 - m_x) | (bit_isset(m_bgp1, 7 - m_x) << 1);
  if (pen == 0)
    return 0;
  else
    return pen | m_bgpal;
}

int NESPPU::draw_sprite(int color, int x, int y) {
  const int sprite_size = (m_reg1.sprite_size ? 15 : 7);
  for (auto it = m_sprites.begin(); it != m_sprites.end(); it++) {
    int idx = *it;
    int iy = m_sram[idx] + 1;
    if (y < iy || y > iy + sprite_size) continue;
    int ix = m_sram[idx + 3];
    if (x < ix || x > ix + 7) continue;
    int obj = m_sram[idx + 1] + (m_reg1.sprite_table * 256);
    const int flags = m_sram[idx + 2];
    iy = y - iy;
    ix = x - ix;
    if (sprite_size > 7) obj &= ~0x01;
    /* Vertical mirror */
    if (bit_isset(flags, 7)) iy = sprite_size - iy;
    /* Horizontal mirror */
    if (bit_isset(flags, 6)) ix = 7 - ix;
    /* Handle large sprite */
    if (iy >= 8) {
      iy -= 8;
      obj++;
    }

    const int pen = get_obj_pixel(obj, ix, iy);
    if (pen != 0) {
      if (idx == m_sram_addr && color != 0 && x != 255)
        m_status.sprite0_hit = 1;
      if (!bit_isset(flags, 5) || color == 0)
        color = 0x10 + ((flags & 0x03) << 2) + pen;
      return color;
    }
  }
  return color;
}

void NESPPU::do_vblank(void) {
  FrameBuffer *screen = machine()->screen();
  screen->flip();
  screen->clear();
  m_vram_locked = false;
  m_status.vblank = 1;
  if (m_reg1.nmi_enabled) {
    machine()->set_line("cpu", Line::NMI, LineState::Pulse);
  }
}

void NESPPU::do_hdraw(void) {
  if (m_vpos <= 20 || (!m_reg2.bg_visible && !m_reg2.spr_visible)) return;

  int sy = m_vpos - 21;
  FrameBuffer *screen = machine()->screen();
  for (int sx = 0; sx < 256; sx++) {
    int color = 0;
    if (m_reg2.bg_visible && (m_reg2.bg_clip || sx >= 8)) color = draw_bg();
    if (m_reg2.spr_visible && (m_reg2.spr_clip || sx >= 8))
      color = draw_sprite(color, sx, sy);
    screen->set(sx, sy, m_palette[color]);
    if (m_x == 0x07) {
      m_x = 0;
      if (m_v.coarse_x == 0x1F) {
        m_v.nt_hselect ^= 1;
        m_v.coarse_x = 0;
      } else
        m_v.coarse_x++;
      cache_bg_tile();
    } else
      m_x++;
  }
}

void NESPPU::do_hend(void) {
  if (m_vpos < 20 || (!m_reg2.bg_visible && !m_reg2.spr_visible)) return;

  /* 256 */
  if (m_v.fine_y == 0x07) {
    m_v.fine_y = 0;
    if (m_v.coarse_y == 29) {
      m_v.coarse_y = 0;
      m_v.nt_vselect ^= 1;
    } else if (m_v.coarse_y == 31) {
      m_v.coarse_y = 0;
    } else
      m_v.coarse_y++;
  } else
    m_v.fine_y++;

  /* 257 */
  m_v.coarse_x = m_t.coarse_x;
  m_x = m_tx;
  m_v.nt_hselect = m_t.nt_hselect;
  machine()->set_line("mapper", Line::INT0, LineState::Pulse);

  /* 280 */
  if (m_vpos == 20) {
    m_vram_locked = true;
    m_v.fine_y = m_t.fine_y;
    m_v.nt_vselect = m_t.nt_vselect;
    m_v.coarse_y = m_t.coarse_y;
    m_status.vblank = 0;
    m_status.sprite0_hit = 0;
  }

  /* 320 */
  cache_bg_tile();

  /* 328 */
  m_sprites.resize(0);
  const int sy = m_vpos - 20;
  const int sprite_size = (m_reg1.sprite_size ? 15 : 7);
  for (int idx = 0; idx < 256; idx += 4) {
    int iy = m_sram[idx] + 1;
    if (sy < iy || sy > iy + sprite_size) continue;
    if (m_sprites.size() == 8) {
      m_status.lost_sprite = 1;
      break;
    }
    m_sprites.push_back(idx);
  }
}

void NESPPU::init_palette(void) {
  /* XXX: VS palette */
  unsigned entry[] = {
      0333, 0014, 0006, 0326, 0403, 0503, 0510, 0420, 0320, 0120, 0031,
      0040, 0022, 0000, 0000, 0000, 0555, 0036, 0027, 0407, 0507, 0704,
      0700, 0630, 0430, 0140, 0040, 0053, 0044, 0000, 0000, 0000, 0777,
      0357, 0447, 0637, 0707, 0737, 0740, 0750, 0660, 0360, 0070, 0276,
      0077, 0000, 0000, 0000, 0777, 0567, 0657, 0757, 0747, 0755, 0764,
      0772, 0773, 0572, 0473, 0276, 0467, 0000, 0000, 0000,
  };

  for (int i = 0; i < 64; i++) {
    m_color_table[i] =
        RGBColor(((entry[i] & 0x1C0) >> 6) * 32, ((entry[i] & 0x038) >> 3) * 32,
                 ((entry[i] & 0x007) >> 0) * 32);
  }

  m_palette[0] = m_color_table[63];
  for (int i = 1; i < 32; i++) m_palette[i] = m_color_table[i % 4 + 4];
}

int NESPPU::get_obj_pixel(int obj, int x, int y) {
  offset_t offset = obj * 16 + y;
  byte_t l = m_ppu_bus->read(offset);
  byte_t h = m_ppu_bus->read(offset + 8);

  return bit_isset(l, 7 - x) | (bit_isset(h, 7 - x) << 1);
}

byte_t NESPPU::ppu_read(offset_t offset) {
  byte_t result = m_latch;
  switch (offset % 8) {
    case 0: /* 0x2000 PPU Control register 1 */
      result = m_reg1.value;
      break;
    case 1: /* 0x2001 PPU Control register 2 */
      result = m_reg2.value;
      break;
    case 2: /* 0x2002 PPU Status register */
      result = m_status.value;
      /* flip_flop is cleared on read */
      m_flip_flop = 0;
      m_latch = 0;
      m_status.vblank = 0;
      break;
    case 3: /* 0x2003 SPR-RAM Address */
      result = m_sram_addr;
      break;
    case 4: /* 0x2004 SPR-RAM Data */
      result = m_sprite_bus->read(m_sram_addr);
      break;
    case 5: /* 0x2005 BG Scroll */
      /* XXX: How do we return 16 bits in one byte? */
      break;
    case 6: /* 0x2006 VRAM Address */
      /* XXX: How do we return 16bits in one byte? */
      break;
    case 7: /* 0x2006 VRAM Data */
      result = m_vram_read;
      if (m_v.d < 0x2000)
        m_vram_read = m_ppu_bus->read(m_v.d);
      else {
        m_vram_read = ppu_nt_read(m_v.d);
        if (m_v.d >= 0x3f00) result = ppu_pal_read(m_v.d);
      }
      if (m_reg1.vram_step)
        m_v.d += 32;
      else
        m_v.d++;
      m_v.d %= 0x4000;
      break;
  }
  return result;
}

void NESPPU::ppu_write(offset_t offset, byte_t value) {
  switch (offset % 8) {
    case 0:
      m_reg1.value = value;
      m_t.nt_hselect = bit_isset(value, 0);
      m_t.nt_vselect = bit_isset(value, 1);
      break;
    case 1:
      m_reg2.value = value;
      break;
    case 2:
      /* XXX: Don't write */
      break;
    case 3:
      m_sram_addr = value;
      break;
    case 4:
      m_sprite_bus->write(m_sram_addr, value);
      m_sram_addr = (m_sram_addr + 1) % 256;
      break;
    case 5:
      m_latch = value;
      if (!m_flip_flop) {
        m_t.coarse_x = value >> 3;
        m_tx = value & 0x07;
      } else {
        m_t.fine_y = value & 0x07;
        m_t.coarse_y = value >> 3;
      }
      m_flip_flop = !m_flip_flop;
      break;
    case 6:
      if (!m_flip_flop) {
        m_t.d = ((value & 0x00ff) << 8) | (m_t.d & 0x00ff);
      } else {
        m_t.d = (m_t.d & 0xff00) | value;
        m_v.d = m_t.d;
      }
      m_flip_flop = !m_flip_flop;
      break;
    case 7:
      m_ppu_bus->write(m_v.d, value);
      if (m_reg1.vram_step)
        m_v.d += 32;
      else
        m_v.d++;
      m_v.d %= 0x4000;
      break;
  }
}

byte_t NESPPU::ppu_nt_read(offset_t offset) {
  NameTableMirroring mirror =
      NameTableMirroring(machine()->read_ioport(m_mirror));
  NameTable nt = NameTable((offset & 0x0C00) >> 10);
  offset &= 0x03ff;
  byte_t result = 0;
  switch (nt) {
    case NT0:
      switch (mirror) {
        case SingleScreenBLK1:
          result = m_blk1[offset];
          break;
        case SingleScreenBLK0:
        case TwoScreenVMirroring:
        case TwoScreenHMirroring:
          result = m_blk0[offset];
          break;
      }
      break;
    case NT1:
      switch (mirror) {
        case SingleScreenBLK1:
        case TwoScreenVMirroring:
          result = m_blk1[offset];
          break;
        case SingleScreenBLK0:
        case TwoScreenHMirroring:
          result = m_blk0[offset];
          break;
      }
      break;
    case NT2:
      switch (mirror) {
        case SingleScreenBLK1:
        case TwoScreenHMirroring:
          result = m_blk1[offset];
          break;
        case SingleScreenBLK0:
        case TwoScreenVMirroring:
          result = m_blk0[offset];
          break;
      }
    case NT3:
      switch (mirror) {
        case SingleScreenBLK0:
          result = m_blk0[offset];
          break;
        case SingleScreenBLK1:
        case TwoScreenVMirroring:
        case TwoScreenHMirroring:
          result = m_blk1[offset];
          break;
      }
      break;
  }
  return result;
}

void NESPPU::ppu_nt_write(offset_t offset, byte_t value) {
  NameTableMirroring mirror =
      NameTableMirroring(machine()->read_ioport(m_mirror));
  NameTable nt = NameTable((offset & 0x0C00) >> 10);
  offset &= 0x03ff;
  switch (nt) {
    case NT0:
      m_blk0[offset] = value;
      break;
    case NT1:
      switch (mirror) {
        case SingleScreenBLK1:
        case TwoScreenVMirroring:
          m_blk1[offset] = value;
          break;
        case SingleScreenBLK0:
        case TwoScreenHMirroring:
          m_blk0[offset] = value;
          break;
      }
      break;
    case NT2:
      switch (mirror) {
        case SingleScreenBLK1:
        case TwoScreenHMirroring:
          m_blk1[offset] = value;
          break;
        case SingleScreenBLK0:
        case TwoScreenVMirroring:
          m_blk0[offset] = value;
          break;
      }
    case NT3:
      m_blk1[offset] = value;
      break;
  }
}

byte_t NESPPU::ppu_pal_read(offset_t offset) {
  offset &= 0x001f;
  return m_palette_bytes[offset];
}

byte_t NESPPU::ppu_bus_read(offset_t offset) {
  if ((offset & 0x1f00) == 0x1F00)
    return ppu_pal_read(offset);
  else
    return ppu_nt_read(offset);
}

void NESPPU::ppu_bus_write(offset_t offset, byte_t value) {
  if ((offset & 0x1f00) == 0x1F00)
    ppu_pal_write(offset, value);
  else
    ppu_nt_write(offset, value);
}

void NESPPU::ppu_pal_write(offset_t offset, byte_t value) {
  offset &= 0x001f;
  value &= 0x3f;
  m_palette_bytes[offset] = value;
  m_palette[offset] = m_color_table[value];
  if ((offset & 0x03) == 0) {
    m_palette_bytes[offset ^ 0x10] = value;
    m_palette[offset ^ 0x10] = m_color_table[value];
  }
}
