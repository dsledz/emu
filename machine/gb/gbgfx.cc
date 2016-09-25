/**
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

#include "machine/gb/gbgfx.h"

using namespace GBMachine;

GBGraphics::GBGraphics(Gameboy *gameboy, ClockDivider divider)
    : ClockedDevice(gameboy, gameboy->clock(), "gfx", divider),
      m_vram(gameboy, "vram", 0x2000),
      m_oam(gameboy, "oam", 0x100),
      m_lcdc(0) {
  m_bus = gameboy->bus();

  m_global_pal[0] = RGBColor(0xff, 0xff, 0xff);
  m_global_pal[1] = RGBColor(0xbb, 0xbb, 0xbb);
  m_global_pal[2] = RGBColor(0x66, 0x66, 0x66);
  m_global_pal[3] = RGBColor(0x00, 0x00, 0x00);

  m_bus->add(0x8000, m_vram.direct(0), 0x2000,
             [&](offset_t offset, byte_t value) {
               if (offset < 0x1800) {
                 auto *o = &m_objs[offset / 16];
                 o->dirty = true;
               }
               m_vram.write8(offset, value);
             });
  m_bus->add(0xFE00, 0xFEFF, READ_CB(RamDevice::read8, &m_oam),
             WRITE_CB(RamDevice::write8, &m_oam));

  m_bus->add(VideoReg::LCDC, &m_lcdc);
  m_bus->add(VideoReg::STAT, &m_stat);
  m_bus->add(VideoReg::SCY, &m_scy);
  m_bus->add(VideoReg::SCX, &m_scx);
  m_bus->add(VideoReg::LY, &m_ly);
  m_bus->add(GBReg::DMA, AddressBus16x8::DefaultRead(),
             [&](offset_t offset, byte_t arg) {
               addr_t src_addr = (addr_t)arg << 8;
               for (unsigned i = 0; i < 160; i++)
                 m_oam.write8(i, m_bus->read(src_addr + i));
             });
  m_bus->add(VideoReg::LYC, &m_lyc);
  m_bus->add(VideoReg::BGP, AddressBus16x8::DataRead(&m_bgp),
             WRITE_CB(GBGraphics::palette_write, this, &m_bg_pal, &m_bgp));
  m_bus->add(VideoReg::OBP0, AddressBus16x8::DataRead(&m_obp0),
             WRITE_CB(GBGraphics::palette_write, this, &m_obj0_pal, &m_obp0));
  m_bus->add(VideoReg::OBP1, AddressBus16x8::DataRead(&m_obp1),
             WRITE_CB(GBGraphics::palette_write, this, &m_obj1_pal, &m_obp1));
  m_bus->add(VideoReg::WY, &m_wy);
  m_bus->add(VideoReg::WX, &m_wx);

  m_tilemap0 = [&](int idx) -> GfxObject<8, 8> * {
    char c = m_vram.read8(VMem::TileMap0 + idx);
    return get_obj(c + 256);
  };
  m_tilemap1 = [&](int idx) -> GfxObject<8, 8> * {
    char c = m_vram.read8(VMem::TileMap1 + idx);
    return get_obj(c + 256);
  };

  for (unsigned i = 0; i < 384; i++) m_objs[i].dirty = true;
}

GBGraphics::~GBGraphics(void) {}

void GBGraphics::palette_write(ColorPalette<4> *pal, byte_t *pal_byte,
                               offset_t offset, byte_t value) {
  (*pal)[0] = trans;
  (*pal)[1] = m_global_pal[(value & 0x0C) >> 2];
  (*pal)[2] = m_global_pal[(value & 0x30) >> 4];
  (*pal)[3] = m_global_pal[(value & 0xC0) >> 6];
  *pal_byte = value;
}

GfxObject<8, 8> *GBGraphics::get_obj(int idx) {
  auto *o = &m_objs[idx];
  if (o->dirty) {
    byte_t *b = m_vram.direct(VMem::ObjTiles + idx * 16);
    int p = 0;
    for (int y = 0; y < o->h; y++) {
      o->data[p++] = (bit_isset(b[y * 2], 7) | bit_isset(b[y * 2 + 1], 7) << 1);
      o->data[p++] = (bit_isset(b[y * 2], 6) | bit_isset(b[y * 2 + 1], 6) << 1);
      o->data[p++] = (bit_isset(b[y * 2], 5) | bit_isset(b[y * 2 + 1], 5) << 1);
      o->data[p++] = (bit_isset(b[y * 2], 4) | bit_isset(b[y * 2 + 1], 4) << 1);
      o->data[p++] = (bit_isset(b[y * 2], 3) | bit_isset(b[y * 2 + 1], 3) << 1);
      o->data[p++] = (bit_isset(b[y * 2], 2) | bit_isset(b[y * 2 + 1], 2) << 1);
      o->data[p++] = (bit_isset(b[y * 2], 1) | bit_isset(b[y * 2 + 1], 1) << 1);
      o->data[p++] = (bit_isset(b[y * 2], 0) | bit_isset(b[y * 2 + 1], 0) << 1);
    }
    o->dirty = false;
  }
  return o;
}

void GBGraphics::draw_scanline(int y) {
  FrameBuffer *screen = machine()->screen();
  if (bit_isset(m_lcdc, LCDCBits::BGDisplay)) {
    auto cb = bit_isset(m_lcdc, LCDCBits::BGTileMap) ? m_tilemap1 : m_tilemap0;
    int iy = ((m_scy + y) % 256);
    for (int x = 0; x < 160; x++) {
      int ix = ((m_scx + x) % 256);
      int idx = (iy / 8) * 32 + ix / 8;
      auto *obj = cb(idx);
      RGBColor pen = m_bg_pal[obj->at(ix % 8, iy % 8)];
      screen->set(x, y, pen);
    }
  }

  if (bit_isset(m_lcdc, LCDCBits::WindowDisplay) && y >= m_wy) {
    auto cb =
        bit_isset(m_lcdc, LCDCBits::WindowTileMap) ? m_tilemap1 : m_tilemap0;
    int iy = (y - m_wy);
    for (int x = 0; x < 160; x++) {
      // Find the correct bg tile.
      int idx = (iy / 8) * 32 + x / 8;
      auto *obj = cb(idx);
      // Find the correct pen.
      RGBColor pen = m_bg_pal[obj->at(x % 8, iy % 8)];
      screen->set(m_wx + x, y, pen);
    }
  }

  int sprites = 0;
  int height = bit_isset(m_lcdc, LCDCBits::OBJSize) ? 16 : 8;
  for (unsigned i = 160; i >= 4 && sprites < 10; i -= 4) {
    byte_t *b = m_oam.direct(i - 4);
    int iy = b[Oam::OamY] - 8;
    int ix = b[Oam::OamX] - 8;
    int idx = b[Oam::OamPattern];
    int flags = b[Oam::OamFlags];

    iy -= y;
    if (iy < 0 || iy >= height) continue;

    sprites++;

    if (!bit_isset(flags, OAMFlags::SpriteFlipY)) iy = height - 1 - iy;

    if (iy >= 8) {
      iy -= 8;
      idx++;
    }

    auto *obj = get_obj(idx);
    auto *pen = &m_obj0_pal;
    if (bit_isset(flags, OAMFlags::SpritePalette)) pen = &m_obj1_pal;

    /* XXX: Handle SpritePriority */
    if (bit_isset(flags, OAMFlags::SpriteFlipX)) {
      for (int x = 0; x < 8; x++) {
        RGBColor color = (*pen)[obj->at(7 - x, iy)];
        if (color != trans && (!bit_isset(flags, OAMFlags::SpritePriority) ||
                               screen->get(ix + x, y) == m_global_pal[0]))
          screen->set(ix + x, y, color);
      }
    } else {
      for (int x = 0; x < 8; x++) {
        RGBColor color = (*pen)[obj->at(x, iy)];
        if (color != trans && (!bit_isset(flags, OAMFlags::SpritePriority) ||
                               screen->get(ix + x, y) == m_global_pal[0]))
          screen->set(ix + x, y, color);
      }
    }
  }
}

void GBGraphics::execute(void) {
  unsigned delta = 64;
  add_icycles(Cycles(delta));
  m_fcycles += delta;

  switch (m_stat & 0x03) {
    case LCDMode::HBlankMode:
      if (m_fcycles > H_BLANK_CYCLES) {
        // Transition to VBlank or OAM
        m_fcycles -= H_BLANK_CYCLES;

        m_ly = (m_ly + 1) % SCANLINES;
        if (m_ly == DISPLAY_LINES) {
          // Wait until our frame is finished
          machine()->screen()->flip();
          machine()->set_line("cpu", make_irq_line(GBInterrupt::VBlank),
                              LineState::Pulse);
          m_stat = (m_stat & 0xfc) | LCDMode::VBlankMode;
          if (bit_isset(m_stat, STATBits::Mode01Int))
            machine()->set_line("cpu", make_irq_line(GBInterrupt::LCDStat),
                                LineState::Pulse);
        } else {
          if (m_ly == 0) machine()->screen()->clear();
          if (m_ly < DISPLAY_LINES) draw_scanline(m_ly);
          m_stat = (m_stat & 0xfc) | LCDMode::OAMMode;
          if (bit_isset(m_stat, STATBits::Mode10Int))
            machine()->set_line("cpu", make_irq_line(GBInterrupt::LCDStat),
                                LineState::Pulse);
        }
        // See if we need to trigger the lcd interrupt.
        if (bit_isset(m_stat, STATBits::LYCInterrupt) &&
            !(bit_isset(m_stat, STATBits::Coincidence) ^ (m_lyc == m_ly)))
          machine()->set_line("cpu", make_irq_line(GBInterrupt::LCDStat),
                              LineState::Pulse);
      }
      break;
    case LCDMode::OAMMode:
      if (m_fcycles > OAM_CYCLES) {
        // Transition to Active
        m_fcycles -= OAM_CYCLES;
        m_stat = (m_stat & 0xfc) | LCDMode::ActiveMode;
      }
      break;
    case LCDMode::ActiveMode:
      if (m_fcycles > ACTIVE_CYCLES) {
        // Transition to HBlank
        m_fcycles -= ACTIVE_CYCLES;
        m_stat = (m_stat & 0xfc) | LCDMode::HBlankMode;
        if (bit_isset(m_stat, STATBits::Mode00Int))
          machine()->set_line("cpu", make_irq_line(GBInterrupt::LCDStat),
                              LineState::Pulse);
      }
      break;
    case LCDMode::VBlankMode:
      if (m_fcycles > V_BLANK_CYCLES) {
        // Transition to OAM
        m_fcycles -= V_BLANK_CYCLES;
        m_stat = (m_stat & 0xfc) | LCDMode::OAMMode;
        if (bit_isset(m_stat, STATBits::Mode10Int))
          machine()->set_line("cpu", make_irq_line(GBInterrupt::LCDStat),
                              LineState::Pulse);
      }
      break;
  }
}
