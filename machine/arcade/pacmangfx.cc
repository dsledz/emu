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

#include "machine/arcade/pacman.h"

using namespace EMU;
using namespace Z80;
using namespace Arcade;

PacmanGfx::PacmanGfx(Machine *machine, const std::string &name,
                     ClockDivider divider, AddressBus16x8 *bus)
    : ScreenDevice(machine, name, divider, 384, 264, 256, 0, 240, 16),
      m_vram(machine, "vram", 0x400),
      m_cram(machine, "cram", 0x400) {}

PacmanGfx::~PacmanGfx(void) {}

void PacmanGfx::init_tile(GfxObject<8, 8> *t, byte_t *b) {
  int p = 0;
  for (int h = 0; h < t->h; h++) {
    t->data[p++] = (bit_isset(b[8 + h], 3) | (bit_isset(b[8 + h], 7)) << 1);
    t->data[p++] = (bit_isset(b[8 + h], 2) | (bit_isset(b[8 + h], 6)) << 1);
    t->data[p++] = (bit_isset(b[8 + h], 1) | (bit_isset(b[8 + h], 5)) << 1);
    t->data[p++] = (bit_isset(b[8 + h], 0) | (bit_isset(b[8 + h], 4)) << 1);
    t->data[p++] = (bit_isset(b[h], 3) | (bit_isset(b[h], 7)) << 1);
    t->data[p++] = (bit_isset(b[h], 2) | (bit_isset(b[h], 6)) << 1);
    t->data[p++] = (bit_isset(b[h], 1) | (bit_isset(b[h], 5)) << 1);
    t->data[p++] = (bit_isset(b[h], 0) | (bit_isset(b[h], 4)) << 1);
  }
}

void PacmanGfx::init_sprite(GfxObject<16, 16> *s, byte_t *b) {
  int p = 0;
  for (unsigned y = 0; y < 16; y++) {
    int h = (y >= 8) ? y + 24 : y;
    s->data[p++] = (bit_isset(b[8 + h], 3) | (bit_isset(b[8 + h], 7)) << 1);
    s->data[p++] = (bit_isset(b[8 + h], 2) | (bit_isset(b[8 + h], 6)) << 1);
    s->data[p++] = (bit_isset(b[8 + h], 1) | (bit_isset(b[8 + h], 5)) << 1);
    s->data[p++] = (bit_isset(b[8 + h], 0) | (bit_isset(b[8 + h], 4)) << 1);
    s->data[p++] = (bit_isset(b[16 + h], 3) | (bit_isset(b[16 + h], 7)) << 1);
    s->data[p++] = (bit_isset(b[16 + h], 2) | (bit_isset(b[16 + h], 6)) << 1);
    s->data[p++] = (bit_isset(b[16 + h], 1) | (bit_isset(b[16 + h], 5)) << 1);
    s->data[p++] = (bit_isset(b[16 + h], 0) | (bit_isset(b[16 + h], 4)) << 1);
    s->data[p++] = (bit_isset(b[24 + h], 3) | (bit_isset(b[24 + h], 7)) << 1);
    s->data[p++] = (bit_isset(b[24 + h], 2) | (bit_isset(b[24 + h], 6)) << 1);
    s->data[p++] = (bit_isset(b[24 + h], 1) | (bit_isset(b[24 + h], 5)) << 1);
    s->data[p++] = (bit_isset(b[24 + h], 0) | (bit_isset(b[24 + h], 4)) << 1);
    s->data[p++] = (bit_isset(b[h], 3) | (bit_isset(b[h], 7)) << 1);
    s->data[p++] = (bit_isset(b[h], 2) | (bit_isset(b[h], 6)) << 1);
    s->data[p++] = (bit_isset(b[h], 1) | (bit_isset(b[h], 5)) << 1);
    s->data[p++] = (bit_isset(b[h], 0) | (bit_isset(b[h], 4)) << 1);
  }
}

/**
 * Color Palette is defined as:
 * RRRGGGBB
 */
static RGBColor convert(uint8_t *b_ptr) {
  uint8_t b = *b_ptr;
  return RGBColor(
      0x21 * bit_isset(b, 0) + 0x47 * bit_isset(b, 1) + 0x97 * bit_isset(b, 2),
      0x21 * bit_isset(b, 3) + 0x47 * bit_isset(b, 4) + 0x97 * bit_isset(b, 5),
      0x21 * 0 + 0x47 * bit_isset(b, 6) + 0x97 * bit_isset(b, 7));
}

void PacmanGfx::init(RomSet *romset) {
  m_colors.init(romset->rom("proms")->direct(0x00), convert);

  /* XXX: 256 bytes of color table */

  byte_t *char_rom = romset->rom("proms")->direct(0x20);

  for (unsigned i = 0; i < 256; i += 4) {
    auto *palette = &m_palettes[i / 4];
    (*palette)[0] = m_colors[(char_rom[i + 0] & 0x0f)];
    (*palette)[1] = m_colors[(char_rom[i + 1] & 0x0f)];
    (*palette)[2] = m_colors[(char_rom[i + 2] & 0x0f)];
    (*palette)[3] = m_colors[(char_rom[i + 3] & 0x0f)];
  }

  /* Gfx */
  /* Decode characters */
  Rom *gfx1 = romset->rom("gfx1");
  for (unsigned idx = 0; idx < 256; idx++) {
    byte_t *b = gfx1->direct(idx * 16);
    auto &tile = m_tiles[idx];
    init_tile(&tile, b);
  }

  /* Decode sprites */
  for (unsigned idx = 0; idx < 64; idx++) {
    byte_t *b = gfx1->direct(0x1000 + idx * 64);
    auto &s = m_sprites[idx];
    init_sprite(&s, b);
  }
}

void PacmanGfx::do_vdraw(void) {
  FrameBuffer *screen = machine()->screen();

  screen->clear();
  draw_bg(screen);
  draw_sprites(screen);
  screen->flip();
}

void PacmanGfx::do_vblank(void) { m_vblank_cb(); }

void PacmanGfx::draw_sprites(FrameBuffer *screen) {
  for (int off = 0; off < 8; off++) {
    int idx = m_spr[off].idx;
    int pen = m_spr[off].color & 0x1f;
    int sy = m_spr[off].y - 31;
    int sx = 272 - m_spr[off].x;
    bool flipx = m_spr[off].xflip;
    bool flipy = m_spr[off].yflip;

    auto *sprite = &m_sprites[idx];
    auto *palette = &m_palettes[pen];
    draw_gfx(screen, palette, sprite, sx, sy, flipx, flipy, m_colors[0x1f]);
    DEVICE_DEBUG("Drawing sprite: ", Hex(off), "(", Hex(sx), ",", Hex(sy),
                 "): ", "Sprite: ", Hex(idx), " Color: ", Hex(pen));
  }
}

void PacmanGfx::draw_bg(FrameBuffer *screen) {
  /* Render the tilemap */
  byte_t *tile_map = m_vram.direct(0x000);
  byte_t *color_map = m_cram.direct(0x000);
  for (int tx = 0; tx < 36; tx++) {
    for (int ty = 0; ty < 28; ty++) {
      int index = 0;
      {
        int row = ty + 2;
        int col = tx - 2;
        if (col & 0x20)
          index = row + ((col & 0x1f) << 5);
        else
          index = col + (row << 5);
      }
      /* XXX: Account for visible */
      auto *tile = &m_tiles[tile_map[index]];
      auto *palette = &m_palettes[color_map[index] & 0x1f];
      int sx = tx * tile->w;
      int sy = ty * tile->h;
      draw_gfx(screen, palette, tile, sx, sy, false, false, m_colors[0x1f]);
    }
  }
}
