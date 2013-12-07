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

#include "machine/dkong.h"

using namespace EMU;
using namespace Z80;
using namespace Arcade;

DonkeyKongGfx::DonkeyKongGfx(Machine *machine, const std::string &name, unsigned hertz,
        AddressBus16 *bus):
    ScreenDevice(machine, name, hertz, 384, 264, 256, 0, 240, 16),
    vram(machine, "vram", 0x0800),
    m_palette_select(0)
{
}

DonkeyKongGfx::~DonkeyKongGfx(void)
{
}

void
DonkeyKongGfx::palette_write(offset_t offset, uint8_t value)
{
    bit_set(m_palette_select, offset, value != 0);
}

void
DonkeyKongGfx::vmem_write(offset_t offset_t, uint8_t value)
{
    vram.write8(offset_t, value);
}

uint8_t
DonkeyKongGfx::vmem_read(offset_t offset)
{
    return vram.read8(offset);
}

void
DonkeyKongGfx::init_tile(GfxObject<8, 8> *t, byte_t *b)
{
    int p = 0;
    for (int i = 0; i < t->h; i++) {
        int h = i;
        int o = 0x0800;
        t->data[p++] = (bit_isset(b[h], 7)    | (bit_isset(b[h+o], 7)) << 1);
        t->data[p++] = (bit_isset(b[h], 6)    | (bit_isset(b[h+o], 6)) << 1);
        t->data[p++] = (bit_isset(b[h], 5)    | (bit_isset(b[h+o], 5)) << 1);
        t->data[p++] = (bit_isset(b[h], 4)    | (bit_isset(b[h+o], 4)) << 1);
        t->data[p++] = (bit_isset(b[h], 3)    | (bit_isset(b[h+o], 3)) << 1);
        t->data[p++] = (bit_isset(b[h], 2)    | (bit_isset(b[h+o], 2)) << 1);
        t->data[p++] = (bit_isset(b[h], 1)    | (bit_isset(b[h+o], 1)) << 1);
        t->data[p++] = (bit_isset(b[h], 0)    | (bit_isset(b[h+o], 0)) << 1);
    }
}

void
DonkeyKongGfx::init_sprite(GfxObject<16, 16> *s, byte_t *b)
{
    int p = 0;
    for (unsigned y = 0; y < 16; y++) {
        for (int x = 0; x < 2; x++) {
            int h = y + x * 0x0800;
            int o = 0x1000;
            s->data[p++] = (bit_isset(b[h], 7)    | (bit_isset(b[h+o], 7)) << 1);
            s->data[p++] = (bit_isset(b[h], 6)    | (bit_isset(b[h+o], 6)) << 1);
            s->data[p++] = (bit_isset(b[h], 5)    | (bit_isset(b[h+o], 5)) << 1);
            s->data[p++] = (bit_isset(b[h], 4)    | (bit_isset(b[h+o], 4)) << 1);
            s->data[p++] = (bit_isset(b[h], 3)    | (bit_isset(b[h+o], 3)) << 1);
            s->data[p++] = (bit_isset(b[h], 2)    | (bit_isset(b[h+o], 2)) << 1);
            s->data[p++] = (bit_isset(b[h], 1)    | (bit_isset(b[h+o], 1)) << 1);
            s->data[p++] = (bit_isset(b[h], 0)    | (bit_isset(b[h+o], 0)) << 1);
        }
    }
}

RGBColor rgb_palette[256] = {
    [0x00] = RGBColor(0,0,0),
    [0x01] = RGBColor(0,0,255),
    [0x02] = RGBColor(0,0,255),
    [0x03] = RGBColor(0,0,255),
    [0x20] = RGBColor(255,0,0),
    [0x40] = RGBColor(255,0,0),
    [0x60] = RGBColor(255,0,0),
    [0x80] = RGBColor(255,0,0),
    [0xA0] = RGBColor(255,0,0),
    [0xC0] = RGBColor(255,0,0),
    [0xE0] = RGBColor(255,0,0),
    [0x04] = RGBColor(0,255,0),
    [0x08] = RGBColor(0,255,0),
    [0x0C] = RGBColor(0,255,0),
    [0x10] = RGBColor(0,255,0),
    [0x14] = RGBColor(0,255,0),
    [0x18] = RGBColor(0,255,0),
    [0x1C] = RGBColor(0,255,0),
};

/**
 * Color Palette is defined as:
 * RRRGGGBB
 */
static inline RGBColor
convert(uint8_t *b_ptr)
{
    uint8_t b = *b_ptr;
    return RGBColor(
            0x21 * bit_isset(b, 0) +
            0x47 * bit_isset(b, 1) +
            0x97 * bit_isset(b, 2),
            0x21 * bit_isset(b, 3) +
            0x47 * bit_isset(b, 4) +
            0x97 * bit_isset(b, 5),
            0x21 * 0 +
            0x47 * bit_isset(b, 6) +
            0x97 * bit_isset(b, 7));
}

void
DonkeyKongGfx::init(RomSet *romset)
{
    /**
     * Color Palette is defined as:
     * RRRGGGBB (invert)
     */
    Rom * palette_rom = romset->rom("palette");
    for (unsigned i = 0; i < 256; i++) {
        if (i % 4 == 0)
            m_palette[i/4][i%4] = RGBColor(0,0,0);
        else {
            uint8_t idx = (palette_rom->read8(i+ 256) << 4) |
                (palette_rom->read8(i) & 0x0F);
            idx ^= 0xFF;
#if 1
            m_palette[i/4][i%4] = RGBColor(
                RGB_4B(bit_isset(idx, 5), 0, bit_isset(idx, 6), bit_isset(idx, 7)),
                RGB_4B(bit_isset(idx, 2), 0, bit_isset(idx, 3), bit_isset(idx, 4)),
                RGB_3B(bit_isset(idx, 0), bit_isset(idx, 1), 0));
#else
            m_palette[i/4][i%4] = convert(&idx);
#endif
        }
        m_palette_index[i] = palette_rom->read8(i+512) & 0x0f;
    }

    /* Decode characters */
    Rom *gfx1_rom = romset->rom("tiles");
    for (unsigned idx = 0; idx < 256; idx++) {
        byte_t *b = gfx1_rom->direct(idx * 8);
        auto &tile = m_tiles[idx];
        init_tile(&tile, b);
    }

    /* Decode sprites */
    Rom *gfx2 = romset->rom("sprites");
    for (unsigned idx = 0; idx < 128; idx++) {
        byte_t *b = gfx2->direct(idx * 16);
        auto &s = m_sprites[idx];
        init_sprite(&s, b);
    }
}

void
DonkeyKongGfx::do_vblank(void)
{
    m_vblank_cb();
}

void
DonkeyKongGfx::do_vdraw(void)
{
    FrameBuffer *screen = machine()->screen();
    if (!screen)
        return;
    screen->clear();
    draw_bg(screen);
    draw_sprites(screen);
    screen->flip();
}

void
DonkeyKongGfx::draw_sprites(FrameBuffer *screen)
{
    uint8_t *ram = vram.direct(0x000);

    for (unsigned off = 0; off < 0x180; off += 4) {
        int sy = (ram[off] + 0x08) & 0xFF;
        int sx = (ram[off+3] + 0xF8) & 0xFF;
        int idx = ram[off+1] & 0x7f;
        int pen = ram[off+2] & 0x0f;
        int flipx = bit_isset(ram[off+2], 7);
        int flipy = bit_isset(ram[off+1], 7);

        sy += 16;
        sy = 0x100 - sy;
        auto *palette = &m_palette[pen + (m_palette_select << 4)];

        draw_gfx(screen, palette, &m_sprites[idx], sx, sy, flipx, flipy, (*palette)[0]);
    }
}

void
DonkeyKongGfx::draw_bg(FrameBuffer *screen)
{
    byte_t *tile_map = vram.direct(0x400);
    int index = 0;
    for (int ty = 0; ty < 32; ty++) {
        for (int tx = 0; tx < 32; tx++, index++) {
            auto *tile = &m_tiles[tile_map[index]];
            auto *palette = &m_palette[m_palette_index[tx + 32 * (ty / 4)] + (m_palette_select << 4)];
            int sx = tx * tile->w;
            int sy = ty * tile->h - 16;
            draw_gfx(screen, palette, tile, sx, sy, false, false, trans);
        }
    }
}


