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

#include "machine/galaga.h"

using namespace EMU;
using namespace Z80;
using namespace Arcade;

GalagaGfx::GalagaGfx(Machine *machine, const std::string &name, unsigned hertz, AddressBus16 *bus):
    GfxDevice(machine, name, hertz),
    vram(machine, "vram", 0x0800),
    m_bus(bus)
{

}

GalagaGfx::~GalagaGfx(void)
{
}

uint8_t
GalagaGfx::vmem_read(offset_t offset)
{
    return vram.read8(offset);
}

void
GalagaGfx::vmem_write(offset_t offset, uint8_t value)
{
    vram.write8(offset, value);
}

void
GalagaGfx::init_tile(GfxObject<8, 8> *t, byte_t *b)
{
    int p = 0;
    for (int h = 0; h < t->h; h++) {
        t->data[p++] = (bit_isset(b[8+h], 3)  | (bit_isset(b[8+h], 7)) << 1);
        t->data[p++] = (bit_isset(b[8+h], 2)  | (bit_isset(b[8+h], 6)) << 1);
        t->data[p++] = (bit_isset(b[8+h], 1)  | (bit_isset(b[8+h], 5)) << 1);
        t->data[p++] = (bit_isset(b[8+h], 0)  | (bit_isset(b[8+h], 4)) << 1);
        t->data[p++] = (bit_isset(b[h], 3)    | (bit_isset(b[h], 7)) << 1);
        t->data[p++] = (bit_isset(b[h], 2)    | (bit_isset(b[h], 6)) << 1);
        t->data[p++] = (bit_isset(b[h], 1)    | (bit_isset(b[h], 5)) << 1);
        t->data[p++] = (bit_isset(b[h], 0)    | (bit_isset(b[h], 4)) << 1);
    }
}

void
GalagaGfx::init_sprite(GfxObject<16, 16> *s, byte_t *b)
{
    int p = 0;
    for (unsigned y = 0; y < 16; y++) {
        int h = (y >= 8) ? y + 24 : y;
        s->data[p++] = (bit_isset(b[h], 3)    | (bit_isset(b[h], 7)) << 1);
        s->data[p++] = (bit_isset(b[h], 2)    | (bit_isset(b[h], 6)) << 1);
        s->data[p++] = (bit_isset(b[h], 1)    | (bit_isset(b[h], 5)) << 1);
        s->data[p++] = (bit_isset(b[h], 0)    | (bit_isset(b[h], 4)) << 1);
        s->data[p++] = (bit_isset(b[8+h], 3)  | (bit_isset(b[8+h], 7)) << 1);
        s->data[p++] = (bit_isset(b[8+h], 2)  | (bit_isset(b[8+h], 6)) << 1);
        s->data[p++] = (bit_isset(b[8+h], 1)  | (bit_isset(b[8+h], 5)) << 1);
        s->data[p++] = (bit_isset(b[8+h], 0)  | (bit_isset(b[8+h], 4)) << 1);
        s->data[p++] = (bit_isset(b[16+h], 3) | (bit_isset(b[16+h], 7)) << 1);
        s->data[p++] = (bit_isset(b[16+h], 2) | (bit_isset(b[16+h], 6)) << 1);
        s->data[p++] = (bit_isset(b[16+h], 1) | (bit_isset(b[16+h], 5)) << 1);
        s->data[p++] = (bit_isset(b[16+h], 0) | (bit_isset(b[16+h], 4)) << 1);
        s->data[p++] = (bit_isset(b[24+h], 3) | (bit_isset(b[24+h], 7)) << 1);
        s->data[p++] = (bit_isset(b[24+h], 2) | (bit_isset(b[24+h], 6)) << 1);
        s->data[p++] = (bit_isset(b[24+h], 1) | (bit_isset(b[24+h], 5)) << 1);
        s->data[p++] = (bit_isset(b[24+h], 0) | (bit_isset(b[24+h], 4)) << 1);
    }
}

/**
 * Color Palette is defined as:
 * RRRGGGBB
 */
static RGBColor
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
GalagaGfx::init(RomSet *romset)
{
    m_colors.init(romset->rom("prom1")->direct(0x00), convert);

    /* XXX: Characters Palette */
    byte_t * char_rom = romset->rom("prom2")->direct(0x00);
    for (unsigned i = 0; i < 64*4; i+=4) {
        auto *palette = &m_tile_palette[i/4];
        (*palette)[0] = m_colors[(char_rom[i+0] & 0x0f) + 0x10];
        (*palette)[1] = m_colors[(char_rom[i+1] & 0x0f) + 0x10];
        (*palette)[2] = m_colors[(char_rom[i+2] & 0x0f) + 0x10];
        (*palette)[3] = m_colors[(char_rom[i+3] & 0x0f) + 0x10];
    }

    /* Decode characters */
    Rom *gfx1_rom = romset->rom("tiles");
    for (unsigned idx = 0; idx < 128; idx++) {
        byte_t *b = gfx1_rom->direct(idx * 16);
        auto &tile = m_tiles[idx];
        init_tile(&tile, b);
    }

    /* XXX: Sprites Palette */
    byte_t * sprite_rom = romset->rom("prom2")->direct(0x0100);
    for (unsigned i = 0; i < 64*4; i+=4) {
        auto *palette = &m_sprite_palette[i/4];
        (*palette)[0] = m_colors[sprite_rom[i+0] & 0x0f];
        (*palette)[1] = m_colors[sprite_rom[i+1] & 0x0f];
        (*palette)[2] = m_colors[sprite_rom[i+2] & 0x0f];
        (*palette)[3] = m_colors[sprite_rom[i+3] & 0x0f];
    }

    /* Decode sprites */
    Rom *gfx2 = romset->rom("sprites");
    for (unsigned idx = 0; idx < 128; idx++) {
        byte_t *b = gfx2->direct(idx * 64);
        auto &s = m_sprites[idx];
        init_sprite(&s, b);
    }
}

void
GalagaGfx::draw_screen(RasterScreen *screen)
{
    screen->clear();

    draw_sprites(screen);
    draw_bg(screen);

    screen->flip();
}

void
GalagaGfx::draw_sprites(RasterScreen *screen)
{
    /**
     * Each sprite is described by 6 bytes:
     * 1 - index into sprite table
     * 2 - palette
     * 3 - y position
     * 4 - x position
     * 5 - flags
     * 6 - Upper bits of x position.
     */
    for (unsigned off = 0; off < 0x80; off += 2) {
        uint8_t b[6];
        b[0] = m_bus->read(0x8B80+off);
        b[1] = m_bus->read(0x8B81+off);
        b[2] = m_bus->read(0x9380+off);
        b[3] = m_bus->read(0x9381+off);
        b[4] = m_bus->read(0x9B80+off);
        b[5] = m_bus->read(0x9B81+off);
        int idx = b[0] & 0x7f;
        int pen = b[1] & 0x3f;
        int sy = 256 - b[2] + 1;
        int sx = b[3] - 40 + 0x100 * (b[5] & 3);
        bool flipx = bit_isset(b[4], 0);
        bool flipy = bit_isset(b[4], 1);
        int sizex = bit_isset(b[4], 2);
        int sizey = bit_isset(b[4], 3);

        /* Adjust the y position */
        sy -= 16 * sizey;
        sy = (sy & 0xff) - 32;

        for (int y = 0; y <= sizey; y++) {
            for (int x = 0; x <= sizex; x++) {
                auto *sprite = &m_sprites[idx];
                auto *palette = &m_sprite_palette[pen];
                draw_gfx(screen, palette,
                    sprite + (y ^ (sizey & flipy)) * 2 + (x ^ (sizex & flipx)),
                    sx + (x * 16), sy + (y * 16),
                    flipx, flipy, m_colors[0xf]);
            }
        }
    }
}

void
GalagaGfx::draw_bg(RasterScreen *screen)
{
    /* Render the tilemap */
    byte_t *tile_map = vram.direct(0x000);
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
            auto *tile = &m_tiles[tile_map[index] & 0x7f];
            auto *palette = &m_tile_palette[tile_map[index + 0x400] & 0x3f];
            int sx = tx * tile->w;
            int sy = ty * tile->h;
            draw_gfx(screen, palette, tile, sx, sy,
                     false, false, m_colors[0x1f]);
        }
    }
}
