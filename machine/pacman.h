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

#include "cpu/z80.h"

using namespace EMU;
using namespace Z80;

namespace Driver {

class Pacman: public Machine
{
public:
    Pacman(const std::string &rom);
    virtual ~Pacman(void);

    virtual void execute(Time interval);

private:

    byte_t rom_read(offset_t offset) {
        return _rom->read8(offset);
    }
    void rom_write(offset_t offset, byte_t value) {
    }

    byte_t vram_read(offset_t offset) {
        return _vram.read8(offset);
    }
    void vram_write(offset_t offset, byte_t value) {
        _vram.write8(offset, value);
    }

    byte_t cram_read(offset_t offset) {
        return _cram.read8(offset);
    }
    void cram_write(offset_t offset, byte_t value) {
        _cram.write8(offset, value);
    }

    byte_t ram_read(offset_t offset) {
        return _ram.read8(offset);
    }
    void ram_write(offset_t offset, byte_t value) {
        if (offset == 0x2f7 && value == 0xff) {
            std::cout << "Sound write " << Hex(value) << std::endl;
        }
        _ram.write8(offset, value);
    }

    byte_t spr_read(offset_t offset) {
        if (offset & 0x01) {
            return _spr[offset >> 1].color;
        } else {
            return _spr[offset >> 1].flags;
        }
    }
    void spr_write(offset_t offset, byte_t value) {
        if (offset & 0x01) {
            _spr[offset >> 1].color = value;
        } else {
            _spr[offset >> 1].flags = value;
        }
    }

    byte_t in0_read(offset_t offset) {
        return read_ioport("IN0");
    }
    void latch_write(offset_t offset, byte_t value);

    void sound_write(offset_t offset, byte_t value) {
    }

    byte_t in1_read(offset_t offset) {
        return read_ioport("IN1");
    }

    byte_t spr_coord_read(offset_t offset) {
        if (offset & 0x01) {
            return _spr[offset >> 1].x;
        } else {
            return _spr[offset >> 1].y;
        }
    }
    void spr_coord_write(offset_t offset, byte_t value) {
        if (offset & 0x01) {
            _spr[offset >> 1].x = value;
        } else {
            _spr[offset >> 1].y = value;
        }
    }

    byte_t dsw1_read(offset_t offset) {
        return read_ioport("DSW1");
    }
    void dsw1_write(offset_t offset, byte_t value) {
    }

    byte_t dsw2_read(offset_t offset) {
        return read_ioport("DSW2");
    }
    void watchdog_write(offset_t offset, byte_t value) {
    }

    byte_t io_read(offset_t offset);
    void io_write(offset_t offset, byte_t value);

    void init_gfx(void);
    void init_sprite(GfxObject<16, 16> *obj, byte_t *b);
    void init_tile(GfxObject<8, 8> *obj, byte_t *b);
    void init_bus(void);
    void init_switches(void);
    void init_controls(void);

    void draw_bg(void);
    void draw_sprites(void);
    void draw_screen(void);

    unsigned _scanline;
    Cycles _avail;
    unsigned _hertz;

    Z80Cpu_ptr _cpu;
    AddressBus16_ptr _bus;
    Ram _vram, _cram, _ram;
    std::unique_ptr<RomSet> _roms;
    Rom *_rom;

    /* latches */
    bool _irq_mask;

    /* Graphics */
    struct Sprite {
        union {
            struct {
                byte_t xflip:1;
                byte_t yflip:1;
                byte_t idx:6;
            };
            byte_t flags;
        };
        byte_t color;
        byte_t x;
        byte_t y;
    };

    Sprite _spr[8];

    ColorPalette<32>  _palette;
    GfxObject<8, 8>   _tiles[256];
    ColorPalette<4>   _palettes[128];
    GfxObject<16, 16> _sprites[64];
};

};
