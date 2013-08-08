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

#include "machine/namco51.h"
#include "machine/namco06.h"
#include "cpu/z80.h"

using namespace EMU;
using namespace Z80;

namespace Driver {

class Galaga: public Machine
{
public:
    Galaga(const std::string &rom);
    virtual ~Galaga(void);

    virtual void execute(Time interval);

private:

    byte_t dips_read(offset_t offset);
    void latch_write(offset_t offset, byte_t value);

    void init_bus(void);
    void init_switches(void);
    void init_controls(void);

    void init_gfx(RomSet *romset);
    void init_sprite(GfxObject<16, 16> *obj, byte_t *b);
    void init_tile(GfxObject<8, 8> *obj, byte_t *b);

    void draw_bg(void);
    void draw_sprites(void);
    void draw_screen(void);

    Ram vram, ram1, ram2, ram3;
    Z80Cpu_ptr _main_cpu;
    Z80Cpu_ptr _sub_cpu;
    Z80Cpu_ptr _snd_cpu;
    Namco51_ptr _namco51;
    Namco06_ptr _namco06;

    AddressBus16_ptr _bus;

    /* Graphics simulation */
    int _scanline;
    Cycles _avail;
    unsigned _hertz;

    /* Interrupt lines */
    bool _main_irq;
    bool _sub_irq;
    bool _snd_nmi;

    /* Graphic Data */
    ColorPalette<32> _palette;
    GfxObject<8,8> _tiles[128];
    ColorPalette<4> _tile_palette[64];
    GfxObject<16,16> _sprites[128];
    ColorPalette<4> _sprite_palette[64];
};

};
