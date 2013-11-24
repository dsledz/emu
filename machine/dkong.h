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

#include "cpu/z80/z80.h"
#include "machine/i8257.h"

using namespace EMU;
using namespace Z80;

namespace Driver {

class DonkeyKongGfx: public GfxDevice
{
public:
    DonkeyKongGfx(Machine *machine, const std::string &name, unsigned hertz, AddressBus16 *bus);
    ~DonkeyKongGfx(void);

    void init(RomSet *romset);

    void draw_screen(RasterScreen *screen);

    void vmem_write(offset_t offset, uint8_t value);
    uint8_t vmem_read(offset_t offset);
    void palette_write(offset_t offset, uint8_t value);

private:

    void init_sprite(GfxObject<16, 16> *obj, byte_t *b);
    void init_tile(GfxObject<8, 8> *obj, byte_t *b);

    void draw_bg(RasterScreen *screen);
    void draw_sprites(RasterScreen *screen);

    Ram vram;

    AddressBus16 *_bus;

    /* Graphic Data */
    byte_t _palette_select;
    std::array<uint8_t, 256> _palette_index;
    std::array<ColorPalette<4>, 64> _palette;
    GfxObject<8,8> _tiles[256];
    GfxObject<16,16> _sprites[128];
};

typedef std::unique_ptr<DonkeyKongGfx> DonkeyKongGfx_ptr;

class DonkeyKong: public Machine
{
public:
    DonkeyKong(const std::string &rom);
    virtual ~DonkeyKong(void);

private:
    void latch_write(offset_t offset, byte_t value);
    byte_t latch_read(offset_t offset);

    void init_bus(void);
    void init_switches(void);
    void init_controls(void);

    Ram ram;
    Z80Cpu_ptr _main_cpu;
    I8257_ptr _i8257;
    DonkeyKongGfx_ptr _gfx;

    AddressBus16_ptr _bus;

    /* Lines */
    bool _nmi_mask;

    unsigned _hertz;
};

};
