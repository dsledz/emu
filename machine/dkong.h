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
#include "emu/screen.h"

#include "cpu/z80/z80.h"
#include "machine/i8257.h"

using namespace EMU;
using namespace Z80;

namespace Arcade {

class DonkeyKongGfx: public ScreenDevice
{
public:
    DonkeyKongGfx(Machine *machine, const std::string &name, unsigned hertz, AddressBus16 *bus);
    ~DonkeyKongGfx(void);

    void init(RomSet *romset);

    void set_vblank_cb(std::function<void (void)> func) {
        m_vblank_cb = func;
    }

    RamDevice &vmem() {
        return vram;
    }

    void vmem_write(offset_t offset, uint8_t value);
    uint8_t vmem_read(offset_t offset);
    void palette_write(offset_t offset, uint8_t value);

private:

    virtual void do_vblank(void);
    virtual void do_vdraw(void);

    void init_sprite(GfxObject<16, 16> *obj, byte_t *b);
    void init_tile(GfxObject<8, 8> *obj, byte_t *b);

    void draw_bg(FrameBuffer *screen);
    void draw_sprites(FrameBuffer *screen);

    RamDevice vram;

    AddressBus16 *m_bus;

    std::function<void (void)> m_vblank_cb;

    /* Graphic Data */
    byte_t m_palette_select;
    std::array<uint8_t, 256> m_palette_index;
    std::array<ColorPalette<4>, 64> m_palette;
    GfxObject<8,8> m_tiles[256];
    GfxObject<16,16> m_sprites[128];
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

    RamDevice m_ram;
    Z80Cpu_ptr m_main_cpu;
    I8257_ptr m_i8257;
    DonkeyKongGfx_ptr m_gfx;

    AddressBus16_ptr m_bus;

    /* Lines */
    bool m_nmi_mask;
};

};
