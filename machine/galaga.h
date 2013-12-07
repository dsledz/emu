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
#include "emu/gfx.h"

#include "machine/namco51.h"
#include "machine/namco06.h"
#include "cpu/z80/z80.h"

using namespace EMU;
using namespace Z80;
using namespace Device;

namespace Arcade {

class GalagaGfx: public ScreenDevice
{
public:
    GalagaGfx(Machine *machine, const std::string &name, unsigned hertz, AddressBus16 *bus);
    ~GalagaGfx(void);

    uint8_t vmem_read(offset_t offset);
    void vmem_write(offset_t offset, uint8_t value);

    void init(RomSet *romset);

protected:
    virtual void do_vdraw(void);

private:

    RamDevice vram;

    void init_sprite(GfxObject<16, 16> *obj, byte_t *b);
    void init_tile(GfxObject<8, 8> *obj, byte_t *b);

    void draw_bg(FrameBuffer *screen);
    void draw_sprites(FrameBuffer *screen);

    AddressBus16 *m_bus;

    /* Graphic Data */
    ColorMap<32, RGBColor> m_colors;
    GfxObject<8,8> m_tiles[128];
    ColorPalette<4> m_tile_palette[64];
    GfxObject<16,16> m_sprites[128];
    ColorPalette<4> m_sprite_palette[64];
};

typedef std::unique_ptr<GalagaGfx> GalagaGfx_ptr;

class Galaga: public Machine
{
public:
    Galaga(const std::string &rom);
    virtual ~Galaga(void);

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

    RamDevice ram1, ram2, ram3;
    Z80Cpu_ptr m_main_cpu;
    Z80Cpu_ptr m_sub_cpu;
    Z80Cpu_ptr m_snd_cpu;
    Namco51_ptr m_namco51;
    Namco06_ptr m_namco06;
    GalagaGfx_ptr m_gfx;

    AddressBus16_ptr m_bus;

    /* Interrupt lines */
    bool m_main_irq;
    bool m_sub_irq;
    bool m_snd_nmi;
};

};
