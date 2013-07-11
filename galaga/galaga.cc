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

#include "emu.h"

#include "galaga.h"

#include "z80.h"
#include "namco06.h"
#include "namco51.h"

using namespace EMU;
using namespace Z80;
using namespace Driver;

class Latches: public Device
{
public:
    Latches(Machine *machine, Device *main, Device *sub, Device *snd):
        Device(machine, "latches"),
        _main_cpu(main), _main_irq(false),
        _sub_cpu(sub), _sub_irq(false),
        _snd_cpu(snd), _snd_nmi(false),
        _snd_timer(new Timer( [&]() {
                if (_snd_nmi)
                    _snd_cpu->set_line(InputLine::NMI, LineState::Pulse);
            }, Time(usec(16501/2)))),
        _vblank_timer(new Timer([&]() {
                if (_main_irq)
                    _main_cpu->set_line(InputLine::INT0, LineState::Assert);
                if (_sub_irq)
                    _sub_cpu->set_line(InputLine::INT0, LineState::Assert);
            }, Time(usec(16501))))
    {
        _machine->add_timer(_vblank_timer);
        _machine->add_timer(_snd_timer);
    }
    ~Latches(void) {
    }

    virtual void save(SaveState &state) {
    }
    virtual void load(LoadState &state) {
    }
    virtual void tick(unsigned cycles) {
    }
    virtual void set_line(InputLine line, LineState state) {
    }
    virtual void write(addr_t addr, byte_t value) {
        switch (addr) {
        case 0x6820:
            _main_irq = bit_isset(value, 0);
            if (!_main_irq)
                _main_cpu->set_line(InputLine::INT0, LineState::Clear);
            break;
        case 0x6821:
            _sub_irq = bit_isset(value, 0);
            if (!_sub_irq)
                _sub_cpu->set_line(InputLine::INT0, LineState::Clear);
            break;
        case 0x6822:
            _snd_nmi = !bit_isset(value, 0);
            break;
        case 0x6823:
            _sub_cpu->set_line(InputLine::RESET, LineState::Pulse);
            _snd_cpu->set_line(InputLine::RESET, LineState::Pulse);
            break;
        default:
            break;
        }
    }
    virtual byte_t read(addr_t addr) {
        return 0;
    }
private:
    Device *_main_cpu;
    bool _main_irq;
    Device *_sub_cpu;
    bool _sub_irq;
    Device *_snd_cpu;
    bool _snd_nmi;
    Timer_ptr _snd_timer;
    Timer_ptr _vblank_timer;
};

Galaga::Galaga(void):
    Machine(18432000),
    _romset("galaga"),
    _screen(288, 384),
    vram(0x0800),
    ram1(0x0400),
    ram2(0x0400),
    ram3(0x0400)
{
    Z80Cpu *main_cpu = new Z80Cpu(this, "maincpu", 6);
    Z80Cpu *sub_cpu = new Z80Cpu(this, "subcpu", 6);
    Z80Cpu *snd_cpu = new Z80Cpu(this, "sndcpu", 6);

    main_cpu->bus()->add_port(0x0000, _romset.rom("gg1-1b.3p"));
    main_cpu->bus()->add_port(0x1000, _romset.rom("gg1-2b.3m"));
    main_cpu->bus()->add_port(0x2000, _romset.rom("gg1-3.2m"));
    main_cpu->bus()->add_port(0x3000, _romset.rom("gg1-4b.2l"));

    sub_cpu->bus()->add_port(0x0000, _romset.rom("gg1-5b.3f"));
    sub_cpu->bus()->add_port(0x1000, 0xF000, IOPort());

    snd_cpu->bus()->add_port(0x0000, _romset.rom("gg1-7b.2c"));

    Namco06 *namco06 = new Namco06(this, dev("maincpu"));
    Namco51 *namco51 = new Namco51(this, namco06);
    namco06->add_child(0, namco51);
    new Latches(this, dev("maincpu"), dev("subcpu"), dev("sndcpu"));

    common_bus(main_cpu->bus());
    common_bus(sub_cpu->bus());
    common_bus(snd_cpu->bus());

    init_gfx();

    add_timer(Time(usec(16501)), [&]() {
        render();
        if (_render_cb)
            _render_cb(&_screen);
        },
        Time(usec(16501)));
}

Galaga::~Galaga(void)
{
}

void
Galaga::common_bus(AddressBus *bus)
{
    bus->add_port(0x8000, &vram);
    bus->add_port(0x8800, &ram1);
    bus->add_port(0x9000, &ram2);
    bus->add_port(0x9800, &ram3);
    /* XXX: Sound */
    /// bus->add_port(0x6800, 0xFFE0, IOPort());
    /* XXX: Latches */
    bus->add_port(0x6800, IOPort());
    bus->add_port(0x6801, IOPort());
    bus->add_port(0x6802, IOPort(
            [=](addr_t addr) { return 0x02; },
            DefaultWrite()));
    bus->add_port(0x6805, IOPort());
    bus->add_port(0x6803, IOPort());
    bus->add_port(0x6804, IOPort(
            [=](addr_t addr) { return 0x02; },
            DefaultWrite()));
    bus->add_port(0x6805, IOPort());
    bus->add_port(0x6806, IOPort());
    bus->add_port(0x6807, IOPort());
    bus->add_port(0x6808, IOPort());
    bus->add_port(0x6809, IOPort());
    bus->add_port(0x680A, IOPort());
    bus->add_port(0x680B, IOPort());
    bus->add_port(0x680C, IOPort());
    bus->add_port(0x680D, IOPort());
    bus->add_port(0x680E, IOPort());
    bus->add_port(0x680F, IOPort());
    bus->add_port(0x6810, IOPort());
    bus->add_port(0x6811, IOPort());
    bus->add_port(0x6812, IOPort());
    bus->add_port(0x6813, IOPort());
    bus->add_port(0x6814, IOPort());
    bus->add_port(0x6815, IOPort());
    bus->add_port(0x6816, IOPort());
    bus->add_port(0x6817, IOPort());
    bus->add_port(0x6818, IOPort());
    bus->add_port(0x6819, IOPort());
    bus->add_port(0x681A, IOPort());
    bus->add_port(0x681B, IOPort());
    bus->add_port(0x681C, IOPort());
    bus->add_port(0x681D, IOPort());
    bus->add_port(0x681E, IOPort());
    bus->add_port(0x681F, IOPort());
    bus->add_port(0x6820, 0xFFF8, IOPort(dev("latches")));
    /* XXX: Watchdog */
    bus->add_port(0x6830, IOPort());
    /* XXX: 06xx chip */
    bus->add_port(0x7000, 0xFF00, IOPort(dev("06xx")));
    bus->add_port(0x7100, 0xFFFF, IOPort(dev("06xx")));
    /* XXX: Star control */
    bus->add_port(0xa000, 0xFFF8, IOPort());
}

void
Galaga::init_gfx(void)
{
    Rom * palette_rom = _romset.rom("prom-5.5n");
    // Rom * sprite_rom = _romset.rom("prom-3.1c");

    /**
     * Color Palette is defined as:
     * RRRGGGBB
     */
    for (unsigned i = 0; i < 32; i++) {
        byte_t b = palette_rom->read8(i);
        _palette[i] = RGBColor(
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

    /* XXX: Characters Palette */
    byte_t * char_rom = _romset.rom("prom-4.2n")->direct(0);
    for (unsigned i = 0; i < 64*4; i+=4) {
        auto *palette = &_tile_palette[i/4];
        (*palette)[0] = _palette[char_rom[i+0] & 0x1f];
        (*palette)[1] = _palette[char_rom[i+1] & 0x1f];
        (*palette)[2] = _palette[char_rom[i+2] & 0x1f];
        (*palette)[3] = _palette[char_rom[i+3] & 0x1f];
    }

    /* XXX: Sprites Palette */
    byte_t * sprite_rom = _romset.rom("prom-3.1c")->direct(0);
    for (unsigned i = 0; i < 64*4; i+=4) {
        auto *palette = &_tile_palette[i/4];
        (*palette)[0] = _palette[sprite_rom[i+0] & 0x1f];
        (*palette)[1] = _palette[sprite_rom[i+1] & 0x1f];
        (*palette)[2] = _palette[sprite_rom[i+2] & 0x1f];
        (*palette)[3] = _palette[sprite_rom[i+3] & 0x1f];
    }

    /* Decode characters */
    Rom *gfx1_rom = _romset.rom("gg1-9.4l");
    for (unsigned idx = 0; idx < 128; idx++) {
        byte_t *b = gfx1_rom->direct(idx * 16);
        auto &tile = _tiles[idx];
        /**
         * Tile layout
         * 64+68 65+69 66+70 67+71 00+04 01+05 02+06 03+07
         * & & & & & & & &
         * & & & & & & & &
         * & & & & & & & &
         * & & & & & & & &
         * & & & & & & & &
         * & & & & & & & &
         * & & & & & & & &
         * & & & & & & & &
         * XXX: Clean this up! I think I either have this or the flip wrong.
         */
        int p = 0;
        for (int h = 0; h < tile.h; h++) {
            tile.data[p++] = (bit_isset(b[8+h], 3) << 1 | bit_isset(b[8+h], 7));
            tile.data[p++] = (bit_isset(b[8+h], 2) << 1 | bit_isset(b[8+h], 6));
            tile.data[p++] = (bit_isset(b[8+h], 1) << 1 | bit_isset(b[8+h], 5));
            tile.data[p++] = (bit_isset(b[8+h], 0) << 1 | bit_isset(b[8+h], 4));
            tile.data[p++] = (bit_isset(b[h], 3) << 1 | bit_isset(b[h], 7));
            tile.data[p++] = (bit_isset(b[h], 2) << 1 | bit_isset(b[h], 6));
            tile.data[p++] = (bit_isset(b[h], 1) << 1 | bit_isset(b[h], 5));
            tile.data[p++] = (bit_isset(b[h], 0) << 1 | bit_isset(b[h], 4));
        }
    }

    /* Decode sprites */
    Rom *gfx2_rom0 = _romset.rom("gg1-11.4d");
    for (unsigned idx = 0; idx < 64; idx++) {
        byte_t *b = gfx2_rom0->direct(idx * 64);
        auto &s = _sprites[idx];
        int p = 0;
        for (unsigned y = 0; y < 16; y++) {
            int h = (y > 8) ? y + 32 : y;
            s.data[p++] = (bit_isset(b[h], 3) << 1    | bit_isset(b[h], 7));
            s.data[p++] = (bit_isset(b[h], 2) << 1    | bit_isset(b[h], 6));
            s.data[p++] = (bit_isset(b[h], 1) << 1    | bit_isset(b[h], 5));
            s.data[p++] = (bit_isset(b[h], 0) << 1    | bit_isset(b[h], 4));
            s.data[p++] = (bit_isset(b[8+h], 3) << 1  | bit_isset(b[8+h], 7));
            s.data[p++] = (bit_isset(b[8+h], 2) << 1  | bit_isset(b[8+h], 6));
            s.data[p++] = (bit_isset(b[8+h], 1) << 1  | bit_isset(b[8+h], 5));
            s.data[p++] = (bit_isset(b[8+h], 0) << 1  | bit_isset(b[8+h], 4));
            s.data[p++] = (bit_isset(b[16+h], 3) << 1 | bit_isset(b[16+h], 7));
            s.data[p++] = (bit_isset(b[16+h], 2) << 1 | bit_isset(b[16+h], 6));
            s.data[p++] = (bit_isset(b[16+h], 1) << 1 | bit_isset(b[16+h], 5));
            s.data[p++] = (bit_isset(b[16+h], 0) << 1 | bit_isset(b[16+h], 4));
            s.data[p++] = (bit_isset(b[24+h], 3) << 1 | bit_isset(b[24+h], 7));
            s.data[p++] = (bit_isset(b[24+h], 2) << 1 | bit_isset(b[24+h], 6));
            s.data[p++] = (bit_isset(b[24+h], 1) << 1 | bit_isset(b[24+h], 5));
            s.data[p++] = (bit_isset(b[24+h], 0) << 1 | bit_isset(b[24+h], 4));
        }
    }

    Rom *gfx2_rom1 = _romset.rom("gg1-10.4f");
    for (unsigned idx = 0; idx < 64; idx++) {
        byte_t *b = gfx2_rom1->direct(idx * 64);
        auto &s = _sprites[64 + idx];
        int p = 0;
        for (unsigned y = 0; y < 16; y++) {
            int h = (y > 8) ? y + 32 : y;
            s.data[p++] = (bit_isset(b[h], 3) << 1    | bit_isset(b[h], 7));
            s.data[p++] = (bit_isset(b[h], 2) << 1    | bit_isset(b[h], 6));
            s.data[p++] = (bit_isset(b[h], 1) << 1    | bit_isset(b[h], 5));
            s.data[p++] = (bit_isset(b[h], 0) << 1    | bit_isset(b[h], 4));
            s.data[p++] = (bit_isset(b[8+h], 3) << 1  | bit_isset(b[8+h], 7));
            s.data[p++] = (bit_isset(b[8+h], 2) << 1  | bit_isset(b[8+h], 6));
            s.data[p++] = (bit_isset(b[8+h], 1) << 1  | bit_isset(b[8+h], 5));
            s.data[p++] = (bit_isset(b[8+h], 0) << 1  | bit_isset(b[8+h], 4));
            s.data[p++] = (bit_isset(b[16+h], 3) << 1 | bit_isset(b[16+h], 7));
            s.data[p++] = (bit_isset(b[16+h], 2) << 1 | bit_isset(b[16+h], 6));
            s.data[p++] = (bit_isset(b[16+h], 1) << 1 | bit_isset(b[16+h], 5));
            s.data[p++] = (bit_isset(b[16+h], 0) << 1 | bit_isset(b[16+h], 4));
            s.data[p++] = (bit_isset(b[24+h], 3) << 1 | bit_isset(b[24+h], 7));
            s.data[p++] = (bit_isset(b[24+h], 2) << 1 | bit_isset(b[24+h], 6));
            s.data[p++] = (bit_isset(b[24+h], 1) << 1 | bit_isset(b[24+h], 5));
            s.data[p++] = (bit_isset(b[24+h], 0) << 1 | bit_isset(b[24+h], 4));
        }
    }


    INFO("Galaga: gfx decoded");
}


void
Galaga::render(void)
{
    /* Render the tilemap */
    byte_t *tile_map = vram.direct(0x400);
    for (unsigned ty = 0; ty < 28; ty++) {
        for (unsigned tx = 0; tx < 48; tx++) {
            byte_t index = ty*tx + tx;
            auto *tile = &_tiles[tile_map[index] & 0x7f];
            auto *palette = &_tile_palette[tile_map[index + 4000] & 0x3f];
            draw_gfx(&_screen, palette, tx * tile->w, ty * tile->h, tile);
        }
    }

    /* Render the sprites */
    byte_t *spriteram_1 = ram1.direct(0x380);
    byte_t *spriteram_2 = ram2.direct(0x380);
    byte_t *spriteram_3 = ram3.direct(0x380);

    for (unsigned off = 0; off < 0x80; off += 2) {
        int idx = spriteram_1[off] & 0x7f;
        int color = spriteram_1[off] & 0x3f;
        int sy = 256 - spriteram_2[off] + 1;
        int sx = spriteram_2[off+1] - 40 + 0x100 * (spriteram_3[off+1] & 3);
        int sizex = bit_isset(spriteram_3[off], 3);
        int sizey = bit_isset(spriteram_3[off], 4);
        for (int y = 0; y <= sizey; y++) {
            for (int x = 0; x <= sizex; x++) {
                /* XXX: Draw diferent sprites */
                auto *sprite = &_sprites[idx];
                auto *palette = &_sprite_palette[color];
                draw_gfx(&_screen, palette, sx, sy, sprite);
            }
        }
    }
}

