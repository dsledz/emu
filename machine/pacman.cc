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

#include "machine/pacman.h"

using namespace EMU;
using namespace Z80;
using namespace Driver;

RomDefinition mspacman_rom(void) {
    RomDefinition rom("mspacman");
    rom.regions.push_back(RomRegion("maincpu", {
        "pacman.6e", "pacman.6f", "pacman.6h", "pacman.6j"
    }));
    /* XXX: u5 + u6 + u7 */

    rom.regions.push_back(RomRegion("gfx1", { "5e", "5f" }));
    rom.regions.push_back(RomRegion("proms", { "82s123.7f", "82s126.4a" }));
    rom.regions.push_back(RomRegion("namco", { "82s126.1m", "82s126.3m" }));

    return rom;
}

RomDefinition pacman_rom(void) {
    RomDefinition rom("pacman");
    rom.regions.push_back(RomRegion("maincpu", {
        "pacman.6e", "pacman.6f", "pacman.6h", "pacman.6j"
    }));

    rom.regions.push_back(RomRegion("gfx1", { "pacman.5e", "pacman.5f" }));
    rom.regions.push_back(RomRegion("proms", { "82s123.7f", "82s126.4a" }));
    rom.regions.push_back(RomRegion("namco", { "82s126.1m", "82s126.3m" }));

    return rom;
}

Pacman::Pacman(const std::string &rom):
    Machine(),
    _scanline(0),
    _avail(Cycles(0)),
    _hertz(18432000),
    _vram(0x400),
    _cram(0x400),
    _ram(0x800)
{
    add_screen(224, 288, RasterScreen::ROT90);

    _bus = AddressBus16_ptr(new AddressBus16());

    if (rom == "pacman") {
        _roms = std::unique_ptr<RomSet>(new RomSet(pacman_rom()));
    } else {
        throw KeyError(rom);
    }

    _cpu = Z80Cpu_ptr(new Z80Cpu(this, "maincpu", _hertz/6, _bus.get()));

    _rom = _roms->rom("maincpu");

    init_gfx();
    init_bus();
    init_switches();
    reset_switches();

    init_controls();

    /* Add IO ports */
    _cpu->io()->add(0x00, 0x0,
        READ_CB(Pacman::io_read, this),
        WRITE_CB(Pacman::io_write, this));
}

Pacman::~Pacman(void)
{
}

byte_t
Pacman::io_read(offset_t offset)
{
    return 0;
}

void
Pacman::io_write(offset_t offset, byte_t value)
{
    _cpu->set_data(value);
    set_line("maincpu", Line::INT0, LineState::Clear);
}

void
Pacman::execute(Time interval)
{
    _avail += interval.to_cycles(Cycles(_hertz/3));

    static const Cycles _cycles_per_scanline(384);
    while (_avail > _cycles_per_scanline) {
        _scanline = (_scanline + 1) % 264;
        _avail -= _cycles_per_scanline;
        switch (_scanline) {
        case 16:
            draw_screen();
            break;
        case 240:
            if (_irq_mask)
                set_line("maincpu", Line::INT0, LineState::Assert);
            break;
        }
    }
}

void
Pacman::init_bus(void)
{
    _bus->add(0x0000, 0x3fff,
        READ_CB(Pacman::rom_read, this),
        WRITE_CB(Pacman::rom_write, this));

    _bus->add(0x4000, 0x43ff,
        READ_CB(Pacman::vram_read, this),
        WRITE_CB(Pacman::vram_write, this));

    _bus->add(0x4400, 0x47ff,
        READ_CB(Pacman::cram_read, this),
        WRITE_CB(Pacman::cram_write, this));

    _bus->add(0x4800, 0x4bff,
        AddressBus16::DefaultRead(),
        AddressBus16::DefaultWrite());

    _bus->add(0x4C00, 0x4fef,
        READ_CB(Pacman::ram_read, this),
        WRITE_CB(Pacman::ram_write, this));

    _bus->add(0x4ff0, 0x4fff,
        READ_CB(Pacman::spr_read, this),
        WRITE_CB(Pacman::spr_write, this));

    _bus->add(0x5000, 0x5007,
        READ_CB(Pacman::in0_read, this),
        WRITE_CB(Pacman::latch_write, this));

    _bus->add(0x5040, 0x505f,
        READ_CB(Pacman::in1_read, this),
        WRITE_CB(Pacman::sound_write, this));

    _bus->add(0x5060, 0x506f,
        READ_CB(Pacman::spr_coord_read, this),
        WRITE_CB(Pacman::spr_coord_write, this));

    _bus->add(0x5070, 0x507f,
        AddressBus16::DefaultRead(),
        AddressBus16::DefaultWrite());

    _bus->add(0x5080, 0x5080,
        READ_CB(Pacman::dsw1_read, this),
        WRITE_CB(Pacman::dsw1_write, this));

    _bus->add(0x50c0, 0x50c0,
        READ_CB(Pacman::dsw2_read, this),
        WRITE_CB(Pacman::watchdog_write, this));

    // Ms. PACMAN
    _bus->add(0x8000, 0xbfff,
        READ_CB(Pacman::rom_read, this),
        WRITE_CB(Pacman::rom_write, this));

    _bus->add(0xC000, 0xC3ff,
        READ_CB(Pacman::vram_read, this),
        WRITE_CB(Pacman::vram_write, this));

    _bus->add(0xC400, 0xC7ff,
        READ_CB(Pacman::cram_read, this),
        WRITE_CB(Pacman::cram_write, this));

    _bus->add(0xCC00, 0xCfef,
        READ_CB(Pacman::ram_read, this),
        WRITE_CB(Pacman::ram_write, this));

    // Pacman waits for an interrupt before setting the stack pointer
    _bus->add(0xFFFD, 0xFFFF,
        AddressBus16::DefaultRead(),
        AddressBus16::DefaultWrite());
}

void
Pacman::init_controls(void)
{
    IOPort *port;

    add_ioport("IN0");
    port = ioport("IN0");
    write_ioport(port, 0xff);
    add_input(InputSignal(InputKey::Joy1Up, port, 0, false));
    add_input(InputSignal(InputKey::Joy1Left, port, 1, false));
    add_input(InputSignal(InputKey::Joy1Right, port, 2, false));
    add_input(InputSignal(InputKey::Joy1Down, port, 3, false));
    add_input(InputSignal(InputKey::Coin1, port, 5, false));
    add_input(InputSignal(InputKey::Coin2, port, 6, false));
    add_input(InputSignal(InputKey::Service, port, 7, false));

    add_ioport("IN1");
    port = ioport("IN1");
    write_ioport(port, 0xff);
    add_input(InputSignal(InputKey::Joy2Up, port, 0, false));
    add_input(InputSignal(InputKey::Joy2Left, port, 1, false));
    add_input(InputSignal(InputKey::Joy2Right, port, 2, false));
    add_input(InputSignal(InputKey::Joy2Down, port, 3, false));
    add_input(InputSignal(InputKey::Start1, port, 5, false));
    add_input(InputSignal(InputKey::Start2, port, 6, false));
}

void
Pacman::init_switches(void)
{
    dipswitch_ptr sw;

    add_ioport("DSW1");
    sw = add_switch("Coinage", "DSW1", 0x03, 0x01);
    sw->add_option("Free play", 0x00);
    sw->add_option("1 Coin  / 1 Credit", 0x01);
    sw->add_option("1 Coin  / 2 Credits", 0x02);
    sw->add_option("2 Coins / 1 Credit", 0x03);

    sw = add_switch("Lives", "DSW1", 0x0c, 0x08);
    sw->add_option("1", 0x00);
    sw->add_option("2", 0x04);
    sw->add_option("3", 0x08);
    sw->add_option("4", 0x0c);

    sw = add_switch("Bonus Life", "DSW1", 0x30, 0x00);
    sw->add_option("10000", 0x00);
    sw->add_option("15000", 0x10);
    sw->add_option("20000", 0x20);
    sw->add_option("None",  0x30);

    sw = add_switch("Difficulty", "DSW1", 0x40, 0x40);
    sw->add_option("Hard", 0x00);
    sw->add_option("Normal", 0x40);

    sw = add_switch("Ghost Names", "DSW1", 0x80, 0x80);
    sw->add_option("Alternate", 0x00);
    sw->add_option("Normal", 0x80);

    add_ioport("DSW2");
}

void
Pacman::init_tile(GfxObject<8, 8> *t, byte_t *b)
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
Pacman::init_sprite(GfxObject<16, 16> *s, byte_t *b)
{
    int p = 0;
    for (unsigned y = 0; y < 16; y++) {
        int h = (y >= 8) ? y + 24 : y;
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
        s->data[p++] = (bit_isset(b[h], 3)    | (bit_isset(b[h], 7)) << 1);
        s->data[p++] = (bit_isset(b[h], 2)    | (bit_isset(b[h], 6)) << 1);
        s->data[p++] = (bit_isset(b[h], 1)    | (bit_isset(b[h], 5)) << 1);
        s->data[p++] = (bit_isset(b[h], 0)    | (bit_isset(b[h], 4)) << 1);
    }
}

void
Pacman::init_gfx(void)
{
    Rom *prom = _roms->rom("proms");

    /* 32 bytes of RRRGGGBB (0-7) */
    for (unsigned i = 0; i < _palette.size; i++) {
        byte_t b = prom->read8(i);
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

    /* XXX: 256 bytes of color table */

    byte_t *char_rom = _roms->rom("proms")->direct(0x20);

    for (unsigned i = 0; i < 256; i+=4) {
        auto *palette = &_palettes[i/4];
        (*palette)[0] = _palette[(char_rom[i+0] & 0x0f)];
        (*palette)[1] = _palette[(char_rom[i+1] & 0x0f)];
        (*palette)[2] = _palette[(char_rom[i+2] & 0x0f)];
        (*palette)[3] = _palette[(char_rom[i+3] & 0x0f)];
    }

    /* Gfx */
    /* Decode characters */
    Rom *gfx1 = _roms->rom("gfx1");
    for (unsigned idx = 0; idx < 256; idx++) {
        byte_t *b = gfx1->direct(idx * 16);
        auto &tile = _tiles[idx];
        init_tile(&tile, b);
    }

    /* Decode sprites */
    for (unsigned idx = 0; idx < 64; idx++) {
        byte_t *b = gfx1->direct(0x1000 + idx * 64);
        auto &s = _sprites[idx];
        init_sprite(&s, b);
    }
}

void
Pacman::draw_screen(void)
{
    screen()->clear();

    draw_bg();
    draw_sprites();
    screen()->flip();
}

void
Pacman::draw_sprites(void)
{
    for (int off = 0; off < 8; off++) {
        int idx = _spr[off].idx;
        int pen = _spr[off].color & 0x1f;
        int sy = _spr[off].y - 31;
        int sx = 272 - _spr[off].x;
        bool flipx = _spr[off].xflip;
        bool flipy = _spr[off].yflip;

        auto *sprite = &_sprites[idx];
        auto *palette = &_palettes[pen];
        draw_gfx(screen(), palette, sprite, sx, sy, flipx, flipy,
                 _palette[0x1f]);
        IF_LOG(Info) {
            std::cout << "Drawing sprite: " << Hex(off)
                << "(" << Hex(sx) << "," << Hex(sy) << "): "
                << "Sprite: " << Hex(idx) << " Color: " << Hex(pen)
                << std::endl;
        }
    }
}

void
Pacman::draw_bg(void)
{
    /* Render the tilemap */
    byte_t *tile_map = _vram.direct(0x000);
    byte_t *color_map = _cram.direct(0x000);
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
            auto *tile = &_tiles[tile_map[index]];
            auto *palette = &_palettes[color_map[index] & 0x1f];
            int sx = tx * tile->w;
            int sy = ty * tile->h;
            draw_gfx(screen(), palette, tile, sx, sy,
                     false, false, _palette[0x1f]);
        }
    }
}


void
Pacman::latch_write(offset_t offset, byte_t value)
{
    switch (offset % 0x08) {
    case 0:
        _irq_mask = bit_isset(value, 0);
        break;
    }
}

MachineInformation pacman_info {
    .name = "Pac-man",
    .year = "1980",
};

static machine_ptr
pacman_create(Options *opts)
{
    return machine_ptr(new Driver::Pacman(opts->rom));
}

MachineDefinition pacman(
    "pacman",
    pacman_info,
    pacman_create);
