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
using namespace Driver;

RomDefinition galaga_rom(void) {
    RomDefinition rom("galaga");
    rom.regions.push_back(RomRegion("maincpu", {
        "gg1_1b.3p", "gg1_2b.3m", "gg1_3.2m", "gg1_4b.2l"
    }));
    rom.regions.push_back(RomRegion("subcpu", { "gg1_5b.3f" }));
    rom.regions.push_back(RomRegion("sndcpu", { "gg1_7b.2c" }));
    rom.regions.push_back(RomRegion("palette", {
        "prom-5.5n", "prom-4.2n", "prom-3.1c" }));
    rom.regions.push_back(RomRegion("tiles", { "gg1_9.4l" }));
    rom.regions.push_back(RomRegion("sprites", { "gg1_11.4d", "gg1_10.4f" }));
    return rom;
};

RomDefinition galagao_rom(void) {
    RomDefinition rom("galaga");
    rom.regions.push_back(RomRegion("maincpu", {
        "gg1-1.3p", "gg1-2.3m", "gg1-3.2m", "gg1-4.2l"
    }));
    rom.regions.push_back(RomRegion("subcpu", { "gg1-5.3f" }));
    rom.regions.push_back(RomRegion("sndcpu", { "gg1-7.2c" }));
    rom.regions.push_back(RomRegion("palette", {
        "prom-5.5n", "prom-4.2n", "prom-3.1c" }));
    rom.regions.push_back(RomRegion("tiles", { "gg1-9.4l" }));
    rom.regions.push_back(RomRegion("sprites", { "gg1-11.4d", "gg1-10.4f" }));
    return rom;
};

Galaga::Galaga(const std::string &rom):
    Machine(),
    vram(0x0800),
    ram1(0x0400),
    ram2(0x0400),
    ram3(0x0400),
    _scanline(0),
    _avail(Cycles(0)),
    _hertz(18432000),
    _main_irq(false),
    _sub_irq(false),
    _snd_nmi(false)
{
    _screen = std::unique_ptr<RasterScreen>(
        new RasterScreen(224, 288, RasterScreen::ROT90));

    _bus = AddressBus16_ptr(new AddressBus16());

    init_switches();
    reset_switches();

    init_controls();

    RomDefinition roms("galaga");
    if (rom == "galagao") {
        roms = galagao_rom();
    } else {
        roms = galaga_rom();
    }

    RomSet romset(roms);

    _main_cpu = Z80Cpu_ptr(new Z80Cpu(this, "maincpu", _hertz/6, _bus.get()));
    _main_cpu->load_rom(romset.rom("maincpu"), 0x0000);

    _sub_cpu = Z80Cpu_ptr(new Z80Cpu(this, "subcpu", _hertz/6, _bus.get()));
    _sub_cpu->load_rom(romset.rom("subcpu"), 0x0000);

    _snd_cpu = Z80Cpu_ptr(new Z80Cpu(this, "sndcpu", _hertz/6, _bus.get()));
    _snd_cpu->load_rom(romset.rom("sndcpu"), 0x0000);

    _namco06 = Namco06_ptr(new Namco06(this, _main_cpu.get()));

    _namco51 = Namco51_ptr(new Namco51(this));
    _namco06->add_child(0, _namco51.get());

    init_bus();

    init_gfx(&romset);
}

Galaga::~Galaga(void)
{
}

byte_t
Galaga::dips_read(offset_t offset)
{
    byte_t dswa = read_ioport("DSWA");
    byte_t dswb = read_ioport("DSWB");
    return (bit_isset(dswa, offset) << 1) | bit_isset(dswb, offset);
}

void
Galaga::latch_write(offset_t offset, byte_t value)
{
    switch (offset) {
    case 0:
        _main_irq = bit_isset(value, 0);
        if (!_main_irq)
            set_line("maincpu", Line::INT0, LineState::Clear);
        break;
    case 1:
        _sub_irq = bit_isset(value, 0);
        if (!_sub_irq)
            set_line("subcpu", Line::INT0, LineState::Clear);
        break;
    case 2:
        _snd_nmi = !bit_isset(value, 0);
        break;
    case 3:
        set_line("subcpu", Line::RESET, LineState::Pulse);
        set_line("sndcpu", Line::RESET, LineState::Pulse);
        break;
    }

}

void
Galaga::init_bus(void)
{
    /* Dipswitches */
    _bus->add(0x6800, 0x6807,
        READ_CB(Galaga::dips_read, this),
        AddressBus16::DefaultWrite());
    /* XXX: Sound */
    _bus->add(0x6808, 0x681F);
    _bus->add(0x6820, 0x6827,
        AddressBus16::DefaultRead(),
        WRITE_CB(Galaga::latch_write, this)
        );
    /* XXX: Watchdog */
    _bus->add(0x6830, 0x6830);

    /* Namco06 (and children) */
    _bus->add(0x7000, 0x70FF,
        READ_CB(Namco06::read_child, _namco06.get()),
        WRITE_CB(Namco06::write_child, _namco06.get())
        );
    _bus->add(0x7100,
        READ_CB(Namco06::read_control, _namco06.get()),
        WRITE_CB(Namco06::write_control, _namco06.get())
        );

    /* Various ram */
    _bus->add(0x8000, 0x87FF, &vram);
    _bus->add(0x8800, 0x8FFF, &ram1);
    _bus->add(0x9000, 0x97FF, &ram2);
    _bus->add(0x9800, 0x9FFF, &ram3);

    /* XXX: Star control */
    _bus->add(0xA000, 0xA007);
}

void
Galaga::execute(Time interval)
{
    _avail += interval.to_cycles(Cycles(_hertz/3));

    /* Each scanline is 384 cycles */
    static const Cycles _cycles_per_scanline(384);

    while (_avail > _cycles_per_scanline) {
        _scanline = (_scanline + 1) % 264;
        _avail -= _cycles_per_scanline;
        switch (_scanline) {
        case 16:
            draw_screen();
            if (_render_cb)
                _render_cb(_screen.get());
            break;
        case 64:
            if (_snd_nmi)
                set_line("sndcpu", Line::NMI, LineState::Pulse);
            break;
        case 196:
            if (_snd_nmi)
                set_line("sndcpu", Line::NMI, LineState::Pulse);
            break;
        case 240:
            /* Vblank start */
            if (_main_irq)
                set_line("maincpu", Line::INT0, LineState::Assert);
            if (_sub_irq)
                set_line("subcpu", Line::INT0, LineState::Assert);
            break;
        }
    }
}

void
Galaga::init_switches(void)
{
    dipswitch_ptr sw;

    add_ioport("DSWA");
    add_ioport("DSWB");

    sw = add_switch("Difficulty", "DSWA", 0x03, 0x00);
    sw->add_option("Easy", 0x03);
    sw->add_option("Medium", 0x00);
    sw->add_option("Hard", 0x01);
    sw->add_option("Hardest", 0x02);

    /* We hang after POST until this is set */
    sw = add_switch("Unknown", "DSWA", 0x04, 0x04);

    sw = add_switch("Demo Sounds", "DSWA", 0x08, 0x08);
    sw->add_option("Off", 0x00);
    sw->add_option("On", 0x08);

    sw = add_switch("Pause", "DSWA", 0x10, 0x10);
    sw->add_option("Off", 0x10);
    sw->add_option("On", 0x00);

    sw = add_switch("Rack Test", "DSWA", 0x20, 0x20);
    sw->add_option("Off", 0x20);
    sw->add_option("On", 0x00);

    sw = add_switch("Cabinet", "DSWA", 0x80, 0x80);
    sw->add_option("Upright", 0x80);
    sw->add_option("Cocktail", 0x00);

    sw = add_switch("Coinage", "DSWB", 0x07, 0x07);
    sw->add_option("4 Coins / 1 Credit", 0x04);
    sw->add_option("3 Coins / 1 Credit", 0x02);
    sw->add_option("2 Coins / 1 Credit", 0x06);
    sw->add_option("1 Coin  / 1 Credit", 0x07);
    sw->add_option("2 Coins / 3 Credit", 0x01);
    sw->add_option("1 Coin  / 2 Credit", 0x03);
    sw->add_option("1 Coin  / 3 Credit", 0x05);
    sw->add_option("Free Play", 0x00);

    sw = add_switch("Bonus Life", "DSWB", 0x38, 0x10);
    sw->add_option("None", 0x00);
    sw->add_option("30K, 100K, Every 100K", 0x08);
    sw->add_option("20, 70K, Every 70K", 0x10);
    sw->add_option("20K and 60K Only", 0x18);
    sw->add_option("20K, 60K, Every 60K", 0x20);
    sw->add_option("30K, 120K, Every 120K", 0x28);
    sw->add_option("20K, 80K, Every 80K", 0x30);
    sw->add_option("30K and 80K Only", 0x38);

    sw = add_switch("Lives", "DSWB", 0xC0, 0x80);
    sw->add_option("2", 0x00);
    sw->add_option("3", 0x80);
    sw->add_option("4", 0x40);
    sw->add_option("5", 0xC0);
}

void
Galaga::init_controls(void)
{
    InputMap *input = &_input;
    IOPort *port;

    add_ioport("IN0");
    add_ioport("IN1");
    add_ioport("IN2");
    add_ioport("IN3");

    port = ioport("IN0");
    write_ioport(port, 0x0f);
    input->add(InputSignal(InputKey::Joy1Btn1, port, 0, false));
    input->add(InputSignal(InputKey::Joy2Btn1, port, 1, false));
    input->add(InputSignal(InputKey::Start1, port, 2, false));
    input->add(InputSignal(InputKey::Start2, port, 3, false));

    port = ioport("IN1");
    write_ioport(port, 0x0f);
    input->add(InputSignal(InputKey::Coin1, port, 0, false));
    input->add(InputSignal(InputKey::Coin2, port, 1, false));
    input->add(InputSignal(InputKey::Service, port, 2, false));

    port = ioport("IN2");
    write_ioport(port, 0x0f);
    input->add(InputSignal(InputKey::Joy1Right, port, 1, false));
    input->add(InputSignal(InputKey::Joy1Left, port, 3, false));

    port = ioport("IN3");
    write_ioport(port, 0x0f);
    input->add(InputSignal(InputKey::Joy2Right, port, 1, false));
    input->add(InputSignal(InputKey::Joy2Left, port, 3, false));
}

void
Galaga::init_tile(GfxObject<8, 8> *t, byte_t *b)
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
Galaga::init_sprite(GfxObject<16, 16> *s, byte_t *b)
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

void
Galaga::init_gfx(RomSet *romset)
{
    /**
     * Color Palette is defined as:
     * RRRGGGBB
     */
    Rom * palette_rom = romset->rom("palette");
    for (unsigned i = 0; i < _palette.size; i++) {
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
    byte_t * char_rom = romset->rom("palette")->direct(0x20);
    for (unsigned i = 0; i < 64*4; i+=4) {
        auto *palette = &_tile_palette[i/4];
        (*palette)[0] = _palette[(char_rom[i+0] & 0x0f) + 0x10];
        (*palette)[1] = _palette[(char_rom[i+1] & 0x0f) + 0x10];
        (*palette)[2] = _palette[(char_rom[i+2] & 0x0f) + 0x10];
        (*palette)[3] = _palette[(char_rom[i+3] & 0x0f) + 0x10];
    }

    /* Decode characters */
    Rom *gfx1_rom = romset->rom("tiles");
    for (unsigned idx = 0; idx < 128; idx++) {
        byte_t *b = gfx1_rom->direct(idx * 16);
        auto &tile = _tiles[idx];
        init_tile(&tile, b);
    }

    /* XXX: Sprites Palette */
    byte_t * sprite_rom = romset->rom("palette")->direct(0x0120);
    for (unsigned i = 0; i < 64*4; i+=4) {
        auto *palette = &_sprite_palette[i/4];
        (*palette)[0] = _palette[sprite_rom[i+0] & 0x0f];
        (*palette)[1] = _palette[sprite_rom[i+1] & 0x0f];
        (*palette)[2] = _palette[sprite_rom[i+2] & 0x0f];
        (*palette)[3] = _palette[sprite_rom[i+3] & 0x0f];
    }

    /* Decode sprites */
    Rom *gfx2 = romset->rom("sprites");
    for (unsigned idx = 0; idx < 128; idx++) {
        byte_t *b = gfx2->direct(idx * 64);
        auto &s = _sprites[idx];
        init_sprite(&s, b);
    }
}

void
Galaga::draw_screen(void)
{
    _screen->clear();

    draw_sprites();
    draw_bg();
}

void
Galaga::draw_sprites(void)
{
    byte_t *spriteram_1 = ram1.direct(0x380);
    byte_t *spriteram_2 = ram2.direct(0x380);
    byte_t *spriteram_3 = ram3.direct(0x380);

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
        int idx = spriteram_1[off] & 0x7f;
        int pen = spriteram_1[off+1] & 0x3f;
        int sy = 256 - spriteram_2[off] + 1;
        int sx = spriteram_2[off+1] - 40 + 0x100 * (spriteram_3[off+1] & 3);
        bool flipx = bit_isset(spriteram_3[off], 0);
        bool flipy = bit_isset(spriteram_3[off], 1);
        int sizex = bit_isset(spriteram_3[off], 2);
        int sizey = bit_isset(spriteram_3[off], 3);

        /* Adjust the y position */
        sy -= 16 * sizey;
        sy = (sy & 0xff) - 32;

        for (int y = 0; y <= sizey; y++) {
            for (int x = 0; x <= sizex; x++) {
                auto *sprite = &_sprites[idx];
                auto *palette = &_sprite_palette[pen];
                draw_gfx(_screen.get(), palette,
                    sprite + (y ^ (sizey & flipy)) * 2 + (x ^ (sizex & flipx)),
                    sx + (x * 16), sy + (y * 16),
                    flipx, flipy, _palette[0xf]);
            }
        }
    }
}

void
Galaga::draw_bg(void)
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
            auto *tile = &_tiles[tile_map[index] & 0x7f];
            auto *palette = &_tile_palette[tile_map[index + 0x400] & 0x3f];
            int sx = tx * tile->w;
            int sy = ty * tile->h;
            draw_gfx(_screen.get(), palette, tile, sx, sy,
                     false, false, _palette[0x1f]);
        }
    }
}

MachineDefinition galaga(
    "galaga",
    [=](Options *opts) -> machine_ptr {
        return machine_ptr(new Driver::Galaga(opts->rom));
    });
