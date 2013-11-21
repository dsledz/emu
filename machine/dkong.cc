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
using namespace Driver;

RomDefinition dkong_rom(void) {
    RomDefinition rom("dkong");
    rom.regions.push_back(RomRegion("maincpu", {
        "c_5et_g.bin", "c_5ct_g.bin", "c_5bt_g.bin", "c_5at_g.bin"
    }));
    rom.regions.push_back(RomRegion("soundcpu", {
        "s_3i_b.bin",
    }));
    rom.regions.push_back(RomRegion("tiles", {
        "v_5h_b.bin", "v_3pt.bin"
    }));
    rom.regions.push_back(RomRegion("sprites", {
        "l_4m_b.bin", "l_4n_b.bin", "l_4r_b.bin", "l_4s_b.bin"
    }));
    rom.regions.push_back(RomRegion("palette", {
        "c-2k.bpr", "c-2j.bpr", "v-5e.bpr"
    }));
    return rom;
}

DonkeyKong::DonkeyKong(const std::string &rom):
    Machine(),
    ram(0x1000),
    _hertz(18432000)
{
    add_screen(224, 256, RasterScreen::ROT90);

    _bus = AddressBus16_ptr(new AddressBus16());

    init_switches();
    reset_switches();

    init_controls();

    RomDefinition roms("dkong");
    roms = dkong_rom();

    RomSet romset(roms);

    _main_cpu = Z80Cpu_ptr(new Z80Cpu(this, "maincpu", _hertz/6, _bus.get()));
    _main_cpu->load_rom(romset.rom("maincpu"), 0x0000);

    _i8257 = I8257_ptr(new I8257(this, "i8257", _hertz/6, _bus.get()));

    _gfx = DonkeyKongGfx_ptr(new DonkeyKongGfx(this, "gfx", _hertz, _bus.get()));
    _gfx->init(&romset);

    init_bus();
}

DonkeyKong::~DonkeyKong(void)
{
}

void
DonkeyKong::init_switches(void)
{
    dipswitch_ptr sw;

    add_ioport("DSW1");

    sw = add_switch("Lives", "DSW1", 0x03, 0x00);
    sw->add_option("3", 0x00);
    sw->add_option("4", 0x01);
    sw->add_option("5", 0x02);
    sw->add_option("6", 0x03);

    sw = add_switch("Bonus At", "DSW1", 0x0C, 0x00);
    sw->add_option("7000", 0x00);
    sw->add_option("10000", 0x04);
    sw->add_option("15000", 0x08);
    sw->add_option("20000", 0x0C);

    sw = add_switch("Cabinet", "DSW1", 0x80, 0x80);
    sw->add_option("Cocktail", 0x00);
    sw->add_option("Upright", 0x80);

    sw = add_switch("Coins/Play", "DSW1", 0x70, 0x00);
    sw->add_option("1 coin 1 play", 0x00);
    sw->add_option("2 coins 1 play", 0x10);
    sw->add_option("1 coint 2 plays", 0x20);
    sw->add_option("3 coins 1 play", 0x30);
    sw->add_option("1 coin 3 plays", 0x40);
    sw->add_option("4 coins 1 play", 0x50);
    sw->add_option("1 coin 4 plays", 0x60);
    sw->add_option("5 coins 1 play", 0x70);
}

void
DonkeyKong::init_controls(void)
{
    IOPort *port;

    add_ioport("IN0");
    add_ioport("IN1");
    add_ioport("IN2");

    port = ioport("IN0");
    add_input(InputSignal(InputKey::Joy1Right, port, 0, true));
    add_input(InputSignal(InputKey::Joy1Left, port, 1, true));
    add_input(InputSignal(InputKey::Joy1Up, port, 2, true));
    add_input(InputSignal(InputKey::Joy1Down, port, 3, true));
    add_input(InputSignal(InputKey::Joy1Btn1, port, 4, true));

    port = ioport("IN1");
    add_input(InputSignal(InputKey::Joy2Right, port, 0, true));
    add_input(InputSignal(InputKey::Joy2Left, port, 1, true));
    add_input(InputSignal(InputKey::Joy2Up, port, 2, true));
    add_input(InputSignal(InputKey::Joy2Down, port, 3, true));
    add_input(InputSignal(InputKey::Joy2Btn1, port, 4, true));

    port = ioport("IN2");
    add_input(InputSignal(InputKey::Coin1, port, 7, true));
    add_input(InputSignal(InputKey::Start2, port, 3, true));
    add_input(InputSignal(InputKey::Start1, port, 2, true));
}

void
DonkeyKong::init_bus(void)
{
    _bus->add(0x6000, 0x6fff, &ram);
    _bus->add(0x7000, 0x77ff,
        READ_CB(DonkeyKongGfx::vmem_read, _gfx.get()),
        WRITE_CB(DonkeyKongGfx::vmem_write, _gfx.get()));
    _bus->add(0x7c00, 0x7fff,
        READ_CB(DonkeyKong::latch_read, this),
        WRITE_CB(DonkeyKong::latch_write, this));
    _bus->add(0x7800, 0x780f,
        READ_CB(I8257::read_cb, _i8257.get()),
        WRITE_CB(I8257::write_cb, _i8257.get()));
}

void
DonkeyKong::latch_write(offset_t offset, byte_t value)
{
    switch (offset) {
    case 0x00: /* Background music */
        break;
    case 0x100: /* Sound - walk */
    case 0x101: /* Sound - jump */
    case 0x102: /* Sound - boom */
    case 0x103: /* Sound - coin */
    case 0x104: /* Gorilla */
    case 0x105: /* Barrel */
        break;
    case 0x184: /* Interrupt */
        _gfx->nmi_mask_write(value);
        break;
    case 0x185:
        if (bit_isset(value, 0))
            _i8257->dma();
        break;
    case 0x186:
        _gfx->palette_write(0, value);
        break;
    case 0x187:
        _gfx->palette_write(1, value);
        break;
    default:
        break;
    }
}

byte_t
DonkeyKong::latch_read(offset_t offset)
{
    switch (offset) {
    case 0x00:
        return read_ioport("IN0");
    case 0x80:
        return read_ioport("IN1");
    case 0x100:
        return read_ioport("IN2");
    case 0x180:
        return read_ioport("DSW1");
    }
    return 0;
}

MachineInformation donkeykong_info {
    .name = "Donkey Kong",
    .year = "1981",
};

MachineDefinition dkong(
    "dkong",
    donkeykong_info,
    [=](Options *opts) -> machine_ptr {
        return machine_ptr(new Driver::DonkeyKong(opts->rom));
    });

