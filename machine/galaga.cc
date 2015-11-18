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
using namespace Device;

RomDefinition galaga_rom(void) {
    RomDefinition rom("galaga");
    rom.regions.push_back(RomRegion("maincpu", {
        "gg1_1b.3p", "gg1_2b.3m", "gg1_3.2m", "gg1_4b.2l"
    }));
    rom.regions.push_back(RomRegion("subcpu",  { "gg1_5b.3f" }));
    rom.regions.push_back(RomRegion("sndcpu",  { "gg1_7b.2c" }));
    rom.regions.push_back(RomRegion("prom1",   { "prom-5.5n" }));
    rom.regions.push_back(RomRegion("prom2",   { "prom-4.2n", "prom-3.1c" }));
    rom.regions.push_back(RomRegion("tiles",   { "gg1_9.4l" }));
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
    ram1(this, "ram1", 0x0400),
    ram2(this, "ram2", 0x0400),
    ram3(this, "ram3", 0x0400),
    m_main_irq(false),
    m_sub_irq(false),
    m_snd_nmi(false)
{
    unsigned hertz = 18432000;

    add_screen(224, 288, FrameBuffer::ROT90);

    m_bus = AddressBus16_ptr(new AddressBus16());

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

    m_main_cpu = Z80Cpu_ptr(new Z80Cpu(this, "maincpu", hertz/6, m_bus.get()));
    m_main_cpu->load_rom(romset.rom("maincpu"), 0x0000);

    m_sub_cpu = Z80Cpu_ptr(new Z80Cpu(this, "subcpu", hertz/6, m_bus.get()));
    m_sub_cpu->load_rom(romset.rom("subcpu"), 0x0000);

    m_snd_cpu = Z80Cpu_ptr(new Z80Cpu(this, "sndcpu", hertz/6, m_bus.get()));
    m_snd_cpu->load_rom(romset.rom("sndcpu"), 0x0000);

    m_namco06 = Namco06_ptr(new Namco06(this, m_main_cpu.get()));

    m_namco51 = Namco51_ptr(new Namco51(this));
    m_namco06->add_child(0, m_namco51.get());

    m_gfx = GalagaGfx_ptr(new GalagaGfx(this, "gfx", hertz, m_bus.get()));
    m_gfx->init(&romset);

    m_gfx->register_callback(64, [&](void) {
        if (m_snd_nmi)
            set_line("sndcpu", Line::NMI, LineState::Pulse);
    });

    m_gfx->register_callback(196, [&](void) {
        if (m_snd_nmi)
            set_line("sndcpu", Line::NMI, LineState::Pulse);
    });

    m_gfx->register_callback(240, [&](void) {
        if (m_main_irq)
            set_line("maincpu", Line::INT0, LineState::Assert);
        if (m_sub_irq)
            set_line("subcpu", Line::INT0, LineState::Assert);
    });

    init_bus();
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
        m_main_irq = bit_isset(value, 0);
        if (!m_main_irq)
            set_line("maincpu", Line::INT0, LineState::Clear);
        break;
    case 1:
        m_sub_irq = bit_isset(value, 0);
        if (!m_sub_irq)
            set_line("subcpu", Line::INT0, LineState::Clear);
        break;
    case 2:
        m_snd_nmi = !bit_isset(value, 0);
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
    m_bus->add(0x6800, 0x6807,
        READ_CB(Galaga::dips_read, this),
        AddressBus16::DefaultWrite());
    /* XXX: Sound */
    m_bus->add(0x6808, 0x681F);
    m_bus->add(0x6820, 0x6827,
        AddressBus16::DefaultRead(),
        WRITE_CB(Galaga::latch_write, this)
        );
    /* XXX: Watchdog */
    m_bus->add(0x6830, 0x6830);

    /* Namco06 (and children) */
    m_bus->add(0x7000, 0x70FF,
        READ_CB(Namco06::read_child, m_namco06.get()),
        WRITE_CB(Namco06::write_child, m_namco06.get())
        );
    m_bus->add(0x7100,
        READ_CB(Namco06::read_control, m_namco06.get()),
        WRITE_CB(Namco06::write_control, m_namco06.get())
        );

    /* Various ram */
    m_bus->add(0x8000, 0x87FF,
        READ_CB(GalagaGfx::vmem_read, m_gfx.get()),
        WRITE_CB(GalagaGfx::vmem_write, m_gfx.get()));

    m_bus->add(0x8800, 0x8BFF,
        READ_CB(RamDevice::read8, &ram1),
        WRITE_CB(RamDevice::write8, &ram1));

    m_bus->add(0x9000, 0x93FF,
        READ_CB(RamDevice::read8, &ram2),
        WRITE_CB(RamDevice::write8, &ram2));

    m_bus->add(0x9800, 0x9BFF,
        READ_CB(RamDevice::read8, &ram3),
        WRITE_CB(RamDevice::write8, &ram3));

    /* XXX: Star control */
    m_bus->add(0xA000, 0xA007);
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
    IOPort *port;

    add_ioport("IN0");
    add_ioport("IN1");
    add_ioport("IN2");
    add_ioport("IN3");

    port = ioport("IN0");
    write_ioport(port, 0x0f);
    add_input(InputSignal(InputKey::Joy1Btn1, port, 0, false));
    add_input(InputSignal(InputKey::Joy2Btn1, port, 1, false));
    add_input(InputSignal(InputKey::Start1, port, 2, false));
    add_input(InputSignal(InputKey::Start2, port, 3, false));

    port = ioport("IN1");
    write_ioport(port, 0x0f);
    add_input(InputSignal(InputKey::Coin1, port, 0, false));
    add_input(InputSignal(InputKey::Coin2, port, 1, false));
    add_input(InputSignal(InputKey::Service, port, 2, false));

    port = ioport("IN2");
    write_ioport(port, 0x0f);
    add_input(InputSignal(InputKey::Joy1Right, port, 1, false));
    add_input(InputSignal(InputKey::Joy1Left, port, 3, false));

    port = ioport("IN3");
    write_ioport(port, 0x0f);
    add_input(InputSignal(InputKey::Joy2Right, port, 1, false));
    add_input(InputSignal(InputKey::Joy2Left, port, 3, false));
}


MachineInformation galaga_info {
    .name = "Galaga",
    .year = "1983",
};

MachineDefinition galaga(
    "galaga",
    galaga_info,
    [](Options *opts) -> machine_ptr {
        return machine_ptr(new Arcade::Galaga(opts->rom));
    });
