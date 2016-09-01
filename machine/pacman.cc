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
using namespace Arcade;

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
    m_hertz(18432000),
    m_ram(this, "ram", 0x800)
{
    add_screen(224, 288, FrameBuffer::ROT90);

    m_bus = AddressBus16_ptr(new AddressBus16());

    if (rom == "pacman") {
        m_roms = std::unique_ptr<RomSet>(new RomSet(pacman_rom()));
    } else {
        throw KeyError(rom);
    }

    m_cpu = Z80Cpu_ptr(new Z80Cpu(this, "maincpu", m_hertz/6, m_bus.get()));
    m_cpu->load_rom(m_roms->rom("maincpu"), 0x0000);

    m_gfx = PacmanGfx_ptr(new PacmanGfx(this, "gfx", m_hertz, m_bus.get()));

    m_gfx->init(m_roms.get());
    m_gfx->set_vblank_cb([&](void) {
        if (m_irq_mask)
            set_line("maincpu", Line::INT0, LineState::Assert);
    });

    init_bus();
    init_switches();
    reset_switches();

    init_controls();

    /* Add IO ports */
    m_cpu->io()->add(0x00, 0x0,
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
    m_cpu->set_data(value);
    set_line("maincpu", Line::INT0, LineState::Clear);
}

void
Pacman::init_bus(void)
{

    m_bus->add(0x4000, &m_gfx->vram());
    m_bus->add(0x4400, 0x47ff, &m_gfx->cram());

    m_bus->add(0x4800, 0x4bff,
        AddressBus16::DefaultRead(),
        AddressBus16::DefaultWrite());

    m_bus->add(0x4C00, 0x4fef,
        READ_CB(Pacman::ram_read, this),
        WRITE_CB(Pacman::ram_write, this));

    m_bus->add(0x4ff0, 0x4fff,
        READ_CB(PacmanGfx::spr_read, m_gfx.get()),
        WRITE_CB(PacmanGfx::spr_write, m_gfx.get()));

    m_bus->add(0x5000, 0x5007,
        READ_CB(Pacman::in0_read, this),
        WRITE_CB(Pacman::latch_write, this));

    m_bus->add(0x5040, 0x505f,
        READ_CB(Pacman::in1_read, this),
        WRITE_CB(Pacman::sound_write, this));

    m_bus->add(0x5060, 0x506f,
        READ_CB(PacmanGfx::spr_coord_read, m_gfx.get()),
        WRITE_CB(PacmanGfx::spr_coord_write, m_gfx.get()));

    m_bus->add(0x5070, 0x507f,
        AddressBus16::DefaultRead(),
        AddressBus16::DefaultWrite());

    m_bus->add(0x5080, 0x5080,
        READ_CB(Pacman::dsw1_read, this),
        WRITE_CB(Pacman::dsw1_write, this));

    m_bus->add(0x50c0, 0x50c0,
        READ_CB(Pacman::dsw2_read, this),
        WRITE_CB(Pacman::watchdog_write, this));

    // Ms. PACMAN
#if 0
    m_bus->add(0x8000, 0xbfff,
        READ_CB(Pacman::rom_read, this),
        WRITE_CB(Pacman::rom_write, this));
#endif

    m_bus->add(0xC000, &m_gfx->vram());

    m_bus->add(0xC400, &m_gfx->cram());

    m_bus->add(0xCC00, 0xCfef,
        READ_CB(Pacman::ram_read, this),
        WRITE_CB(Pacman::ram_write, this));

    // Pacman waits for an interrupt before setting the stack pointer
    m_bus->add(0xFFFD, 0xFFFF,
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
Pacman::latch_write(offset_t offset, byte_t value)
{
    switch (offset % 0x08) {
    case 0:
        m_irq_mask = bit_isset(value, 0);
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
    return machine_ptr(new Arcade::Pacman(opts->rom));
}

MachineDefinition pacman(
    "pacman",
    pacman_info,
    pacman_create);
