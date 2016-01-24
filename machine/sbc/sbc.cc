/*
 * Copyright (c) 2016, Dan Sledz
 */

#include "emu/emu.h"
#include "machine/sbc/sbc.h"

using namespace EMU;
using namespace Z80;
using namespace Arcade;
using namespace Device;

RomDefinition sbc_rom(void) {
    RomDefinition rom("sbc");
    rom.regions.push_back(RomRegion("cpu", { "INTMINI.HEX" }));
    return rom;
}

SingleBoardZ80::SingleBoardZ80(const std::string &rom):
    Machine(),
    m_ram(this, "ram", 0x7000),
    m_acia_irq(false)
{
    unsigned hertz = 7372800;

    m_bus = AddressBus16_ptr(new AddressBus16());

    RomDefinition roms("sbc");
    roms = sbc_rom();
    RomSet romset(roms);

    m_cpu = Z80Cpu_ptr(new Z80Cpu(this, "cpu", hertz, m_bus.get()));
    m_cpu->load_rom(romset.rom("cpu"), 0x0000);

}

SingleBoardZ80::~SingleBoardZ80(void)
{
}

byte_t
SingleBoardZ80::io_read(offset_t offset)
{
    byte_t value = 0;
    if ((offset & 0xC0) == 0x80) {
        bool rs = offset & 0x01;
        value = m_acia->read(rs);
    }
    return value;
}

void
SingleBoardZ80::io_write(offset_t offset, byte_t value)
{
    if ((offset & 0xC0) == 0x80) {
        bool rs = offset & 0x01;
        m_acia->write(rs, value);
    }
}

void
SingleBoardZ80::init_bus(void)
{
    m_bus->add(0x1000,  0x7000,
        READ_CB(SingleBoardZ80::ram_read, this),
        WRITE_CB(SingleBoardZ80::ram_write, this));

    m_cpu->io()->add(0x00, 0xff,
            READ_CB(SingleBoardZ80::io_read, this),
            WRITE_CB(SingleBoardZ80::io_write, this));
}

