/* * Copyright (c) 2013, Dan Sledz
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

#include "machine/tg16.h"

using namespace TG16Driver;

/* XXX wrong */
#define MASTER_CLOCK 21477270

TG16::TG16(const std::string &rom):
    Machine(),
    _cpu_bus(),
    _ram(0x2000),
    _vdc(this, MASTER_CLOCK),
    _psg(this)
{
    IF_LOG(Info)
        std::cout << "Loading: " << rom << std::endl;

    EMU::read_rom(rom, _rom);
    _rom_offset = _rom.size() % 8192;

    if (bank_read(0x1FFF) < 0xe0)
        throw RomException("Encrypted Rom");

    init_joypad();

    /* XXX: Is this screen correct? */
    add_screen(256, 240);

    _cpu = std::unique_ptr<M6502::hu6280Cpu>(
        new M6502::hu6280Cpu(this, "cpu", MASTER_CLOCK/3, &_cpu_bus));

    _cpu_bus.add(0x000000, 0x0FFFFF,
        READ_CB(TG16::bank_read, this),
        WRITE_CB(TG16::bank_write, this));

    _cpu_bus.add(0x1F0000, 0x1F1FFF,
        READ_CB(TG16::ram_read, this),
        WRITE_CB(TG16::ram_write, this));

    _cpu_bus.add(0x1FE000, 0x1FE3FF,
        READ_CB(VDC::read, &_vdc),
        WRITE_CB(VDC::write, &_vdc));

    _cpu_bus.add(0x1FE400, 0x1FE7FF,
        READ_CB(VDC::vce_read, &_vdc),
        WRITE_CB(VDC::vce_write, &_vdc));

    _cpu_bus.add(0x1FE800, 0x1FEBFF,
        READ_CB(PSG::read, &_psg),
        WRITE_CB(PSG::write, &_psg));

    _cpu_bus.add(0x1FEC00, 0x1FEFFF,
        READ_CB(M6502::hu6280Cpu::timer_read, _cpu.get()),
        WRITE_CB(M6502::hu6280Cpu::timer_write, _cpu.get()));

    _cpu_bus.add(0x1FF000, 0x1FF3FF,
        READ_CB(TG16::joypad_read, this),
        WRITE_CB(TG16::joypad_write, this));

    _cpu_bus.add(0x1FF400, 0x1FF7FF,
        READ_CB(M6502::hu6280Cpu::irq_read, _cpu.get()),
        WRITE_CB(M6502::hu6280Cpu::irq_write, _cpu.get()));
}

TG16::~TG16(void)
{
}

enum TG16Key {
    I = 0,
    II = 1,
    Select = 2,
    Run = 3,
    Up = 4,
    Right = 5,
    Down = 6,
    Left = 7,
    Size = 8
};

void
TG16::init_joypad(void)
{
    IOPort *port = NULL;

    add_ioport("JOYPAD1");
    port = ioport("JOYPAD1");
    add_input(InputSignal(InputKey::Joy1Btn2,  port, TG16Key::II, false));
    add_input(InputSignal(InputKey::Joy1Btn1,  port, TG16Key::I, false));
    add_input(InputSignal(InputKey::Select1,   port, TG16Key::Select, false));
    add_input(InputSignal(InputKey::Start1,    port, TG16Key::Run, false));
    add_input(InputSignal(InputKey::Joy1Up,    port, TG16Key::Up, false));
    add_input(InputSignal(InputKey::Joy1Down,  port, TG16Key::Down, false));
    add_input(InputSignal(InputKey::Joy1Left,  port, TG16Key::Left, false));
    add_input(InputSignal(InputKey::Joy1Right, port, TG16Key::Right, false));
    _joypad_data = 0;
    _joypad = 0;
}

byte_t
TG16::bank_read(offset_t offset)
{
    /* XXX: Horrible hack */
    if (_rom.size() - _rom_offset == 0x60000) {
        int bank = offset >> 17;
        offset &= 0x1FFFF;
        switch (bank) {
        case 0:
        case 2:
            return _rom.at(offset + _rom_offset);
        case 1:
        case 3:
            return _rom.at(0x20000 + offset + _rom_offset);
        default:
            return _rom.at(0x40000 + offset + _rom_offset);
        }
    } else {
        /* XXX: Is this correct? */
        return _rom.at((offset % _rom.size()) + _rom_offset);
    }
}

void
TG16::bank_write(offset_t offset, byte_t value)
{
    DBG("bank_write");
}

byte_t
TG16::joypad_read(offset_t offset)
{
    byte_t result = 0xFF;
    if (_joypad == 0) {
        result = read_ioport("JOYPAD1");
    }

    /* XXX: Is this correct? */
    if (_joypad_data)
        result >>= 4;

    result = (result & 0x0F) | 0x30;

    return result;
}

void
TG16::joypad_write(offset_t offset, byte_t value)
{
    if (!_joypad_data && bit_isset(value, 0))
        _joypad = (_joypad + 1) % 8;

    _joypad_data = bit_isset(value, 0);

    if (bit_isset(value, 1))
        _joypad = 0;
}

byte_t
TG16::ram_read(offset_t offset)
{
    return _ram.read8(offset);
}

void
TG16::ram_write(offset_t offset, byte_t value)
{
    _ram.write8(offset, value);
}

PSG::PSG(TG16 *tg16):
    Device(tg16, "psg")
{
}

PSG::~PSG(void)
{
}

void
PSG::execute(Time interval)
{
}

byte_t
PSG::read(offset_t offset)
{
    return 0;
}

void
PSG::write(offset_t offset, byte_t value)
{
}

MachineInformation tg16_info {
    .name = "TurboGrafx-16",
    .year = "1989",
    .cartridge = true,
    .extension = "pce",
};

MachineDefinition tg16(
    "tg16",
    tg16_info,
    [=](Options *opts) -> machine_ptr {
        return machine_ptr(new TG16Driver::TG16(opts->rom));
    });

