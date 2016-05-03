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

#include "machine/gb/gb.h"
#include "machine/gb/gbgfx.h"
#include "machine/gb/gbmbc.h"

using namespace EMU;
using namespace GBMachine;

class GBJoypad: public Device {
    enum GBKey {
        A = 0,
        B = 1,
        Select = 2,
        Start = 3,
        Right = 4,
        Left = 5,
        Up = 6,
        Down = 7,
        Size = 8,
    };

    enum KeysReg {
        ButtonsSelect = 5,
        ArrowSelect   = 4,
        DownOrSelect  = 3,
        UpOrSelect    = 2,
        LeftOrB       = 1,
        RightOrA      = 0,
    };

public:
    GBJoypad(Gameboy *gb):
        Device(gb, "joypad"),
        m_value(0x00)
    {
        gb->add_ioport("IN0");

        m_port = gb->ioport("IN0");
        gb->write_ioport(m_port, 0xFF);
        gb->add_input(InputSignal(InputKey::Joy1Left, m_port, GBKey::Left,
                                  false));
        gb->add_input(InputSignal(InputKey::Joy1Right, m_port, GBKey::Right,
                                  false));
        gb->add_input(InputSignal(InputKey::Joy1Up, m_port, GBKey::Up,
                                  false));
        gb->add_input(InputSignal(InputKey::Joy1Down,  m_port, GBKey::Down,
                                  false));
        gb->add_input(InputSignal(InputKey::Joy1Btn1,  m_port, GBKey::A,
                                  false));
        gb->add_input(InputSignal(InputKey::Joy1Btn2,  m_port, GBKey::B,
                                  false));
        gb->add_input(InputSignal(InputKey::Select1, m_port, GBKey::Select,
                                  false));
        gb->add_input(InputSignal(InputKey::Start1, m_port, GBKey::Start,
                                  false));

        gb->bus()->add(GBReg::KEYS,
            [&](offset_t offset) -> byte_t {
                return m_value;
            },
            [&](offset_t offset, byte_t arg) {
                byte_t keys = machine()->read_ioport("IN0");
                if (bit_isset(arg, ButtonsSelect))
                    arg = (arg & 0xF0) | ((keys & 0xF0) >> 4);
                if (bit_isset(arg, ArrowSelect))
                    arg = (arg & 0xF0) | (keys & 0x0F);
                m_value = arg;
            });
    }
    virtual ~GBJoypad(void) {
    }

private:
    IOPort *m_port;
    byte_t m_value;
};

class GBSerialIO: public Device {
public:
    GBSerialIO(Gameboy *gameboy):
        Device(gameboy, "serial") {
        gameboy->bus()->add(GBReg::SB, &m_sb);
        gameboy->bus()->add(GBReg::SC, &m_sc);
    }
    virtual ~GBSerialIO(void) {
    }
private:
    uint8_t m_sb;
    uint8_t m_sc;
};

class GBTimer: public ClockedDevice {
public:
    GBTimer(Gameboy *gameboy, unsigned hertz):
        ClockedDevice(gameboy, "timer", hertz)
    {
        gameboy->bus()->add(GBReg::TIMA, &m_tima);
        gameboy->bus()->add(GBReg::TMA, &m_tma);
        gameboy->bus()->add(GBReg::TAC, &m_tac);
        gameboy->bus()->add(GBReg::DIV, &m_div);
    }
    virtual ~GBTimer(void) {
    }

    virtual void execute(void) {
        unsigned delta = 64;
        add_icycles(delta);
        m_cycles += delta;
        m_dcycles += delta;
        m_tcycles += delta;

        if (m_dcycles > 256) {
            m_dcycles -= 256;
            m_div++;
            // Divider register triggered
        }
        if (m_tac & 0x04) {
            unsigned limit = 1024;
            switch (m_tac & 0x3) {
            case 0: limit = 1024; break;
            case 1: limit = 16; break;
            case 2: limit = 64; break;
            case 3: limit = 256; break;
            }
            if (m_tcycles > limit) {
                m_tcycles -= limit;
                if (m_tima == 0xff) {
                    machine()->set_line("cpu",
                        make_irq_line(GBInterrupt::Timeout), LineState::Pulse);
                    // Reset the overflow
                    m_tima = m_tma;
                } else
                    m_tima++;
            }
        }
    }

    virtual void set_line(Line line, LineState state) {
        switch (line) {
        case Line::RESET:
            reset();
        default:
            break;
        }
    }

    virtual void reset(void) {
        m_cycles = 0;
        m_dcycles = 0;
        m_tcycles = 0;
        m_tima = 0;
        m_tma = 0;
        m_tac = 0;
    }

private:

    unsigned m_cycles;
    unsigned m_dcycles;
    unsigned m_tcycles;
    byte_t m_div;
    byte_t m_tima;
    byte_t m_tma;
    byte_t m_tac;
};

Gameboy::Gameboy(const std::string &rom_name):
    Machine()
{
    add_screen(160, 144);

    m_bus = AddressBus16_ptr(new AddressBus16());

    m_cpu = std::unique_ptr<LR35902Cpu>(
        new LR35902Cpu(this, "cpu", 4194304, m_bus.get()));

    m_timer = Device_ptr(new GBTimer(this, 4194304));
    m_serial = Device_ptr(new GBSerialIO(this));
    m_joypad = Device_ptr(new GBJoypad(this));
    m_gfx = std::unique_ptr<GBGraphics>(new GBGraphics(this, 4194304));

    m_mbc = std::unique_ptr<GBMBC>(new GBMBC(this));
    m_mbc->load_rom(rom_name);

    m_ram = std::unique_ptr<RamDevice>(new RamDevice(this, "ram", 0x2000));
    /* XXX: Ram isn't mirrored */
    m_bus->add(0xC000, 0xDFFF,
        READ_CB(RamDevice::read8, m_ram.get()),
        WRITE_CB(RamDevice::write8, m_ram.get()));
    m_bus->add(0xE000, 0xFDFF,
        READ_CB(RamDevice::read8, m_ram.get()),
        WRITE_CB(RamDevice::write8, m_ram.get()));

    m_hiram = std::unique_ptr<RamDevice>(new RamDevice(this, "hiram", 0x80));
    m_bus->add(0xFF80, 0xFFFF,
        READ_CB(RamDevice::read8, m_hiram.get()),
        WRITE_CB(RamDevice::write8, m_hiram.get()));

    /* XXX: Some games need this. */
    m_bus->add(0xFF7F, 0xFF7F);

    // XXX: Sound
    m_bus->add(0xFF10, 0xFF3F);
}

Gameboy::~Gameboy(void)
{

}

MachineInformation gb_info {
    "Gameboy",
    "1989",
    "gb",
    true,
};

MachineDefinition gb(
    "gb",
    gb_info,
    [](Options *opts) -> machine_ptr {
        return machine_ptr(new Gameboy(opts->rom));
    });

