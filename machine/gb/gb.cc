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
        _value(0x00)
    {
        gb->add_ioport("IN0");

        _port = gb->ioport("IN0");
        gb->write_ioport(_port, 0xFF);
        gb->add_input(InputSignal(InputKey::Joy1Left,_port, GBKey::Left,
                                  false));
        gb->add_input(InputSignal(InputKey::Joy1Right, _port, GBKey::Right,
                                  false));
        gb->add_input(InputSignal(InputKey::Joy1Up, _port, GBKey::Up,
                                  false));
        gb->add_input(InputSignal(InputKey::Joy1Down,  _port, GBKey::Down,
                                  false));
        gb->add_input(InputSignal(InputKey::Joy1Btn1,  _port, GBKey::A,
                                  false));
        gb->add_input(InputSignal(InputKey::Joy1Btn2,  _port, GBKey::B,
                                  false));
        gb->add_input(InputSignal(InputKey::Select1, _port, GBKey::Select,
                                  false));
        gb->add_input(InputSignal(InputKey::Start1, _port, GBKey::Start,
                                  false));

        gb->bus()->add(GBReg::KEYS,
            [&](offset_t offset) -> byte_t {
                return _value;
            },
            [&](offset_t offset, byte_t arg) {
                byte_t keys = machine()->read_ioport("IN0");
                if (bit_isset(arg, ButtonsSelect))
                    arg = (arg & 0xF0) | ((keys & 0xF0) >> 4);
                if (bit_isset(arg, ArrowSelect))
                    arg = (arg & 0xF0) | (keys & 0x0F);
                _value = arg;
            });
    }
    virtual ~GBJoypad(void) {
    }

private:
    IOPort *_port;
    byte_t _value;
};

class GBSerialIO: public Device {
public:
    GBSerialIO(Gameboy *gameboy):
        Device(gameboy, "serial") {
        gameboy->bus()->add(GBReg::SB, &_sb);
        gameboy->bus()->add(GBReg::SC, &_sc);
    }
    virtual ~GBSerialIO(void) {
    }
private:
    uint8_t _sb;
    uint8_t _sc;
};

class GBTimer: public ClockedDevice {
public:
    GBTimer(Gameboy *gameboy, unsigned hertz):
        ClockedDevice(gameboy, "timer", hertz)
    {
        gameboy->bus()->add(GBReg::TIMA, &_tima);
        gameboy->bus()->add(GBReg::TMA, &_tma);
        gameboy->bus()->add(GBReg::TAC, &_tac);
        gameboy->bus()->add(GBReg::DIV, &_div);
    }
    virtual ~GBTimer(void) {
    }

    virtual void execute(void) {
        _cycles += m_avail.v;
        _dcycles += m_avail.v;
        _tcycles += m_avail.v;
        m_avail = Cycles(0);

        if (_dcycles > 256) {
            _dcycles -= 256;
            _div++;
            // Divider register triggered
        }
        if (_tac & 0x04) {
            unsigned limit = 1024;
            switch (_tac & 0x3) {
            case 0: limit = 1024; break;
            case 1: limit = 16; break;
            case 2: limit = 64; break;
            case 3: limit = 256; break;
            }
            if (_tcycles > limit) {
                _tcycles -= limit;
                if (_tima == 0xff) {
                    machine()->set_line("cpu",
                        make_irq_line(GBInterrupt::Timeout), LineState::Pulse);
                    // Reset the overflow
                    _tima = _tma;
                } else
                    _tima++;
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
        _cycles = 0;
        _dcycles = 0;
        _tcycles = 0;
        _tima = 0;
        _tma = 0;
        _tac = 0;
    }

private:

    unsigned _hertz;
    unsigned _cycles;
    unsigned _dcycles;
    unsigned _tcycles;
    byte_t _div;
    byte_t _tima;
    byte_t _tma;
    byte_t _tac;
};

Gameboy::Gameboy(const std::string &rom_name):
    Machine()
{
    add_screen(160, 144);

    _bus = AddressBus16_ptr(new AddressBus16());

    _cpu = std::unique_ptr<LR35902Cpu>(
        new LR35902Cpu(this, "cpu", 4194304, _bus.get()));

    _timer = device_ptr(new GBTimer(this, 4194304));
    _serial = device_ptr(new GBSerialIO(this));
    _joypad = device_ptr(new GBJoypad(this));
    _gfx = std::unique_ptr<GBGraphics>(new GBGraphics(this, 4194304));

    _mbc = std::unique_ptr<GBMBC>(new GBMBC(this));
    _mbc->load_rom(rom_name);

    _ram = std::unique_ptr<RamDevice>(new RamDevice(this, "ram", 0x2000));
    /* XXX: Ram isn't mirrored */
    _bus->add(0xC000, 0xDFFF,
        READ_CB(RamDevice::read8, _ram.get()),
        WRITE_CB(RamDevice::write8, _ram.get()));
    _bus->add(0xE000, 0xFDFF,
        READ_CB(RamDevice::read8, _ram.get()),
        WRITE_CB(RamDevice::write8, _ram.get()));

    _hiram = std::unique_ptr<RamDevice>(new RamDevice(this, "hiram", 0x80));
    _bus->add(0xFF80, 0xFFFF,
        READ_CB(RamDevice::read8, _hiram.get()),
        WRITE_CB(RamDevice::write8, _hiram.get()));

    /* XXX: Some games need this. */
    _bus->add(0xFF7F, 0xFF7F);

    // XXX: Sound
    _bus->add(0xFF10, 0xFF3F);
}

Gameboy::~Gameboy(void)
{

}

MachineInformation gb_info {
    .name = "Gameboy",
    .year = "1989",
    .cartridge = true,
    .extension = "gb",
};

MachineDefinition gb(
    "gb",
    gb_info,
    [=](Options *opts) -> machine_ptr {
        return machine_ptr(new Gameboy(opts->rom));
    });

