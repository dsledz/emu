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

#include "nes.h"

#include "m6502.h"

#define MASTER_CLOCK 21477270

using namespace EMU;
using namespace NESDriver;

enum NESKey {
    A = 0,
    B = 1,
    Select = 2,
    Start = 3,
    Up = 4,
    Down = 5,
    Left = 6,
    Right = 7,
    Size = 8,
};

const std::vector<std::string> NES::ports = {
    "MIRRORING",
    "JOYPAD1",
    "JOYPAD2",
};

NES::NES(const std::string &rom):
    Machine(),
    _ram(0x0800),
    _joy1_shift(0),
    _joy2_shift(0)
{
    _screen = std::unique_ptr<RasterScreen>(new RasterScreen(256, 240));

    for (auto it = ports.begin(); it != ports.end(); it++)
        add_input_port(*it);

    _cpu = std::unique_ptr<M6502::n2A03Cpu>(
        new M6502::n2A03Cpu(this, "cpu", MASTER_CLOCK/12, cpu_bus()));

    _ppu = std::unique_ptr<NESPPU>(
        new NESPPU(this, "ppu", MASTER_CLOCK/4));

    /* Cartridge */
    _mapper = load_cartridge(this, rom);

    /* Program rom is on the CPU bus */
    _cpu_bus.add(0x8000, 0x8000,
        [&](addr_t addr) -> byte_t {
            return _mapper->prg_read(addr);
        },
        [&](addr_t addr, byte_t value) {
            _mapper->prg_write(addr, value);
        });

    /* Chr rom is on the PPU bus */
    _ppu_bus.add(0x0000, 0xE000,
        [&](addr_t addr) -> byte_t {
            return _mapper->chr_read(addr);
        },
        [&](addr_t addr, byte_t value) {
            _mapper->chr_write(addr, value);
        });

    /* SRAM is on the CPU bus */
    _cpu_bus.add(0x6000, 0xE000,
        [&](addr_t addr) -> byte_t {
            return _mapper->sram_read(addr);
        },
        [&](addr_t addr, byte_t value) {
            _mapper->sram_write(addr, value);
        });

    /* XXX: APU registers 0x4000 - 0x4017 */
    _cpu_bus.add(0x4000, 0xE000,
        [&](addr_t addr) -> byte_t {
            byte_t result = 0;
            switch (addr) {
            case 0x0016: {
                byte_t keys = input_port("JOYPAD1")->value;
                if (_joy1_shift < 8)
                    result = bit_isset(keys, _joy1_shift);
                _joy1_shift++;
                break;
            }
            case 0x0017: {
                byte_t keys = input_port("JOYPAD2")->value;
                if (_joy2_shift < 8)
                    result = bit_isset(keys, _joy2_shift);
                _joy2_shift++;
                break;
            }
            }
            return result;
        },
        [&](addr_t addr, byte_t value) {
            switch (addr) {
            case 0x0014: {
                /* XXX: We need to charge the CPU somehow */
                addr = value << 8;
                byte_t sram = _cpu_bus.read(0x2003);
                for (int i = 0; i < 256; i++)
                    _sprite_bus.write((i + sram % 256), _cpu_bus.read(addr++));
                break;
            }
            case 0x0016:
                _joy1_shift = 0;
                _joy2_shift = 0;
                break;
            }
        });

    /* Joypad 1 */
    InputDevice *input = &_input;

    InputPort *port = input_port("JOYPAD1");
    input->add(InputSignal(InputKey::Joy1Btn1, port, NESKey::A, true));
    input->add(InputSignal(InputKey::Joy1Btn2, port, NESKey::B, true));
    input->add(InputSignal(InputKey::Select1, port, NESKey::Select, true));
    input->add(InputSignal(InputKey::Start1, port, NESKey::Start, true));
    input->add(InputSignal(InputKey::Joy1Up, port, NESKey::Up, true));
    input->add(InputSignal(InputKey::Joy1Down, port, NESKey::Down, true));
    input->add(InputSignal(InputKey::Joy1Left, port, NESKey::Left, true));
    input->add(InputSignal(InputKey::Joy1Right, port, NESKey::Right, true));

    port = input_port("JOYPAD2");
    input->add(InputSignal(InputKey::Joy2Btn1, port, NESKey::A, true));
    input->add(InputSignal(InputKey::Joy2Btn2, port, NESKey::B, true));
    input->add(InputSignal(InputKey::Select2, port, NESKey::Select, true));
    input->add(InputSignal(InputKey::Start2, port, NESKey::Start, true));
    input->add(InputSignal(InputKey::Joy2Up, port, NESKey::Up, true));
    input->add(InputSignal(InputKey::Joy2Down, port, NESKey::Down, true));
    input->add(InputSignal(InputKey::Joy2Left, port, NESKey::Left, true));
    input->add(InputSignal(InputKey::Joy2Right, port, NESKey::Right, true));

    /* 2K ram mirrored 4x */
    _cpu_bus.add(0x0000, 0xF800, &_ram);
    _cpu_bus.add(0x0800, 0xF800, &_ram);
    _cpu_bus.add(0x1000, 0xF800, &_ram);
    _cpu_bus.add(0x1800, 0xF800, &_ram);

    /* XXX: Cartridge Expansion 0x4018 - 0x5FFF */

    set_line("cpu", InputLine::RESET, LineState::Pulse);
}

NES::~NES(void)
{
}

MachineDefinition nes(
    "nes",
    [=](Options *opts) -> machine_ptr {
        return machine_ptr(new NESDriver::NES(opts->rom));
    });
