/**
 * Copyright (c) 2016, Dan Sledz
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

#include "machine/c64/c64.h"
#include "machine/c64/m6510.h"

using namespace EMU;
using namespace C64Machine;

RomDefinition c64_rom(void) {
  RomDefinition rom("c64");
  rom.regions.push_back(RomRegion("basic", {"basic.bin"}));
  rom.regions.push_back(RomRegion("char", {"characters.bin"}));
  rom.regions.push_back(RomRegion("kernal", {"kernal.bin"}));
  return rom;
}

/*
 * XXX:
 * Color clock is 14.318180Mhz
 * DOT clock is 8.181800Mhz
 * Cpu is divided by 8.
 */
C64::C64(void) : Machine(Hertz(8181800)),
    m_ram(this, "ram", 0x10000),
    m_romset(),
    m_cpu(),
    m_bus()
{
  // 40 x 25 of 8x8 characters
  add_screen(320, 200);

  m_bus = C64Bus_ptr(new C64Bus());
  m_cpu = C64Cpu_ptr(new M6510Cpu(this, "cpu", ClockDivider(8), m_bus.get()));
  m_vic = VIC2_ptr(new VIC2(this));
  m_keyboard = C64Keyboard_ptr(new C64Keyboard(this));
  m_cia1 = C64CIA_ptr(new C64CIA(this, "cia1", Line::INT0));
  //m_cia2 = C64CIA_ptr(new C64CIA(this, "cia2", Line::NMI));

  m_bus->add(0x0000, m_ram.direct(0), 0x10000, false);
  m_bus->add(0x0000, READ_CB(C64::direction_port_read, this),
             WRITE_CB(C64::direction_port_write, this));
  m_bus->add(0x0001, READ_CB(C64::io_port_read, this),
             WRITE_CB(C64::io_port_write, this));

  m_bus->add(0xdc00, 0xdcff, READ_CB(C64CIA::cia_read, m_cia1.get()),
                    WRITE_CB(C64CIA::cia_write, m_cia1.get()));
  m_bus->add(0xd000, 0xd3ff, READ_CB(VIC2::vic2_read, m_vic.get()),
             WRITE_CB(VIC2::vic2_write, m_vic.get()));
}

C64::~C64() {}

void C64::load_rom(const std::string &rom) {
  RomDefinition roms = c64_rom();

  m_romset.load(roms);
  m_bus->add(0xA000, m_romset.rom("basic"));
  m_bus->add(0xE000, m_romset.rom("kernal"));

  m_vic->load_rom();
}

uint8_t C64::direction_port_read(offset_t offset)
{
  return m_direction_port;
}

void C64::direction_port_write(offset_t offset, uint8_t value)
{

}

uint8_t C64::io_port_read(offset_t offset)
{
  LOG_DEBUG("IO PORT READ:", Hex(m_io_port));
  return m_io_port;
}

void C64::io_port_write(offset_t offset, uint8_t value)
{
  LOG_DEBUG("IO PORT WRITE:", Hex(value));
  uint8_t bits = value & m_direction_port;
  if (bits & 0x01) {

  }
  if (bits & 0x02) {

  }
  if (bits & 0x04) {
  }
  // XXX:
  m_io_port = value;

}


MachineInformation c64_info{
    .name = "C64", .year = "1983",
};

MachineDefinition c64("c64", c64_info, [](Options *opts) -> machine_ptr {
  return machine_ptr(new C64Machine::C64());
});
