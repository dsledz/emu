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

#include "machine/nes/nes.h"

#define MASTER_CLOCK 21477270

using namespace EMU;
using namespace NESMachine;

const std::vector<std::string> NES::ports = {
    "MIRRORING"
};

NES::NES(void)
    : Machine(Hertz(MASTER_CLOCK)),
      m_joypad1(this),
      m_joypad2(this),
      m_ram(this, "ram", 0x0800) {
  add_screen(256, 240);

  for (auto it = ports.begin(); it != ports.end(); it++) add_ioport(*it);

  m_cpu = std::unique_ptr<N2a03Cpu>(
      new N2a03Cpu(this, "cpu", ClockDivider(12), cpu_bus()));

  m_ppu = std::unique_ptr<NESPPU>(new NESPPU(this, "ppu", ClockDivider(4)));

  /* XXX: APU registers 0x4000 - 0x4017 */
  m_cpu_bus.add(0x4000, 0x5FFF, READ_CB(NES::latch_read, this),
                WRITE_CB(NES::latch_write, this));


  /* 2K ram mirrored 4x */
  m_cpu_bus.add(0x0000, &m_ram);
  m_cpu_bus.add(0x0800, &m_ram);
  m_cpu_bus.add(0x1000, &m_ram);
  m_cpu_bus.add(0x1800, &m_ram);

  /* XXX: Cartridge Expansion 0x4018 - 0x5FFF */
}

NES::~NES(void) {}

void NES::load_rom(const std::string &rom) {
  /* Cartridge */
  m_mapper = load_cartridge(this, rom);
}

uint8_t NES::latch_read(offset_t offset) {
  byte_t result = 0;
  switch (offset) {
    case 0x0016: {
      result = m_joypad1.latch_read();
      break;
    }
    case 0x0017: {
      result = m_joypad2.latch_read();
      break;
    }
  }
  return result;
}

void NES::latch_write(offset_t offset, uint8_t value) {
  switch (offset) {
    case 0x0014: {
      /* XXX: We need to charge the CPU somehow */
      addr_t addr = value << 8;
      byte_t sram = m_cpu_bus.read(0x2003);
      for (int i = 0; i < 256; i++)
        m_sprite_bus.write((i + sram % 256), m_cpu_bus.read(addr++));
      break;
    }
    case 0x0016:
      m_joypad1.latch_reset();
      m_joypad2.latch_reset();
      break;
  }
}

MachineInformation nes_info{
    .name = "Nintendo Entertainment System",
    .year = "1985",
    .extension = "nes",
    .cartridge = true,
};

MachineDefinition nes("nes", nes_info, [](Options *opts) -> machine_ptr {
  return machine_ptr(new NESMachine::NES());
});
