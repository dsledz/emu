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
/*
 * Memory bank Controller
 */
#include "emu/emu.h"

#include "machine/gb/gbmbc.h"

using namespace EMU;
using namespace GBMachine;

GBMBC::GBMBC(Gameboy *gameboy)
    : Device(gameboy, "mbc"),
    m_bus(gameboy->bus()),
    m_rom_bank(1),
    m_rom_size(0),
    m_rom(),
    m_ram_bank(0),
    m_ram_size(0),
    m_ram() {
  m_bus->add(GBReg::DMG_RESET, AddressBus16x8::DefaultRead(),
             [&](offset_t offset, byte_t arg) {
               memcpy(m_rom.data(), m_dmg.data(), m_dmg.size());
             });
}

GBMBC::~GBMBC(void) {}

void GBMBC::rom_bank(offset_t offset, byte_t value) {
  if (offset < 0x2000) {
    if (value & 0x0A)
      m_bus->add(0xA000, &m_ram[m_ram_bank * 0x2000],
                 m_ram_size > 0x2000 ? 0x2000 : m_ram_size, false);
    else
      m_bus->remove(0xA000, 0xBFFF);
    return;
  }
  if (m_type == MBC3RRB)
    m_rom_bank = value & 0x7f;
  else
    m_rom_bank = value & 0x1f;
  if (m_rom_bank == 0 || (m_rom_bank * 0x4000) > m_rom_size) m_rom_bank = 1;
  m_bus->add(0x4000, &m_rom[m_rom_bank * 0x4000], 0x4000,
             PAGE_FAULT_CB(GBMBC::ram_bank, this));
}

void GBMBC::ram_bank(offset_t offset, byte_t value) {
  if (offset > 0x6000)
    return;
  m_ram_bank = value & 0x03;
  m_bus->add(0xA000, &m_ram[m_ram_bank * 0x2000], 0x2000, false);
}

void GBMBC::load_rom(const std::string &name) {
  bvec boot_rom;
  read_rom(name, m_rom);
  read_rom("DMG_ROM.bin", boot_rom);
  m_dmg.resize(boot_rom.size());
  memcpy(m_dmg.data(), m_rom.data(), boot_rom.size());
  memcpy(m_rom.data(), boot_rom.data(), boot_rom.size());

  if ((m_rom[0x0143] & 0xC0) == 0xC0) {
    std::cout << "Color Gameboy Only" << std::endl;
    throw RomException(name);
  }

  // Cartridge Header
  m_name.clear();
  for (unsigned i = 0x0134; i < 0x0142; i++) m_name += m_rom[i];
  m_type = static_cast<Cartridge>(m_rom[0x0147]);

  switch (m_type) {
    case Cartridge::RomOnly:
    case Cartridge::MBC1:
    case Cartridge::MBC1R:
    case Cartridge::MBC1RB:
    case Cartridge::MBC3RRB:
      break;
    default:
      throw RomException(name);
      break;
  }

  switch (m_rom[0x0148]) {
    case 0:
      m_rom_size = 32 * 1024;
      break;
    case 1:
      m_rom_size = 64 * 1024;
      break;
    case 2:
      m_rom_size = 128 * 1024;
      break;
    case 3:
      m_rom_size = 256 * 1024;
      break;
    case 4:
      m_rom_size = 512 * 1024;
      break;
    case 5:
      m_rom_size = 1024 * 1024;
      break;
    case 6:
      m_rom_size = 2048 * 1024;
      break;
    case 0x52:
      m_rom_size = 1152 * 1024;
      break;
    case 0x53:
      m_rom_size = 1280 * 1024;
      break;
    case 0x54:
      m_rom_size = 1536 * 1024;
      break;
  }
  if (m_rom_size != m_rom.size()) throw RomException(name);

  switch (m_rom[0x0149]) {
    case 0:
      m_ram_size = 2 * 1024;
    case 1:
      m_ram_size = 2 * 1024;
      break;
    case 2:
      m_ram_size = 8 * 1024;
      break;
    case 3:
      m_ram_size = 32 * 1024;
      break;
    case 4:
      m_ram_size = 128 * 1024;
      break;
  }
  m_ram.resize(m_ram_size);

  LOG_DEBUG("Cartridge Header:", "Name: ", m_name, " MBC: ", m_type,
            " Rom Size: ", m_rom_size);
  reset();
}

void GBMBC::save(SaveState &state) {
  state << m_type;
  state << m_rom_bank;
  state << m_rom_size;
  state << m_rom; /*XXX: lame */
  state << m_ram_bank;
  state << m_ram_size;
  state << m_ram;
}

void GBMBC::load(LoadState &state) {
  state >> m_type;
  state >> m_rom_bank;
  state >> m_rom_size;
  state >> m_rom; /*XXX: lame */
  state >> m_ram_bank;
  state >> m_ram_size;
  state >> m_ram;
  /* XXX: Restore bus */
}

void GBMBC::reset(void) {
  memset(&m_ram[0], 0, m_ram.size());
  m_ram_bank = 0;
  m_rom_bank = 1;
  m_bus->add(0x0000, &m_rom[0], 0x4000, PAGE_FAULT_CB(GBMBC::rom_bank, this));
  m_bus->add(0x4000, &m_rom[m_rom_bank * 0x4000], 0x4000,
             PAGE_FAULT_CB(GBMBC::ram_bank, this));
}
