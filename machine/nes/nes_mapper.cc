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

using namespace EMU;
using namespace NESMachine;

enum {
  ROM_BANK_SIZE = 0x4000,
  CHR_BANK_SIZE = 0x2000,
};

NESMapper::NESMapper(NES *nes, const iNesHeader *header, bvec &rom)
    : Device(nes, "mapper"),
      m_nes(nes),
      m_header(*header),
      m_rom(std::move(rom)) {
  memcpy(&m_header, header, sizeof(m_header));

/* XXX: My horizontal mirroring never works */
#if 0
    if (m_header.hmirroring)
        machine->write_ioport("MIRRORING", TwoScreenHMirroring);
    else
        machine->write_ioport("MIRRORING", TwoScreenVMirroring);
#else
  m_nes->write_ioport("MIRRORING", TwoScreenVMirroring);
#endif
}

NESMapper::~NESMapper(void) {}

size_t NESMapper::prg_bank(int bank) {
  return sizeof(m_header) + (ROM_BANK_SIZE * bank);
}

size_t NESMapper::chr_bank(int bank) {
  return sizeof(m_header) + (ROM_BANK_SIZE * m_header.rom_banks) +
         (CHR_BANK_SIZE * bank);
}

size_t NESMapper::prg_bank8k(int bank) {
  return sizeof(m_header) + (ROM_BANK_SIZE / 2) * bank;
}

size_t NESMapper::chr_bank1k(int bank) {
  return sizeof(m_header) + (ROM_BANK_SIZE * m_header.rom_banks) +
         (1024 * bank);
}

class NESMapperNone : public NESMapper {
 public:
  NESMapperNone(NES *nes, const iNesHeader *header, bvec &rom)
      : NESMapper(nes, header, rom),
        m_prg_offset(prg_bank(0)),
        m_chr_offset(chr_bank(0)) {
    if (m_header.vrom_banks == 0) {
      m_ram.resize(0x2000);
      m_nes->ppu_bus()->add(0x0000, m_ram);
    } else {
      m_nes->ppu_bus()->add(0x0000, &m_rom[m_chr_offset], 0x2000, true);
    }
    if (m_header.rom_banks == 1) {
      m_nes->cpu_bus()->add(0x8000, &m_rom[m_prg_offset], 0x4000, true);
      m_nes->cpu_bus()->add(0xC000, &m_rom[m_prg_offset], 0x4000, true);
    } else {
      m_nes->cpu_bus()->add(0x8000, &m_rom[m_prg_offset], 0x8000, true);
    }
  }

  ~NESMapperNone(void) {}

  virtual void reset() {
    m_prg_offset = prg_bank(0);
    m_chr_offset = chr_bank(0);
  }

 private:
  size_t m_prg_offset;
  size_t m_chr_offset;
  bvec m_ram;
};

class NESMapperUNROM : public NESMapper {
 public:
  NESMapperUNROM(NES *nes, const iNesHeader *header, bvec &rom)
      : NESMapper(nes, header, rom),
        m_prg_offset0(prg_bank(0)),
        m_prg_offset1(prg_bank(header->rom_banks - 1)) {
    m_nes->ppu_bus()->add(0x0000, m_chr);
  }

  ~NESMapperUNROM(void) {}

  virtual void reset(void) {
    m_prg_offset0 = prg_bank(0);
    m_prg_offset1 = prg_bank(m_header.rom_banks - 1);
    m_nes->cpu_bus()->add(0x8000, &m_rom[m_prg_offset0], 0x4000,
                          PAGE_FAULT_CB(NESMapperUNROM::page_fault, this));
    m_nes->cpu_bus()->add(0xC000, &m_rom[m_prg_offset1], 0x4000,
                          PAGE_FAULT_CB(NESMapperUNROM::page_fault, this));
    std::fill(m_chr.begin(), m_chr.end(), 0);
  }

  void page_fault(offset_t offset, byte_t value) {
    int prg = (value & 0x0f);
    m_prg_offset0 = prg_bank(prg);
    m_nes->cpu_bus()->add(0x8000, &m_rom[m_prg_offset0], 0x4000,
                          PAGE_FAULT_CB(NESMapperUNROM::page_fault, this));
  }

 private:
  size_t m_prg_offset0;
  size_t m_prg_offset1;
  bvec m_chr;
};

class NESMapperGNROM : public NESMapper {
 public:
  NESMapperGNROM(NES *nes, const iNesHeader *header, bvec &rom)
      : NESMapper(nes, header, rom),
        m_prg_offset(prg_bank(0)),
        m_chr_offset(chr_bank(0)) {}

  ~NESMapperGNROM(void) {}

  virtual void reset(void) {
    m_prg_offset = prg_bank(0);
    m_chr_offset = chr_bank(0);
    m_nes->cpu_bus()->add(0x8000, &m_rom[m_prg_offset], 0x8000,
                          PAGE_FAULT_CB(NESMapperGNROM::page_fault, this));
    m_nes->ppu_bus()->add(0x0000, &m_rom[m_chr_offset], 0x2000, false);
  }

  void page_fault(offset_t offset, byte_t value) {
    int chr = (value & 0x03);
    int prg = (value & 0x30) >> 3;
    m_prg_offset = prg_bank(prg);
    m_chr_offset = chr_bank(chr);
    m_nes->cpu_bus()->add(0x8000, &m_rom[m_prg_offset], 0x8000,
                          PAGE_FAULT_CB(NESMapperGNROM::page_fault, this));
    m_nes->ppu_bus()->add(0x0000, &m_rom[m_chr_offset], 0x2000, false);
  }

 private:
  size_t m_prg_offset;
  size_t m_chr_offset;
};

class NESMapperMMC1 : public NESMapper {
 public:
  NESMapperMMC1(NES *nes, const iNesHeader *header, bvec &rom)
      : NESMapper(nes, header, rom),
        m_prg_offset0(prg_bank(0)),
        m_prg_offset1(prg_bank(header->rom_banks - 1)),
        m_chr_offset(chr_bank(0)),
        m_shift(0),
        m_shift_count(0) {
    m_ram.resize(0x4000);

    if (m_header.battery) {
      m_battery_ram.resize(0x2000);
    }
    update_mappings();
  }

  ~NESMapperMMC1(void) {}

  virtual void reset(void) {
    m_prg_offset0 = prg_bank(0);
    m_prg_offset1 = prg_bank(m_header.rom_banks - 1);
    m_chr_offset = chr_bank(0);
    m_shift = 0;
    m_shift_count = 0;
    std::fill(m_ram.begin(), m_ram.end(), 0);
    std::fill(m_battery_ram.begin(), m_battery_ram.end(), 0);

    update_mappings();
  }

  void page_fault(offset_t offset, byte_t value) {
    if (value & 0x80) {
      m_shift_count = 0;
      m_shift = 0;
    } else if (m_shift_count < 4) {
      m_shift = bit_isset(value, 0) << m_shift_count | m_shift;
      m_shift_count++;
    } else {
      switch (offset >> 13) {
        case 0: {
          machine()->write_ioport("MIRRORING", m_shift & 0x03);
          /* XXX: Remaing bits */
          break;
        }
        case 1: {
          int bank = (m_shift & 0x1f);
          m_chr_offset = chr_bank(bank);
          break;
        }
        case 2:
          break;
        case 3: {
          int bank = (m_shift & 0x0f);
          m_prg_offset0 = prg_bank(bank);
          break;
        }
      }
      m_shift_count = 0;
      m_shift = 0;
    }
    update_mappings();
  }

  void update_mappings(void) {
    m_nes->cpu_bus()->add(0x8000, &m_rom[m_prg_offset0], 0x4000,
                          PAGE_FAULT_CB(NESMapperMMC1::page_fault, this));
    m_nes->cpu_bus()->add(0xC000, &m_rom[m_prg_offset1], 0x4000,
                          PAGE_FAULT_CB(NESMapperMMC1::page_fault, this));
    m_nes->ppu_bus()->add(0x0000, &m_ram[m_chr_offset], 0x2000, false);
    m_nes->cpu_bus()->add(0x6000, &m_battery_ram[0], 0x2000, false);
  }

 private:
  size_t m_prg_offset0;
  size_t m_prg_offset1;
  size_t m_chr_offset;

  byte_t m_shift;
  int m_shift_count;

  bvec m_ram;

  bvec m_battery_ram;
};

class NESMapperMMC3 : public NESMapper {
 public:
  NESMapperMMC3(NES *nes, const iNesHeader *header, bvec &rom)
      : NESMapper(nes, header, rom),
        m_irq_counter(0),
        m_irq_reload(0),
        m_irq_enable(false) {
    m_ram.resize(0x4000);
    m_battery_ram.resize(0x2000);

    m_nes->cpu_bus()->add(0x6000, &m_battery_ram[0], 0x2000, false);
    reset();
  }

  ~NESMapperMMC3(void) {}

  virtual void reset(void) {
    m_irq_counter = 0;
    m_irq_reload = 0;
    m_irq_enable = 0;

    std::fill(m_ram.begin(), m_ram.end(), 0);
    std::fill(m_battery_ram.begin(), m_battery_ram.end(), 0);

    for (int i = 0; i < 8; i++) m_chr_offset[i] = chr_bank1k(i);

    m_prg_offset[0] = m_prg_offset[2] =
        prg_bank8k((m_header.rom_banks - 1) * 2);
    m_prg_offset[1] = m_prg_offset[3] =
        prg_bank8k((m_header.rom_banks - 1) * 2 + 1);
    update_mappings();
  }

  virtual void line(Line line, LineState state) {
    switch (line) {
      case Line::RESET:
        reset();
        break;
      case Line::INT0:
        if (state == LineState::Pulse) {
          if (m_irq_counter == 0) {
            m_irq_counter = m_irq_reload;
          } else if (--m_irq_counter == 0 && m_irq_enable) {
            machine()->set_line("cpu", Line::INT0, LineState::Assert);
          }
        }
        break;
      default:
        break;
    }
  }

  void page_fault(offset_t offset, byte_t value) {
    switch (offset) {
      case 0x8000:
        m_command = value;
        break;
      case 0x8001: {
        int bank = 0;
        switch (m_command & 0x07) {
          case 0:
            if (bit_isset(m_command, 7)) bank += 4;
            m_chr_offset[bank + 0] = chr_bank1k(value);
            m_chr_offset[bank + 1] = chr_bank1k(value + 1);
            break;
          case 1:
            if (bit_isset(m_command, 7)) bank += 4;
            m_chr_offset[bank + 2] = chr_bank1k(value);
            m_chr_offset[bank + 3] = chr_bank1k(value + 1);
            break;
          case 2:
          case 3:
          case 4:
          case 5:
            bank = (m_command & 0x07) + 2;
            if (bit_isset(m_command, 7)) bank -= 4;
            m_chr_offset[bank] = chr_bank1k(value);
            break;
          case 6:
            if (bit_isset(m_command, 6))
              m_prg_offset[2] = prg_bank8k(value);
            else
              m_prg_offset[0] = prg_bank8k(value);
            break;
          case 7:
            m_prg_offset[1] = prg_bank8k(value);
            break;
        }
        break;
      }
      case 0xA000:
        machine()->write_ioport("MIRRORING", bit_isset(value, 0)
                                                 ? TwoScreenHMirroring
                                                 : TwoScreenVMirroring);
        break;
      case 0xA001:
        /* XXX: SRAM */
        break;
      case 0xC000:
        m_irq_reload = value - 1;
        break;
      case 0xC001:
        m_irq_counter = 0;
        break;
      case 0xE000:
        m_irq_enable = false;
        machine()->set_line("cpu", Line::INT0, LineState::Clear);
        break;
      case 0xE001:
        m_irq_enable = true;
        break;
    }
    // XXX: Offsets are wrong
    update_mappings();
  }

  void update_mappings(void) {
    for (int i = 0; i < 4; i++)
      m_nes->cpu_bus()->add((i * 0x2000) + 0x8000, &m_rom[m_prg_offset[i]],
                            0x2000,
                            PAGE_FAULT_CB(NESMapperMMC3::page_fault, this));
    for (int i = 0; i < 8; i++)
      m_nes->ppu_bus()->add((i * 0x0400), &m_rom[m_chr_offset[i]], 0x0400,
                            false);
  }

 private:
  size_t m_prg_offset[4];
  size_t m_chr_offset[8];

  byte_t m_command;

  byte_t m_irq_counter;
  byte_t m_irq_reload;
  bool m_irq_enable;

  bvec m_ram;
  bvec m_battery_ram;
};
enum class Mapper {
  None = 0,
  MMC1 = 1,
  UNROM = 2,
  MMC3 = 4,
  RAMBO1 = 64,
  GNROM = 66,
};

mapper_ptr NESMachine::load_cartridge(NES *nes, const std::string &name) {
  bvec rom;
  iNesHeader header;
  mapper_ptr mapper;

  read_rom(name, rom);

  memcpy(&header, rom.data(), sizeof(header));

  Mapper mapper_type;
  /* Handle broken !DiskDude! format */
  if (rom[7] == 0x44)
    mapper_type = Mapper(header.mapper_low);
  else
    mapper_type = Mapper(header.mapper_low | (header.mapper_high << 4));

  switch (mapper_type) {
    case Mapper::None:
      mapper = mapper_ptr(new NESMapperNone(nes, &header, rom));
      break;
    case Mapper::MMC1:
      mapper = mapper_ptr(new NESMapperMMC1(nes, &header, rom));
      break;
    case Mapper::UNROM:
      mapper = mapper_ptr(new NESMapperUNROM(nes, &header, rom));
      break;
    case Mapper::MMC3:
      mapper = mapper_ptr(new NESMapperMMC3(nes, &header, rom));
      break;
    case Mapper::GNROM:
    case Mapper::RAMBO1:
      mapper = mapper_ptr(new NESMapperGNROM(nes, &header, rom));
      break;
    default:
      throw CpuFeatureFault("mapper", "unknown mapper");
  }

  return mapper;
}
