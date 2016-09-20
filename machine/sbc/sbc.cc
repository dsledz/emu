/*
 * Copyright (c) 2016, Dan Sledz
 */

#include "machine/sbc/sbc.h"
#include "emu/emu.h"

using namespace EMU;
using namespace Z80;
using namespace Arcade;
using namespace Device;

RomDefinition sbc_rom(void) {
  RomDefinition rom("sbc");
  rom.regions.push_back(RomRegion("cpu", {"INTMINI2.BIN", "BASIC.BIN"}));
  // rom.regions.push_back(RomRegion("cpu", { "INTMINI.bin" }));
  return rom;
}

SingleBoardZ80::SingleBoardZ80(void)
    : Machine(Hertz(7372800)),
      m_hertz(7372800),
      m_state(),
      m_bus(nullptr),
      m_io(nullptr),
      m_ram(this, "ram", 0xD000),
      m_romset(sbc_rom()),
      m_acia(nullptr),
      m_acia_irq(false) {

  m_bus = AddressBus16x8_ptr(new AddressBus16x8());
  m_io = std::unique_ptr<AddressBus8x8>(new AddressBus8x8());
  m_acia = M6850_ptr(new M6850(this, "m6850", m_hertz.v));
  m_state.bus = m_bus.get();
  m_state.io = m_io.get();

  m_cpu = Z80Cpu_ptr(new Z80Cpu(this, "cpu", m_hertz.v, &m_state));

  init_bus();
}

SingleBoardZ80::~SingleBoardZ80(void) {}

byte_t SingleBoardZ80::io_read(offset_t offset) {
  byte_t value = 0;
  /* XXX: Fix address check */
  if (true || (offset & 0xC0) == 0x80) {
    bool rs = offset & 0x01;
    value = m_acia->read(rs);
  }
  return value;
}

void SingleBoardZ80::io_write(offset_t offset, byte_t value) {
  /* XXX: Fix address check */
  if (true || (offset & 0xC0) == 0x80) {
    bool rs = offset & 0x01;
    m_acia->write(rs, value);
    m_state.yield = true;
  }
}

void SingleBoardZ80::init_bus(void) {
  m_bus->add(0x0000, m_romset.rom("cpu"));
  m_bus->add(0x2000, &m_ram);

  m_io->add(0x00, 0xff, READ_CB(SingleBoardZ80::io_read, this),
            WRITE_CB(SingleBoardZ80::io_write, this));
}
