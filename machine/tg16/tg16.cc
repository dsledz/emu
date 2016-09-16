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

#include "machine/tg16/tg16.h"

using namespace TG16Machine;

/* XXX wrong */
#define MASTER_CLOCK 21477270

TG16::TG16(void)
    : Machine(),
      m_cpu_bus(),
      m_ram(this, "ram", 0x2000),
      m_vdc(this, MASTER_CLOCK),
      m_psg(this),
      m_rom(0x100000) {

  init_joypad();

  /* XXX: Is this screen correct? */
  add_screen(256, 240);

  m_cpu = std::unique_ptr<cpu_type>(
      new cpu_type(this, "cpu", MASTER_CLOCK / 3, &m_cpu_bus));

  m_cpu_bus.add(0x000000, m_rom);

  m_cpu_bus.add(0x1F0000, &m_ram);

  m_cpu_bus.add(0x1FE000, 0x1FE3FF, READ_CB(VDC::read, &m_vdc),
                WRITE_CB(VDC::write, &m_vdc));

  m_cpu_bus.add(0x1FE400, 0x1FE7FF, READ_CB(VDC::vce_read, &m_vdc),
                WRITE_CB(VDC::vce_write, &m_vdc));

  m_cpu_bus.add(0x1FE800, 0x1FEBFF, READ_CB(PSG::read, &m_psg),
                WRITE_CB(PSG::write, &m_psg));

  m_cpu_bus.add(0x1FEC00, 0x1FEFFF,
                READ_CB(cpu_type::timer_read, m_cpu.get()),
                WRITE_CB(cpu_type::timer_write, m_cpu.get()));

  m_cpu_bus.add(0x1FF000, 0x1FF3FF, READ_CB(TG16::joypad_read, this),
                WRITE_CB(TG16::joypad_write, this));

  m_cpu_bus.add(0x1FF400, 0x1FF7FF,
                READ_CB(cpu_type::irq_read, m_cpu.get()),
                WRITE_CB(cpu_type::irq_write, m_cpu.get()));
}

TG16::~TG16(void) {}

void
TG16::load_rom(const std::string &rom) {
  LOG_INFO("Loading: ", rom);
  bvec rom_data;

  EMU::read_rom(rom, rom_data);
  offset_t rom_offset = rom_data.size() % 8192;

  if (rom_data.at(rom_offset + 0x1FFF) < 0xe0)
    throw RomException("Encrypted Rom");
  if (rom_data.size() - rom_offset == 0x60000) {
    /* int bank = offset >> 17; */
    /* 0 & 2 -> 0x00000 */
    memcpy(&m_rom[0x00000], &rom_data[rom_offset], 0x20000);
    memcpy(&m_rom[0x40000], &rom_data[rom_offset], 0x20000);
    /* 1 & 3 -> 0x20000 */
    memcpy(&m_rom[0x20000], &rom_data[rom_offset + 0x20000], 0x20000);
    memcpy(&m_rom[0x60000], &rom_data[rom_offset + 0x20000], 0x20000);
    /* rest -> 0x40000 */
    memcpy(&m_rom[0x80000], &rom_data[rom_offset + 0x40000], 0x20000);
    memcpy(&m_rom[0xA0000], &rom_data[rom_offset + 0x40000], 0x20000);
    memcpy(&m_rom[0xC0000], &rom_data[rom_offset + 0x40000], 0x20000);
    memcpy(&m_rom[0xE0000], &rom_data[rom_offset + 0x40000], 0x20000);
  } else if (rom_data.size() - rom_offset > 0x100000) {
    throw RomException("Unsupported ROM size");
  } else {
    memcpy(&m_rom.front(), &rom_data[rom_offset], rom_data.size() - rom_offset);
  }
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

void TG16::init_joypad(void) {
  IOPort *port = NULL;

  add_ioport("JOYPAD1");
  port = ioport("JOYPAD1");
  add_input(InputSignal(InputKey::Joy1Btn2, port, TG16Key::II, false));
  add_input(InputSignal(InputKey::Joy1Btn1, port, TG16Key::I, false));
  add_input(InputSignal(InputKey::Select1, port, TG16Key::Select, false));
  add_input(InputSignal(InputKey::Start1, port, TG16Key::Run, false));
  add_input(InputSignal(InputKey::Joy1Up, port, TG16Key::Up, false));
  add_input(InputSignal(InputKey::Joy1Down, port, TG16Key::Down, false));
  add_input(InputSignal(InputKey::Joy1Left, port, TG16Key::Left, false));
  add_input(InputSignal(InputKey::Joy1Right, port, TG16Key::Right, false));
  m_joypad_data = 0;
  m_joypad = 0;
}

byte_t TG16::joypad_read(offset_t offset) {
  byte_t result = 0xFF;
  if (m_joypad == 0) {
    result = read_ioport("JOYPAD1");
  }

  /* XXX: Is this correct? */
  if (m_joypad_data) result >>= 4;

  result = (result & 0x0F) | 0x30;

  return result;
}

void TG16::joypad_write(offset_t offset, byte_t value) {
  if (!m_joypad_data && bit_isset(value, 0)) m_joypad = (m_joypad + 1) % 8;

  m_joypad_data = bit_isset(value, 0);

  if (bit_isset(value, 1)) m_joypad = 0;
}

PSG::PSG(TG16 *tg16) : Device(tg16, "psg") {}

PSG::~PSG(void) {}

byte_t PSG::read(offset_t offset) { return 0; }

void PSG::write(offset_t offset, byte_t value) {}

MachineInformation tg16_info{
    .name = "TurboGrafx-16",
    .year = "1989",
    .cartridge = true,
    .extension = "pce",
};

MachineDefinition tg16("tg16", tg16_info, [](Options *opts) -> machine_ptr {
  return machine_ptr(new TG16Machine::TG16());
});
