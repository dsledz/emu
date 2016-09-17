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

#include "machine/galaga.h"

using namespace EMU;
using namespace Z80;
using namespace Arcade;
using namespace Device;

RomDefinition galaga_rom(void) {
  RomDefinition rom("galaga");
  rom.regions.push_back(RomRegion(
      "maincpu", {"gg1_1b.3p", "gg1_2b.3m", "gg1_3.2m", "gg1_4b.2l"}));
  rom.regions.push_back(RomRegion("subcpu", {"gg1_5b.3f"}));
  rom.regions.push_back(RomRegion("sndcpu", {"gg1_7b.2c"}));
  rom.regions.push_back(RomRegion("prom1", {"prom-5.5n"}));
  rom.regions.push_back(RomRegion("prom2", {"prom-4.2n", "prom-3.1c"}));
  rom.regions.push_back(RomRegion("tiles", {"gg1_9.4l"}));
  rom.regions.push_back(RomRegion("sprites", {"gg1_11.4d", "gg1_10.4f"}));
  return rom;
};

RomDefinition galagao_rom(void) {
  RomDefinition rom("galaga");
  rom.regions.push_back(
      RomRegion("maincpu", {"gg1-1.3p", "gg1-2.3m", "gg1-3.2m", "gg1-4.2l"}));
  rom.regions.push_back(RomRegion("subcpu", {"gg1-5.3f"}));
  rom.regions.push_back(RomRegion("sndcpu", {"gg1-7.2c"}));
  rom.regions.push_back(
      RomRegion("palette", {"prom-5.5n", "prom-4.2n", "prom-3.1c"}));
  rom.regions.push_back(RomRegion("tiles", {"gg1-9.4l"}));
  rom.regions.push_back(RomRegion("sprites", {"gg1-11.4d", "gg1-10.4f"}));
  return rom;
};

Galaga::Galaga(void)
    : Machine(),
      ram1(this, "ram1", 0x0400),
      ram2(this, "ram2", 0x0400),
      ram3(this, "ram3", 0x0400),
      m_main_cpu_state(),
      m_main_cpu(nullptr),
      m_bus1(nullptr),
      m_sub_cpu_state(),
      m_sub_cpu(nullptr),
      m_bus2(nullptr),
      m_snd_cpu_state(),
      m_snd_cpu(nullptr),
      m_bus3(nullptr),
      m_iobus(nullptr),
      m_main_irq(false),
      m_sub_irq(false),
      m_snd_nmi(false) {
  unsigned hertz = 18432000;

  add_screen(224, 288, GfxScale::None, FrameBuffer::ROT90);

  m_iobus = Z80IOBus_ptr(new Z80IOBus());
  m_bus1 = Z80Bus_ptr(new Z80Bus());
  m_main_cpu_state.bus = m_bus1.get();
  m_main_cpu_state.io = m_iobus.get();
  m_main_cpu =
      Z80Cpu_ptr(new Z80Cpu(this, "maincpu", hertz / 6, &m_main_cpu_state));

  m_bus2 = Z80Bus_ptr(new Z80Bus());
  m_sub_cpu_state.bus = m_bus2.get();
  m_sub_cpu_state.io = m_iobus.get();
  m_sub_cpu =
      Z80Cpu_ptr(new Z80Cpu(this, "subcpu", hertz / 6, &m_sub_cpu_state));

  m_bus3 = Z80Bus_ptr(new Z80Bus());
  m_snd_cpu_state.bus = m_bus3.get();
  m_snd_cpu_state.io = m_iobus.get();
  m_snd_cpu =
      Z80Cpu_ptr(new Z80Cpu(this, "sndcpu", hertz / 6, &m_snd_cpu_state));

  init_switches();
  reset_switches();

  init_controls();

  m_namco06 = Namco06_ptr(new Namco06(this, m_main_cpu.get()));

  m_namco51 = Namco51_ptr(new Namco51(this));
  m_namco06->add_child(0, m_namco51.get());

  m_gfx = GalagaGfx_ptr(new GalagaGfx(this, "gfx", hertz, m_bus1.get()));

  m_gfx->register_callback(64, [&](void) {
    if (m_snd_nmi) set_line("sndcpu", Line::NMI, LineState::Pulse);
  });

  m_gfx->register_callback(196, [&](void) {
    if (m_snd_nmi) set_line("sndcpu", Line::NMI, LineState::Pulse);
  });

  m_gfx->register_callback(240, [&](void) {
    if (m_main_irq) set_line("maincpu", Line::INT0, LineState::Assert);
    if (m_sub_irq) set_line("subcpu", Line::INT0, LineState::Assert);
  });

  init_bus(m_bus1.get());
  init_bus(m_bus2.get());
  init_bus(m_bus3.get());
}

Galaga::~Galaga(void) {}

void Galaga::load_rom(const std::string &rom) {

  RomDefinition roms("galaga");
  if (rom == "galagao") {
    roms = galagao_rom();
  } else {
    roms = galaga_rom();
  }

  m_romset.load(roms);

  m_bus1->add(0x0000, m_romset.rom("maincpu"));
  m_bus2->add(0x0000, m_romset.rom("subcpu"));
  m_bus3->add(0x0000, m_romset.rom("sndcpu"));
  m_gfx->init(&m_romset);
}

byte_t Galaga::dips_read(offset_t offset) {
  byte_t dswa = read_ioport("DSWA");
  byte_t dswb = read_ioport("DSWB");
  return (bit_isset(dswa, offset) << 1) | bit_isset(dswb, offset);
}

void Galaga::latch_write(offset_t offset, byte_t value) {
  switch (offset) {
    case 0:
      m_main_irq = bit_isset(value, 0);
      if (!m_main_irq) set_line("maincpu", Line::INT0, LineState::Clear);
      break;
    case 1:
      m_sub_irq = bit_isset(value, 0);
      if (!m_sub_irq) set_line("subcpu", Line::INT0, LineState::Clear);
      break;
    case 2:
      m_snd_nmi = !bit_isset(value, 0);
      break;
    case 3:
      set_line("subcpu", Line::RESET, LineState::Pulse);
      set_line("sndcpu", Line::RESET, LineState::Pulse);
      break;
  }
}

void Galaga::init_bus(Z80Bus *bus) {
  /* Dipswitches */
  bus->add(0x6800, 0x6807, READ_CB(Galaga::dips_read, this),
             Z80Bus::DefaultWrite());
  /* XXX: Sound */
  bus->add(0x6808, 0x681F);
  bus->add(0x6820, 0x6827, Z80Bus::DefaultRead(),
             WRITE_CB(Galaga::latch_write, this));
  /* XXX: Watchdog */
  bus->add(0x6830, 0x6830);

  /* Namco06 (and children) */
  bus->add(0x7000, 0x70FF, READ_CB(Namco06::read_child, m_namco06.get()),
             WRITE_CB(Namco06::write_child, m_namco06.get()));
  bus->add(0x7100, READ_CB(Namco06::read_control, m_namco06.get()),
             WRITE_CB(Namco06::write_control, m_namco06.get()));

  /* Various ram */
  bus->add(0x8000, 0x87FF, READ_CB(GalagaGfx::vmem_read, m_gfx.get()),
             WRITE_CB(GalagaGfx::vmem_write, m_gfx.get()));

  bus->add(0x8800, &ram1);
  bus->add(0x9000, &ram2);
  bus->add(0x9800, &ram3);

  /* XXX: Star control */
  bus->add(0xA000, 0xA007);
}

void Galaga::init_switches(void) {
  dipswitch_ptr sw;

  add_ioport("DSWA");
  add_ioport("DSWB");

  sw = add_switch("Difficulty", "DSWA", 0x03, 0x00);
  sw->add_option("Easy", 0x03);
  sw->add_option("Medium", 0x00);
  sw->add_option("Hard", 0x01);
  sw->add_option("Hardest", 0x02);

  /* We hang after POST until this is set */
  sw = add_switch("Unknown", "DSWA", 0x04, 0x04);

  sw = add_switch("Demo Sounds", "DSWA", 0x08, 0x08);
  sw->add_option("Off", 0x00);
  sw->add_option("On", 0x08);

  sw = add_switch("Pause", "DSWA", 0x10, 0x10);
  sw->add_option("Off", 0x10);
  sw->add_option("On", 0x00);

  sw = add_switch("Rack Test", "DSWA", 0x20, 0x20);
  sw->add_option("Off", 0x20);
  sw->add_option("On", 0x00);

  sw = add_switch("Cabinet", "DSWA", 0x80, 0x80);
  sw->add_option("Upright", 0x80);
  sw->add_option("Cocktail", 0x00);

  sw = add_switch("Coinage", "DSWB", 0x07, 0x07);
  sw->add_option("4 Coins / 1 Credit", 0x04);
  sw->add_option("3 Coins / 1 Credit", 0x02);
  sw->add_option("2 Coins / 1 Credit", 0x06);
  sw->add_option("1 Coin  / 1 Credit", 0x07);
  sw->add_option("2 Coins / 3 Credit", 0x01);
  sw->add_option("1 Coin  / 2 Credit", 0x03);
  sw->add_option("1 Coin  / 3 Credit", 0x05);
  sw->add_option("Free Play", 0x00);

  sw = add_switch("Bonus Life", "DSWB", 0x38, 0x10);
  sw->add_option("None", 0x00);
  sw->add_option("30K, 100K, Every 100K", 0x08);
  sw->add_option("20, 70K, Every 70K", 0x10);
  sw->add_option("20K and 60K Only", 0x18);
  sw->add_option("20K, 60K, Every 60K", 0x20);
  sw->add_option("30K, 120K, Every 120K", 0x28);
  sw->add_option("20K, 80K, Every 80K", 0x30);
  sw->add_option("30K and 80K Only", 0x38);

  sw = add_switch("Lives", "DSWB", 0xC0, 0x80);
  sw->add_option("2", 0x00);
  sw->add_option("3", 0x80);
  sw->add_option("4", 0x40);
  sw->add_option("5", 0xC0);
}

void Galaga::init_controls(void) {
  IOPort *port;

  add_ioport("IN0");
  add_ioport("IN1");
  add_ioport("IN2");
  add_ioport("IN3");

  port = ioport("IN0");
  write_ioport(port, 0x0f);
  add_input(InputSignal(InputKey::Joy1Btn1, port, 0, false));
  add_input(InputSignal(InputKey::Joy2Btn1, port, 1, false));
  add_input(InputSignal(InputKey::Start1, port, 2, false));
  add_input(InputSignal(InputKey::Start2, port, 3, false));

  port = ioport("IN1");
  write_ioport(port, 0x0f);
  add_input(InputSignal(InputKey::Coin1, port, 0, false));
  add_input(InputSignal(InputKey::Coin2, port, 1, false));
  add_input(InputSignal(InputKey::Service, port, 2, false));

  port = ioport("IN2");
  write_ioport(port, 0x0f);
  add_input(InputSignal(InputKey::Joy1Right, port, 1, false));
  add_input(InputSignal(InputKey::Joy1Left, port, 3, false));

  port = ioport("IN3");
  write_ioport(port, 0x0f);
  add_input(InputSignal(InputKey::Joy2Right, port, 1, false));
  add_input(InputSignal(InputKey::Joy2Left, port, 3, false));
}

MachineInformation galaga_info{
    .name = "Galaga", .year = "1983",
};

MachineDefinition galaga("galaga", galaga_info,
                         [](Options *opts) -> machine_ptr {
                           return machine_ptr(new Arcade::Galaga());
                         });
