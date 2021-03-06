/*
 * Copyright (c) 2016, Dan Sledz
 */

#include "cpu/z80/z80.h"
#include "emu/device.h"
#include "emu/emu.h"

#include "machine/sbc/m6850.h"

using namespace EMU;
using namespace Z80;
using namespace Device;

namespace Arcade {

class SingleBoardZ80 : public Machine {
 public:
  SingleBoardZ80(void);
  virtual ~SingleBoardZ80(void);

  virtual void load_rom(const std::string &rom) {}

 private:
  byte_t io_read(offset_t offset);
  void io_write(offset_t offset, byte_t value);

 private:
  void init_bus();

  Hertz m_hertz;
  Z80State m_state;
  Z80Cpu_ptr m_cpu;
  Z80Bus_ptr m_bus;
  Z80IOBus_ptr m_io;
  RamDevice m_ram;
  RomSet m_romset;

 public:
  M6850_ptr m_acia;

  bool m_acia_irq;
};
};
