/*
 * Copyright (c) 2016, Dan Sledz
 */

#include "cpu/z80/z80v2.h"
#include "emu/device.h"
#include "emu/emu.h"

#include "machine/sbc/m6850.h"

using namespace EMU;
using namespace Z80v2;
using namespace Device;

namespace Arcade {

class SingleBoardZ80 : public Machine {
 public:
  SingleBoardZ80(const std::string &rom);
  virtual ~SingleBoardZ80(void);

 private:
  byte_t io_read(offset_t offset);
  void io_write(offset_t offset, byte_t value);

 private:
  void init_bus();

  Hertz m_hertz;
  Z80State m_state;
  Z80Cpu_ptr m_cpu;
  AddressBus16x8_ptr m_bus;
  std::unique_ptr<AddressBus8x8> m_io;
  RamDevice m_ram;
  RomSet m_romset;

 public:
  M6850_ptr m_acia;

  bool m_acia_irq;
};
};
