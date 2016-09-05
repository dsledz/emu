/*
 * Copyright (c) 2016, Dan Sledz
 */

#include "emu/emu.h"
#include "emu/device.h"
#include "cpu/z80/z80.h"

#include "machine/sbc/m6850.h"

using namespace EMU;
using namespace Z80;
using namespace Device;

namespace Arcade {

class SingleBoardZ80: public Machine
{
public:
    SingleBoardZ80(const std::string &rom);
    virtual ~SingleBoardZ80(void);

private:

    byte_t io_read(offset_t offset);
    void io_write(offset_t offset, byte_t value);

private:

    void init_bus();

    Z80Cpu_ptr m_cpu;
    AddressBus16_ptr m_bus;
    RamDevice m_ram;

public:
    M6850_ptr m_acia;

    bool m_acia_irq;
};

};
