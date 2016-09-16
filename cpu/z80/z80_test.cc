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

#include "emu/rom.h"
#include "emu/test.h"

#include "cpu/z80/z80.h"

using namespace EMU;
using namespace EMUTest;
using namespace Z80;

#if 0
typedef TestMachine<Z80Cpu, 0x0000> Z80Machine;

TEST(Z80Test, opcode_dd)
{
    Z80Machine machine;

    LOAD4(0xDD, 0x21, 0x34, 0x12);  /* LD IX, $1234 */

}
#endif

class Zexall : public Machine {
 public:
  Zexall(void)
      : bus(new AddressBus16x8()),
        cpu(new Z80Cpu(this, "cpu", 1000000, bus.get())),
        rom("zex.bin"),
        m_data(0),
        m_req(0),
        m_req_last(0),
        m_ack(0) {
    cpu->load_rom(&rom, 0x0000);

    bus->add(0xFFFF, &m_data);
    bus->add(0xFFFE, 0xFFFF, READ_CB(Zexall::req_read, this),
             WRITE_CB(Zexall::req_write, this));
    bus->add(0xFFFD, &m_ack);

    cpu->io()->add(0x01, 0x01, AddressBus8x8::DefaultRead(),
                   AddressBus8x8::DefaultWrite());
  }
  ~Zexall(void) {}

  byte_t req_read(byte_t vlaue) { return m_req; }

  void req_write(offset_t offset, byte_t value) {
    if (m_req_last != value) {
      m_ack++;
      std::cout << m_data;
      std::cout.flush();
    }
    m_req_last = m_req;
    m_req = value;
  }

  std::unique_ptr<AddressBus16x8> bus;
  std::unique_ptr<Z80Cpu> cpu;
  Rom rom;
  byte_t m_data, m_req, m_req_last, m_ack;
};

TEST(Zexall_test, test) {
  Zexall machine;
  EmuTime runtime = get_runtime("Z80_RUNTIME");

  Core::log.set_level(LogLevel::Debug);

  // Run the first few seconds of the rom
  machine.poweron();
  machine.reset();
  machine.run_forward(runtime);
  machine.poweroff();
}
