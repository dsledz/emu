/*
 * Copyright (c) 2016, Dan Sledz
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

#include "cpu/z80/z80.h"
#include "emu/test.h"

using namespace EMU;
using namespace EMUTest;
using namespace Z80;

class Zexall : public Machine {
 public:
  Zexall(void)
      : Machine(Hertz(18432000)),
        bus(new Z80Bus()),
        io(new Z80IOBus()),
        state(bus.get(), io.get()),
        cpu(new Z80Cpu(this, "cpu", ClockDivider(2), &state)),
        rom("z80", "zex.bin"),
        ram(),
        m_data(0),
        m_req(0),
        m_req_last(0),
        m_ack(0) {
    ram.assign(rom.cbegin(), rom.cend());
    ram.resize(0xF000);
    bus->add(0x0000, ram);

    bus->add(0xFFFF, &m_data);
    bus->add(0xFFFE, 0xFFFF, READ_CB(Zexall::req_read, this),
             WRITE_CB(Zexall::req_write, this));
    bus->add(0xFFFD, &m_ack);

    io->add(0x01, 0x01, AddressBus8x8::DefaultRead(),
            AddressBus8x8::DefaultWrite());
  }
  ~Zexall(void) {}

  virtual void load_rom(const std::string &rom) { }
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
  std::unique_ptr<AddressBus8x8> io;
  Z80State state;
  std::unique_ptr<Z80Cpu> cpu;
  Rom rom;
  bvec ram;
  byte_t m_data, m_req, m_req_last, m_ack;
};

TEST(Zexall_test, test) {
  machine_test<Zexall>();
}
