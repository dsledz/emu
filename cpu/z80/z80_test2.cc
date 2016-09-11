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

#include "cpu/z80/z80v2.h"
#include "emu/test.h"

using namespace EMU;
using namespace EMUTest;
using namespace Z80v2;

class Zexall2 : public Machine {
 public:
  Zexall2(void)
      : bus(new AddressBus16()),
        io(new AddressBus8()),
        state(),
        cpu(new Z80Cpu(this, "cpu", 1000000, &state)),
        rom("zex.bin"),
        ram(),
        m_data(0),
        m_req(0),
        m_req_last(0),
        m_ack(0) {
    state.bus = bus.get();
    state.io = io.get();
    state.Phase = CpuPhase::Interrupt;

    ram.assign(rom.cbegin(), rom.cend());
    bus->add(0x0000, ram);

    bus->add(0xFFFF, &m_data);
    bus->add(0xFFFE, 0xFFFF, READ_CB(Zexall2::req_read, this),
             WRITE_CB(Zexall2::req_write, this));
    bus->add(0xFFFD, &m_ack);

    io->add(0x01, 0x01, AddressBus8x8::DefaultRead(),
            AddressBus8x8::DefaultWrite());
  }
  ~Zexall2(void) {}

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

  std::unique_ptr<AddressBus16> bus;
  std::unique_ptr<AddressBus8> io;
  Z80State state;
  std::unique_ptr<Z80Cpu> cpu;
  Rom rom;
  bvec ram;
  byte_t m_data, m_req, m_req_last, m_ack;
};

TEST(Zexall2_test, test) {
  Zexall2 zex;

  Core::log.set_level(LogLevel::Debug);

  // Run the first few seconds of the rom
  zex.poweron();
  zex.reset();
  zex.run_forward(sec(6000));
}
