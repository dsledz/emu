/*
 * Copyright (c) 2013, Dan Sledz * All rights reserved.
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
/**
 * Cpu Abstraction.
 */
#pragma once

#include "emu/device.h"

using namespace EMU;

namespace CPU3 {

template <class _bus_type, class _state_type, typename latch_func,
          typename op_func>
class Cpu : public ClockedDevice {
  typedef _bus_type bus_type;
  typedef _state_type state_type;
  typedef typename bus_type::addr_type addr_type;
  typedef typename bus_type::data_type data_type;

  Cpu(Machine *machine, const std::string &name, ClockDivider divider,
      bus_type *bus)
      : ClockedDevice(machine, name, divider), m_bus(bus) {}
  virtual ~Cpu(void) {}
  Cpu(const Cpu &cpu) = delete;

  void execute(void) {
    while (true) {
      OpFunc(state(), bus());
    }
  }

  virtual void line(Line line, LineState state) { Device::line(line, state); }

  bus_type *bus(void) { return m_bus; }

  state_type *state(void) { return &m_state; }

  data_type bus_read(addr_type addr) {
    data_type tmp = m_bus->read(addr);
    return tmp;
  }

  void bus_write(addr_type addr, data_type value) { m_bus->write(addr, value); }

 protected:
  bus_type *m_bus;
  addr_type m_address;
  data_type m_data;
  state_type m_state;
};
};
