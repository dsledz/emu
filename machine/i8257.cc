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

#include "machine/i8257.h"

using namespace EMU;

I8257::I8257(Machine *machine, const std::string &name, unsigned hertz,
             AddressBus16 *bus)
    : ClockedDevice(machine, name, hertz), _flip_flop(0), _bus(bus) {}

I8257::~I8257(void) {}

void I8257::execute(void) {
  while (true) {
    add_icycles(100);
  }
}

void I8257::write_cb(offset_t offset, byte_t value) {
  switch (offset) {
    case 0x00:
    case 0x02:
    case 0x04:
    case 0x06:
      set_byte(_channels[offset >> 1].address, _flip_flop, value);
      _flip_flop = !_flip_flop;
      break;
    case 0x01:
    case 0x03:
    case 0x05:
    case 0x07:
      set_byte(_channels[offset >> 1].count, _flip_flop, value);
      _flip_flop = !_flip_flop;
      break;
    case 0x08:
      _mode.val = value;
      break;
    default:
      throw DeviceFault(name(), "Invalid write");
  }
}

byte_t I8257::read_cb(offset_t offset) {
  switch (offset) {
    case 0x09:
      return _status;
  }
  throw DeviceFault(name(), "Invalid read");
}

void I8257::dma(void) {
  Channel *rd = NULL;
  Channel *wr = NULL;
  for (int i = 0; i < 4; i++) {
    if (bit_isset(_mode.val, i)) {
      if (rd == NULL && (_channels[i].count.d & 0x4000))
        rd = &_channels[i];
      else if (wr == NULL && (_channels[i].count.d & 0x8000))
        wr = &_channels[i];
    }
    if (rd && wr) break;
  }
  if (!rd || !wr) return;

  for (unsigned i = 0; i < (rd->count.d & 0x3fff); i++)
    _bus->write(wr->address.d + i, _bus->read(rd->address.d + i));
}
