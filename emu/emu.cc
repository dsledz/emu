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

#include "emu.h"

using namespace EMU;

AddressBus::AddressBus(void)
{
}

AddressBus::~AddressBus(void)
{
}

void
AddressBus::write(addr_t addr, byte_t arg)
{
    _map.find(addr).write(addr, arg);
}

byte_t
AddressBus::read(addr_t addr)
{
    return _map.find(addr).read(addr);
}

void
AddressBus::add_port(addr_t addr, const IOPort &port)
{
    _map.add(addr, port);
}

void
AddressBus::add_port(addr_t addr, addr_t mask, const IOPort &port)
{
    _map.add(addr, mask, port);
}

void
AddressBus::add_port(addr_t base, IODevice *dev)
{
    IOPort port(
        [=](addr_t addr) -> byte_t {
            assert(addr >= base);
            addr -= base;
            return dev->read8(addr);
        },
        [=](addr_t addr, byte_t v) {
              assert(addr >= base);
              addr -= base;
              dev->write8(addr, v);
        });
    addr_t mask = 0xFFFF & ~(dev->size() - 1);
    _map.add(base, mask, port);
}

Device::Device(Machine *machine, const std::string &name):
    _machine(machine), _name(name)
{
    _machine->add_device(this);
}


Device::~Device(void)
{
    _machine->remove_device(this);
}

const Time EMU::time_zero = Time(sec(0));

Debug EMU::log = Debug(std::cout);

