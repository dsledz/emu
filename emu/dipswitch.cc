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

#include "core/bits.h"

#include "emu/dipswitch.h"
#include "emu/machine.h"

using namespace EMU;

Dipswitch::Dipswitch(const std::string &name, const std::string &port,
                     byte_t mask, byte_t def):
    _name(name),
    _port(port),
    _mask(mask),
    _def(def)
{
}

Dipswitch::~Dipswitch(void)
{
}

void
Dipswitch::add_option(const std::string &name, byte_t value)
{
    _options.insert(make_pair(name, value));
}

void
Dipswitch::select(Machine *machine, const std::string &value)
{
    auto it = _options.find(value);
    if (it == _options.end())
        throw DipswitchValueException(value);
    IOPort *port = machine->ioport(_port);
    bit_setmask(port->value, _mask, it->second);
}

void
Dipswitch::set_default(Machine *machine)
{
    IOPort *port = machine->ioport(_port);
    bit_setmask(port->value, _mask, _def);
}

