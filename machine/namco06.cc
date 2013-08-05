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
#include "machine/namco06.h"

using namespace EMU;

Namco06::Namco06(Machine *machine, Device *parent):
    Device(machine, "06xx"),
    _parent(parent),
    _children(),
    _control(0x00)
{
    _timer = Timer_ptr(new Timer(
        [=]() {
            _machine->set_line(_parent, Line::NMI, LineState::Pulse);
        },
        Time(usec(200))));
}

Namco06::~Namco06(void)
{
}

void
Namco06::add_child(int pos, IODevice *child)
{
    _children[pos] = child;
}

byte_t
Namco06::read_child(addr_t addr)
{
    byte_t result = 0xff;
    if ((_control & 0x10) != 0x10)
        return 0;
    for (int dev = 0; dev < 4; dev++)
        if (bit_isset(_control, dev) && _children[dev] != NULL)
            result &= _children[dev]->read8(0);
    return result;
}

void
Namco06::write_child(addr_t addr, byte_t value)
{
    if ((_control & 0x10) != 0x00)
        return;
    for (int dev = 0; dev < 4; dev++)
        if (bit_isset(_control, dev) && _children[dev] != NULL)
            _children[dev]->write8(0, value);
}

byte_t
Namco06::read_control(addr_t addr)
{
    return _control;
}

void
Namco06::write_control(addr_t addr, byte_t value)
{
    _control = value;
    _machine->remove_timer(_timer);
    if ((_control & 0x0F) != 0)
        _machine->add_timer(_timer);
}

void
Namco06::line(Line line, LineState state)
{
    switch (line) {
    case Line::RESET:
        DBG("06xx RESET");
        break;
    default:
        DBG("Unrecognized signal");
        break;
    }
}


