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
#include "namco06.h"

using namespace EMU;

Namco06::Namco06(Machine *machine, Device *parent):
    Device(machine, "06xx"),
    _parent(parent),
    _children(),
    _timer(new Timer( [=]() {
            _parent->set_line(InputLine::NMI, LineState::Pulse);
        },
        Time(usec(200)))),
    _control(0x00)
{
}

Namco06::~Namco06(void)
{
}

void
Namco06::add_child(int pos, Device *child)
{
    _children[pos] = child;
}

void
Namco06::set_line(InputLine line, LineState state)
{
    switch (line) {
    case InputLine::INT0:
        DEBUG("06xx INT0");
        break;
    case InputLine::INT1:
        DEBUG("06xx INT1");
        break;
    case InputLine::INT2:
        DEBUG("06xx INT2");
        break;
    case InputLine::INT3:
        DEBUG("06xx INT3");
        break;
    case InputLine::RESET:
        DEBUG("06xx RESET");
        break;
    case InputLine::NMI:
        DEBUG("06xx NMI");
        break;
    default:
        DEBUG("Unrecognized signal");
        break;
    }
}

void
Namco06::write(addr_t addr, byte_t value)
{
    switch (addr) {
    case 0x7100:
        _control = value;
        _machine->remove_timer(_timer);
        if (_control == 0x10) {
            DEBUG("06xx Disabled");
        } else if ((_control & 0x0F) != 0) {
            DEBUG("06xx Enabled");
            _machine->add_timer(_timer);
        }
        break;
    default:
        if ((_control & 0x10) != 0x00) {
            DEBUG("Write while in read mode!");
            return;
        }
        DEBUG("06xx WRITE");
        for (int dev = 0; dev < 4; dev++) {
            if (bit_isset(_control, dev) && _children[dev] != NULL)
                _children[dev]->write(0, value);
        }
        break;
    }
}

byte_t
Namco06::read(addr_t addr)
{
    byte_t result = 0xff;
    switch (addr) {
    case 0x7100:
        result = _control;
        break;
    default:
        if ((_control & 0x10) != 0x10) {
            DEBUG("Read while in write mode!");
            return 0;
        }
        for (int dev = 0; dev < 4; dev++) {
            if (bit_isset(_control, dev) && _children[dev] != NULL)
                result &= _children[dev]->read(0);
        }
        break;
    }
    return result;
}


