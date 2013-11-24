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

namespace EMU {

template<class _bus_type>
class Cpu: public ClockedDevice {
public:
    typedef _bus_type bus_type;
    typedef typename bus_type::addr_type addr_type;
    typedef typename bus_type::data_type data_type;

    Cpu(Machine *machine, const std::string &name, unsigned hertz,
        bus_type *bus):
        ClockedDevice(machine, name, hertz),
        _icycles(0),
        _bus(bus)
    {
    }
    virtual ~Cpu(void)
    {
    }
    Cpu(const Cpu &cpu) = delete;

    virtual void execute(void)
    {
        while (true) {
            step();
        }
    }

    virtual void line(Line line, LineState state)
    {
        Device::line(line, state);
    }

    bus_type *bus() {
        return _bus;
    }

    data_type bus_read(addr_type addr) {
        data_type tmp = _bus->read(addr);
        /* XXX: Should we always incur a cycle here? */
        add_icycles(1);
        return tmp;
    }

    void bus_write(addr_type addr, data_type value) {
        _bus->write(addr, value);
        add_icycles(1);
    }

    virtual void test_step(void) {
        step();
    }

    /* Process a single clock cycle */
    virtual void step(void) = 0;
    virtual std::string dasm(addr_type addr) = 0;

private:
    Cycles _icycles;
    bus_type *_bus;
};

};
