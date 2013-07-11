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
#pragma once

#include "bits.h"
#include "state.h"
#include "input.h"
#include <list>

namespace EMU {

enum class DeviceState {
    Running = 0,
    Halted = 1,
    Stopped = 2,
    Fault = 3,
};

class Machine;

/**
 * Emulation device. Specific chips implement the device class.
 */
class Device {
public:
    Device(Machine *machine, const std::string &name);
    virtual ~Device(void);

    Machine *machine(void) {
        return _machine;
    }
    const std::string &name(void) {
        return _name;
    }
    /**
     * Save the device state.
     */
    virtual void save(SaveState &state) = 0;
    /**
     * Restore the device state.
     */
    virtual void load(LoadState &state) = 0;
    /**
     * Run the device for @a cycles.
     */
    virtual void tick(unsigned cycles) = 0;
    /**
     * Signal one of the external lines.
     */
    virtual void set_line(InputLine line, LineState state) = 0;
    /**
     * Write a single byte to the IO port.
     */
    virtual void write(addr_t addr, byte_t value) = 0;
    /**
     * Read a single byte from the IO port.
     */
    virtual byte_t read(addr_t addr) = 0;

protected:
    Machine *_machine;
    std::string _name;
};

class CpuDevice {
    CpuDevice(Machine *machine, const std::string &name);
    virtual ~CpuDevice(void);

};

};

