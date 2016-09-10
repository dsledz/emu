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
#include "cpu/lib/jit.h"

using namespace EMU;

namespace CPU2 {

enum class CpuPhase {
    Interrupt,
    Decode,
    Dispatch
};

/**
 * CPU Fault
 */
struct CpuFault: public DeviceFault {
    CpuFault(const std::string &cpu, const std::string &msg):
        DeviceFault(cpu, msg) { }
};

/**
 * CPU Opcode error
 */
struct CpuOpcodeFault: public CpuFault {
    CpuOpcodeFault(const std::string &cpu, unsigned opcode, unsigned addr):
        CpuFault(cpu, "invalid opcode"),
        op(opcode),
        pc(addr)
    {
        std::stringstream ss;
        ss << " " << Hex(opcode);
        if (addr != 0)
            ss << " at address " << Hex(addr);
        msg += ss.str();
    }
    unsigned op;
    unsigned pc;
};

struct CpuRegisterFault: public CpuFault {
    CpuRegisterFault(const std::string &cpu, unsigned index):
        CpuFault(cpu, "invalid regsiter")
    {
        std::stringstream ss;
        ss << " " << Hex(index);
        msg += ss.str();
    }
};

struct CpuFeatureFault: public CpuFault {
    CpuFeatureFault(const std::string &cpu, const std::string &feature=""):
        CpuFault(cpu, "unsupported feature")
    {
        std::stringstream ss;
        if (feature != "") {
            ss << " " << feature;
            msg += ss.str();
        }
    }
};

template<class _state_type>
struct CpuOpcode {
    uint8_t code;
    const char *name;
    int cycles;
    int bytes;
    void (*func)(_state_type *state);
};

template<class _bus_type, class _state_type, class _opcode_type>
class Cpu: public ClockedDevice {
public:
    typedef _bus_type bus_type;
    typedef _state_type state_type;
    typedef _opcode_type opcode_type;
    typedef typename bus_type::addr_type addr_type;
    typedef typename bus_type::data_type data_type;

    Cpu(Machine *machine, const std::string &name, unsigned hertz,
        state_type *state):
        ClockedDevice(machine, name, hertz),
        m_state(state) {
    }
    virtual ~Cpu(void) { }
    Cpu(const Cpu &cpu) = delete;

    state_type *state(void) {
        return m_state;
    }

    virtual void reset(void) {
        m_state->reset();
    }

    virtual std::string dasm(addr_type addr) { return ""; }

protected:

    virtual void execute(void) = 0;

    state_type *m_state;
};

};
