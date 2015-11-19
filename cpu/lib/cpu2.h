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

template<class _bus_type, class _state_type, typename _opcode_type>
class Cpu: public ClockedDevice {
public:
    typedef _bus_type bus_type;
    typedef _state_type state_type;
    typedef _opcode_type opcode_type;
    typedef typename bus_type::addr_type pc_type;
    typedef typename bus_type::addr_type addr_type;
    typedef typename bus_type::data_type data_type;

    struct Opcode {
        opcode_type code;
        const char *name;
        int bytes;
        int cycles;
        std::function<void (state_type *)> addr_mode;
        std::function<void (state_type *)> operation;
    };

    Cpu(Machine *machine, const std::string &name, unsigned hertz,
        bus_type *bus):
        ClockedDevice(machine, name, hertz),
        m_icycles(0),
        m_bus(bus)
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

    bus_type *bus(void) {
        return m_bus;
    }

    state_type *state(void) {
        return &m_state;
    }

    data_type bus_read(addr_type addr) {
        data_type tmp = m_bus->read(addr);
        /* XXX: Should we always incur a cycle here? */
        //add_icycles(1);
        return tmp;
    }

    void bus_write(addr_type addr, data_type value) {
        m_bus->write(addr, value);
        //add_icycles(1);
    }

    virtual void test_step(void) {
        step();
    }

    /* Process a single clock cycle */
    virtual void step(void) = 0;
    virtual std::string dasm(addr_type addr) = 0;

protected:

    void dispatch(pc_type pc)
    {
        opcode_type opcode = bus_read(m_state.PC.d++);
        auto it = m_opcodes.find(opcode);
        if (it == m_opcodes.end()) {
            DEVICE_ERROR("Unknown opcode");
            throw CpuOpcodeFault(name(), opcode, pc);
        }

        Opcode *op = &it->second;

        m_state.bus = bus();
        m_state.icycles = 0;

        op->addr_mode(&m_state);
        op->operation(&m_state);
        add_icycles(op->cycles + m_state.icycles);

        return;
    }

    Cycles m_icycles;
    bus_type *m_bus;
    addr_type m_address;
    data_type m_data;
    state_type m_state;
    std::unordered_map<opcode_type, Opcode> m_opcodes;
};

};
