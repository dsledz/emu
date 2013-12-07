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
using namespace JITx64;

namespace CPU {

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
        std::function<void (JITEmitter *, std::function<addr_type (addr_type)>, addr_type)> jit_address;
        std::function<bool (JITEmitter *, pc_type)> jit_op;
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

    static data_type jit_bus_read(void *ctx, addr_type addr)
    {
            auto *cpu = static_cast<Cpu<bus_type, state_type, opcode_type> *>(ctx);
            return cpu->bus_read(addr);
    }

    static void jit_bus_write(void *ctx, addr_type addr, data_type value)
    {
            auto *cpu = static_cast<Cpu<bus_type, state_type, opcode_type> *>(ctx);
            cpu->bus_write(addr, value);
    }

    static void jit_bus_get_flags(void *ctx, uint16_t flags)
    {
            auto *cpu = static_cast<Cpu<bus_type, state_type, opcode_type> *>(ctx);
    }

    void jit_dispatch(pc_type pc)
    {
        using namespace std::placeholders;
        JITBlock *block = NULL;

        auto it = m_jit_cache.find(pc);
        if (it == m_jit_cache.end()) {
            jit_block_ptr b = jit_compile(pc);
            block = b.get();
            m_jit_cache.insert(std::make_pair(pc, std::move(b)));
        } else if (!it->second->valid(std::bind(jit_bus_read, this, _1))) {
            m_jit_cache.erase(it);
            jit_block_ptr b = jit_compile(pc);
            block = b.get();
            m_jit_cache.insert(std::make_pair(pc, std::move(b)));
        } else {
            block = it->second.get();
        }

        bit_set(m_state.NativeFlags.d, Flags::CF, m_state.F.C);
        bit_set(m_state.NativeFlags.d, Flags::ZF, m_state.F.Z);
        bit_set(m_state.NativeFlags.d, Flags::OF, m_state.F.V);
        bit_set(m_state.NativeFlags.d, Flags::SF, m_state.F.N);

        JITState jit_state(reinterpret_cast<uintptr_t>(this),
                           jit_bus_read, jit_bus_write);

        block->execute(reinterpret_cast<uintptr_t>(&m_state),
                       reinterpret_cast<uintptr_t>(&jit_state));

        // Update our flags
        m_state.F.C = bit_isset(m_state.NativeFlags.d, Flags::CF);
        m_state.F.Z = bit_isset(m_state.NativeFlags.d, Flags::ZF);
        m_state.F.V = bit_isset(m_state.NativeFlags.d, Flags::OF);
        m_state.F.N = bit_isset(m_state.NativeFlags.d, Flags::SF);

        // Account for the extra cycles if we branched
        add_icycles(block->cycles);
        if (m_state.PC.d != (block->pc + block->len))
            add_icycles(2);
    }

    jit_block_ptr jit_compile(pc_type start_pc)
    {
        using namespace std::placeholders;
        JITEmitter jit;
        bool done = false;
        uint32_t len = 0;
        uint32_t cycles = 0;
        auto br = std::bind(&jit_bus_read, this, _1);
        bvec source;

        while (!done) {
            pc_type pc = start_pc + len;
            opcode_type code = bus_read(pc);

            auto it = m_opcodes.find(code);
            if (it == m_opcodes.end()) {
                std::cout << "Unknown opcode: " << Hex(code) << std::endl;
                throw CpuOpcodeFault(name(), code, pc);
            }
            Opcode *op = &it->second;

            for (unsigned i = 0; i < op->bytes; i++) {
                source.push_back(bus_read(pc + i));
            }

            len += op->bytes;
            cycles += op->cycles;
            op->jit_address(&jit, br, pc);
            done = !op->jit_op(&jit, pc);
        }

        jit.xMOV16(RegIdx16::RegDX, start_pc + len);
        jit.xSETPC(RegIdx16::RegDX);
        return jit_block_ptr(new JITBlock(jit.code(), start_pc, source, len, cycles));
    }

    Cycles m_icycles;
    bus_type *m_bus;
    state_type m_state;
    std::unordered_map<opcode_type, Opcode> m_opcodes;
    std::unordered_map<pc_type, jit_block_ptr> m_jit_cache;
};


};
