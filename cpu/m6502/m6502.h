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

#include "emu/emu.h"
#include "cpu/lib/cpu.h"
#include "cpu/lib/jit.h"

using namespace EMU;
using namespace CPU;
using namespace JITx64;

namespace M6502v2 {

struct M6502State
{
    M6502State(void) = default;

    uint8_t bus_read(uint16_t addr) {
        return bus->read(addr);
    }
    void bus_write(uint16_t addr, uint8_t value) {
        bus->write(addr, value);
    }
    uint8_t get_flags(uint16_t flags) {
        F.C = bit_isset(flags, Flags::CF);
        F.Z = bit_isset(flags, Flags::ZF);
        F.V = bit_isset(flags, Flags::OF);
        F.N = bit_isset(flags, Flags::SF);

        return SR;
    }
    void reset(void) {
        A = 0; SP = 0; X = 0; Y = 0; SR = 0; ZPG = 0;
        PC = 0; NativeFlags = 0; EA = 0; ARG = 0;
    }

    reg8_t A;
    reg8_t SP;
    reg8_t X;
    reg8_t Y;
    union {
        byte_t SR;
        struct {
            byte_t C:1;
            byte_t Z:1;
            byte_t I:1;
            byte_t D:1;
            byte_t B:1; /* XXX: Break flag */
            byte_t E:1;
            byte_t V:1;
            byte_t N:1;
        } F;
    };
    byte_t ZPG;
    reg16_t PC;

    reg16_t NativeFlags;

    reg16_t EA;  /* %r8 */
    reg8_t  ARG; /* %rdx */

    AddressBus16 *bus;
    uint8_t icycles;
} __attribute__((packed));

class M6502Cpu;

class M6502Cpu: public Cpu<AddressBus16, M6502State, uint8_t>
{
public:
    M6502Cpu(Machine *machine, const std::string &name, unsigned hertz,
             bus_type *bus);
    virtual ~M6502Cpu(void);
    M6502Cpu(const M6502Cpu &cpu) = delete;

    virtual void line(Line line, LineState state);
    virtual void reset(void);

    M6502State *get_state(void) {
        return &m_state;
    }

    void log_state(void);
    void log_op(const Opcode *op, uint16_t pc, const uint8_t *instr);

    virtual void test_step(void);
    virtual void step(void);
    virtual std::string dasm(addr_type addr);

protected:
    LineState m_nmi_line;
    LineState m_irq_line;
    LineState m_reset_line;
};

};
