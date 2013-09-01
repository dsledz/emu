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
#include "emu/jit.h"

using namespace EMU;
using namespace JITx64;

namespace M6502v2 {

struct M6502State
{
    M6502State(void) = default;

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
} __attribute__((packed));

class M6502Cpu;

struct M6502Opcode
{
    uint8_t code;
    const char *name;
    int bytes;
    int cycles;
    std::function<void (M6502Cpu *, M6502State *)> addr_mode;
    std::function<void (M6502Cpu *, M6502State *)> operation;
    std::function<void (JITEmitter *, std::function<uint8_t (uint16_t)>, uint16_t)> jit_address;
    std::function<bool (JITEmitter *, uint16_t)> jit_op;
};

class M6502Cpu: public Cpu<AddressBus16>
{
public:
    M6502Cpu(Machine *machine, const std::string &name, unsigned hertz,
             bus_type *bus);
    ~M6502Cpu(void);
    M6502Cpu(const M6502Cpu &cpu) = delete;

    virtual void line(Line line, LineState state);

    M6502State *get_state(void) {
        return &_state;
    }

    void log_state(void);
    void log_op(const M6502Opcode *op, uint16_t pc, const uint8_t *instr);

    virtual void test_step(void);
    virtual Cycles step(void);
    virtual std::string dasm(addr_type addr);

    byte_t fetch(void) {
        _state.ARG = bus_read(_state.EA.d);
        return _state.ARG;
    }

    void store(byte_t value) {
        bus_write(_state.EA.d, value);
    }

    byte_t pc_read(void) {
        return bus_read(_state.PC.d++);
    }

    void push(byte_t value) {
        bus_write((_state.ZPG << 8) + 0x0100 + _state.SP, value);
        _state.SP--;
    }

    byte_t pop(void) {
        _state.SP++;
        return bus_read((_state.ZPG << 8) + 0x0100 + _state.SP);
    }

protected:
    void _reset(void);
    Cycles dispatch(uint16_t pc);
    Cycles jit_dispatch(uint16_t pc);
    jit_block_ptr jit_compile(uint16_t pc);

    LineState _nmi_line;
    LineState _irq_line;
    M6502State _state;
    JITEmitter _jit;
    JITState   _jit_state;
    std::unordered_map<uint32_t, jit_block_ptr> _jit_cache;
    std::unordered_map<uint8_t, M6502Opcode> _opcodes;
};

class M65C02Cpu: public M6502Cpu
{
public:
    M65C02Cpu(Machine *machine, const std::string &name, unsigned clock,
              bus_type *bus):
        M6502Cpu(machine, name, clock, bus)
    {
    }
    virtual ~M65C02Cpu(void)
    {
    }

protected:

};

};
