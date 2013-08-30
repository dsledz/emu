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

using namespace EMU;

namespace M6502JIT {

struct M6502State
{
    M6502State(void) = default;

    reg8_t A;
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
    byte_t SP;
    byte_t ZPG;
    reg16_t PC;

    reg16_t EA;  /* %r8 */
    reg8_t  ARG; /* %rdx */
};

struct M6502Opcode
{
    uint8_t code;
    const char *name;
    std::function<void (void)> addr_mode;
    std::function<void (void)> operation;
};

#define OP(op) void op(void)
#define ADDR(addr) void addr(void)

struct M6502Ops
{
};

class M6502Cpu: public Cpu<AddressBus16>
{
public:
    M6502Cpu(Machine *machine, const std::string &name, unsigned hertz,
             bus_type *bus);
    ~M6502Cpu(void);
    M6502Cpu(const M6502Cpu&cpu) = delete;

    virtual void line(Line line, LineState state);

    M6502State *get_state(void) {
        return &state;
    }

    void log_op(uint16_t pc, const M6502Opcode *op);

    virtual Cycles step(void);
    virtual std::string dasm(addr_type addr);

protected:
    byte_t fetch(void) {
        state.ARG = bus_read(state.EA.d);
        return state.ARG;
    }

    void store(byte_t value) {
        bus_write(state.EA.d, value);
    }

    inline void set_sz(byte_t result) {
        state.F.N = bit_isset(result, 7);
        state.F.Z = (result == 0);
    }

    byte_t pc_read(void) {
        return bus_read(state.PC.d++);
    }

    void push(byte_t value) {
        bus_write((state.ZPG << 8) + 0x0100 + state.SP, value);
        state.SP--;
    }

    byte_t pop(void) {
        state.SP++;
        return bus_read((state.ZPG << 8) + 0x0100 + state.SP);
    }

protected:
    ADDR(Inherent) {
    }
    ADDR(Absolute) {
        state.EA.b.l = pc_read();
        state.EA.b.h = pc_read();
    }
    ADDR(AbsoluteX) {
        state.EA.b.l = pc_read();
        state.EA.b.h = pc_read();
        state.EA.d += state.X;
        if (state.EA.b.l < state.X)
            add_icycles(1);
    }
    ADDR(AbsoluteY) {
        state.EA.b.l = pc_read();
        state.EA.b.h = pc_read();
        state.EA.d += state.Y;
        if (state.EA.b.l < state.Y)
            add_icycles(1);
    }
    ADDR(Indirect) {
        reg16_t addr;
        addr.b.l = pc_read();
        addr.b.h = pc_read();
        state.EA.b.l = bus_read(addr.d);
        addr.d++;
        state.EA.b.h = bus_read(addr.d);
    }
    ADDR(IndirectY) {
        reg16_t addr;
        addr.b.l = pc_read();
        addr.b.h = state.ZPG;
        state.EA.b.l = bus_read(addr.d);
        addr.b.l++;
        state.EA.b.h = bus_read(addr.d);
        state.EA.d += state.Y;
        if (state.EA.b.l < state.Y)
            add_icycles(1);
    }
    ADDR(XIndirect) {
        reg16_t addr;
        addr.b.l = pc_read() + state.X;
        addr.b.h = state.ZPG;
        state.EA.b.l = bus_read(addr.d);
        addr.b.l++;
        state.EA.b.h = bus_read(addr.d);
    }
    ADDR(ZeroPageIndirect) {
        reg16_t addr;
        addr.b.l = pc_read();
        addr.b.h = state.ZPG;
        state.EA.b.l = bus_read(addr.d);
        addr.b.l++;
        state.EA.b.h = bus_read(addr.d);
    }
    ADDR(ZeroPage) {
        state.EA.b.l = pc_read();
        state.EA.b.h = state.ZPG;
    }
    ADDR(ZeroPageX) {
        state.EA.b.l = pc_read() + state.X;
        state.EA.b.h = state.ZPG;
    }
    ADDR(ZeroPageY) {
        state.EA.b.l = pc_read() + state.Y;
        state.EA.b.h = state.ZPG;
    }
    ADDR(Immediate) {
        state.EA = state.PC;
        state.PC.d++;
    }
    ADDR(Relative) {
        char tmp = pc_read();
        state.EA = state.PC;
        state.EA.d += tmp;
    }

protected:
    OP(ADC) {
        fetch();
        if (state.F.D)
            throw CpuFeatureFault("m6502", "decimal");
        int result = state.A + state.ARG + state.F.C;
        set_sz(result);
        state.F.C = bit_isset(result, 8);
        state.F.V = bit_isset(
            (state.A^state.ARG^0x80) & (state.ARG^result), 7);
        state.A = result;
    }
    OP(AND) {
        fetch();
        int result = state.A & state.ARG;
        set_sz(result);
        state.A = result;
    }
    OP(ASL) {
        fetch();
        int result = state.ARG << 1;
        state.F.C = bit_isset(state.ARG, 7);
        set_sz(result);
        store(result);
    }
    OP(ASLA) {
        state.ARG = state.A;
        int result = state.ARG << 1;
        state.F.C = bit_isset(state.ARG, 7);
        set_sz(result);
        state.A = result;
    }
    OP(BCC) {
        if (state.F.C == 0) {
            state.PC = state.EA;
            add_icycles(2);
        }
    }
    OP(BCS) {
        if (state.F.C != 0) {
            state.PC = state.EA;
            add_icycles(2);
        }
    }
    OP(BEQ) {
        if (state.F.Z != 0) {
            state.PC = state.EA;
            add_icycles(2);
        }
    }
    OP(BIT) {
        fetch();
        int result = state.ARG;
        state.F.N = bit_isset(result, 7);
        state.F.V = bit_isset(result, 6);
        state.F.Z = (state.A & result) == 0;
    }
    OP(BMI) {
        if (state.F.N != 0) {
            state.PC = state.EA;
            add_icycles(2);
        }
    }
    OP(BNE) {
        if (state.F.Z == 0) {
            state.PC = state.EA;
            add_icycles(2);
        }
    }
    OP(BPL) {
        if (state.F.N == 0) {
            state.PC = state.EA;
            add_icycles(2);
        }
    }
    OP(BRK) {
        state.F.B = 1;
        state.PC.d++;
        push(state.PC.b.h);
        push(state.PC.b.l);
        push(state.SR);
        state.PC.b.l = bus_read(0xFFFE);
        state.PC.b.h = bus_read(0xFFFF);
    }
    OP(BVC) {
        if (state.F.V == 0) {
            state.PC = state.EA;
            add_icycles(2);
        }
    }
    OP(BVS) {
        if (state.F.V != 0) {
            state.PC = state.EA;
            add_icycles(2);
        }
    }
    OP(CLC) {
        state.F.C = 0;
    }
    OP(CLD) {
        state.F.D = 0;
    }
    OP(CLI) {
        state.F.I = 0;
    }
    OP(CLV) {
        state.F.V = 0;
    }
    OP(CMP) {
        fetch();
        int result = state.A - state.ARG;
        set_sz(result);
        state.F.C = state.ARG <= state.A;
    }
    OP(CPX) {
        fetch();
        int result = state.X - state.ARG;
        set_sz(result);
        state.F.C = state.ARG <= state.X;
    }
    OP(CPY) {
        fetch();
        int result = state.Y - state.ARG;
        set_sz(result);
        state.F.C = state.ARG <= state.Y;
    }
    OP(DEC) {
        fetch();
        int result = state.ARG - 1;
        set_sz(result);
        store(result);
    }
    OP(DEA) {
        int result = state.A - 1;
        set_sz(result);
        state.A = result;
    }
    OP(DEX) {
        byte_t result = state.X - 1;
        set_sz(result);
        state.X = result;
    }
    OP(DEY) {
        int result = state.Y - 1;
        set_sz(result);
        state.Y = result;
    }
    OP(EOR) {
        fetch();
        int result = state.A ^ state.ARG;
        set_sz(result);
        state.A = result;
    }
    OP(INC) {
        fetch();
        int result = state.ARG + 1;
        set_sz(result);
        store(result);
    }
    OP(INA) {
        int result = state.A + 1;
        set_sz(result);
        state.A = result;
    }
    OP(INX) {
        byte_t result = state.X + 1;
        set_sz(result);
        state.X = result;
    }
    OP(INY) {
        int result = state.Y + 1;
        set_sz(result);
        state.Y = result;
    }
    OP(JMP) {
        state.PC = state.EA;
    }
    OP(JSR) {
        state.PC.d--;
        push(state.PC.b.h);
        push(state.PC.b.l);
        state.PC = state.EA;
    }
    OP(LDA) {
        int result = fetch();
        set_sz(result);
        state.A = result;
    }
    OP(LDX) {
        int result = fetch();
        set_sz(result);
        state.X = result;
    }
    OP(LDY) {
        int result = fetch();
        set_sz(result);
        state.Y = result;
    }
    OP(LSR) {
        fetch();
        int result = state.ARG >> 1;
        state.F.C = bit_isset(state.ARG, 0);
        set_sz(result);
        store(result);
    }
    OP(LSRA) {
        state.ARG = state.A;
        int result = state.ARG >> 1;
        state.F.C = bit_isset(state.ARG, 0);
        set_sz(result);
        state.A = result;
    }
    OP(NOP) {
    }
    OP(ORA) {
        fetch();
        int result = state.A | state.ARG;
        set_sz(result);
        state.A = result;
    }
    OP(PHA) {
        push(state.A);
    }
    OP(PHP) {
        push(state.SR);
    }
    OP(PLA) {
        state.A = pop();
        set_sz(state.A);
    }
    OP(PLP) {
        state.SR = pop();
        state.F.B = 1;
        state.F.E = 1;
    }
    OP(ROL) {
        fetch();
        int result = state.ARG << 1 | state.F.C;
        state.F.C = bit_isset(state.ARG, 7);
        set_sz(result);
        store(result);
    }
    OP(ROLA) {
        state.ARG = state.A;
        int result = state.ARG << 1 | state.F.C;
        state.F.C = bit_isset(state.ARG, 7);
        set_sz(result);
        state.A = result;
    }
    OP(ROR) {
        fetch();
        int result = state.ARG >> 1 | (state.F.C << 7);
        state.F.C = bit_isset(state.ARG, 0);
        set_sz(result);
        store(result);
    }
    OP(RORA) {
        state.ARG = state.A;
        int result = state.ARG >> 1 | (state.F.C << 7);
        state.F.C = bit_isset(state.ARG, 0);
        set_sz(result);
        state.A = result;
    }
    OP(RTI) {
        state.SR = pop();
        state.F.E = 1;
        state.PC.b.l = pop();
        state.PC.b.h = pop();
    }
    OP(RTS) {
        state.PC.b.l = pop();
        state.PC.b.h = pop();
        state.PC.d++;
    }
    OP(SBC) {
        fetch();
        uint32_t result = state.A - state.ARG - !state.F.C;
        set_sz(result);
        state.F.C = result < 0x100;
        state.F.V = bit_isset((result^state.A) & (state.A^state.ARG), 7);
        state.A = result;
    }
    OP(SEC) {
        state.F.C = 1;
    }
    OP(SED) {
        state.F.D = 1;
    }
    OP(SEI) {
        state.F.I = 1;
    }
    OP(STA) {
        store(state.A);
    }
    OP(STX) {
        store(state.X);
    }
    OP(STY) {
        store(state.Y);
    }
    OP(TAX) {
        state.X = state.A;
        set_sz(state.X);
    }
    OP(TAY) {
        state.Y = state.A;
        set_sz(state.Y);
    }
    OP(TSX) {
        state.X = state.SP;
        set_sz(state.X);
    }
    OP(TXA) {
        state.A = state.X;
        set_sz(state.A);
    }
    OP(TYA) {
        state.A = state.Y;
        set_sz(state.A);
    }
    OP(TXS) {
        state.SP = state.X;
        /* XXX: set_sz(state.SP) */
    }

protected:
    void _reset(void);
    Cycles dispatch(void);

    LineState _nmi_line;
    LineState _irq_line;
    M6502State state;
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

    OP(TSB) {
        fetch();
        int result = state.A | state.ARG;
        state.F.Z = (state.A & state.ARG) == 0;
        store(result);
    }

    OP(TRB) {
        fetch();
        int result = ~state.A & state.ARG;
        state.F.Z = (state.A & state.ARG) == 0;
        store(result);
    }

    OP(BITIMM) {
        fetch();
        state.F.Z = (state.A & state.ARG) == 0;
    }

    OP(INA) {
        int result = state.A + 1;
        set_sz(result);
        state.A = result;
    }

    OP(DEA) {
        int result = state.A - 1;
        set_sz(result);
        state.A = result;
    }

    OP(PHX) {
        push(state.X);
    }

    OP(PLX) {
        int result = pop();
        set_sz(result);
        state.X = result;
    }

    OP(PHY) {
        push(state.Y);
    }

    OP(PLY) {
        int result = pop();
        set_sz(result);
        state.Y = result;
    }

    OP(SMB) {
        Immediate();
        int bit = fetch();
        ZeroPage();
        fetch();
        int result = state.ARG | (1 << bit);
        store(result);
    }

    OP(RMB) {
        Immediate();
        int bit = fetch();
        ZeroPage();
        fetch();
        int result = state.ARG & ~(1 << bit);
        store(result);
    }

    OP(BBR) {
        Immediate();
        int bit = fetch();
        ZeroPage();
        fetch();
        Relative();
        if (!bit_isset(state.ARG, bit)) {
            state.PC = state.EA;
            add_icycles(2);
        }
    }

    OP(BBS) {
        Immediate();
        int bit = fetch();
        ZeroPage();
        fetch();
        Relative();
        if (bit_isset(state.ARG, bit)) {
            state.PC = state.EA;
            add_icycles(2);
        }
    }

};

};
