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

namespace M6502JIT {

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

#define RegA   RegIdx8::RegAL
#define RegSP  RegIdx8::RegAH
#define RegX   RegIdx8::RegBL
#define RegY   RegIdx8::RegBH
#define RegTmp RegIdx8::RegDL
#define RegEA  RegIdx16::RegCX
#define RegEAh RegIdx8::RegCH
#define RegEAl RegIdx8::RegCL
//#define RegPC  RegIdx16::RegCX

struct M6502Opcode
{
    uint8_t code;
    const char *name;
    std::function<void (void)> addr_mode;
    std::function<void (void)> operation;
    std::function<int (uint16_t)> jit_address;
    std::function<bool (void)> jit_op;
};

#define OP2(op) bool op ##_jit(void) { return false; } void op(void)
#define ADDR(addr) void addr(void)
#define OP(op) void op(void)
#define JIT_OP(op) bool op ##_jit(void)
#define JIT_ADDR(addr) int addr ##_jit(uint16_t pc)

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
    JIT_ADDR(Inherent) {
        return 1;
    }
    ADDR(Absolute) {
        state.EA.b.l = pc_read();
        state.EA.b.h = pc_read();
    }
    JIT_ADDR(Absolute) {
        uint16_t tmp = bus_read(pc + 1) | (bus_read(pc + 2) << 8);
        _jit.xMOV16(RegEA, tmp);
        return 3;
    }
    ADDR(AbsoluteX) {
        state.EA.b.l = pc_read();
        state.EA.b.h = pc_read();
        state.EA.d += state.X;
        if (state.EA.b.l < state.X)
            add_icycles(1);
    }
    JIT_ADDR(AbsoluteX) {
        uint16_t tmp = bus_read(pc + 1) | (bus_read(pc + 2) << 8);
        _jit.xAbsolute(RegEA, RegX, tmp);
        return 3;
    }
    ADDR(AbsoluteY) {
        state.EA.b.l = pc_read();
        state.EA.b.h = pc_read();
        state.EA.d += state.Y;
        if (state.EA.b.l < state.Y)
            add_icycles(1);
    }
    JIT_ADDR(AbsoluteY) {
        uint16_t tmp = bus_read(pc + 1) | (bus_read(pc + 2) << 8);
        _jit.xAbsolute(RegEA, RegY, tmp);
        return 3;
    }
    ADDR(Indirect) {
        reg16_t addr;
        addr.b.l = pc_read();
        addr.b.h = pc_read();
        state.EA.b.l = bus_read(addr.d);
        addr.d++;
        state.EA.b.h = bus_read(addr.d);
    }
    JIT_ADDR(Indirect) {
        throw DeviceFault(_name, "JIT");
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
    JIT_ADDR(IndirectY) {
        throw DeviceFault(_name, "JIT");
    }
    ADDR(XIndirect) {
        reg16_t addr;
        addr.b.l = pc_read() + state.X;
        addr.b.h = state.ZPG;
        state.EA.b.l = bus_read(addr.d);
        addr.b.l++;
        state.EA.b.h = bus_read(addr.d);
    }
    JIT_ADDR(XIndirect) {
        throw DeviceFault(_name, "JIT");
    }
    ADDR(ZeroPageIndirect) {
        reg16_t addr;
        addr.b.l = pc_read();
        addr.b.h = state.ZPG;
        state.EA.b.l = bus_read(addr.d);
        addr.b.l++;
        state.EA.b.h = bus_read(addr.d);
    }
    JIT_ADDR(ZeroPageIndirect) {
        throw DeviceFault(_name, "JIT");
    }
    ADDR(ZeroPage) {
        state.EA.b.l = pc_read();
        state.EA.b.h = state.ZPG;
    }
    JIT_ADDR(ZeroPage) {
        uint16_t tmp = bus_read(pc + 1) + (state.ZPG << 8);
        _jit.xMOV16(RegEA, tmp);
        return 2;
    }
    ADDR(ZeroPageX) {
        state.EA.b.l = pc_read() + state.X;
        state.EA.b.h = state.ZPG;
    }
    JIT_ADDR(ZeroPageX) {
        uint16_t tmp = bus_read(pc + 1) + (state.ZPG << 8);
        _jit.xAbsolute(RegEA, RegX, tmp);
        return 2;
    }
    ADDR(ZeroPageY) {
        state.EA.b.l = pc_read() + state.Y;
        state.EA.b.h = state.ZPG;
    }
    JIT_ADDR(ZeroPageY) {
        uint16_t tmp = bus_read(pc + 1) + (state.ZPG << 8);
        _jit.xAbsolute(RegEA, RegY, tmp);
        return 2;
    }
    ADDR(Immediate) {
        state.EA = state.PC;
        state.PC.d++;
    }
    JIT_ADDR(Immediate) {
        uint16_t tmp = pc + 1;
        _jit.xMOV16(RegEA, tmp);
        return 2;
    }
    ADDR(Relative) {
        char tmp = pc_read();
        state.EA = state.PC;
        state.EA.d += tmp;
    }
    JIT_ADDR(Relative) {
        char tmp = bus_read(pc + 1);
        uint16_t addr = pc + 2 + tmp;
        _jit.xMOV16(RegEA, addr);
        return 2;
    }

protected:
    OP(ADC) {
        int result;
        fetch();
        if (state.F.D) {
            uint8_t l = (state.A & 0x0F) + (state.ARG & 0x0F) + state.F.C;
            if (l > 9)
                l += 6;
            uint8_t h = (state.A >> 4) + (state.ARG >> 4) + (l >> 4);
            result = (h << 4) | (l & 0x0F);
        } else {
            result = state.A + state.ARG + state.F.C;
        }
        set_sz(result);
        state.F.C = bit_isset(result, 8);
        state.F.V = bit_isset(
            (state.A^state.ARG^0x80) & (state.ARG^result), 7);
        state.A = result;
    }
    JIT_OP(ADC) {
        _jit.xADC(RegA, RegEA);
        return true;
    }

    OP(AND) {
        fetch();
        int result = state.A & state.ARG;
        set_sz(result);
        state.A = result;
    }
    JIT_OP(AND) {
        _jit.xAND(RegA, RegEA);
        return true;
    }

    OP(ASL) {
        fetch();
        int result = state.ARG << 1;
        state.F.C = bit_isset(state.ARG, 7);
        set_sz(result);
        store(result);
    }
    JIT_OP(ASL) {
        _jit.xSHL(RegEA);
        return true;
    }

    OP(ASLA) {
        state.ARG = state.A;
        int result = state.ARG << 1;
        state.F.C = bit_isset(state.ARG, 7);
        set_sz(result);
        state.A = result;
    }
    JIT_OP(ASLA) {
        _jit.xSHL(RegA);
        return true;
    }

    OP(BCC) {
        if (state.F.C == 0) {
            state.PC = state.EA;
            add_icycles(2);
        }
    }
    JIT_OP(BCC) {
        _jit.xBR(Condition::CFClear, RegEA);
        return false;
    }
    OP(BCS) {
        if (state.F.C != 0) {
            state.PC = state.EA;
            add_icycles(2);
        }
    }
    JIT_OP(BCS) {
        _jit.xBR(Condition::CFSet, RegEA);
        return false;
    }
    OP(BEQ) {
        if (state.F.Z != 0) {
            state.PC = state.EA;
            add_icycles(2);
        }
    }
    JIT_OP(BEQ) {
        _jit.xBR(Condition::ZFSet, RegEA);
        return false;
    }
    OP2(BIT) {
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
    JIT_OP(BMI) {
        _jit.xBR(Condition::SFSet, RegEA);
        return false;
    }
    OP(BNE) {
        if (state.F.Z == 0) {
            state.PC = state.EA;
            add_icycles(2);
        }
    }
    JIT_OP(BNE) {
        _jit.xBR(Condition::ZFClear, RegEA);
        return false;
    }
    OP(BPL) {
        if (state.F.N == 0) {
            state.PC = state.EA;
            add_icycles(2);
        }
    }
    JIT_OP(BPL) {
        _jit.xBR(Condition::SFClear, RegEA);
        return false;
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
    JIT_OP(BRK) {
        /* XXX: BRK */
        return false;
    }
    OP(BVC) {
        if (state.F.V == 0) {
            state.PC = state.EA;
            add_icycles(2);
        }
    }
    JIT_OP(BVC) {
        _jit.xBR(Condition::OFClear, RegEA);
        return false;
    }
    OP(BVS) {
        if (state.F.V != 0) {
            state.PC = state.EA;
            add_icycles(2);
        }
    }
    JIT_OP(BVS) {
        _jit.xBR(Condition::OFSet, RegEA);
        return false;
    }
    OP(CLC) {
        state.F.C = 0;
    }
    JIT_OP(CLC) {
        _jit.xCLC();
        return true;
    }
    OP2(CLD) {
        state.F.D = 0;
    }
    OP2(CLI) {
        state.F.I = 0;
    }
    OP2(CLV) {
        state.F.V = 0;
    }
    OP(CMP) {
        fetch();
        int result = state.A - state.ARG;
        set_sz(result);
        state.F.C = state.ARG <= state.A;
    }
    JIT_OP(CMP) {
        _jit.xCMP(RegA, RegEA);
        return true;
    }
    OP(CPX) {
        fetch();
        int result = state.X - state.ARG;
        set_sz(result);
        state.F.C = state.ARG <= state.X;
    }
    JIT_OP(CPX) {
        _jit.xCMP(RegX, RegEA);
        return true;
    }
    OP(CPY) {
        fetch();
        int result = state.Y - state.ARG;
        set_sz(result);
        state.F.C = state.ARG <= state.Y;
    }
    JIT_OP(CPY) {
        _jit.xCMP(RegY, RegEA);
        return true;
    }
    OP(DEC) {
        fetch();
        int result = state.ARG - 1;
        set_sz(result);
        store(result);
    }
    JIT_OP(DEC) {
        _jit.xDEC(RegEA);
        return true;
    }
    OP(DEA) {
        int result = state.A - 1;
        set_sz(result);
        state.A = result;
    }
    JIT_OP(DEA) {
        _jit.xDEC(RegA);
        return true;
    }
    OP(DEX) {
        byte_t result = state.X - 1;
        set_sz(result);
        state.X = result;
    }
    JIT_OP(DEX) {
        _jit.xDEC(RegX);
        return true;
    }
    OP(DEY) {
        int result = state.Y - 1;
        set_sz(result);
        state.Y = result;
    }
    JIT_OP(DEY) {
        _jit.xDEC(RegY);
        return true;
    }
    OP(EOR) {
        fetch();
        int result = state.A ^ state.ARG;
        set_sz(result);
        state.A = result;
    }
    JIT_OP(EOR) {
        _jit.xXOR(RegA, RegEA);
        return true;
    }
    OP(INC) {
        fetch();
        int result = state.ARG + 1;
        set_sz(result);
        store(result);
    }
    JIT_OP(INC) {
        _jit.xINC(RegEA);
        return true;
    }
    OP(INA) {
        int result = state.A + 1;
        set_sz(result);
        state.A = result;
    }
    JIT_OP(INA) {
        _jit.xINC(RegA);
        return true;
    }
    OP(INX) {
        byte_t result = state.X + 1;
        set_sz(result);
        state.X = result;
    }
    JIT_OP(INX) {
        _jit.xINC(RegX);
        return true;
    }
    OP(INY) {
        int result = state.Y + 1;
        set_sz(result);
        state.Y = result;
    }
    JIT_OP(INY) {
        _jit.xINC(RegY);
        return true;
    }
    OP(JMP) {
        state.PC = state.EA;
    }
    JIT_OP(JMP) {
        _jit.xSETPC(RegEA);
        return false;
    }
    OP2(JSR) {
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
    JIT_OP(LDA) {
        _jit.xLOAD(RegA, RegEA);
        return true;
    }
    OP(LDX) {
        int result = fetch();
        set_sz(result);
        state.X = result;
    }
    JIT_OP(LDX) {
        _jit.xLOAD(RegX, RegEA);
        return true;
    }
    OP(LDY) {
        int result = fetch();
        set_sz(result);
        state.Y = result;
    }
    JIT_OP(LDY) {
        _jit.xLOAD(RegY, RegEA);
        return true;
    }
    OP(LSR) {
        fetch();
        int result = state.ARG >> 1;
        state.F.C = bit_isset(state.ARG, 0);
        set_sz(result);
        store(result);
    }
    JIT_OP(LSR) {
        _jit.xSHR(RegEA);
        return true;
    }
    OP(LSRA) {
        state.ARG = state.A;
        int result = state.ARG >> 1;
        state.F.C = bit_isset(state.ARG, 0);
        set_sz(result);
        state.A = result;
    }
    JIT_OP(LSRA) {
        _jit.xSHR(RegA);
        return true;
    }
    OP(NOP) {
    }
    JIT_OP(NOP) {
        /* XXX: NOP */
        return true;
    }
    OP(ORA) {
        fetch();
        int result = state.A | state.ARG;
        set_sz(result);
        state.A = result;
    }
    JIT_OP(ORA) {
        _jit.xOR(RegA, RegEA);
        return true;
    }
    OP(PHA) {
        push(state.A);
    }
    JIT_OP(PHA) {
        uint8_t tmp = (state.ZPG + 0x01) << 8;
        _jit.xMOV8(RegEAh, tmp);
        _jit.xMOV8(RegEAl, RegSP);
        _jit.xSTORE(RegEA, RegA);
        _jit.xDEC(RegSP);
        return true;
    }
    OP2(PHP) {
        push(state.SR);
    }
    OP(PLA) {
        state.A = pop();
        set_sz(state.A);
    }
    JIT_OP(PLA) {
        uint8_t tmp = (state.ZPG + 0x01) << 8;
        _jit.xINC(RegSP);
        _jit.xMOV8(RegEAh, tmp);
        _jit.xMOV8(RegEAl, RegSP);
        _jit.xLOAD(RegA, RegEA);
        /* XXX: flags */
        return true;
    }
    OP2(PLP) {
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
    JIT_OP(ROL) {
        _jit.xRCL(RegEA);
        return true;
    }
    OP(ROLA) {
        state.ARG = state.A;
        int result = state.ARG << 1 | state.F.C;
        state.F.C = bit_isset(state.ARG, 7);
        set_sz(result);
        state.A = result;
    }
    JIT_OP(ROLA) {
        _jit.xRCL(RegA);
        return true;
    }
    OP(ROR) {
        fetch();
        int result = state.ARG >> 1 | (state.F.C << 7);
        state.F.C = bit_isset(state.ARG, 0);
        set_sz(result);
        store(result);
    }
    JIT_OP(ROR) {
        _jit.xRCR(RegEA);
        return true;
    }
    OP(RORA) {
        state.ARG = state.A;
        int result = state.ARG >> 1 | (state.F.C << 7);
        state.F.C = bit_isset(state.ARG, 0);
        set_sz(result);
        state.A = result;
    }
    JIT_OP(RORA) {
        _jit.xRCR(RegA);
        return true;
    }
    OP2(RTI) {
        state.SR = pop();
        state.F.E = 1;
        state.PC.b.l = pop();
        state.PC.b.h = pop();
    }
    OP2(RTS) {
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
    JIT_OP(SBC) {
        _jit.xSBC(RegA, RegEA);
        return true;
    }
    OP(SEC) {
        state.F.C = 1;
    }
    JIT_OP(SEC) {
        _jit.xSTC();
        return true;
    }
    OP2(SED) {
        state.F.D = 1;
    }
    OP2(SEI) {
        state.F.I = 1;
    }
    OP(STA) {
        store(state.A);
    }
    JIT_OP(STA) {
        _jit.xSTORE(RegEA, RegA);
        return true;
    }
    OP(STX) {
        store(state.X);
    }
    JIT_OP(STX) {
        _jit.xSTORE(RegEA, RegX);
        return true;
    }
    OP(STY) {
        store(state.Y);
    }
    JIT_OP(STY) {
        _jit.xSTORE(RegEA, RegY);
        return true;
    }
    OP(TAX) {
        state.X = state.A;
        set_sz(state.X);
    }
    JIT_OP(TAX) {
        /* XXX: flags */
        _jit.xMOV8(RegX, RegA);
        return true;
    }
    OP(TAY) {
        state.Y = state.A;
        set_sz(state.Y);
    }
    JIT_OP(TAY) {
        /* XXX: flags */
        _jit.xMOV8(RegY, RegA);
        return true;
    }
    OP(TSX) {
        state.X = state.SP;
        set_sz(state.X);
    }
    JIT_OP(TSX) {
        _jit.xMOV8(RegX, RegSP);
        return true;
    }
    OP(TXA) {
        state.A = state.X;
        set_sz(state.A);
    }
    JIT_OP(TXA) {
        _jit.xMOV8(RegA, RegX);
        return true;
    }
    OP(TYA) {
        state.A = state.Y;
        set_sz(state.A);
    }
    JIT_OP(TYA) {
        _jit.xMOV8(RegA, RegY);
        return true;
    }
    OP(TXS) {
        state.SP = state.X;
        /* XXX: set_sz(state.SP) */
    }
    JIT_OP(TXS) {
        _jit.xMOV8(RegSP, RegX);
        return true;
    }

protected:
    void _reset(void);
    Cycles dispatch(void);
    Cycles jit_dispatch(JITBlock *block);
    JITBlock compile(uint16_t pc);

    LineState _nmi_line;
    LineState _irq_line;
    M6502State state;
    JITEmitter _jit;
    JITState   _jit_state;
    std::unordered_map<uint32_t, JITBlock> _jit_cache;
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

    OP2(TSB) {
        fetch();
        int result = state.A | state.ARG;
        state.F.Z = (state.A & state.ARG) == 0;
        store(result);
    }

    OP2(TRB) {
        fetch();
        int result = ~state.A & state.ARG;
        state.F.Z = (state.A & state.ARG) == 0;
        store(result);
    }

    OP2(BITIMM) {
        fetch();
        state.F.Z = (state.A & state.ARG) == 0;
    }

    OP2(INA) {
        int result = state.A + 1;
        set_sz(result);
        state.A = result;
    }

    OP2(DEA) {
        int result = state.A - 1;
        set_sz(result);
        state.A = result;
    }

    OP2(PHX) {
        push(state.X);
    }

    OP2(PLX) {
        int result = pop();
        set_sz(result);
        state.X = result;
    }

    OP2(PHY) {
        push(state.Y);
    }

    OP2(PLY) {
        int result = pop();
        set_sz(result);
        state.Y = result;
    }

    OP2(SMB) {
        Immediate();
        int bit = fetch();
        ZeroPage();
        fetch();
        int result = state.ARG | (1 << bit);
        store(result);
    }

    OP2(RMB) {
        Immediate();
        int bit = fetch();
        ZeroPage();
        fetch();
        int result = state.ARG & ~(1 << bit);
        store(result);
    }

    OP2(BBR) {
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

    OP2(BBS) {
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
