#pragma once

#include "emu/emu.h"

#define ADDR(addr) void addr(M6502Cpu *cpu, M6502State *state)
#define OP(op) void op(M6502Cpu *cpu, M6502State *state)

namespace M6502v2
{

    static inline void set_sz(M6502State *state, byte_t result)
    {
        state->F.N = bit_isset(result, 7);
        state->F.Z = (result == 0);
    }

    static inline uint8_t pc_read(M6502Cpu *cpu, M6502State *state)
    {
        return cpu->bus_read(state->PC.d++);
    }

    static inline void push(M6502Cpu *cpu, M6502State *state, uint8_t value)
    {
        cpu->bus_write((state->ZPG << 8) + 0x0100 + state->SP, value);
        state->SP--;
    }

    static inline uint8_t pop(M6502Cpu *cpu, M6502State *state)
    {
        state->SP++;
        return cpu->bus_read((state->ZPG << 8) + 0x0100 + state->SP);
    }

    ADDR(Inherent) {
    }
    ADDR(Absolute) {
        state->EA.b.l = pc_read(cpu, state);
        state->EA.b.h = pc_read(cpu, state);
    }
    ADDR(AbsoluteX) {
        state->EA.b.l = pc_read(cpu, state);
        state->EA.b.h = pc_read(cpu, state);
        state->EA.d += state->X;
        if (state->EA.b.l < state->X)
            cpu->add_icycles(1);
    }
    ADDR(AbsoluteY) {
        state->EA.b.l = pc_read(cpu, state);
        state->EA.b.h = pc_read(cpu, state);
        state->EA.d += state->Y;
        if (state->EA.b.l < state->Y)
            cpu->add_icycles(1);
    }
    ADDR(Indirect) {
        reg16_t addr;
        addr.b.l = pc_read(cpu, state);
        addr.b.h = pc_read(cpu, state);
        state->EA.b.l = cpu->bus_read(addr.d);
        addr.b.l++;
        state->EA.b.h = cpu->bus_read(addr.d);
    }
    ADDR(IndirectY) {
        reg16_t addr;
        addr.b.l = pc_read(cpu, state);
        addr.b.h = state->ZPG;
        state->EA.b.l = cpu->bus_read(addr.d);
        addr.b.l++;
        state->EA.b.h = cpu->bus_read(addr.d);
        state->EA.d += state->Y;
        if (state->EA.b.l < state->Y)
            cpu->add_icycles(1);
    }
    ADDR(XIndirect) {
        reg16_t addr;
        addr.b.l = pc_read(cpu, state) + state->X;
        addr.b.h = state->ZPG;
        state->EA.b.l = cpu->bus_read(addr.d);
        addr.b.l++;
        state->EA.b.h = cpu->bus_read(addr.d);
    }
    ADDR(ZeroPageIndirect) {
        reg16_t addr;
        addr.b.l = pc_read(cpu, state);
        addr.b.h = state->ZPG;
        state->EA.b.l = cpu->bus_read(addr.d);
        addr.b.l++;
        state->EA.b.h = cpu->bus_read(addr.d);
    }
    ADDR(ZeroPage) {
        state->EA.b.l = pc_read(cpu, state);
        state->EA.b.h = state->ZPG;
    }
    ADDR(ZeroPageX) {
        state->EA.b.l = pc_read(cpu, state) + state->X;
        state->EA.b.h = state->ZPG;
    }
    ADDR(ZeroPageY) {
        state->EA.b.l = pc_read(cpu, state) + state->Y;
        state->EA.b.h = state->ZPG;
    }
    ADDR(Immediate) {
        state->EA = state->PC;
        state->PC.d++;
    }
    ADDR(Relative) {
        char tmp = pc_read(cpu, state);
        state->EA = state->PC;
        state->EA.d += tmp;
    }

    OP(ADC) {
        int result;
        cpu->fetch();
        if (state->F.D) {
            uint8_t l = (state->A & 0x0F) + (state->ARG & 0x0F) + state->F.C;
            if (l > 9)
                l += 6;
            uint8_t h = (state->A >> 4) + (state->ARG >> 4) + (l >> 4);
            result = (h << 4) | (l & 0x0F);
        } else {
            result = state->A + state->ARG + state->F.C;
        }
        set_sz(state, result);
        state->F.C = bit_isset(result, 8);
        state->F.V = bit_isset(
            (state->A^state->ARG^0x80) & (state->ARG^result), 7);
        state->A = result;
    }

    OP(AND) {
        cpu->fetch();
        int result = state->A & state->ARG;
        set_sz(state, result);
        state->A = result;
    }

    OP(ASL) {
        cpu->fetch();
        int result = state->ARG << 1;
        state->F.C = bit_isset(state->ARG, 7);
        set_sz(state, result);
        cpu->store(result);
    }

    OP(ASLA) {
        state->ARG = state->A;
        int result = state->ARG << 1;
        state->F.C = bit_isset(state->ARG, 7);
        set_sz(state, result);
        state->A = result;
    }

    OP(BCC) {
        if (state->F.C == 0) {
            state->PC = state->EA;
            cpu->add_icycles(2);
        }
    }
    OP(BCS) {
        if (state->F.C != 0) {
            state->PC = state->EA;
            cpu->add_icycles(2);
        }
    }
    OP(BEQ) {
        if (state->F.Z != 0) {
            state->PC = state->EA;
            cpu->add_icycles(2);
        }
    }
    OP(BIT) {
        cpu->fetch();
        int result = state->ARG;
        state->F.N = bit_isset(result, 7);
        state->F.V = bit_isset(result, 6);
        state->F.Z = (state->A & result) == 0;
    }
    OP(BMI) {
        if (state->F.N != 0) {
            state->PC = state->EA;
            cpu->add_icycles(2);
        }
    }
    OP(BNE) {
        if (state->F.Z == 0) {
            state->PC = state->EA;
            cpu->add_icycles(2);
        }
    }
    OP(BPL) {
        if (state->F.N == 0) {
            state->PC = state->EA;
            cpu->add_icycles(2);
        }
    }
    OP(BRK) {
        state->F.B = 1;
        state->PC.d++;
        push(cpu, state, state->PC.b.h);
        push(cpu, state, state->PC.b.l);
        push(cpu, state, state->SR);
        state->PC.b.l = cpu->bus_read(0xFFFE);
        state->PC.b.h = cpu->bus_read(0xFFFF);
    }
    OP(BVC) {
        if (state->F.V == 0) {
            state->PC = state->EA;
            cpu->add_icycles(2);
        }
    }
    OP(BVS) {
        if (state->F.V != 0) {
            state->PC = state->EA;
            cpu->add_icycles(2);
        }
    }
    OP(CLC) {
        state->F.C = 0;
    }
    OP(CLD) {
        state->F.D = 0;
    }
    OP(CLI) {
        state->F.I = 0;
    }
    OP(CLV) {
        state->F.V = 0;
    }
    OP(CMP) {
        cpu->fetch();
        int result = state->A - state->ARG;
        set_sz(state, result);
        state->F.C = state->ARG <= state->A;
    }
    OP(CPX) {
        cpu->fetch();
        int result = state->X - state->ARG;
        set_sz(state, result);
        state->F.C = state->ARG <= state->X;
    }
    OP(CPY) {
        cpu->fetch();
        int result = state->Y - state->ARG;
        set_sz(state, result);
        state->F.C = state->ARG <= state->Y;
    }
    OP(DEC) {
        cpu->fetch();
        int result = state->ARG - 1;
        set_sz(state, result);
        cpu->store(result);
    }
    OP(DEA) {
        int result = state->A - 1;
        set_sz(state, result);
        state->A = result;
    }
    OP(DEX) {
        byte_t result = state->X - 1;
        set_sz(state, result);
        state->X = result;
    }
    OP(DEY) {
        int result = state->Y - 1;
        set_sz(state, result);
        state->Y = result;
    }
    OP(EOR) {
        cpu->fetch();
        int result = state->A ^ state->ARG;
        set_sz(state, result);
        state->A = result;
    }
    OP(INC) {
        cpu->fetch();
        int result = state->ARG + 1;
        set_sz(state, result);
        cpu->store(result);
    }
    OP(INX) {
        byte_t result = state->X + 1;
        set_sz(state, result);
        state->X = result;
    }
    OP(INY) {
        int result = state->Y + 1;
        set_sz(state, result);
        state->Y = result;
    }
    OP(JMP) {
        state->PC = state->EA;
    }
    OP(JSR) {
        state->PC.d--;
        push(cpu, state, state->PC.b.h);
        push(cpu, state, state->PC.b.l);
        state->PC = state->EA;
    }
    OP(LDA) {
        int result = cpu->fetch();
        set_sz(state, result);
        state->A = result;
    }
    OP(LDX) {
        int result = cpu->fetch();
        set_sz(state, result);
        state->X = result;
    }
    OP(LDY) {
        int result = cpu->fetch();
        set_sz(state, result);
        state->Y = result;
    }
    OP(LSR) {
        cpu->fetch();
        int result = state->ARG >> 1;
        state->F.C = bit_isset(state->ARG, 0);
        set_sz(state, result);
        cpu->store(result);
    }
    OP(LSRA) {
        state->ARG = state->A;
        int result = state->ARG >> 1;
        state->F.C = bit_isset(state->ARG, 0);
        set_sz(state, result);
        state->A = result;
    }
    OP(NOP) {
    }
    OP(ORA) {
        cpu->fetch();
        int result = state->A | state->ARG;
        set_sz(state, result);
        state->A = result;
    }
    OP(PHA) {
        push(cpu, state, state->A);
    }
    OP(PHP) {
        push(cpu, state, state->SR);
    }
    OP(PLA) {
        state->A = pop(cpu, state);
        set_sz(state, state->A);
    }
    OP(PLP) {
        state->SR = pop(cpu, state);
        state->F.B = 1;
        state->F.E = 1;
    }
    OP(ROL) {
        cpu->fetch();
        int result = state->ARG << 1 | state->F.C;
        state->F.C = bit_isset(state->ARG, 7);
        set_sz(state, result);
        cpu->store(result);
    }
    OP(ROLA) {
        state->ARG = state->A;
        int result = state->ARG << 1 | state->F.C;
        state->F.C = bit_isset(state->ARG, 7);
        set_sz(state, result);
        state->A = result;
    }
    OP(ROR) {
        cpu->fetch();
        int result = state->ARG >> 1 | (state->F.C << 7);
        state->F.C = bit_isset(state->ARG, 0);
        set_sz(state, result);
        cpu->store(result);
    }
    OP(RORA) {
        state->ARG = state->A;
        int result = state->ARG >> 1 | (state->F.C << 7);
        state->F.C = bit_isset(state->ARG, 0);
        set_sz(state, result);
        state->A = result;
    }
    OP(RTI) {
        state->SR = pop(cpu, state);
        state->F.E = 1;
        state->PC.b.l = pop(cpu, state);
        state->PC.b.h = pop(cpu, state);
    }
    OP(RTS) {
        state->PC.b.l = pop(cpu, state);
        state->PC.b.h = pop(cpu, state);
        state->PC.d++;
    }
    OP(SBC) {
        cpu->fetch();
        uint32_t result = state->A + (~state->ARG) + state->F.C;
        set_sz(state, result);
        state->F.C = result < 0x100;
        state->F.V = bit_isset((result^state->A) & (state->A^state->ARG), 7);
        state->A = result;
    }
    OP(SEC) {
        state->F.C = 1;
    }
    OP(SED) {
        state->F.D = 1;
    }
    OP(SEI) {
        state->F.I = 1;
    }
    OP(STA) {
        cpu->store(state->A);
    }
    OP(STX) {
        cpu->store(state->X);
    }
    OP(STY) {
        cpu->store(state->Y);
    }
    OP(TAX) {
        state->X = state->A;
        set_sz(state, state->X);
    }
    OP(TAY) {
        state->Y = state->A;
        set_sz(state, state->Y);
    }
    OP(TSX) {
        state->X = state->SP;
        set_sz(state, state->X);
    }
    OP(TXA) {
        state->A = state->X;
        set_sz(state, state->A);
    }
    OP(TYA) {
        state->A = state->Y;
        set_sz(state, state->A);
    }
    OP(TXS) {
        state->SP = state->X;
    }

};

/* 65C02 Ops */
namespace M65C02v2
{
    using namespace M6502v2;

    OP(TSB) {
        cpu->fetch();
        int result = state->A | state->ARG;
        state->F.Z = (state->A & state->ARG) == 0;
        cpu->store(result);
    }

    OP(TRB) {
        cpu->fetch();
        int result = ~state->A & state->ARG;
        state->F.Z = (state->A & state->ARG) == 0;
        cpu->store(result);
    }

    OP(BITIMM) {
        cpu->fetch();
        state->F.Z = (state->A & state->ARG) == 0;
    }

    OP(INA) {
        int result = state->A + 1;
        set_sz(state, result);
        state->A = result;
    }

    OP(PHX) {
        push(cpu, state, state->X);
    }

    OP(PLX) {
        int result = pop(cpu, state);
        set_sz(state, result);
        state->X = result;
    }

    OP(PHY) {
        push(cpu, state, state->Y);
    }

    OP(PLY) {
        int result = pop(cpu, state);
        set_sz(state, result);
        state->Y = result;
    }

    OP(SMB) {
        Immediate(cpu, state);
        int bit = cpu->fetch();
        ZeroPage(cpu, state);
        cpu->fetch();
        int result = state->ARG | (1 << bit);
        cpu->store(result);
    }

    OP(RMB) {
        Immediate(cpu, state);
        int bit = cpu->fetch();
        ZeroPage(cpu, state);
        cpu->fetch();
        int result = state->ARG & ~(1 << bit);
        cpu->store(result);
    }

    OP(BBR) {
        Immediate(cpu, state);
        int bit = cpu->fetch();
        ZeroPage(cpu, state);
        cpu->fetch();
        Relative(cpu, state);
        if (!bit_isset(state->ARG, bit)) {
            state->PC = state->EA;
            cpu->add_icycles(2);
        }
    }

    OP(BBS) {
        Immediate(cpu, state);
        int bit = cpu->fetch();
        ZeroPage(cpu, state);
        cpu->fetch();
        Relative(cpu, state);
        if (bit_isset(state->ARG, bit)) {
            state->PC = state->EA;
            cpu->add_icycles(2);
        }
    }

};

