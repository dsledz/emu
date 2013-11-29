#pragma once

#include "cpu/m6502/m6502_ops.h"

namespace HuC6280
{
    using namespace M6502v2;

    static inline void op_tstart(M6502State *state, reg16_t *src, reg16_t *dest, reg16_t *len) {
        src->b.l = pc_read(state);
        src->b.h = pc_read(state);
        dest->b.l = pc_read(state);
        dest->b.h = pc_read(state);
        len->b.l = pc_read(state);
        len->b.h = pc_read(state);
        push(state, state->Y);
        push(state, state->A);
        push(state, state->X);
    }

    OP(SXY) {
        reg8_t tmp = state->X;
        state->X = state->Y;
        state->Y = tmp;
    }

    OP(SAX) {
        reg8_t tmp = state->A;
        state->A = state->X;
        state->X = tmp;
    }

    OP(SAY) {
        reg8_t tmp = state->A;
        state->A = state->Y;
        state->Y = tmp;
    }

    OP(SET) {
        throw CpuFeatureFault("HuC6280", "T Flag");

    }

    OP(TDD) {
        reg16_t src, dest, len;
        op_tstart(state, &src, &dest, &len);

        do {
            byte_t value = state->bus_read(src.d);
            state->bus_write(dest.d, value);
            dest.d -= 1;
            src.d -= 1;
            len.d -= 1;
        } while (len.d != 0);

        state->X = pop(state);
        state->A = pop(state);
        state->Y = pop(state);
    }
    OP(TII) {
        reg16_t src, dest, len;
        op_tstart(state, &src, &dest, &len);

        do {
            byte_t value = state->bus_read(src.d);
            state->bus_write(dest.d, value);
            dest.d += 1;
            src.d += 1;
            len.d -= 1;
        } while (len.d != 0);

        state->X = pop(state);
        state->A = pop(state);
        state->Y = pop(state);
    }
    OP(TIN) {
        reg16_t src, dest, len;
        op_tstart(state, &src, &dest, &len);

        do {
            byte_t value = state->bus_read(src.d);
            state->bus_write(dest.d, value);
            src.d += 1;
            len.d -= 1;
        } while (len.d != 0);

        state->X = pop(state);
        state->A = pop(state);
        state->Y = pop(state);
    }
    OP(TAI) {
        reg16_t src, dest, len;
        op_tstart(state, &src, &dest, &len);
        int b = 0;

        do {
            byte_t value = state->bus_read(src.d + b);
            state->bus_write(dest.d, value);
            dest.d += 1;
            len.d -= 1;
            b ^= 1;
        } while (len.d != 0);

        state->X = pop(state);
        state->A = pop(state);
        state->Y = pop(state);
    }
    OP(TIA) {
        reg16_t src, dest, len;
        op_tstart(state, &src, &dest, &len);
        int b = 0;

        do {
            byte_t value = state->bus_read(src.d);
            state->bus_write(dest.d + b, value);
            src.d += 1;
            len.d -= 1;
            b ^= 1;
        } while (len.d != 0);

        state->X = pop(state);
        state->A = pop(state);
        state->Y = pop(state);
    }

    OP(CSL) {
        throw CpuFeatureFault("HuC6280", "Speed");
    }

    OP(CSH) {
        throw CpuFeatureFault("HuC6280", "Spped");
    }

    OP(CLA) {
        state->A = 0;
    }

    OP(CLX) {
        state->X = 0;
    }

    OP(CLY) {
        state->Y = 0;
    }

    OP(BSR) {
        state->PC.d--;
        push(state, state->PC.b.h);
        push(state, state->PC.b.l);
        state->PC = state->EA;
    }

    OP(TST) {
        throw CpuFeatureFault("HuC6280", "NYI");
        /*
        Immediate();
        int value = fetch(state);
        fetch(state);
        state->F.N = bit_isset(state->ARG, 7);
        state->F.V = bit_isset(state->ARG, 6);
        state->F.Z = (value & state->ARG) == 0;
        */
    }

    OP(TAM) {
        throw CpuFeatureFault("HuC6280", "MMU");
    }

    OP(TMA) {
        throw CpuFeatureFault("HuC6280", "MMU");
    }

    OP(ST0) {
        fetch(state);
        state->bus_write(0x0000, state->ARG);
    }

    OP(ST1) {
        fetch(state);
        state->bus_write(0x0002, state->ARG);
    }

    OP(ST2) {
        fetch(state);
        state->bus_write(0x0003, state->ARG);
    }
}
