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
#include "cpu/m6502v2.h"

using namespace M6502v2;

class HuC6280Cpu: public M65C02Cpu
{
public:
    HuC6280Cpu(Machine *machine, const std::string &name, unsigned clock,
              bus_type *bus);
    virtual ~HuC6280Cpu(void);

    void build_table(void);

protected:
    OP(SXY) {
        std::swap(state.X, state.Y);
    }

    OP(SAX) {
        std::swap(state.A, state.X);
    }

    OP(SAY) {
        std::swap(state.A, state.Y);
    }

    OP(SET) {
        throw CpuFeatureFault(name(), "T flag");
    }

    void op_tstart(reg16_t *src, reg16_t *dest, reg16_t *len) {
        src->b.l = pc_read();
        src->b.h = pc_read();
        dest->b.l = pc_read();
        dest->b.h = pc_read();
        len->b.l = pc_read();
        len->b.h = pc_read();
        push(state.Y);
        push(state.A);
        push(state.X);
    }
    OP(TDD) {
        reg16_t src, dest, len;
        op_tstart(&src, &dest, &len);

        do {
            byte_t value = bus_read(src.d);
            bus_write(dest.d, value);
            dest.d -= 1;
            src.d -= 1;
            len.d -= 1;
        } while (len.d != 0);

        state.X = pop();
        state.A = pop();
        state.Y = pop();
    }
    OP(TII) {
        reg16_t src, dest, len;
        op_tstart(&src, &dest, &len);

        do {
            byte_t value = bus_read(src.d);
            bus_write(dest.d, value);
            dest.d += 1;
            src.d += 1;
            len.d -= 1;
        } while (len.d != 0);

        state.X = pop();
        state.A = pop();
        state.Y = pop();
    }
    OP(TIN) {
        reg16_t src, dest, len;
        op_tstart(&src, &dest, &len);

        do {
            byte_t value = bus_read(src.d);
            bus_write(dest.d, value);
            src.d += 1;
            len.d -= 1;
        } while (len.d != 0);

        state.X = pop();
        state.A = pop();
        state.Y = pop();
    }
    OP(TAI) {
        reg16_t src, dest, len;
        op_tstart(&src, &dest, &len);
        int b = 0;

        do {
            byte_t value = bus_read(src.d + b);
            bus_write(dest.d, value);
            dest.d += 1;
            len.d -= 1;
            b ^= 1;
        } while (len.d != 0);

        state.X = pop();
        state.A = pop();
        state.Y = pop();
    }
    OP(TIA) {
        reg16_t src, dest, len;
        op_tstart(&src, &dest, &len);
        int b = 0;

        do {
            byte_t value = bus_read(src.d);
            bus_write(dest.d + b, value);
            src.d += 1;
            len.d -= 1;
            b ^= 1;
        } while (len.d != 0);

        state.X = pop();
        state.A = pop();
        state.Y = pop();
    }

    OP(TAM) {
        throw CpuFeatureFault(name(), "MMU");
    }

    OP(TMA) {
        throw CpuFeatureFault(name(), "MMU");
    }

    OP(TST) {
        Immediate();
        int value = fetch();
        throw CpuFeatureFault(name(), "NYI");
        fetch();
        state.F.N = bit_isset(state.ARG, 7);
        state.F.V = bit_isset(state.ARG, 6);
        state.F.Z = (value & state.ARG) == 0;
    }

    OP(CSL) {
        throw CpuFeatureFault(name(), "Speed");
    }

    OP(CSH) {
        throw CpuFeatureFault(name(), "Spped");
    }

    OP(CLA) {
        state.A = 0;
    }

    OP(CLX) {
        state.X = 0;
    }

    OP(CLY) {
        state.Y = 0;
    }

    OP(BSR) {
        state.PC.d--;
        push(state.PC.b.h);
        push(state.PC.b.l);
        state.PC = state.EA;
    }

    OP(ST0) {
        fetch();
        bus_write(0x0000, state.ARG);
    }

    OP(ST1) {
        fetch();
        bus_write(0x0002, state.ARG);
    }

    OP(ST2) {
        fetch();
        bus_write(0x0003, state.ARG);
    }

};

