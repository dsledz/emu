#pragma once

#include "emu/emu.h"

#define OP(op, ...) void op(Z80State *state, ##__VA_ARGS__)

namespace Z80
{
    uint8_t set_parity(byte_t result) {
        return (0 == (bit_isset(result, 0) ^ bit_isset(result, 1) ^
                      bit_isset(result, 2) ^ bit_isset(result, 3) ^
                      bit_isset(result, 4) ^ bit_isset(result, 5) ^
                      bit_isset(result, 6) ^ bit_isset(result, 7))) << 2;
    }

    uint8_t set_flags(byte_t result)
    {
        /*
         * _flags.S = bit_isset(result, 7);
         * _flags.Z = (result & 0xff) == 0;
         * _flags.Y = bit_isset(result, 5);
         * _flags.H = false;
         * _flags.X = bit_isset(result, 3);
         * _set_parity(result);
         * _flags.N = false;
         * _flags.C = false;
         */
        return ((result & 0xA8) | (result == 0) << 6 | set_parity(result));
    }

    uint8_t set_flags(uint16_t result, byte_t arg1, byte_t arg2)
    {
        /*  _flags.S = bit_isset(result, 7);
         *  _flags.Z = (result & 0xff) == 0;
         *  _flags.Y = bit_isset(result, 5);
         *  _flags.H = bit_isset(dest ^ arg ^ result, 4);
         *  _flags.X = bit_isset(result, 3);
         *  _flags.V = bit_isset(dest ^ arg ^ result, 7);
         *  _flags.N = false;
         *  _flags.C = bit_isset(result, 8);
         */
        return ((result & 0xA8) |
                ((arg1 ^ arg2 ^ result) & 0x10) |
                (bit_isset(arg1 ^ arg2 ^ result, 7) << 2) |
                ((result & 0x100) >> 8));
    }

    OP2(LD8)
    {
        byte_t res = cpu->fetch8(r2);
        cpu->store8(r1, res);
    }

    OP2(LD16)
    {
        uint16_t result = cpu->fetch16(r2);
        cpu->store16(r1, result);
    }

    OP1(ADD)
    {
        uint8_t arg = cpu->fetch8(r1);
        uint16_t result = state->AF.h + arg;

        state->AF.l = set_flags(result, state->AF.h, arg);
        state->AF.h = result;
    }

    OP1(ADC)
    {
        uint8_t arg = cpu->fetch8(r1);
        uint16_t result = state->AF.h + arg + state->AF.f.C;

        state->AF.l = set_flags(result, state->AF.h, arg);
        state->AF.h = result;
    }

    OP1(INC)
    {
        uint8_t arg = cpu->fetch8(r1);
        uint16_t result = arg + 1;

        state->AF.l = set_flags(result, arg, 1);

        cpu->store8(r1, result);
    }

    OP1(ORA)
    {
        byte_t result = state->AF.h | cpu->fetch8(r1);

        state->AF.l = set_flags(result);
        state->AF.h = result;
    }

    OP1(SUB)
    {
        uint8_t arg1 = state->AF.h;
        uint8_t arg2 = cpu->fetch8(r1);
        uint16_t result = arg1 - arg2;

        state->AF.l = set_flags(result);
        state->AF.h = result;
    }

    OP1(SBC)
    {
        uint8_t arg1 = state->AF.h;
        uint8_t arg2 = cpu->fetch8(r1);
        uint16_t result = arg1 - arg2 - state->AF.f.C;

        state->AF.l = set_flags(result);
        state->AF.h = result;
    }

    OP1(CP)
    {
        uint8_t arg1 = state->AF.h;
        uint8_t arg2 = cpu->fetch8(r1);
        uint16_t result = arg1 - arg2;

        state->AF.l = set_flags(result);
    }

    OP1(XOR)
    {
        uint8_t arg1 = state->AF.h;
        uint8_t arg2 = cpu->fetch8(r1);
        uint16_t result = arg1 ^ arg2;

        state->AF.l = set_flags(result);
        state->AF.h = result;
    }

    OP1(RL)
    {
        uint8_t arg1 = cpu->fetch8(r1);
        uint16_t result = (arg1 << 1) | state->AF.f.C;

        state->AF.l = set_flags(result);
        state->AF.h = result;
        state->AF.f.C = bit_isset(result, 7);
    }

};

