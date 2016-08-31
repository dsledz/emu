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

#include "core/bits.h"
#include "emu/emu.h"
#include "cpu/lib/cpu2.h"

namespace Z80 {

enum Z80Arg {
    ArgRegB  = 0,
    ArgRegC  = 1,
    ArgRegD  = 2,
    ArgRegE  = 3,
    ArgRegH  = 4,
    ArgRegL  = 5,
    ArgRegHL = 6,
    ArgRegA  = 7
};

enum Z80Arg16 {
    RegBC = 0,
    RegDE = 1,
    RegHL = 2,
    RegIX = 3,
    RegIY = 4
};

typedef ClockedBus16 Z80Bus;

struct Z80State;

enum Z80Prefix {
    NoPrefix = 0x00,
    DDPrefix = 0xDD,
    FDPrefix = 0xFD,
};

/* Operation specific state */
struct Z80Op {
    Z80Op(void): name("NONE") { }

    void reset(void) {
        prefix = Z80Prefix::NoPrefix;
        d8 = i8 = 0;
        pc = opcode = d16 = i16 = yield = 0;
    }

    Z80Prefix prefix;
    uint16_t pc;
    byte_t opcode;
    byte_t d8;
    byte_t i8;
    uint16_t d16;
    uint16_t i16;
    int yield;

    std::string name;
};

struct Z80State;

struct Z80Opcode
{
    uint8_t code;
    const char *name;
    void (*func)(Z80State *state);
};

struct Z80State {
    Z80State(void) = default;

    union {
        struct {
            union {
                struct {
                    byte_t C:1;   /**< carry flag */
                    byte_t N:1;   /**< add/substract */
                    byte_t V:1;  /**< parity/overflow */
                    byte_t X:1;
                    byte_t H:1;   /**< half carry */
                    byte_t Y:1;
                    byte_t Z:1;   /**< zero flag */
                    byte_t S:1;   /**< sign flag */
                } f;
                reg8_t l;
            };
            reg8_t h;
        } b;
        uint16_t d;
    } AF;
    reg16_t BC;
    reg16_t DE;
    reg16_t HL;
    reg16_t IX;
    reg16_t IY;
    reg16_t SP;
    reg16_t PC;

    reg8_t I;
    reg8_t R;

    reg16_t AF2;
    reg16_t BC2;
    reg16_t DE2;
    reg16_t HL2;

    reg16_t EA;

    Z80Opcode *Op;

    reg8_t d8;
    reg8_t i8;
    reg16_t d16;
    reg16_t i16;

    CPU2::CpuPhase Phase;
    uint8_t icycles;
    Z80Bus *bus;
} __attribute__((packed));

};
