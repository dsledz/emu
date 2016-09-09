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
#include "cpu/z80/z80_state.h"

using namespace Core;
using namespace EMU;
using namespace CPU2;
using namespace Z80;

namespace Z80v2 {

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

typedef std::unique_ptr<Z80Bus> Z80Bus_ptr;

class Z80Class
{
public:
    Z80Class();
    ~Z80Class();

    void Interrupt(ClockedDevice *cpu, Z80State *state);
    void Decode(ClockedDevice *cpu, Z80State *state);
    void Dispatch(ClockedDevice *cpu, Z80State *state);
    std::string Log(Z80State *state);

private:
};

typedef CPU2::Cpu<Z80Bus, Z80State, Z80Opcode, Z80Class> Z80Cpu;
typedef std::unique_ptr<Z80Cpu> Z80Cpu_ptr;
};
