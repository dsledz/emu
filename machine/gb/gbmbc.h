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
/*
 * Memory bank Controller
 */
#pragma once

#include "emu/emu.h"

#include "machine/gb/gb.h"

using namespace EMU;

namespace GBMachine {

enum Cartridge {
    RomOnly = 0x00,
    MBC1    = 0x01,
    MBC1R   = 0x02,
    MBC1RB  = 0x03,
    MBC3RRB = 0x13
};

class GBMBC: public Device {
    public:
        GBMBC(Gameboy *gameboy);
        virtual ~GBMBC(void);

        virtual void save(SaveState &state);
        virtual void load(LoadState &state);
        virtual void line(Line line, LineState state) { }

        void load_rom(const std::string &name);

    private:
        void _reset(void);

        std::string _name;
        Cartridge   _type;
        unsigned    _rom_bank;
        unsigned    _rom_size;
        bvec        _rom;
        unsigned    _ram_bank;
        unsigned    _ram_size;
        bvec        _ram;
};

};
