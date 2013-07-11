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

#include "emu.h"

using namespace EMU;

/* XXX: This seems wrong */
namespace EMU {

enum class Namco51Command {
    Nop0 = 0,
    Coinage = 1,
    Credit = 2,
    DisableRemap = 3,
    EnableRemap = 4,
    SwitchMode = 5,
    Nop6 = 6,
    Nop7 = 7
};

enum class Namco51Mode {
    Unknown = 0,
    Switch = 1,
    Credit = 2,
};

/**
 * Namco 51xx device. Used as an IO board.
 */
class Namco51: public Device
{
public:
    Namco51(Machine *machine, Device *parent);
    ~Namco51(void);

    virtual void save(SaveState &state);
    virtual void load(LoadState &state);
    virtual void tick(unsigned cycles);
    virtual void set_line(InputLine line, LineState state);
    virtual void write(addr_t addr, byte_t value);
    virtual byte_t read(addr_t addr);

private:
    Namco51Mode _mode;
    bool _remap;
    int _coinage_bytes;
    int _read_count;
    byte_t _in[4];
};

};
