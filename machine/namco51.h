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

/* XXX: This seems wrong */
namespace EMU {

/**
 * Namco 51xx device. Used as an IO board.
 */
class Namco51: public IODevice
{
public:
    Namco51(Machine *machine);
    virtual ~Namco51(void);

    virtual void write8(offset_t offset, byte_t value);
    virtual byte_t read8(offset_t offset);
    virtual size_t size(void) {
        return 0x4;
    }
    virtual byte_t *direct(offset_t offset) {
        throw DeviceFault("namco51");
    }

private:
    byte_t read_port(const std::string &port) {
        return _machine->read_ioport(port);
    }

    enum class Command {
        Nop0 = 0,
        Coinage = 1,
        Credit = 2,
        DisableRemap = 3,
        EnableRemap = 4,
        SwitchMode = 5,
        Nop6 = 6,
        Nop7 = 7
    };

    enum class Mode {
        Unknown = 0,
        Switch = 1,
        Credit = 2,
    };

    Mode _mode;
    bool _remap;
    int _credits;
    int _coinage_bytes;
    int _read_count;
    int _last_coin;
    int _last_joy;

    Machine *_machine;
};

typedef std::unique_ptr<Namco51> Namco51_ptr;

};
