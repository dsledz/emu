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

#include "emu/bits.h"
#include "emu/exception.h"
#include "emu/device.h"
#include "emu/io.h"

#include <fstream>
#include <sys/stat.h>

namespace EMU {

void read_rom(const std::string &name, bvec &rom);

struct RomException: public EmuException {
    RomException(const std::string &path):
        EmuException("Missing rom: "),
        path(path)
    {
        msg += path;
    }
    std::string path;
};

class Rom: public IODevice {
public:
    Rom(void) { }
    Rom(const std::string &path);
    ~Rom(void);

    virtual void write8(offset_t offset, byte_t arg);
    virtual byte_t read8(offset_t offset);
    virtual size_t size(void);
    virtual byte_t *direct(offset_t offset);

    void append(const bvec &data) {
        _rom.insert(_rom.end(), data.begin(), data.end());
    }

private:
    bvec _rom;
};

struct RomRegion {
    RomRegion(const std::string &name, std::initializer_list<std::string> roms):
        name(name), roms(roms) { }

    std::string name;

    std::list<std::string> roms;
};

struct RomDefinition {
    RomDefinition(const std::string &name): name(name) { }

    std::string name;
    std::list<RomRegion> regions;
};

class RomSet {
public:
    RomSet(const std::string &path);
    RomSet(const RomDefinition &definition);
    ~RomSet(void);

    Rom *rom(const std::string &name);

private:
    std::unordered_map<std::string, Rom> _roms;
};

};
