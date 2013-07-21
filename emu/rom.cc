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

#include "rom.h"

#include "dirent.h"

using namespace EMU;

void
EMU::read_rom(const std::string &name, bvec &rom)
{
    struct stat sb;
    if (stat(name.c_str(), &sb) == -1)
        throw RomException(name);
    rom.resize(sb.st_size);
    try {
        std::ifstream file(name, std::ios::in | std::ios::binary);
        file.read((char *)&rom[0], rom.size());
    } catch (std::ifstream::failure e) {
        throw RomException(name);
    }
}

Rom::Rom(const std::string &path) {
    read_rom(path, _rom);
}

Rom::~Rom(void)
{
}

void
Rom::write8(offset_t offset, byte_t arg)
{
    _rom[offset] = arg;
}

byte_t
Rom::read8(offset_t offset)
{
    return _rom[offset];
}

size_t
Rom::size(void)
{
    return _rom.size();
}

byte_t *
Rom::direct(offset_t offset)
{
    return &_rom[offset];
}

RomSet::RomSet(const std::string &path)
{
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir(path.c_str())) == NULL)
        throw RomException(path.c_str());
    while ((ent = readdir(dir)) != NULL) {
        std::string name(ent->d_name);
        if (name == "." || name == "..")
            continue;
        std::string full_path = path + "/" + name;
        Rom rom(full_path);
        _roms.insert(std::make_pair(name, rom));
    }
}

RomSet::RomSet(const RomDefinition &def)
{
    for (auto it = def.regions.begin(); it != def.regions.end(); it++) {
        Rom rom;
        for (auto it2 = it->roms.begin(); it2 != it->roms.end(); it2++) {
            std::string path = def.name + "/" + (*it2);
            bvec data;

            read_rom(path, data);

            rom.append(data);
        }
        _roms.insert(make_pair(it->name, rom));
    }
}

RomSet::~RomSet(void)
{
}

Rom *
RomSet::rom(const std::string &name)
{
    auto it = _roms.find(name);
    if (it == _roms.end())
        throw RomException(name);
    /* XXX This is unstable and unsafe */
    return &it->second;
}

