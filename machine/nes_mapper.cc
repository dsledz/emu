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

#include "nes.h"

using namespace EMU;
using namespace NESDriver;

enum {
    ROM_BANK_SIZE = 0x4000,
    CHR_BANK_SIZE = 0x2000,
};

NESMapper::NESMapper(Machine *machine, const iNesHeader *header, bvec &rom):
    Device(machine, "mapper"),
    _machine(machine),
    _header(*header),
    _rom(std::move(rom))
{
    memcpy(&_header, header, sizeof(_header));

    InputPort *mirror = _machine->input_port("MIRRORING");

    /* XXX: My horizontal mirroring never works */
#if 0
    if (_header.hmirroring)
        mirror->value = TwoScreenHMirroring;
    else
        mirror->value = TwoScreenVMirroring;
#else
    mirror->value = TwoScreenVMirroring;
#endif
}

NESMapper::~NESMapper(void)
{
}

size_t
NESMapper::prg_bank(int bank)
{
    return sizeof(_header) + (ROM_BANK_SIZE * bank);
}

size_t
NESMapper::chr_bank(int bank)
{
    return sizeof(_header) + (ROM_BANK_SIZE * _header.rom_banks) +
        (CHR_BANK_SIZE * bank);
}

class NESMapperNone: public NESMapper
{
public:
    NESMapperNone(Machine *machine, const iNesHeader *header, bvec &rom):
        NESMapper(machine, header, rom),
        _prg_offset(prg_bank(0)),
        _chr_offset(chr_bank(0))
    {
    }

    ~NESMapperNone(void)
    {
    }

    byte_t prg_read(offset_t offset)
    {
        if (_header.rom_banks == 1)
            offset &= 0x3fff;
        return _rom[_prg_offset + offset];
    }

    byte_t chr_read(offset_t offset)
    {
        return _rom[_chr_offset + offset];
    }

private:
    size_t _prg_offset;
    size_t _chr_offset;
};

class NESMapperUNROM: public NESMapper
{
public:
    NESMapperUNROM(Machine *machine, const iNesHeader *header, bvec &rom):
        NESMapper(machine, header, rom),
        _prg_offset0(prg_bank(0)),
        _prg_offset1(prg_bank(header->rom_banks - 1))
    {
        _chr.resize(0x2000);
    }

    ~NESMapperUNROM(void)
    {
    }

    byte_t prg_read(offset_t offset)
    {
        if (offset & 0x4000) {
            offset &= 0x3FFF;
            return _rom[_prg_offset1 + offset];
        } else
            return _rom[_prg_offset0 + offset];
    }

    void prg_write(offset_t offset, byte_t value)
    {
        int prg  = (value & 0x0f);
        _prg_offset0 = prg_bank(prg);
    }

    void chr_write(offset_t offset, byte_t value)
    {
        _chr[offset] = value;
    }

    byte_t chr_read(offset_t offset)
    {
        return _chr[offset];
    }

private:
    size_t _prg_offset0;
    size_t _prg_offset1;
    bvec _chr;
};

class NESMapperGNROM: public NESMapper
{
public:
    NESMapperGNROM(Machine *machine, const iNesHeader *header, bvec &rom):
        NESMapper(machine, header, rom),
        _prg_offset(prg_bank(0)),
        _chr_offset(chr_bank(0))
    {
    }

    ~NESMapperGNROM(void)
    {
    }

    byte_t prg_read(offset_t offset)
    {
        return _rom[_prg_offset + offset];
    }

    void prg_write(offset_t offset, byte_t value)
    {
        int chr = (value & 0x03);
        int prg  = (value & 0x30) >> 3;
        _prg_offset = prg_bank(prg);
        _chr_offset = chr_bank(chr);
    }

    byte_t chr_read(offset_t offset)
    {
        return _rom[_chr_offset + offset];
    }

private:
    size_t _prg_offset;
    size_t _chr_offset;
};

class NESMapperMMC1: public NESMapper
{
public:
    NESMapperMMC1(Machine *machine, const iNesHeader *header, bvec &rom):
        NESMapper(machine, header, rom),
        _prg_offset0(prg_bank(0)),
        _prg_offset1(prg_bank(header->rom_banks - 1)),
        _chr_offset(chr_bank(0)),
        _shift(0),
        _shift_count(0)
    {
        _ram.resize(0x4000);

        if (_header.battery) {
            _battery_ram.resize(0x2000);
        }
    }

    ~NESMapperMMC1(void)
    {
    }

    byte_t prg_read(offset_t offset)
    {
        if ((offset & 0x4000) == 0)
            return _rom[_prg_offset0 + offset];
        else {
            offset &= 0x3fff;
            return _rom[_prg_offset1 + offset];
        }
    }

    void prg_write(offset_t offset, byte_t value)
    {
        if (value & 0x80) {
            _shift_count = 0;
            _shift = 0;
        } else if (_shift_count < 4) {
            _shift = bit_isset(value, 0) << _shift_count | _shift;
            _shift_count++;
        } else {
            switch (offset >> 13) {
            case 0: {
                _machine->input_port("MIRRORING")->value =
                    NameTableMirroring(_shift & 0x03);
                /* XXX: Remaing bits */
                break;
            }
            case 1: {
                int bank = (_shift & 0x1f);
                _chr_offset = chr_bank(bank);
                break;
            }
            case 2:
                break;
            case 3: {
                int bank = (_shift & 0x0f);
                _prg_offset0 = prg_bank(bank);
                break;
            }
            }
            _shift_count = 0;
            _shift = 0;
        }

    }

    void chr_write(offset_t offset, byte_t value)
    {
        _ram[offset] = value;
    }

    byte_t chr_read(offset_t offset)
    {
        return _ram[offset];
    }

    byte_t sram_read(offset_t offset)
    {
        return _battery_ram[offset];
    }

    void sram_write(offset_t offset, byte_t value)
    {
        _battery_ram[offset] = value;
    }

private:
    size_t _prg_offset0;
    size_t _prg_offset1;
    size_t _chr_offset;

    byte_t _shift;
    int _shift_count;

    bvec _ram;

    bvec _battery_ram;
};

enum class Mapper {
    None = 0,
    MMC1 = 1,
    UNROM = 2,
    RAMBO1 = 64,
    GNROM = 66,
};

mapper_ptr
NESDriver::load_cartridge(NES *nes, const std::string &name)
{
    bvec rom;
    iNesHeader header;
    mapper_ptr mapper;

    read_rom(name, rom);

    memcpy(&header, rom.data(), sizeof(header));

    Mapper mapper_type;
    /* Handle broken !DiskDude! format */
    if (rom[7] == 0x44)
        mapper_type = Mapper(header.mapper_low);
    else
        mapper_type = Mapper(header.mapper_low | (header.mapper_high << 4));

    switch (mapper_type) {
    case Mapper::None:
        mapper = mapper_ptr(new NESMapperNone(nes, &header, rom));
        break;
    case Mapper::MMC1:
        mapper = mapper_ptr(new NESMapperMMC1(nes, &header, rom));
        break;
    case Mapper::UNROM:
        mapper = mapper_ptr(new NESMapperUNROM(nes, &header, rom));
        break;
    case Mapper::GNROM:
    case Mapper::RAMBO1:
        mapper = mapper_ptr(new NESMapperGNROM(nes, &header, rom));
        break;
    default:
        throw EmuException();
    }

    return mapper;
}

