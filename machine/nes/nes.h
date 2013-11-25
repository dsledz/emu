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

#include "emu/emu.h"


using namespace EMU;
#if 0
#include "cpu/m6502/m6502.h"
using namespace M6502v2;
#else
#include "cpu/m6502.h"
using namespace M6502;
#endif

namespace NESDriver {

enum NameTableMirroring {
    SingleScreenBLK0 = 0,
    SingleScreenBLK1 = 1,
    TwoScreenVMirroring = 2,
    TwoScreenHMirroring = 3
};

struct iNesHeader {
    char magic[4];
    byte_t rom_banks;
    byte_t vrom_banks;
    byte_t hmirroring:1;
    byte_t battery:1;
    byte_t trainer:1;
    byte_t four_screen:1;
    byte_t mapper_low:4;
    byte_t _reserved:4;
    byte_t mapper_high:4;
    byte_t ram_banks;
    byte_t pal_flag;
    byte_t reserved[6];
};

enum NameTable {
    NT0 = 0,
    NT1 = 1,
    NT2 = 2,
    NT3 = 3,
};

class NES;

class NESMapper: public Device
{
public:

    NESMapper(Machine *machine, const iNesHeader *header, bvec &rom);
    virtual ~NESMapper(void);

    virtual void reset(void) = 0;

    virtual byte_t prg_read(offset_t offset) = 0;

    virtual void prg_write(offset_t offset, byte_t value) { }

    virtual byte_t chr_read(offset_t offset) = 0;

    virtual void chr_write(offset_t offset, byte_t value) { }

    virtual byte_t sram_read(offset_t offset) {
        throw DeviceFault(_name, "sram read");
    }

    virtual void sram_write(offset_t offset, byte_t value) {
        throw DeviceFault(_name, "sram write");
    }

    /* Convert a prg bank into an offset */
    size_t prg_bank(int bank);
    size_t prg_bank8k(int bank);

    /* Convert a chr bank into an offset */
    size_t chr_bank(int bank);
    size_t chr_bank1k(int bank);

protected:

    Machine *_machine;
    iNesHeader _header;
    bvec _rom;
};

class NESPPU: public ClockedDevice
{
public:
    NESPPU(NES *machine, const std::string &name, unsigned hertz);
    virtual ~NESPPU(void);

    virtual void execute(Time interval);
    virtual void reset(void);

    int draw_bg(void);
    int draw_sprite(int color, int x, int y);

    void step(void);
private:

    void init_palette(void);

    void cache_bg_tile(void);
    int get_obj_pixel(int obj, int x, int y);

    byte_t ppu_read(offset_t offset);
    void ppu_write(offset_t offset, byte_t value);

    byte_t ppu_bus_read(offset_t offset);
    void ppu_bus_write(offset_t offset, byte_t value);

    byte_t ppu_nt_read(offset_t offset);
    void ppu_nt_write(offset_t offset, byte_t value);

    byte_t ppu_pal_read(offset_t offset);
    void ppu_pal_write(offset_t offset, byte_t value);

    std::vector<RGBColor> _color_table;
    std::vector<RGBColor> _palette;
    bvec _palette_bytes;

    union {
        struct {
            byte_t htable: 1;
            byte_t vtable: 1;
            byte_t vram_step: 1;
            byte_t sprite_table: 1;
            byte_t bg_table: 1;
            byte_t sprite_size: 1;
            byte_t _reserved0: 1;
            byte_t nmi_enabled: 1;
        };
        byte_t value;
    } _reg1;

    union {
        struct {
            byte_t monochrome_mode: 1;
            byte_t bg_clip: 1;
            byte_t spr_clip: 1;
            byte_t bg_visible: 1;
            byte_t spr_visible: 1;
            byte_t color_emph: 3;
        };
        byte_t value;
    } _reg2;

    union {
        struct {
            byte_t _reserved1:5;
            byte_t lost_sprite:1;
            byte_t sprite0_hit:1;
            byte_t vblank:1;
        };
        byte_t value;
    } _status;
    byte_t _sram_addr;
    byte_t _vram_read;
    union {
        struct {
            addr_t coarse_x: 5;
            addr_t coarse_y: 5;
            addr_t nt_hselect: 1;
            addr_t nt_vselect: 1;
            addr_t fine_y: 3;
            addr_t _unused: 1;
        };
        addr_t d;
    } _v;
    union {
        struct {
            addr_t coarse_x: 5;
            addr_t coarse_y: 5;
            addr_t nt_hselect: 1;
            addr_t nt_vselect: 1;
            addr_t fine_y: 3;
            addr_t _unused: 1;
        };
        addr_t d;
    } _t;
    byte_t _x;
    byte_t _latch;
    bool _flip_flop;
    bool _vram_locked;

    std::vector<int> _sprites;

    int _hpos; /* Current pixel gun x position 0 - 340 */
    int _vpos; /* Current pixel gun y position 0 - 261 */

    byte_t _bgp0; /* Plane 0 of background */
    byte_t _bgp1; /* Plane 1 of background */
    byte_t _bgpal; /* background palette */

    bvec _sram; /* Sprite memory */
    bvec _blk0; /* Name Table 0 */
    bvec _blk1; /* Name Table 1 */

    IOPort *_mirror;

    AddressBus16 *_cpu_bus;
    AddressBus16 *_ppu_bus;
    AddressBus8 *_sprite_bus;
};

typedef std::unique_ptr<NESMapper> mapper_ptr;
mapper_ptr load_cartridge(NES *nes, const std::string &rom);

class NES: public Machine
{
public:
    NES(void);
    virtual ~NES(void);

    virtual void load_rom(const std::string &rom);

    AddressBus16 *cpu_bus(void) {
        return &_cpu_bus;
    }

    AddressBus16 *ppu_bus(void) {
        return &_ppu_bus;
    }

    AddressBus8 *sprite_bus(void) {
        return &_sprite_bus;
    }

private:
    static const std::vector<std::string> ports;

    std::vector<device_ptr> _our_devs;

    std::unique_ptr<M6502Cpu> _cpu;
    std::unique_ptr<NESPPU> _ppu;
    mapper_ptr _mapper;
    Ram _ram;

    AddressBus16 _cpu_bus;
    AddressBus16 _ppu_bus;
    AddressBus8 _sprite_bus;

    int _joy1_shift;
    int _joy2_shift;
};

};
