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

NESPPU::NESPPU(NES *machine, const std::string &name, unsigned hertz):
    CpuDevice(machine, name, hertz),
    _color_table(), _palette(), _palette_bytes(),
    _reg1({}), _reg2({}), _status({}),
    _sram_addr(0),
    _vram_addr(0),
    _latch(0),
    _flip_flop(false),
    _hpos(0), _vpos(0), _bgx(0), _bgy(0),
    _hscroll(0), _htable(0), _vscroll(0), _vtable(0),
    _sram(), _blk0(), _blk1()
{
    /* 256 Bytes of sram */
    _sram.resize(256);
    /* 2K of memory in two chunks */
    _blk0.resize(0x0400);
    _blk1.resize(0x0400);
    _palette_bytes.resize(0x0020);

    _mirror = _machine->input_port("MIRRORING");

    _cpu_bus = machine->cpu_bus();
    _ppu_bus = machine->ppu_bus();
    _sprite_bus = machine->sprite_bus();

    _cpu_bus->add(0x2000, 0xE000,
        [&](offset_t offset) -> byte_t {
            return ppu_read(offset);
        },
        [&](offset_t offset, byte_t value) {
            ppu_write(offset, value);
        });

    _ppu_bus->add(0x2000, 0xE000,
        [&](offset_t offset) -> byte_t {
            if ((offset & 0x1f00) == 0x1F00)
                return ppu_pal_read(offset);
            else
                return ppu_nt_read(offset);
        },
        [&](offset_t offset, byte_t value) {
            if ((offset & 0x1f00) == 0x1F00)
                ppu_pal_write(offset, value);
            else
                ppu_nt_write(offset, value);
        });

    machine->sprite_bus()->add(0x00, 0x00,
        [&](offset_t offset) -> byte_t {
            return _sram[offset];
        },
        [&](offset_t offset, byte_t value) {
            _sram[offset] = value;
        });

    init_palette();
}

NESPPU::~NESPPU(void)
{

}

void
NESPPU::execute(Time interval)
{
    _avail += interval.to_cycles(Cycles(_hertz));

    while (_avail > 0) {
        step();
        _avail -= Cycles(1);
    }
}

byte_t
NESPPU::read_atable(int x, int y)
{
    const int ix = x / 32;
    const int iy = y / 32;
    return ppu_nt_read(_vtable << 11 | _htable << 10 | (iy * 8 + ix) + 0x3C0);
}

byte_t
NESPPU::read_ntable(int x, int y)
{
    const int ix = x / 8;
    const int iy = y / 8;
    return ppu_nt_read(_vtable << 11 | _htable << 10 | (iy * 32 + ix));
}

void
NESPPU::cache_bg_tile(int x, int y)
{
    int obj = read_ntable(x, y) + (_reg1.bg_table * 256);
    byte_t b = read_atable(x, y);
    int bit = 0;
    if (x & 0x10)
        bit += 2;
    if (y & 0x10)
        bit += 4;

    offset_t offset = obj * 16 + (y % 8);
    _bgp0 = _ppu_bus->read(offset);
    _bgp1 = _ppu_bus->read(offset+8);
    _bgpal = (bit_isset(b, bit) << 2) | (bit_isset(b, bit + 1) << 3);
}

void
NESPPU::draw_bg(RGBColor *pixel, int bgx, int bgy)
{
    int ix = bgx % 8;
    if ((bgx % 8) == 0)
        cache_bg_tile(bgx, bgy);

    int pen = bit_isset(_bgp0, 7 - ix) | (bit_isset(_bgp1, 7 - ix) << 1);
    if (pen == 0)
        *pixel = _palette[0];
    else
        *pixel = _palette[pen | _bgpal];
}

void
NESPPU::draw_sprite(RGBColor *pixel, int x, int y)
{
    const int sprite_size = (_reg1.sprite_size ? 15 : 7);
    for (auto it = _sprites.begin(); it != _sprites.end(); it++) {
        int idx = *it;
        int iy = _sram[idx] + 1;
        if (y < iy || y > iy + sprite_size)
            continue;
        int ix = _sram[idx + 3];
        if (x < ix || x > ix + 7)
            continue;
        int obj = _sram[idx + 1] + (_reg1.sprite_table * 256);
        const int flags = _sram[idx + 2];
        iy = y - iy;
        ix = x - ix;
        if (sprite_size > 7)
            obj &= ~0x01;
        /* Vertical mirror */
        if (bit_isset(flags, 7))
            iy = sprite_size - iy;
        /* Horizontal mirror */
        if (bit_isset(flags, 6))
            ix = 7 - ix;
        /* Handle large sprite */
        if (iy >= 8) {
            iy -= 8;
            obj++;
        }

        const int pen = get_obj_pixel(obj, ix, iy);
        if (pen != 0) {
            if (idx == _sram_addr && *pixel != _palette[0])
                _status.sprite0_hit = 1;
            if (!bit_isset(flags, 5) || *pixel == _palette[0])
                *pixel = _palette[0x10 + ((flags & 0x03) << 2) | pen];
            break;
        }
    }
}

void
NESPPU::step(void)
{
    _hpos++;
    if (_hpos > 340) {
        _hpos = 0;
        _vpos = (_vpos + 1) % 261;
        if (_vpos == 0) {
            _machine->render();
            _machine->screen()->clear();
            _status.vblank = 1;
            if (_reg1.nmi_enabled)
                _machine->set_line("cpu", InputLine::NMI, LineState::Pulse);
        } else if (_vpos == 20) {
            _status.vblank = 0;
            _status.sprite0_hit = 0;
            _bgy = _vscroll;
            _vtable = _reg1.vtable;
            _machine->set_line("mapper", InputLine::INT0, LineState::Pulse);
        } else if (_vpos > 21) {
            /* Reset the horizontal table */
            _bgx = _hscroll;
            _htable = _reg1.htable;
            _bgy++;
            if (_bgy >= 240) {
                _bgy -= 240;
                _vtable = !_vtable;
            }
            _machine->set_line("mapper", InputLine::INT0, LineState::Pulse);
        }
    } else if (_hpos == 65) {
        /* Calculate the available sprites */
        _sprites.resize(0);
        const int sy = _vpos - 21;
        const int sprite_size = (_reg1.sprite_size ? 15 : 7);
        for (int idx = 0; idx < 256; idx += 4) {
            int iy = _sram[idx] + 1;
            if (sy < iy || sy > iy + sprite_size)
                continue;
            if (_sprites.size() == 8) {
                _status.lost_sprite = 1;
                break;
            }
            _sprites.push_back(idx);
        }
    } else if (_hpos == 80) {
        /* Cache the first background tile entry */
        cache_bg_tile(_bgx, _bgy);
    } else if (_hpos >= 85 && _vpos > 21 && _vpos < 261) {
        int sx = _hpos - 85;
        int sy = _vpos - 21;
        RGBColor *pixel = _machine->screen()->at(sx, sy);
        if (_reg2.bg_visible && (_reg2.bg_clip || sy >= 8))
            draw_bg(pixel, _bgx, _bgy);
        if (_reg2.spr_visible && (_reg2.spr_clip || sx >= 8))
            draw_sprite(pixel, sx, sy);
        _bgx++;
        if (_bgx >= 256) {
            _bgx -= 256;
            _htable = !_htable;
        }
    }
}

void
NESPPU::init_palette(void)
{
    /* XXX: VS palette */
    unsigned entry[] = {
        0333,0014,0006,0326,0403,0503,0510,0420,
        0320,0120,0031,0040,0022,0000,0000,0000,
        0555,0036,0027,0407,0507,0704,0700,0630,
        0430,0140,0040,0053,0044,0000,0000,0000,
        0777,0357,0447,0637,0707,0737,0740,0750,
        0660,0360,0070,0276,0077,0000,0000,0000,
        0777,0567,0657,0757,0747,0755,0764,0772,
        0773,0572,0473,0276,0467,0000,0000,0000,
    };

    for (int i = 0; i < 64; i++) {
        _color_table[i] = RGBColor(
            ((entry[i] & 0x1C0) >> 6) * 32,
            ((entry[i] & 0x038) >> 3) * 32,
            ((entry[i] & 0x007) >> 0) * 32);
    }

    _palette[0] = _color_table[63];
    for (int i = 1; i < 32; i++)
        _palette[i] = _color_table[i % 4 + 4];

}

int
NESPPU::get_obj_pixel(int obj, int x, int y)
{
    offset_t offset = obj * 16 + y;
    byte_t l = _ppu_bus->read(offset);
    byte_t h = _ppu_bus->read(offset+8);

    return bit_isset(l, 7 - x) | (bit_isset(h, 7 - x) << 1);
}

byte_t
NESPPU::ppu_read(offset_t offset)
{
    byte_t result = _latch;
    switch (offset % 8) {
    case 0: /* 0x2000 PPU Control register 1 */
        result = _reg1.value;
        break;
    case 1: /* 0x2001 PPU Control register 2 */
        result = _reg2.value;
        break;
    case 2: /* 0x2002 PPU Status register */
        result = _status.value;
        /* flip_flop is cleared on read */
        _flip_flop = 0;
        _latch = 0;
        _status.vblank = 0;
        break;
    case 3: /* 0x2003 SPR-RAM Address */
        result = _sram_addr;
        break;
    case 4: /* 0x2004 SPR-RAM Data */
        result = _sprite_bus->read(_sram_addr);
        break;
    case 5: /* 0x2005 BG Scroll */
        /* XXX: How do we return 16 bits in one byte? */
        break;
    case 6: /* 0x2006 VRAM Address */
        /* XXX: How do we return 16bits in one byte? */
        break;
    case 7: /* 0x2006 VRAM Data */
        result = _vram_read;
        if (_vram_addr < 0x2000)
            _vram_read = _ppu_bus->read(_vram_addr);
        else {
            _vram_read = ppu_nt_read(_vram_addr);
            if (_vram_addr >= 0x3f00)
                result = ppu_pal_read(_vram_addr);
        }
        if (_reg1.vram_step)
            _vram_addr += 32;
        else
            _vram_addr++;
        _vram_addr %= 0x4000;
        break;
    }
    return result;
}

void
NESPPU::ppu_write(offset_t offset, byte_t value)
{
    switch (offset % 8) {
    case 0:
        _reg1.value = value;
        break;
    case 1:
        _reg2.value = value;
        break;
    case 2:
        /* XXX: Don't write */
        break;
    case 3:
        _sram_addr = value;
        break;
    case 4:
        _sprite_bus->write(_sram_addr, value);
        _sram_addr++;
        break;
    case 5:
        _latch = value;
        if (!_flip_flop) {
            _hscroll = value;
        } else {
            if (value > 239)
                throw CpuFault();
            _vscroll = value;
        }
        _flip_flop = !_flip_flop;
        break;
    case 6:
        if (_flip_flop)
            _vram_addr = ((_latch & 0x3f) << 8) | value;
        else
            _latch = value;
        _flip_flop = !_flip_flop;
        break;
    case 7:
        _ppu_bus->write(_vram_addr, value);
        if (_reg1.vram_step)
            _vram_addr += 32;
        else
            _vram_addr++;
        _vram_addr %= 0x4000;

        break;
    }
}

byte_t
NESPPU::ppu_nt_read(offset_t offset)
{
    NameTableMirroring mirror = NameTableMirroring(_mirror->value);
    NameTable nt = NameTable((offset & 0x0C00) >> 10);
    offset &= 0x03ff;
    byte_t result = 0;
    switch (nt) {
    case NT0:
        switch (mirror) {
        case SingleScreenBLK1:
            result = _blk1[offset];
            break;
        case SingleScreenBLK0:
        case TwoScreenVMirroring:
        case TwoScreenHMirroring:
            result = _blk0[offset];
            break;
        }
        break;
    case NT1:
        switch (mirror) {
        case SingleScreenBLK1:
        case TwoScreenVMirroring:
            result = _blk1[offset];
            break;
        case SingleScreenBLK0:
        case TwoScreenHMirroring:
            result = _blk0[offset];
            break;
        }
        break;
    case NT2:
        switch (mirror) {
        case SingleScreenBLK1:
        case TwoScreenHMirroring:
            result = _blk1[offset];
            break;
        case SingleScreenBLK0:
        case TwoScreenVMirroring:
            result = _blk0[offset];
            break;
        }
    case NT3:
        switch (mirror) {
        case SingleScreenBLK0:
            result = _blk0[offset];
            break;
        case SingleScreenBLK1:
        case TwoScreenVMirroring:
        case TwoScreenHMirroring:
            result = _blk1[offset];
            break;
        }
        break;
    }
    return result;
}

void
NESPPU::ppu_nt_write(offset_t offset, byte_t value)
{
    NameTableMirroring mirror = NameTableMirroring(_mirror->value);
    NameTable nt = NameTable((offset & 0x0C00) >> 10);
    offset &= 0x03ff;
    switch (nt) {
    case NT0:
        _blk0[offset] = value;
        break;
    case NT1:
        switch (mirror) {
        case SingleScreenBLK1:
        case TwoScreenVMirroring:
            _blk1[offset] = value;
            break;
        case SingleScreenBLK0:
        case TwoScreenHMirroring:
            _blk0[offset] = value;
            break;
        }
        break;
    case NT2:
        switch (mirror) {
        case SingleScreenBLK1:
        case TwoScreenHMirroring:
            _blk1[offset] = value;
            break;
        case SingleScreenBLK0:
        case TwoScreenVMirroring:
            _blk0[offset] = value;
            break;
        }
    case NT3:
        _blk1[offset] = value;
        break;
    }
}

byte_t
NESPPU::ppu_pal_read(offset_t offset)
{
    offset &= 0x001f;
    return _palette_bytes[offset];
}

void
NESPPU::ppu_pal_write(offset_t offset, byte_t value)
{
    offset &= 0x001f;
    value &= 0x3f;
    _palette_bytes[offset] = value;
    _palette[offset] = _color_table[value];
    if ((offset & 0x03) == 0) {
        _palette_bytes[offset ^ 0x10] = value;
        _palette[offset ^ 0x10] = _color_table[value];
    }
}

