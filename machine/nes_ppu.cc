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
    _v(),
    _t(),
    _x(),
    _latch(0),
    _flip_flop(false),
    _vram_locked(false),
    _hpos(0), _vpos(0),
    _sram(), _blk0(), _blk1()
{
    /* 256 Bytes of sram */
    _sram.resize(256);
    /* 2K of memory in two chunks */
    _blk0.resize(0x0400);
    _blk1.resize(0x0400);
    _palette_bytes.resize(0x0020);

    _mirror = _machine->ioport("MIRRORING");

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

void
NESPPU::cache_bg_tile(void)
{
    int obj = ppu_nt_read(0x2000 | (_v.d & 0xFFF)) + (_reg1.bg_table * 256);
    byte_t b = ppu_nt_read(0x23C0 | (_v.d & 0x0C00) |
                           ((_v.d >> 4) & 0x38) | ((_v.d >> 2) & 0x07));
    /* XXX: calculate */
    int bit = 0;
    if (_v.coarse_x & 0x02)
        bit += 2;
    if (_v.coarse_y & 0x02)
        bit += 4;

    offset_t offset = obj * 16 + _v.fine_y;
    _bgp0 = _ppu_bus->read(offset);
    _bgp1 = _ppu_bus->read(offset+8);
    _bgpal = (bit_isset(b, bit) << 2) | (bit_isset(b, bit + 1) << 3);
}

int
NESPPU::draw_bg(void)
{
    int pen = bit_isset(_bgp0, 7 - _x) |
        (bit_isset(_bgp1, 7 - _x) << 1);
    if (pen == 0)
        return 0;
    else
        return pen | _bgpal;
}

int
NESPPU::draw_sprite(int color, int x, int y)
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
            if (idx == _sram_addr && color != 0 && x != 255)
                _status.sprite0_hit = 1;
            if (!bit_isset(flags, 5) || color == 0)
                color = 0x10 + ((flags & 0x03) << 2) + pen;
            return color;
        }
    }
    return color;
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
            _vram_locked = false;
            _status.vblank = 1;
            if (_reg1.nmi_enabled)
                _machine->set_line("cpu", Line::NMI, LineState::Pulse);
        }
    }
    if (_vpos < 20 || (!_reg2.bg_visible && !_reg2.spr_visible))
        return;
    if (_vpos >= 21 && _hpos == 0) {
        int sy = _vpos - 21;
        for (int sx = 0; sx < 256; sx++) {
            int color = 0;
            if (_reg2.bg_visible && (_reg2.bg_clip || sx >= 8))
                color = draw_bg();
            if (_reg2.spr_visible && (_reg2.spr_clip || sx >= 8))
                color = draw_sprite(color, sx, sy);
            _machine->screen()->set(sx, sy, _palette[color]);
            if (_x == 0x07) {
                _x = 0;
                if (_v.coarse_x == 0x1F) {
                    _v.nt_hselect ^= 1;
                    _v.coarse_x = 0;
                } else
                    _v.coarse_x++;
                cache_bg_tile();
            } else
                _x++;
        }
    } else if (_hpos == 256) {
        DEBUG("Line Start");
        if (_v.fine_y == 0x07) {
            _v.fine_y = 0;
            if (_v.coarse_y == 29) {
                _v.coarse_y = 0;
                _v.nt_vselect ^= 1;
            } else if (_v.coarse_y == 31) {
                _v.coarse_y = 0;
            } else
                _v.coarse_y++;
        } else
            _v.fine_y++;
    } else if (_hpos == 257) {
        _v.coarse_x = _t.coarse_x;
        _v.nt_hselect = _t.nt_hselect;
        _machine->set_line("mapper", Line::INT0, LineState::Pulse);
    } else if (_hpos == 280 && _vpos == 20) {
        _vram_locked = true;
        _v.fine_y = _t.fine_y;
        _v.nt_vselect = _t.nt_vselect;
        _v.coarse_y = _t.coarse_y;
        _status.vblank = 0;
        _status.sprite0_hit = 0;
    } else if (_hpos == 320) {
        cache_bg_tile();
    } else if (_hpos == 328) {
        /* Calculate the available sprites (for the next line) */
        _sprites.resize(0);
        const int sy = _vpos - 20;
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
        if (_v.d < 0x2000)
            _vram_read = _ppu_bus->read(_v.d);
        else {
            _vram_read = ppu_nt_read(_v.d);
            if (_v.d >= 0x3f00)
                result = ppu_pal_read(_v.d);
        }
        if (_reg1.vram_step)
            _v.d += 32;
        else
            _v.d++;
        _v.d %= 0x4000;
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
        _t.nt_hselect = bit_isset(value, 0);
        _t.nt_vselect = bit_isset(value, 1);
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
        _sram_addr = (_sram_addr + 1) % 256;
        break;
    case 5:
        _latch = value;
        if (!_flip_flop) {
            _t.coarse_x = value >> 3;
            _x = value & 0x07;
        } else {
            _t.fine_y = value & 0x07;
            _t.coarse_y = value >> 3;
        }
        _flip_flop = !_flip_flop;
        break;
    case 6:
        if (!_flip_flop) {
            _t.d = ((value & 0x00ff) << 8) | (_t.d & 0x00ff);
        } else {
            _t.d = (_t.d & 0xff00) | value;
            _v.d = _t.d;
        }
        _flip_flop = !_flip_flop;
        break;
    case 7:
        _ppu_bus->write(_v.d, value);
        if (_reg1.vram_step)
            _v.d += 32;
        else
            _v.d++;
        _v.d %= 0x4000;
        break;
    }
}

byte_t
NESPPU::ppu_nt_read(offset_t offset)
{
    NameTableMirroring mirror = NameTableMirroring(
        _machine->read_ioport(_mirror));
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
    NameTableMirroring mirror = NameTableMirroring(
        _machine->read_ioport(_mirror));
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

