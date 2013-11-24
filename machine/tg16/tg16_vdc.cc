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

#include "tg16.h"

using namespace TG16Driver;

VDC::VDC(TG16 *tg16, unsigned hertz):
    CpuDevice(tg16, "vdc", hertz),
    _sprites(0),
    _hpos(0),
    _vpos(0),
    _status(0),
    _reg_idx(0),
    _vram(0x8000),
    _sat(0x100),
    _palette(0x200),
    _pal_bytes(0x200),
    _pal_idx(0)
{
    for (int i = 0; i < 20; i++)
        _reg[i].d = 0;
    /* XXX: Set a few registers to sane values. */
    _reg[HSR].d = 0x0202;
    _reg[HDR].d = 0x041f;
    _reg[VPR].d = 0x0f02;
    _reg[VDW].d = 0x00ef;
    _reg[VCR].d = 0x0004;
}

VDC::~VDC(void)
{
}

byte_t
VDC::status_read(offset_t offset)
{
    byte_t result = _status;
    _status = 0;
    /* XXX: How often should be clear this? */
    _machine->set_line("cpu", Line::INT1, LineState::Clear);

    return result;
}

void
VDC::status_write(offset_t offset, byte_t value)
{
    _reg_idx = 0x1F & value;
    if (_reg_idx & 0x10)
        _reg_idx &= 0x13;
}

uint8_t vram_inc[] = { 1, 32, 64, 128 };

byte_t
VDC::data_read(offset_t offset)
{
    const int b = offset & 0x01;
    byte_t result = 0;
    switch (_reg_idx) {
    case VxR: {
        result = get_byte(_vram.at(_reg[MARR].d % 0x7FFF), b);
        int inc = vram_inc[(_reg[CR].d & 0x0C00) >> 11];
        if (b == 1)
            _reg[MARR].d += inc;
        break;
    }
    default:
        throw DeviceFault(_name, "data_read");
        break;
    }
    return result;
}

void
VDC::data_write(offset_t offset, byte_t value)
{
    const int b = offset & 0x01;
    /* XXX: Is this correct? */
    set_byte(_reg[_reg_idx], b, value);

    switch (_reg_idx) {
    default:
        IF_LOG(Trace)
            std::cout << "VDC: " << Hex(_reg_idx) << " <- "
                      << Hex(_reg[_reg_idx].d) << std::endl;
        break;
    case VxR: {
        set_byte(_vram.at(_reg[MAWR].d & 0x7FFF), b, value);
        if (b == 1)
            _reg[MAWR].d += vram_inc[(_reg[CR].d & 0x0C00) >> 11];
        break;
    }
    case BYR:
#if 0
        if (b == 1) {
            _bgvtile = _reg[BYR].d / 8;
            _bgvtile &= (bit_isset(_reg[MWR].d, 6) ? 0x3F : 0x1F);
            _bgvidx = _reg[BYR].d % 8;
        }
#endif
        break;
    case SATB:
        _satb_write = true;
        break;
    case LENR:
        INFO("DMA");
        if (b == 1) {
            /* XXX: Cycles are wrong */
            for (uint16_t i = 0; i < _reg[LENR].d; i++)
                _vram[_reg[DESR].d + i] = _vram[_reg[SOUR].d + i];
        }
        break;
    }
}

byte_t
VDC::read(offset_t offset)
{
    byte_t result = 0;
    offset &= 0x03;
    switch (offset) {
    case 0:
        result = status_read(offset);
        break;
    case 2:
    case 3:
        result = data_read(offset);
        break;
    }
    return result;
}

void
VDC::write(offset_t offset, byte_t value)
{
    offset &= 0x03;
    switch (offset) {
    case 0:
    case 1:
        status_write(offset, value);
        break;
    case 2:
    case 3:
        data_write(offset, value);
        break;
    }
}

static uint8_t bg_width[] = { 32, 64, 128, 128 };
//static uint8_t bg_height[] = { 32, 64 };

void
VDC::bg_cache(void)
{
    const int width = bg_width[(_reg[MWR].d >> 4) & 0x3];
    assert(_bghtile < width);
    _bgaddr = _bgvtile * width + _bghtile;
    uint16_t bat = _vram.at(_bgaddr).d;
#if 0
    IF_LOG(Trace) {
        std::cout << "BGTile: " << Hex(_bgvtile) << " : " << Hex(_bghtile)
                  << " " << Hex(_bgaddr) << " : " << Hex(bat)<< std::endl;
    }
#endif
    uint16_t address = ((bat & 0x0FFF) << 4) + _bgvidx;

    _bgp0 = _vram.at(address & 0x7FFF );
    _bgp1 = _vram.at((address + 8) & 0x7FFF);
    _bgpal = (bat & 0xF000) >> 12;
}

uint8_t
VDC::bg_pixel(void)
{
    int idx = 7 - _bghidx;

    uint8_t color =
        (bit_isset(_bgp0.b.l, idx) << 0) |
        (bit_isset(_bgp0.b.h, idx) << 1) |
        (bit_isset(_bgp1.b.l, idx) << 2) |
        (bit_isset(_bgp1.b.h, idx) << 3);
    return (color != 0) ? color | (_bgpal << 4) : color;
}

void
VDC::execute(Time interval)
{
    _avail += interval.to_cycles(Cycles(_hertz));

    const Cycles used(1);

    while (_avail > 0) {
        step();
        _avail -= used;
    }
}

static const int VMAX = 263;
static const int VBSTART = 260;
static const int VBEND = 18;
static const int HMAX = 1365;
static const int HBSTART = 1152;
static const int HBEND = 64;

void
VDC::step(void)
{
    _hpos++;
    if (_hpos > HMAX) {
        _hpos = 0;
        _vpos = (_vpos + 1) % VMAX;
        if (_vpos == VBSTART) {
            _flags.vblank = 1;
            if (bit_isset(_reg[CR].d, 3))
                _machine->set_line("cpu", Line::INT1, LineState::Assert);
            DBG("vblank start");
            if (_satb_write || bit_isset(_reg[DCR].d, 4)) {
                for (int i = 0; i < 256; i++)
                    _sat[i] = _vram[_reg[SATB].d + i];
                _satb_write = 0;
            }
        } else if (_vpos == VBEND) {
            _machine->screen()->flip();
            _machine->screen()->clear();
            _flags.vblank = 0;
            _flags.scanline_irq = 0;
            _flags.sprite_overflow = 0;
            _flags.sprite_hit = 0;
            _bgvtile = _reg[BYR].d / 8;
            _bgvtile &= (bit_isset(_reg[MWR].d, 6) ? 0x3F : 0x1F);
            _bgvidx = _reg[BYR].d % 8;

            IF_LOG(Debug) {
                std::cout << "Screen Start: ";
                std::cout << " BYR: " << Hex(_reg[BYR]);
                std::cout << " BXR: " << Hex(_reg[BXR]);
                std::cout << std::endl;
            }
        } else if (_vpos > VBEND && _vpos < VBSTART) {
            /* XXX: Scanline interrupt */
            if ((_vpos - VBEND + 63 == _reg[RCR].d) && bit_isset(_reg[CR].d, 2)) {
                _flags.scanline_irq = 1;
                _machine->set_line("cpu", Line::INT1, LineState::Assert);
            } else {
                _flags.scanline_irq = 0;
            }
            if (_bgvidx == 7) {
                _bgvtile++;
                _bgvtile &= (bit_isset(_reg[MWR].d, 6) ? 0x3F : 0x1F);
                _bgvidx = 0;
            } else
                _bgvidx++;
        }
    }
    if (_vpos <= VBEND || _vpos >= VBSTART)
        return;
    if (_hpos > HBSTART) {

    } else if (_hpos == HBSTART) {
        /* XXX: DMA? */
    } else if (_hpos > HBEND &&  _hpos < HBSTART) {
        RasterScreen *screen = _machine->screen();
        uint8_t bg = 0;
        if (bit_isset(_reg[CR].d, 7))
            bg = bg_pixel();
        uint8_t spr = 0;
        if (bit_isset(_reg[CR].d, 6))
            spr = sprite_pixel(bg);
        if (spr != 0)
            screen->set(_hpos - HBEND, _vpos - VBEND, vce(spr, true));
        else
            screen->set(_hpos - HBEND, _vpos - VBEND, vce(bg, false));
        if (_bghidx == 7) {
            _bghidx = 0;
            _bghtile++;
            _bghtile &= bg_width[(_reg[MWR].d >> 4) & 0x3] - 1;
            bg_cache();
        } else
            _bghidx++;
    } else if (_hpos == HBEND) {
        /* Start drawing */
        _bghtile = _reg[BXR].d / 8;
        _bghtile &= bg_width[(_reg[MWR].d >> 4) & 0x3] - 1;
        _bghidx = _reg[BXR].d % 8;
        bg_cache();
        _sprites.clear();
        if (bit_isset(_reg[CR].d, 6))
            sprite_cache();
    }
    /* Output a pixel to the VCE */
}

void
VDC::sprite_cache(void)
{
    /* 64 sprites? */
    int sy = _vpos - VBEND;
    for (int i = 0; i < 64; i++) {
        Sprite sprite(&_sat.at(i * 4));
        if (sprite.matchy(sy)) {
            _sprites.push_back(sprite);
        }
    }
}

uint8_t
VDC::sprite_pixel(uint8_t bg)
{
    int sx = _hpos - HBEND;
    int sy = _vpos - VBEND;
    uint8_t color = 0;
    for (auto it = _sprites.begin(); it != _sprites.end() && color == 0; it++) {
        if (it->matchx(sx)) {
            color = it->pixel(_vram, sx, sy);
            if (it->spbg == 0 && bg != 0)
                color = 0;
        }
    }
    return color;
}

RGBColor
VDC::vce(uint8_t color, bool sprite)
{
    uint16_t idx = color | (sprite << 8);
    return _palette[idx];
}

byte_t
VDC::vce_read(offset_t offset)
{
    byte_t result = 0;
    switch (offset % 8) {
    case 4:
        result = _pal_bytes[_pal_idx.d].b.l;
        break;
    case 5:
        result = _pal_bytes[_pal_idx.d].b.h;
        _pal_idx.d++;
    default:
        break;
    }
    return result;
}

void
VDC::vce_write(offset_t offset, byte_t value)
{
    IF_LOG(Debug)
        std::cout << "VCE:" << Hex(offset) << ": " << Hex(value) << std::endl;

    switch (offset % 8) {
    case 0:
        break;
    case 2:
        _pal_idx.b.l = value;
        break;
    case 3:
        _pal_idx.b.h = value;
        break;
    case 4:
        _pal_bytes[_pal_idx.d].b.l = value;
        break;
    case 5:
        _pal_bytes[_pal_idx.d].b.h = value;
        /* XXXXXXXGGGRRRBBB */
        _palette[_pal_idx.d] = RGBColor(
            ((_pal_bytes[_pal_idx.d].d & 0x038) >> 3) * 32,
            ((_pal_bytes[_pal_idx.d].d & 0x1C0) >> 6) * 32,
            ((_pal_bytes[_pal_idx.d].d & 0x007) >> 0) * 32);
        _pal_idx.d++;
        break;
    default:
        break;
    }
}

