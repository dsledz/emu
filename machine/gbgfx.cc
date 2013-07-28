/**
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

#include "gbgfx.h"

using namespace Driver;

GBGraphics::GBGraphics(Gameboy *gameboy, unsigned hertz):
    Device(gameboy, "gfx"),
    _vram(0x2000),
    _oam(0x100),
    _lcdc(0),
    _hertz(hertz)
{
    _screen = _machine->screen();

    _bus = gameboy->bus();

    _global_pal[0] = RGBColor(0xff, 0xff, 0xff);
    _global_pal[1] = RGBColor(0xbb, 0xbb, 0xbb);
    _global_pal[2] = RGBColor(0x66, 0x66, 0x66);
    _global_pal[3] = RGBColor(0x00, 0x00, 0x00);

    _bus->add(0x8000, 0xE000,
        [&] (offset_t offset) -> byte_t {
            return _vram.read8(offset);
        },
        [&] (offset_t offset, byte_t value) {
            if (offset < 0x1800) {
                auto *o = &_objs[offset / 16];
                o->dirty = true;
            }
            _vram.write8(offset, value);
        });
    _bus->add(0xFE00, 0xFF00, &_oam);

    _bus->add(VideoReg::LCDC, &_lcdc);
    _bus->add(VideoReg::STAT, &_stat);
    _bus->add(VideoReg::SCY, &_scy);
    _bus->add(VideoReg::SCX, &_scx);
    _bus->add(VideoReg::LY, &_ly);
    _bus->add(GBReg::DMA, 0xFFFF,
        AddressBus16::DefaultRead(),
        [&] (offset_t offset, byte_t arg) {
            addr_t src_addr = (addr_t)arg << 8;
            for (unsigned i = 0; i < 160; i++)
                _oam.write8(i, _bus->read(src_addr + i));
        });
    _bus->add(VideoReg::LYC, &_lyc);
    _bus->add(VideoReg::BGP, 0xFFFF,
        AddressBus16::DataRead(&_bgp),
        WRITE_CB(GBGraphics::palette_write, this, &_bg_pal, &_bgp)
        );
    _bus->add(VideoReg::OBP0, 0xFFFF,
        AddressBus16::DataRead(&_obp0),
        WRITE_CB(GBGraphics::palette_write, this, &_obj0_pal, &_obp0)
        );
    _bus->add(VideoReg::OBP1, 0xFFFF,
        AddressBus16::DataRead(&_obp1),
        WRITE_CB(GBGraphics::palette_write, this, &_obj1_pal, &_obp1)
        );
    _bus->add(VideoReg::WY, &_wy);
    _bus->add(VideoReg::WX, &_wx);

    _tilemap0 = [&](int idx) -> GfxObject<8,8> *{
        char c = _vram.read8(VMem::TileMap0 + idx);
        return get_obj(c + 256);
    };
    _tilemap1 = [&](int idx) -> GfxObject<8,8> *{
        char c = _vram.read8(VMem::TileMap1 + idx);
        return get_obj(c + 256);
    };

    for (unsigned i = 0; i < 384; i++)
        _objs[i].dirty = true;
}

GBGraphics::~GBGraphics(void)
{
}

void
GBGraphics::palette_write(ColorPalette<4> *pal, byte_t *pal_byte,
                          offset_t offset, byte_t value)
{
    (*pal)[0] = trans;
    (*pal)[1] = _global_pal[(value & 0x0C) >> 2];
    (*pal)[2] = _global_pal[(value & 0x30) >> 4];
    (*pal)[3] = _global_pal[(value & 0xC0) >> 6];
    *pal_byte = value;
}

GfxObject<8, 8> *
GBGraphics::get_obj(int idx)
{
    auto *o = &_objs[idx];
    if (o->dirty) {
        byte_t *b = _vram.direct(VMem::ObjTiles + idx*16);
        int p = 0;
        for (int y = 0; y < o->h; y++) {
            o->data[p++] = (bit_isset(b[y*2], 7) | bit_isset(b[y*2+1], 7) << 1);
            o->data[p++] = (bit_isset(b[y*2], 6) | bit_isset(b[y*2+1], 6) << 1);
            o->data[p++] = (bit_isset(b[y*2], 5) | bit_isset(b[y*2+1], 5) << 1);
            o->data[p++] = (bit_isset(b[y*2], 4) | bit_isset(b[y*2+1], 4) << 1);
            o->data[p++] = (bit_isset(b[y*2], 3) | bit_isset(b[y*2+1], 3) << 1);
            o->data[p++] = (bit_isset(b[y*2], 2) | bit_isset(b[y*2+1], 2) << 1);
            o->data[p++] = (bit_isset(b[y*2], 1) | bit_isset(b[y*2+1], 1) << 1);
            o->data[p++] = (bit_isset(b[y*2], 0) | bit_isset(b[y*2+1], 0) << 1);
        }
        o->dirty = false;
    }
    return o;
}

void
GBGraphics::draw_scanline(int y)
{
    if (bit_isset(_lcdc, LCDCBits::BGDisplay)) {
        auto cb = bit_isset(_lcdc, LCDCBits::BGTileMap) ?
                _tilemap1 : _tilemap0;
        int iy = ((_scy + y) % 256);
        for (int x = 0; x < 160; x++) {
            int ix = ((_scx + x) % 256);
            int idx = (iy / 8) * 32 + ix/8;
            auto *obj = cb(idx);
            RGBColor pen = _bg_pal[obj->at(ix % 8, iy % 8)];
            _screen->set(x, y, pen);
        }
    }

    if (bit_isset(_lcdc, LCDCBits::WindowDisplay) && y >= _wy) {
        auto cb = bit_isset(_lcdc, LCDCBits::WindowTileMap) ?
                _tilemap1 : _tilemap0;
        int iy = (y - _wy);
        for (int x = 0; x < 160; x++) {
            // Find the correct bg tile.
            int idx = (iy / 8) * 32 + x/8;
            auto *obj = cb(idx);
            // Find the correct pen.
            RGBColor pen = _bg_pal[obj->at(x % 8, iy % 8)];
            _screen->set(_wx + x, y, pen);
        }
    }

    int sprites = 0;
    bool large a_unused = bit_isset(_lcdc, LCDCBits::OBJSize);
    /* XXX: Large sprites are 8x16 and need a different handler */
    if (large)
        throw CpuFeatureFault(_name, "large sprites");
    for (unsigned i = 160; i >= 4 && sprites < 10; i -= 4) {
        byte_t *b = _oam.direct(i-4);
        int iy = b[Oam::OamY] - 8;
        int ix = b[Oam::OamX] - 8;
        int idx = b[Oam::OamPattern];
        int flags = b[Oam::OamFlags];

        iy -= y;
        if (iy < 0 || iy >= 8)
            continue;

        sprites++;

        if (!bit_isset(flags, OAMFlags::SpriteFlipY))
            iy = 7 - iy;

        auto *obj = get_obj(idx);
        auto *pen = &_obj0_pal;
        if (bit_isset(flags, OAMFlags::SpritePalette))
            pen = &_obj1_pal;

        /* XXX: Handle SpritePriority */
        if (bit_isset(flags, OAMFlags::SpriteFlipX)) {
            for (int x = 0; x < 8; x++) {
                RGBColor color = (*pen)[obj->at(7 - x, iy)];
                if (color != trans &&
                    (!bit_isset(flags, OAMFlags::SpritePriority) ||
                     _screen->get(ix + x, y) == _global_pal[0]))
                    _screen->set(ix + x, y, color);
            }
        } else {
            for (int x = 0; x < 8; x++) {
                RGBColor color = (*pen)[obj->at(x, iy)];
                if (color != trans &&
                    (!bit_isset(flags, OAMFlags::SpritePriority) ||
                     _screen->get(ix + x, y) == _global_pal[0]))
                    _screen->set(ix + x, y, color);
            }
        }
    }
}

void
GBGraphics::execute(Time interval)
{
    _fcycles += interval.to_cycles(Cycles(_hertz)).v;

    switch (_stat & 0x03) {
    case LCDMode::HBlankMode:
        if (_fcycles > H_BLANK_CYCLES) {
            // Transition to VBlank or OAM
            _fcycles -= H_BLANK_CYCLES;

            _ly = (_ly + 1) % SCANLINES;
            if (_ly == DISPLAY_LINES) {
                // Wait until our frame is finished
                _machine->render();
                _machine->set_line("cpu",
                    make_irq_line(GBInterrupt::VBlank),
                    LineState::Pulse);
                _stat = (_stat & 0xfc) | LCDMode::VBlankMode;
                if (bit_isset(_stat, STATBits::Mode01Int))
                    _machine->set_line("cpu",
                        make_irq_line(GBInterrupt::LCDStat),
                        LineState::Pulse);
            } else {
                if (_ly == 0)
                    _screen->clear();
                if (_ly < DISPLAY_LINES)
                    draw_scanline(_ly);
                _stat = (_stat & 0xfc) | LCDMode::OAMMode;
                if (bit_isset(_stat, STATBits::Mode10Int))
                    _machine->set_line("cpu",
                        make_irq_line(GBInterrupt::LCDStat),
                        LineState::Pulse);
            }
            // See if we need to trigger the lcd interrupt.
            if (bit_isset(_stat, STATBits::LYCInterrupt) &&
                !(bit_isset(_stat, STATBits::Coincidence) ^ (_lyc == _ly)))
                _machine->set_line("cpu",
                    make_irq_line(GBInterrupt::LCDStat),
                    LineState::Pulse);
        }
        break;
    case LCDMode::OAMMode:
        if (_fcycles > OAM_CYCLES) {
            // Transition to Active
            _fcycles -= OAM_CYCLES;
            _stat = (_stat & 0xfc) | LCDMode::ActiveMode;
        }
        break;
    case LCDMode::ActiveMode:
        if (_fcycles > ACTIVE_CYCLES) {
            // Transition to HBlank
            _fcycles -= ACTIVE_CYCLES;
            _stat = (_stat & 0xfc) | LCDMode::HBlankMode;
            if (bit_isset(_stat, STATBits::Mode00Int))
                _machine->set_line("cpu",
                    make_irq_line(GBInterrupt::LCDStat),
                    LineState::Pulse);
        }
        break;
    case LCDMode::VBlankMode:
        if (_fcycles > V_BLANK_CYCLES) {
            // Transition to OAM
            _fcycles -= V_BLANK_CYCLES;
            _stat = (_stat & 0xfc) | LCDMode::OAMMode;
            if (bit_isset(_stat, STATBits::Mode10Int))
                _machine->set_line("cpu",
                    make_irq_line(GBInterrupt::LCDStat),
                    LineState::Pulse);
        }
        break;
    }
}

