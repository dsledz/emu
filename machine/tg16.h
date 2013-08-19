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

#include "cpu/m6502.h"

namespace TG16Driver {

enum VDCReg {
    MAWR = 0,
    MARR = 1,
    VxR = 2,
    CR = 5,
    RCR = 6,
    BXR = 7,
    BYR = 8,
    MWR = 9,
    HSR = 10,
    HDR = 11,
    VPR = 12,
    VDW = 13,
    VCR = 14,
    DCR = 15,
    SOUR = 16,
    DESR = 17,
    LENR = 18,
    SATB = 19
};

class TG16;

class VDC: public CpuDevice
{
public:
    VDC(TG16 *tg16, unsigned hertz);
    virtual ~VDC(void);

    virtual void execute(Time interval);

    byte_t read(offset_t offset);
    void write(offset_t offset, byte_t value);

    byte_t vce_read(offset_t offset);
    void vce_write(offset_t offset, byte_t value);

private:

    byte_t status_read(offset_t offset);
    void status_write(offset_t offset, byte_t value);

    byte_t data_read(offset_t offset);
    void data_write(offset_t offset, byte_t value);

    void bg_cache(void);
    uint8_t bg_pixel(void);

    void sprite_cache(void);
    uint8_t sprite_pixel(uint8_t bg);

    RGBColor vce(uint8_t color, bool sprite);

    void step(void);

    struct Sprite {
        Sprite(void) = default;
        Sprite(reg16_t *data):
            y(data[0].d & 0x03FF),
            x(data[1].d & 0x03FF),
            pattern(data[2].d & 0x07FF),
            attrs(data[3].d)
        {
            y -= 64;
            x -= 32;
            pattern &= ~cgy;
            pattern &= ~cgx;
            pattern >>= 1;
        }

        bool matchy(int sy) {
            int yend = y + (cgy + 1) * 16;
            return (y <= sy && sy < yend);
        };

        bool matchx(int sx) {
            int xend = x + (cgx + 1) * 16;
            return (x <= sx && sx < xend);
        }

        uint8_t pixel(const std::vector<reg16_t> &vram, int sx, int sy) {
            uint16_t p = pattern;
            sy -= y;
            sx -= x;
            if (xflip)
                sx = ((cgx + 1) * 16) - 1 - sx;
            while (sx >= 16) {
                sx -= 16;
                p++;
            }
            sx = 15 - sx;
            if (yflip)
                sy = ((cgy + 1) * 16) - 1 -  sy;
            while (sy >= 16) {
                sy -= 16;
                p += (cgx + 1);
            }
            p <<= 6;
            p += sy;
            uint8_t color =
                (bit_isset(vram[p].d, sx) << 0) |
                (bit_isset(vram[p + 16].d, sx) << 1) |
                (bit_isset(vram[p + 32].d, sx) << 2) |
                (bit_isset(vram[p + 48].d, sx) << 3);
            return (color != 0) ? color | (pal << 4): 0;
        }

        uint16_t y;
        uint16_t x;
        uint16_t pattern;
        union {
            struct {
                uint16_t pal:4;
                uint16_t _u0:3;
                uint16_t spbg:1;
                uint16_t cgx:1;
                uint16_t _u1:2;
                uint16_t xflip:1;
                uint16_t cgy:2;
                uint16_t _u2:1;
                uint16_t yflip:1;
            };
            uint16_t attrs;
        };
    };
    std::vector<Sprite> _sprites;

    /* Internal State */
    int _hpos;
    int _vpos;

    /* Background Tiles */
    uint16_t _bgaddr;
    uint8_t _bgpal;
    reg16_t _bgp0;
    reg16_t _bgp1;
    uint16_t _bgy;
    uint8_t _bghtile;
    uint8_t _bghidx;
    uint8_t _bgvtile;
    uint8_t _bgvidx;

    /* External registers */
    union {
        struct {
            byte_t sprite_hit:1;
            byte_t sprite_overflow:1;
            byte_t scanline_irq:1;
            byte_t vram_satb_end:1;
            byte_t vram_dma_end:1;
            byte_t vblank:1;
            byte_t _unused:2;
        } _flags;
        byte_t _status;
    };
    int _reg_idx;
    reg16_t _reg[20];

    bool _satb_write;
    std::vector<reg16_t> _vram;
    std::vector<reg16_t> _sat;

    /* XXX: Move to VCE */
    std::vector<RGBColor> _palette;
    std::vector<reg16_t> _pal_bytes;
    reg16_t _pal_idx;
};

class PSG: public Device
{
public:
    PSG(TG16 *tg16);
    virtual ~PSG(void);

    virtual void execute(Time interval);

    byte_t read(offset_t offset);
    void write(offset_t offset, byte_t value);
};

class TG16: public Machine
{
public:
    TG16(const std::string &rom);
    virtual ~TG16(void);

private:

    void init_joypad(void);

    byte_t bank_read(offset_t offset);
    void bank_write(offset_t offset, byte_t value);

    byte_t joypad_read(offset_t offset);
    void joypad_write(offset_t offset, byte_t value);

    byte_t ram_read(offset_t offset);
    void ram_write(offset_t offset, byte_t value);

    std::unique_ptr<M6502::hu6280Cpu> _cpu;
    AddressBus21 _cpu_bus;
    Ram _ram;
    VDC _vdc;
    PSG _psg;
    bvec _rom;
    offset_t _rom_offset;

    int _joypad;
    int _joypad_data;
};

};