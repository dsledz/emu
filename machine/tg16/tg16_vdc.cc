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

using namespace TG16Machine;

VDC::VDC(TG16 *tg16, unsigned hertz):
    ClockedDevice(tg16, "vdc", hertz),
    m_sprites(0),
    m_hpos(0),
    m_vpos(0),
    m_status(0),
    m_reg_idx(0),
    m_vram(0x8000),
    m_sat(0x100),
    m_palette(0x200),
    m_pal_bytes(0x200),
    m_pal_idx(0)
{
    for (int i = 0; i < 20; i++)
        m_reg[i].d = 0;
    /* XXX: Set a few registers to sane values. */
    m_reg[HSR].d = 0x0202;
    m_reg[HDR].d = 0x041f;
    m_reg[VPR].d = 0x0f02;
    m_reg[VDW].d = 0x00ef;
    m_reg[VCR].d = 0x0004;
}

VDC::~VDC(void)
{
}

byte_t
VDC::status_read(offset_t offset)
{
    byte_t result = m_status;
    m_status = 0;
    /* XXX: How often should be clear this? */
    machine()->set_line("cpu", Line::INT1, LineState::Clear);

    return result;
}

void
VDC::status_write(offset_t offset, byte_t value)
{
    m_reg_idx = 0x1F & value;
    if (m_reg_idx & 0x10)
        m_reg_idx &= 0x13;
}

uint8_t vram_inc[] = { 1, 32, 64, 128 };

byte_t
VDC::data_read(offset_t offset)
{
    const int b = offset & 0x01;
    byte_t result = 0;
    switch (m_reg_idx) {
    case VxR: {
        result = get_byte(m_vram.at(m_reg[MARR].d % 0x7FFF), b);
        int inc = vram_inc[(m_reg[CR].d & 0x0C00) >> 11];
        if (b == 1)
            m_reg[MARR].d += inc;
        break;
    }
    default:
        throw DeviceFault(name(), "data_read");
        break;
    }
    return result;
}

void
VDC::data_write(offset_t offset, byte_t value)
{
    const int b = offset & 0x01;
    /* XXX: Is this correct? */
    set_byte(m_reg[m_reg_idx], b, value);

    switch (m_reg_idx) {
    default:
        IF_LOG(Trace)
            std::cout << "VDC: " << Hex(m_reg_idx) << " <- "
                      << Hex(m_reg[m_reg_idx].d) << std::endl;
        break;
    case VxR: {
        set_byte(m_vram.at(m_reg[MAWR].d & 0x7FFF), b, value);
        if (b == 1)
            m_reg[MAWR].d += vram_inc[(m_reg[CR].d & 0x0C00) >> 11];
        break;
    }
    case BYR:
        if (b == 1) {
            m_bgvtile = m_reg[BYR].d / 8;
            m_bgvtile &= (bit_isset(m_reg[MWR].d, 6) ? 0x3F : 0x1F);
            m_bgvidx = m_reg[BYR].d % 8;
        }
        break;
    case SATB:
        m_satb_write = true;
        break;
    case LENR:
        DEVICE_INFO("DMA");
        if (b == 1) {
            /* XXX: Cycles are wrong */
            for (uint16_t i = 0; i < m_reg[LENR].d; i++)
                m_vram[m_reg[DESR].d + i] = m_vram[m_reg[SOUR].d + i];
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
    const int width = bg_width[(m_reg[MWR].d >> 4) & 0x3];
    assert(m_bghtile < width);
    m_bgaddr = m_bgvtile * width + m_bghtile;
    uint16_t bat = m_vram.at(m_bgaddr).d;
#if 0
    IF_LOG(Debug) {
        LOG_DEBUG("BGTile: ", Hex(m_bgvtile), " : ", Hex(m_bghtile),
                  " ", Hex(m_bgaddr), " : ", Hex(bat));
    }
#endif
    uint16_t address = ((bat & 0x0FFF) << 4) + m_bgvidx;

    m_bgp0 = m_vram.at(address & 0x7FFF );
    m_bgp1 = m_vram.at((address + 8) & 0x7FFF);
    m_bgpal = (bat & 0xF000) >> 12;
}

uint8_t
VDC::bg_pixel(void)
{
    int idx = 7 - m_bghidx;

    uint8_t color =
        (bit_isset(m_bgp0.b.l, idx) << 0) |
        (bit_isset(m_bgp0.b.h, idx) << 1) |
        (bit_isset(m_bgp1.b.l, idx) << 2) |
        (bit_isset(m_bgp1.b.h, idx) << 3);
    return (color != 0) ? color | (m_bgpal << 4) : color;
}

void
VDC::execute(void)
{
    while (true) {
        add_icycles(1000);
        for (int i = 0; i < 1000; i++)
            step();
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
    m_hpos++;
    if (m_hpos > HMAX) {
        m_hpos = 0;
        m_vpos = (m_vpos + 1) % VMAX;
        if (m_vpos == VBSTART) {
            m_flags.vblank = 1;
            if (bit_isset(m_reg[CR].d, 3))
                machine()->set_line("cpu", Line::INT1, LineState::Assert);
            DEVICE_DEBUG("vblank start");
            if (m_satb_write || bit_isset(m_reg[DCR].d, 4)) {
                for (int i = 0; i < 256; i++)
                    m_sat[i] = m_vram[m_reg[SATB].d + i];
                m_satb_write = 0;
            }
        } else if (m_vpos == VBEND) {
            machine()->screen()->flip();
            machine()->screen()->clear();
            m_flags.vblank = 0;
            m_flags.scanline_irq = 0;
            m_flags.sprite_overflow = 0;
            m_flags.sprite_hit = 0;
            m_bgvtile = m_reg[BYR].d / 8;
            m_bgvtile &= (bit_isset(m_reg[MWR].d, 6) ? 0x3F : 0x1F);
            m_bgvidx = m_reg[BYR].d % 8;

            IF_LOG(Debug) {
                std::cout << "Screen Start: ";
                std::cout << " BYR: " << Hex(m_reg[BYR]);
                std::cout << " BXR: " << Hex(m_reg[BXR]);
                std::cout << std::endl;
            }
        } else if (m_vpos > VBEND && m_vpos < VBSTART) {
            /* XXX: Scanline interrupt */
            if ((m_vpos - VBEND + 63 == m_reg[RCR].d) && bit_isset(m_reg[CR].d, 2)) {
                m_flags.scanline_irq = 1;
                machine()->set_line("cpu", Line::INT1, LineState::Assert);
            } else {
                m_flags.scanline_irq = 0;
            }
            if (m_bgvidx == 7) {
                m_bgvtile++;
                m_bgvtile &= (bit_isset(m_reg[MWR].d, 6) ? 0x3F : 0x1F);
                m_bgvidx = 0;
            } else
                m_bgvidx++;
        }
    }
    if (m_vpos <= VBEND || m_vpos >= VBSTART)
        return;
    if (m_hpos > HBSTART) {

    } else if (m_hpos == HBSTART) {
        /* XXX: DMA? */
    } else if (m_hpos > HBEND &&  m_hpos < HBSTART) {
        FrameBuffer *screen = machine()->screen();
        uint8_t bg = 0;
        if (bit_isset(m_reg[CR].d, 7))
            bg = bg_pixel();
        uint8_t spr = 0;
        if (bit_isset(m_reg[CR].d, 6))
            spr = sprite_pixel(bg);
        if (spr != 0)
            screen->set(m_hpos - HBEND, m_vpos - VBEND, vce(spr, true));
        else
            screen->set(m_hpos - HBEND, m_vpos - VBEND, vce(bg, false));
        if (m_bghidx == 7) {
            m_bghidx = 0;
            m_bghtile++;
            m_bghtile &= bg_width[(m_reg[MWR].d >> 4) & 0x3] - 1;
            bg_cache();
        } else
            m_bghidx++;
    } else if (m_hpos == HBEND) {
        /* Start drawing */
        m_bghtile = m_reg[BXR].d / 8;
        m_bghtile &= bg_width[(m_reg[MWR].d >> 4) & 0x3] - 1;
        m_bghidx = m_reg[BXR].d % 8;
        bg_cache();
        m_sprites.clear();
        if (bit_isset(m_reg[CR].d, 6))
            sprite_cache();
    }
    /* Output a pixel to the VCE */
}

void
VDC::sprite_cache(void)
{
    /* 64 sprites? */
    int sy = m_vpos - VBEND;
    for (int i = 0; i < 64; i++) {
        Sprite sprite(&m_sat.at(i * 4));
        if (sprite.matchy(sy)) {
            m_sprites.push_back(sprite);
        }
    }
}

uint8_t
VDC::sprite_pixel(uint8_t bg)
{
    int sx = m_hpos - HBEND;
    int sy = m_vpos - VBEND;
    uint8_t color = 0;
    for (auto it = m_sprites.begin(); it != m_sprites.end() && color == 0; it++) {
        if (it->matchx(sx)) {
            color = it->pixel(m_vram, sx, sy);
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
    return m_palette[idx];
}

byte_t
VDC::vce_read(offset_t offset)
{
    byte_t result = 0;
    switch (offset % 8) {
    case 4:
        result = m_pal_bytes[m_pal_idx.d].b.l;
        break;
    case 5:
        result = m_pal_bytes[m_pal_idx.d].b.h;
        m_pal_idx.d++;
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
        m_pal_idx.b.l = value;
        break;
    case 3:
        m_pal_idx.b.h = value;
        break;
    case 4:
        m_pal_bytes[m_pal_idx.d].b.l = value;
        break;
    case 5:
        m_pal_bytes[m_pal_idx.d].b.h = value;
        /* XXXXXXXGGGRRRBBB */
        m_palette[m_pal_idx.d] = RGBColor(
            ((m_pal_bytes[m_pal_idx.d].d & 0x038) >> 3) * 32,
            ((m_pal_bytes[m_pal_idx.d].d & 0x1C0) >> 6) * 32,
            ((m_pal_bytes[m_pal_idx.d].d & 0x007) >> 0) * 32);
        m_pal_idx.d++;
        break;
    default:
        break;
    }
}

