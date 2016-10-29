/**
 * Copyright (c) 2016, Dan Sledz
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

#include "machine/c64/c64.h"

using namespace EMU;
using namespace C64Machine;

enum VICReg {
  M0X = 0x00,
  M0Y = 0x01,
  M1X = 0x02,
  M1Y = 0x03,
  M2X = 0x04,
  M2Y = 0x05,
  M3X = 0x06,
  M3Y = 0x07,
  M4X = 0x08,
  M4Y = 0x09,
  M5X = 0x0A,
  M5Y = 0x0B,
  M6X = 0x0C,
  M6Y = 0x0D,
  M7X = 0x0E,
  M7Y = 0x0F,
  MSBX = 0x10,
  CR1 = 0x11,  // RST8, ECM, BMM, DEN, RSEL, YSCROLL(3)
  RASTER = 0x12,
  LPX = 0x13, // Light pen X
  LPY = 0x14, // Light pen Y
  ME = 0x15, // Sprite enabled
  CR2 = 0x16, // -, -, RES, MCM, CSEL, XSCROLL(3)
  MYE = 0x17,
  MP = 0x18, // VM13, VM12, VM11, VM10, CB13, CB12, CB11, -
  IR = 0x19, // IRQ, -, -, -, ILP, IMMC, IMBC, IRST
  IE = 0x1A, // IRQ, -, -, -, ELP, EMMC, EMBC, ERST
  MDP = 0x1B, // Sprite data priority
  MMC = 0x1C, // Sprite Multicolour
  MXE = 0x1D, // Sprite X expansion
  MM = 0x1E, // Sprite-sprite collision
  MD = 0x1F, // Sprite date collision
  BORDER = 0x20, // Border colour
  B0C = 0x21, // Background colour
  B1C = 0x22,
  B2C = 0x23,
  B3C = 0x24,
  MM0 = 0x25, // Sprite multicolour
  MM1 = 0x26,
  M0C = 0x27, // Sprite colour
  M1C = 0x28,
  M2C = 0x29,
  M3C = 0x2A,
  M4C = 0x2B,
  M5C = 0x2C,
  M6C = 0x2D,
  M7C = 0x2E
};

// Based on  6567R56A timings
VIC2::VIC2(C64 *c64)
  : ScreenDevice(c64, "vic-ii", ClockDivider(1), 512, 262, 388, 488, 13, 40),
    m_c64(c64),
    m_ram(c64->ram()),
    m_palette(),
    m_bus()
{
  init_palette();
  m_bus.add(0x0000, m_ram->direct(0), 0x10000, false);
}

VIC2::~VIC2() {}

void VIC2::load_rom(void) {
  Rom *rom = m_c64->romset()->rom("char");
  m_bus.add(0x1000, rom->direct(0), 0x1000, true);
}

byte_t VIC2::vic2_read(offset_t offset) {
  offset &= 0x3f;
  if (offset > 0x2f)
    return 0xff;
  else
    return m_regs[offset];
}

void VIC2::vic2_write(offset_t offset, byte_t value) {
  offset &= 0x3f;
  if (offset < 0x2f)
    m_regs[offset] = value;
}

void VIC2::init_palette(void) {
  //  0 black
  //  1 white
  //  2 red
  //  3 cyan
  //  4 pink
  //  5 green
  //  6 blue
  //  7 yellow
  //  8 orange
  //  9 brown
  //  10 light red
  //  11 dark gray
  //  12 medium gray
  //  13 light green
  //  14 light blue
  //  15 light gray
  m_palette[0] = RGBColor(0, 0, 0);
  m_palette[1] = RGBColor(255, 255, 255);
  m_palette[2] = RGBColor(255, 0, 0);
  m_palette[3] = RGBColor(0, 255, 255);
  m_palette[4] = RGBColor(255, 192, 203);
  m_palette[5] = RGBColor(0, 255, 0);
  m_palette[6] = RGBColor(0, 0, 255);
  m_palette[7] = RGBColor(255, 255, 0);
  m_palette[8] = RGBColor(255, 165, 0);
  m_palette[9] = RGBColor(165, 42, 42);
  m_palette[10] = RGBColor(255, 42, 42);
  m_palette[11] = RGBColor(169, 169, 169);
  m_palette[12] = RGBColor(128, 128, 128);
  m_palette[13] = RGBColor(128, 255, 128);
  m_palette[14] = RGBColor(128, 128, 255);
  m_palette[15] = RGBColor(211, 211, 211);
}

void VIC2::do_hend(void) {
  // XXX: Update the raster line
  DEVICE_TRACE("hend: ", m_vpos);
  bit_set(m_regs[VICReg::CR1], 7, (m_vpos & 0x100) != 0);
  m_regs[VICReg::RASTER] = m_vpos;
}

void VIC2::do_hdraw(void) { }

void VIC2::do_vnext(void) {
}

void VIC2::do_vdraw(void) {
  // Temporary hack for drawing the screen
  FrameBuffer *fb = machine()->screen();
  offset_t sb = 0x0400;
  offset_t cb = 0xD800;
  for (int y = 0; y < 25; y++) {
    for (int x = 0; x < 40; x++,sb++,cb++) {
      uint16_t b = m_bus.read(sb);
      byte_t c = m_bus.read(cb) & 0x0f;
      for (int iy = 0; iy < 8; iy++) {
        byte_t r = m_bus.read(0x1000 + (b * 8 + iy));
        for (int ix = 0; ix < 8; ix++) {
          RGBColor color;
          if (!bit_isset(r, 7 - ix))
            color = m_palette[c];
          else
            color = m_palette[0];
          fb->set((x * 8) + ix, (y * 8) + iy, color);
        }
      }
    }
  }
}

void VIC2::do_vblank(void) {
  FrameBuffer *fb = machine()->screen();
  fb->flip();
  fb->clear();
  //machine()->set_line("cpu", Line::NMI, LineState::Pulse);
}



