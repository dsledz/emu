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
#pragma once

#include "machine/gb/gb.h"

namespace GBMachine {

// 201 - 207
#define H_BLANK_CYCLES Cycles(201)
// 77 - 83
#define OAM_CYCLES Cycles(77)
// 169 - 175
#define ACTIVE_CYCLES Cycles(169)
// 4560
#define V_BLANK_CYCLES Cycles(4560)

#define SCANLINE_CYCLES Cycles(456)

#define SCANLINES 156
#define DISPLAY_LINES 144

enum VideoReg {
  LCDC = 0xFF40,
  STAT = 0xFF41,
  SCY = 0xFF42,
  SCX = 0xFF43,
  LY = 0xFF44,
  LYC = 0xFF45,
  BGP = 0xFF47,
  OBP0 = 0xFF48,
  OBP1 = 0xFF49,
  WY = 0xFF4A,
  WX = 0xFF4B,
};

enum Oam {
  OamY = 0,
  OamX = 1,
  OamPattern = 2,
  OamFlags = 3,
};

enum VMem {
  ObjTiles = 0x0000,
  BGTiles = 0x0800,
  TileMap0 = 0x1800,
  TileMap1 = 0x1C00,
};

enum LCDCBits {
  BGDisplay = 0,
  OBJDisplay = 1,
  OBJSize = 2,
  BGTileMap = 3,
  BGTileData = 4,
  WindowDisplay = 5,
  WindowTileMap = 6,
  LCDEnabled = 7,
};

enum OAMFlags {
  SpritePalette = 4,
  SpriteFlipX = 5,
  SpriteFlipY = 6,
  SpritePriority = 7,
};

enum LCDMode {
  HBlankMode = 0,
  VBlankMode = 1,
  OAMMode = 2,
  ActiveMode = 3,
};

enum STATBits {
  LYCInterrupt = 6,
  Mode10Int = 5,
  Mode01Int = 4,
  Mode00Int = 3,
  Coincidence = 2,
  LCDModeBit1 = 1,
  LCDModeBit0 = 0
};

class GBGraphics : public ClockedDevice {
 public:
  GBGraphics(Gameboy *gameboy, ClockDivider divider);
  virtual ~GBGraphics(void);

  virtual void save(SaveState &state) {}
  virtual void load(LoadState &state) {}
  virtual void execute(void);
  virtual void line(Line line, LineState state) {}

 private:
  byte_t vram_read(offset_t offset);
  void vram_write(offset_t offset, byte_t value);
  void dma_write(offset_t offset, byte_t value);
  void palette_write(ColorPalette<4> *pal, bool bg, byte_t *pal_data, offset_t offset,
                     byte_t value);

  GfxObject<8, 8> *get_obj(int idx);

  typedef std::function<GfxObject<8, 8> *(int idx)> obj_cb;
  obj_cb m_tilemap0;
  obj_cb m_tilemap1;

  void draw_scanline(int sy);

  AddressBus16x8 *m_bus;

  RamDevice m_vram;
  RamDevice m_oam;

  byte_t m_lcdc;
  byte_t m_stat;
  byte_t m_scy;
  byte_t m_scx;
  byte_t m_ly;
  byte_t m_lyc;
  byte_t m_bgp;
  byte_t m_obp0;
  byte_t m_obp1;
  byte_t m_wy;
  byte_t m_wx;

  ColorPalette<4> m_global_pal, m_obj0_pal, m_obj1_pal, m_bg_pal;
  GfxObject<8, 8> m_objs[384];
};
};
