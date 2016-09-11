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

#include "core/bits.h"
#include "emu/device.h"

namespace EMU {

/**
 * Defines an absolute color in the RGB space
 */
struct RGBColor {
  RGBColor(void) : v(0) {}
  ~RGBColor(void) = default;
  RGBColor(const RGBColor &rhs) : v(rhs.v) {}
  RGBColor(unsigned r, unsigned g, unsigned b, unsigned a = 0xff)
      : r(r), g(g), b(b), a(a) {}
  RGBColor(uint32_t c) : v(c) {}

  inline bool operator!=(const RGBColor &rhs) const { return v != rhs.v; };
  inline bool operator==(const RGBColor &rhs) const { return v == rhs.v; };
  inline RGBColor &operator=(const RGBColor &rhs) {
    this->v = rhs.v;
    return *this;
  }
  union {
    struct {
      uint8_t r;
      uint8_t g;
      uint8_t b;
      uint8_t a;
    };
    uint32_t v;
  };
};

static inline RGBColor operator*(const RGBColor &rhs, unsigned i) {
  RGBColor res(rhs);
  res.r *= i;
  res.g *= i;
  res.b *= i;
  return res;
}

#define RGB_3B(a, b, c) (0x21 * c + 0x47 * b + 0x97 * a)
#define RGB_4B(a, b, c, d) (0x14 * d + 0x24 * c + 0x43 * b + 0x83 * a)

static const RGBColor trans(0x00, 0x00, 0x00, 0x00);

template <int width>
struct ColorPalette {
  inline RGBColor &operator[](unsigned i) { return m_colors[i]; }
  static const unsigned size = width;

 private:
  std::array<RGBColor, width> m_colors;
};

/* XXX: GfxObject should be based on a palette */
template <int width, int height, typename entry_t = byte_t>
struct GfxObject {
  GfxObject(void) : dirty(true) {}
  const int w = width;
  const int h = height;
  bool dirty;
  inline entry_t at(int x, int y) { return data[y * w + x]; }
  std::array<entry_t, width * height> data;
};

/**
 * Emulated Screen Abstraction.
 *
 * The FrameBuffer provides a generic interface to describe a screen.  For most
 * machines, this exists outside the emulated hardware and instead stands in
 * for video DAC (digital to analog converter).
 */
class FrameBuffer {
 public:
  enum Rotation { ROT0 = 0, ROT90 = 1, ROT180 = 2, ROT270 = 3 };
  FrameBuffer(Rotation rot = ROT0);

  FrameBuffer(short width, short height, Rotation rot = ROT0);
  virtual ~FrameBuffer(void);

  void set_rotation(Rotation rot);

  virtual void resize(short width, short height);
  virtual void render(void);
  virtual void flip(void);

  void set(int x, int y, RGBColor color);
  const RGBColor get(int x, int y) const;

  void clear(void);

  inline short width(void) const { return m_width; }

  inline short height(void) const { return m_height; }

  inline short pitch(void) const { return m_pitch; }

  inline const byte_t *fb(void) const {
    return reinterpret_cast<const byte_t *>(m_data.data());
  }

 protected:
  void do_resize(short width, short height);

 private:
  short m_width;  /* Width of the screen */
  short m_pitch;  /* Line pitch */
  short m_height; /* Height of the screen */
  Rotation m_rot;

  std::vector<RGBColor> m_data;
  RGBColor m_empty;
};

template <class gfx, class palette>
static inline void draw_gfx(FrameBuffer *screen, palette *pal, gfx *obj, int sx,
                            int sy, bool flipx = false, bool flipy = false,
                            RGBColor transparent = trans) {
  if (flipx) {
    if (flipy) {
      for (int y = 0; y < obj->h; y++) {
        for (int x = 0; x < obj->w; x++) {
          RGBColor pen = (*pal)[obj->at(obj->w - x - 1, obj->h - y - 1)];
          if (pen != transparent) screen->set(sx + x, sy + y, pen);
        }
      }
    } else {
      for (int y = 0; y < obj->h; y++) {
        for (int x = 0; x < obj->w; x++) {
          RGBColor pen = (*pal)[obj->at(obj->w - x - 1, y)];
          if (pen != transparent) screen->set(sx + x, sy + y, pen);
        }
      }
    }
  } else {
    if (flipy) {
      for (int y = 0; y < obj->h; y++) {
        for (int x = 0; x < obj->w; x++) {
          RGBColor pen = (*pal)[obj->at(x, obj->h - y - 1)];
          if (pen != transparent) screen->set(sx + x, sy + y, pen);
        }
      }
    } else {
      for (int y = 0; y < obj->h; y++) {
        for (int x = 0; x < obj->w; x++) {
          RGBColor pen = (*pal)[obj->at(x, y)];
          if (pen != transparent) screen->set(sx + x, sy + y, pen);
        }
      }
    }
  }
}

/* Draw the inverse */
template <class gfx, class palette>
static inline void draw_gfx_overlay(FrameBuffer *screen, palette *pal, gfx *obj,
                                    int sx, int sy, RGBColor target,
                                    bool flipx = false, bool flipy = false,
                                    RGBColor transparent = trans) {
  if (flipx) {
    if (flipy) {
      for (int y = 0; y < obj->h; y++) {
        for (int x = 0; x < obj->w; x++) {
          RGBColor pen = (*pal)[obj->at(obj->w - x - 1, obj->h - y - 1)];
          if (pen != transparent && screen->get(sx + x, sy + y) == target)
            screen->set(sx + x, sy + y, pen);
        }
      }
    } else {
      for (int y = 0; y < obj->h; y++) {
        for (int x = 0; x < obj->w; x++) {
          RGBColor pen = (*pal)[obj->at(obj->w - x - 1, y)];
          if (pen != transparent && screen->get(sx + x, sy + y) == target)
            screen->set(sx + x, sy + y, pen);
        }
      }
    }
  } else {
    if (flipy) {
      for (int y = 0; y < obj->h; y++) {
        for (int x = 0; x < obj->w; x++) {
          RGBColor pen = (*pal)[obj->at(x, obj->h - y - 1)];
          if (pen != transparent && screen->get(sx + x, sy + y) == target)
            screen->set(sx + x, sy + y, pen);
        }
      }
    } else {
      for (int y = 0; y < obj->h; y++) {
        for (int x = 0; x < obj->w; x++) {
          RGBColor pen = (*pal)[obj->at(x, y)];
          if (pen != transparent && screen->get(sx + x, sy + y) == target)
            screen->set(sx + x, sy + y, pen);
        }
      }
    }
  }
}
};
