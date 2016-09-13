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

#include "emu/video.h"

using namespace EMU;

FrameBuffer::FrameBuffer(short width, short height, GfxScale scale,
                         Rotation rot)
    : m_width(width),
      m_pitch(width * sizeof(RGBColor)),
      m_height(height),
      m_rot(rot),
      m_transform(get_transform(scale, m_width, m_height)),
      m_data(m_pitch * m_height),
      m_empty() {}

FrameBuffer::~FrameBuffer(void) {}

void FrameBuffer::set(int x, int y, RGBColor color) {
  int sy, sx;
  switch (m_rot) {
    case ROT0:
      sx = x;
      sy = y;
      break;
    case ROT90:
      sx = m_width - y - 1;
      sy = x;
      break;
    case ROT180:
      sx = x;
      sy = m_height - y - 1;
      break;
    case ROT270:
      sx = m_width - y - 1;
      sy = m_height - x - 1;
      break;
  }

  if ((sy >= 0 && sy < m_height) && (sx >= 0 && sx < m_width))
    m_data[sy * m_width + sx] = color.v;
}

const RGBColor FrameBuffer::get(int x, int y) const {
  int sy, sx;
  switch (m_rot) {
    case ROT0:
      sx = x;
      sy = y;
      break;
    case ROT90:
      sx = m_width - y - 1;
      sy = x;
      break;
    case ROT180:
      sx = x;
      sy = m_height - y - 1;
      break;
    case ROT270:
      sx = m_width - y - 1;
      sy = m_height - x - 1;
      break;
  }

  if ((sy >= 0 && sy < m_height) && (sx >= 0 && sx < m_width))
    return m_data[sy * m_width + sx];
  else
    return m_empty;
}

void FrameBuffer::clear(void) {
  std::fill(m_data.begin(), m_data.end(), m_empty);
}

class GfxTransformNone : public GfxTransform {
 public:
  GfxTransformNone(short width, short height) : GfxTransform(width, height) {}
  virtual ~GfxTransformNone(void) {}

  virtual void render(FrameBuffer *screen) {
    const byte_t *src = screen->fb();
    byte_t *dest = reinterpret_cast<byte_t *>(fb());
    const short dest_pitch = pitch();

    for (int y = 0; y < screen->height(); y++)
      memcpy(&dest[y * dest_pitch], &src[y * screen->pitch()], screen->pitch());
  }
};

class GfxTransformScanline2x : public GfxTransform {
 public:
  GfxTransformScanline2x(short width, short height)
      : GfxTransform(width * 2, height * 2) { }
  virtual ~GfxTransformScanline2x(void) {}

  virtual void render(FrameBuffer *screen) {
    const byte_t *src = screen->fb();
    byte_t *dest = reinterpret_cast<byte_t *>(fb());
    const short dest_pitch = pitch();

    for (int y = 0; y < screen->height(); y++) {
      unsigned *d0 = reinterpret_cast<unsigned *>(dest);
      dest += dest_pitch;
      unsigned *d1 = reinterpret_cast<unsigned *>(dest);
      dest += dest_pitch;
      const unsigned *s = reinterpret_cast<const unsigned *>(src);
      for (int x = 1; x < screen->width() - 1; x++) {
        d0[x * 2] = s[x];
        d0[x * 2 + 1] = s[x];
        d1[x * 2] = d1[x * 2 + 1] = 0xff000000;
      }
      src += screen->pitch();
    }
  }
};

class GfxTransform2x : public GfxTransform {
 public:
  GfxTransform2x(short width, short height)
      : GfxTransform(width * 2, height * 2) {}
  virtual ~GfxTransform2x(void) {}

  virtual void render(FrameBuffer *screen) {
    const byte_t *src = screen->fb();
    byte_t *dest = reinterpret_cast<byte_t *>(fb());
    const short dest_pitch = pitch();

    for (int y = 0; y < screen->height(); y++) {
      unsigned *d0 = reinterpret_cast<unsigned *>(dest);
      dest += dest_pitch;
      unsigned *d1 = reinterpret_cast<unsigned *>(dest);
      dest += dest_pitch;
      const unsigned *s = reinterpret_cast<const unsigned *>(src);
      for (int x = 1; x < screen->width() - 1; x++) {
        d0[x * 2] = s[x];
        d0[x * 2 + 1] = s[x];
        d1[x * 2] = s[x];
        d1[x * 2 + 1] = s[x];
      }
      src += screen->pitch();
    }
  }
};

class GfxTransformScale2x : public GfxTransform {
 public:
  GfxTransformScale2x(short width, short height)
      : GfxTransform(width * 2, height * 2) {}
  virtual ~GfxTransformScale2x(void) {}

  virtual void render(FrameBuffer *screen) {
    const byte_t *src = screen->fb();
    byte_t *dest = reinterpret_cast<byte_t *>(fb());
    const short dest_pitch = pitch();

    const unsigned *sl = reinterpret_cast<const unsigned *>(src);
    const unsigned *s, *sh;

    for (int y = 0; y < screen->height(); y++) {
      int x = 0;
      unsigned *d0 = reinterpret_cast<unsigned *>(dest);
      dest += dest_pitch;
      unsigned *d1 = reinterpret_cast<unsigned *>(dest);
      dest += dest_pitch;
      s = reinterpret_cast<const unsigned *>(src);
      if (y < screen->height() - 1)
        sh = reinterpret_cast<const unsigned *>(src + screen->pitch());
      d0[x * 2] = s[x];
      d0[x * 2 + 1] = s[x];
      d1[x * 2] = s[x];
      d1[x * 2 + 1] = s[x];
      for (x = 1; x < screen->width() - 1; x++) {
        d0[x * 2] = s[x];
        d0[x * 2 + 1] = s[x];
        d1[x * 2] = s[x];
        d1[x * 2 + 1] = s[x];
        if (s[x - 1] == sl[x] && s[x - 1] != sh[x] && sl[x] != s[x + 1])
          d0[x * 2] = sl[x];
        if (sl[x] == s[x + 1] && sl[x] != s[x - 1] && s[x + 1] != sh[x])
          d0[x * 2 + 1] = s[x + 1];
        if (sh[x] == s[x - 1] && s[x + 1] != sl[x] && s[x - 1] != sl[x])
          d1[x * 2] = s[x - 1];
        if (s[x + 1] == sh[x] && s[x + 1] != sl[x] && sh[x] != s[x - 1])
          d1[x * 2 + 1] = sh[x];
      }
      d0[x * 2] = s[x];
      d0[x * 2 + 1] = s[x];
      d1[x * 2] = s[x];
      d1[x * 2 + 1] = s[x];
      sl = s;
      src += screen->pitch();
    }
  }
};

GfxTransform_ptr EMU::get_transform(GfxScale scale, short width, short height) {
  GfxTransform_ptr transform;

  switch (scale) {
    case GfxScale::None:
      transform = GfxTransform_ptr(new GfxTransformNone(width, height));
      break;
    case GfxScale::Scaneline2x:
      transform = GfxTransform_ptr(new GfxTransformScanline2x(width, height));
      break;
    case GfxScale::Nearest2x:
      transform = GfxTransform_ptr(new GfxTransform2x(width, height));
      break;
    case GfxScale::Scale2x:
      transform = GfxTransform_ptr(new GfxTransformScale2x(width, height));
      break;
  }

  return transform;
}
