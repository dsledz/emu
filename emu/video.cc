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

FrameBuffer::FrameBuffer(Rotation rot) : m_rot(rot) { resize(0, 0); }

FrameBuffer::FrameBuffer(short width, short height, Rotation rot) : m_rot(rot) {
  resize(width, height);
}

FrameBuffer::~FrameBuffer(void) {}

void FrameBuffer::set_rotation(Rotation rot) { m_rot = rot; }

void FrameBuffer::resize(short width, short height) {
  do_resize(width, height);
}

void FrameBuffer::render(void) {}

void FrameBuffer::flip(void) {}

void FrameBuffer::do_resize(short width, short height) {
  m_width = width;
  m_height = height;
  m_pitch = width * sizeof(uint32_t);
  m_data.resize(m_width * m_height);
}

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
