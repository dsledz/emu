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

FrameBuffer::FrameBuffer(Rotation rot):
    _rot(rot)
{
    resize(0, 0);
}

FrameBuffer::FrameBuffer(short width, short height, Rotation rot):
    _rot(rot)
{
    resize(width, height);
}

FrameBuffer::~FrameBuffer(void)
{

}

void
FrameBuffer::set_rotation(Rotation rot)
{
    _rot = rot;
}

void
FrameBuffer::resize(short width, short height)
{
    do_resize(width, height);
}

void
FrameBuffer::render(void)
{
}

void
FrameBuffer::flip(void)
{
}

void
FrameBuffer::do_resize(short width, short height)
{
    _width = width;
    _height = height;
    _pitch = width * sizeof(uint32_t);
    _data.resize(_width * _height);
}

void
FrameBuffer::set(int x, int y, RGBColor color)
{
    int sy, sx;
    switch (_rot) {
    case ROT0: sx = x; sy = y; break;
    case ROT90: sx = _width - y - 1; sy = x; break;
    case ROT180: sx = x; sy = _height - y -1; break;
    case ROT270: sx = _width - y - 1; sy = _height - x -1; break;
    }

    if ((sy >= 0 && sy < _height) && (sx >= 0 && sx < _width))
        _data[sy * _width + sx] = color.v;
}

const RGBColor
FrameBuffer::get(int x, int y) const
{
    int sy, sx;
    switch (_rot) {
    case ROT0: sx = x; sy = y; break;
    case ROT90: sx = _width - y - 1; sy = x; break;
    case ROT180: sx = x; sy = _height - y -1; break;
    case ROT270: sx = _width - y - 1; sy = _height - x -1; break;
    }

    if ((sy >= 0 && sy < _height) && (sx >= 0 && sx < _width))
        return _data[sy * _width +sx];
    else
        return _empty;
}

void
FrameBuffer::clear(void)
{
    std::fill(_data.begin(), _data.end(), _empty);
}
