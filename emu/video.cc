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

#include "video.h"

using namespace EMU;

RasterScreen::RasterScreen(short width, short height, Rotation rot):
    width(width), height(height), rot(rot)
{
    data.resize(width*height);
    pitch = width * 4;
}

void
RasterScreen::set(int x, int y, RGBColor color)
{
    // X = 3 Y = 4 (w = 20, h = 10)
    // **********
    // **********
    // ******B***
    // **A*******
    // **********
    // **********
    // **********
    // **********
    // **********
    // A(2, 3) B(7, 2)
    // XXX: This is wrong.
    int sy, sx;
    switch (rot) {
    case ROT0: sx = x; sy = y; break;
    case ROT90: sx = width - y - 1; sy = x; break;
    case ROT180: sx = x; sy = height - y -1; break;
    case ROT270: sx = width - y - 1; sy = height - x -1; break;
    }

    if ((sy >= 0 && sy < height) && (sx >= 0 && sx < width))
        data[sy * width + sx] = color.v;
}

const RGBColor
RasterScreen::get(int x, int y) const
{
    int sy, sx;
    switch (rot) {
    case ROT0: sx = x; sy = y; break;
    case ROT90: sx = width - y - 1; sy = x; break;
    case ROT180: sx = x; sy = height - y -1; break;
    case ROT270: sx = width - y - 1; sy = height - x -1; break;
    }

    if ((sy >= 0 && sy < height) && (sx >= 0 && sx < width))
        return data[sy * width +sx];
    else
        return RGBColor(0, 0, 0);
}

RGBColor*
RasterScreen::at(int x, int y)
{
    int sy, sx;
    switch (rot) {
    case ROT0: sx = x; sy = y; break;
    case ROT90: sx = width - y - 1; sy = x; break;
    case ROT180: sx = x; sy = height - y -1; break;
    case ROT270: sx = width - y - 1; sy = height - x -1; break;
    }

    if ((sy >= 0 && sy < height) && (sx >= 0 && sx < width))
        return &data[sy * width + sx];
    else
        return &empty;
}
void
RasterScreen::clear(void)
{
    std::fill(data.begin(), data.end(), RGBColor(0x00,0x00,0x00));
}
