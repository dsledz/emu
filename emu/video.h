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

#include "bits.h"
#include "device.h"

namespace EMU {

/**
 * Defines an absolute color in the RGB space
 */
struct RGBColor {
    RGBColor() = default;
    RGBColor(unsigned r, unsigned g, unsigned b): _r(r), _g(g), _b(b), _a(0) {
    };
    union {
        struct { uint8_t _a; uint8_t _b; uint8_t _g; uint8_t _r; };
        uint32_t v;
    };
};

template<int width>
struct ColorPalette {
    inline RGBColor &operator[](unsigned i) {
        return _colors[i];
    }

private:
    RGBColor _colors[width];
};

template<int width, int height, typename entry_t = byte_t>
struct GfxObject {
    int w = width;
    int h = height;
    entry_t at(int x, int y) {
        return data[y * w + x];
    }
    entry_t data[width * height];
};

/**
 * Emulated screen.
 */
class RasterScreen
{
public:
    RasterScreen(short width, short height);

    short width;   /* Width of the screen */
    short pitch;   /* Line pitch */
    short height;  /* Height of the screen */

    void set(int x, int y, RGBColor color);

    std::vector<uint32_t> data;
};

typedef std::function<void (RasterScreen *)> render_cb;

template<class gfx, class palette> static inline void
draw_gfx(RasterScreen *screen, palette *pal, int sx, int sy, gfx *obj)
{
    for (int y = 0; y < obj->h; y++) {
        for (int x = 0; x < obj->w; x++) {
            RGBColor pen = (*pal)[obj->at(x,y)];
            screen->set(sx + x, sy + y, pen);
        }
    }
}

};
