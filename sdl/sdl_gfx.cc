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

#include "sdl_gfx.h"

SDLGfx::SDLGfx(void):
    _scale(GfxScale::Scale2x)
{
}

SDLGfx::~SDLGfx(void)
{
}

void
SDLGfx::init(RasterScreen *screen)
{
    short width = screen->width;
    short height = screen->height;

    switch (_scale) {
    case GfxScale::None:
        break;
    case GfxScale::Scale2x:
        width *= 2;
        height *= 2;
        break;
    }

    _window = surface_ptr(SDL_SetVideoMode(width, height, 32,
        SDL_HWSURFACE | SDL_DOUBLEBUF));
    if (_window == NULL)
        throw SDLException();
}

void
SDLGfx::render_scale2x(RasterScreen *screen, SDL_Surface *surface)
{
    byte_t *dest = reinterpret_cast<byte_t *>(surface->pixels);
    byte_t *src = reinterpret_cast<byte_t *>(screen->data.data());

    unsigned *sl = reinterpret_cast<unsigned *>(src);
    unsigned *s, *sh;

    for (int y = 0; y < screen->height; y++) {
        int x = 0;
        unsigned *d0 = reinterpret_cast<unsigned *>(dest);
        dest += surface->pitch;
        unsigned *d1 = reinterpret_cast<unsigned *>(dest);
        dest += surface->pitch;
        s = reinterpret_cast<unsigned *>(src);
        if (y < screen->height - 1)
            sh = reinterpret_cast<unsigned *>(src + screen->pitch);
        d0[x*2] = s[x];
        d0[x*2+1] = s[x];
        d1[x*2] = s[x];
        d1[x*2+1] = s[x];
        for (x = 1; x < screen->width - 1; x++) {
            d0[x*2] = s[x];
            d0[x*2+1] = s[x];
            d1[x*2] = s[x];
            d1[x*2+1] = s[x];
            if (s[x-1] == sl[x] && s[x-1] != sh[x] && sl[x] != s[x+1])
                d0[x*2] = sl[x];
            if (sl[x] == s[x+1] && sl[x] != s[x-1] && s[x+1] != sh[x])
                d0[x*2+1] = s[x+1];
            if (sh[x] == s[x-1] && s[x+1] != sl[x] && s[x-1] != sl[x])
                d1[x*2] = s[x-1];
            if (s[x+1] == sh[x] && s[x+1] != sl[x] && sh[x] != s[x-1])
                d1[x*2+1] = sh[x];
        }
        d0[x*2] = s[x];
        d0[x*2+1] = s[x];
        d1[x*2] = s[x];
        d1[x*2+1] = s[x];
        sl = s;
        src += screen->pitch;
    }
}

void
SDLGfx::render_none(RasterScreen *screen, SDL_Surface *surface)
{
    byte_t *dest = reinterpret_cast<byte_t *>(surface->pixels);
    byte_t *src = reinterpret_cast<byte_t *>(screen->data.data());

    for (int y = 0; y < screen->height; y++)
        memcpy(&dest[y * surface->pitch],
               &src[y * screen->pitch],
               screen->pitch);
}

void
SDLGfx::render(RasterScreen *screen)
{
    /* Convert the screen into an SDL surface */
    surface_ptr surface = surface_ptr(SDL_CreateRGBSurface(SDL_SWSURFACE,
        screen->width * 2, screen->height * 2, 32, 0x000000ff,
        0x0000ff00, 0x00ff0000, 0x00000000));
    SDL_LockSurface(surface.get());

    switch (_scale) {
    case GfxScale::None:
        render_none(screen, surface.get());
        break;
    case GfxScale::Scale2x:
        render_scale2x(screen, surface.get());
        break;
    }
    SDL_UnlockSurface(surface.get());

    SDL_Rect rect;
    rect.x = 0;
    rect.y = 0;

    SDL_BlitSurface(surface.get(), NULL, _window.get(), &rect);

    SDL_Flip(_window.get());
}
