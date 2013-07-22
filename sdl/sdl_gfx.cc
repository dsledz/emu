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
    _scale(GfxScale::Scaneline2x)
{
}

SDLGfx::~SDLGfx(void)
{
}

void
SDLGfx::init(RasterScreen *screen)
{
    _width = screen->width;
    _height = screen->height;

    switch (_scale) {
    case GfxScale::None:
        break;
    case GfxScale::Nearest2x:
    case GfxScale::Scale2x:
    case GfxScale::Scaneline2x:
        _width *= 2;
        _height *= 2;
        break;
    }

    _window = surface_ptr(SDL_SetVideoMode(_width, _height, 32,
        SDL_HWSURFACE | SDL_OPENGL));
    if (_window == NULL)
        throw SDLException();

    glClearColor(0.0,0.0,0.0,0.0);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, _width, _height, 1.0, -1.0, 1.0);
    glEnable(GL_BLEND);
    glEnable(GL_TEXTURE_2D);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    glGenTextures(1,&_frame);
    glBindTexture(GL_TEXTURE_2D, _frame);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

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
SDLGfx::render_2x(RasterScreen *screen, SDL_Surface *surface)
{
    byte_t *dest = reinterpret_cast<byte_t *>(surface->pixels);
    byte_t *src = reinterpret_cast<byte_t *>(screen->data.data());

    for (int y = 0; y < screen->height; y++) {
        unsigned *d0 = reinterpret_cast<unsigned *>(dest);
        dest += surface->pitch;
        unsigned *d1 = reinterpret_cast<unsigned *>(dest);
        dest += surface->pitch;
        unsigned * s = reinterpret_cast<unsigned *>(src);
        for (int x = 1; x < screen->width - 1; x++) {
            d0[x*2] = s[x];
            d0[x*2+1] = s[x];
            d1[x*2] = s[x];
            d1[x*2+1] = s[x];
        }
        src += screen->pitch;
    }
}

void
SDLGfx::render_scanline2x(RasterScreen *screen, SDL_Surface *surface)
{
    byte_t *dest = reinterpret_cast<byte_t *>(surface->pixels);
    byte_t *src = reinterpret_cast<byte_t *>(screen->data.data());

    for (int y = 0; y < screen->height; y++) {
        unsigned *d0 = reinterpret_cast<unsigned *>(dest);
        dest += surface->pitch;
        unsigned *d1 = reinterpret_cast<unsigned *>(dest);
        dest += surface->pitch;
        unsigned * s = reinterpret_cast<unsigned *>(src);
        for (int x = 1; x < screen->width - 1; x++) {
            d0[x*2] = s[x];
            d0[x*2+1] = s[x];
            d1[x*2] = d1[x*2+1] = 0xff000000;
        }
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
    surface_ptr surface = surface_ptr(SDL_CreateRGBSurface(
        SDL_SWSURFACE, _width, _height, 32,
        0x000000ff, 0x0000ff00, 0x00ff0000, 0xff000000));
    SDL_LockSurface(surface.get());

    switch (_scale) {
    case GfxScale::None:
        render_none(screen, surface.get());
        break;
    case GfxScale::Scale2x:
        render_scale2x(screen, surface.get());
        break;
    case GfxScale::Nearest2x:
        render_2x(screen, surface.get());
        break;
    case GfxScale::Scaneline2x:
        render_scanline2x(screen, surface.get());
        break;
    }

    glBindTexture(GL_TEXTURE_2D,_frame);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, surface->w, surface->h,
                 0, GL_RGBA, GL_UNSIGNED_BYTE, surface->pixels);

    SDL_UnlockSurface(surface.get());

    glClear(GL_COLOR_BUFFER_BIT);
    glBindTexture(GL_TEXTURE_2D, _frame);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex2f(0, 0);
    glTexCoord2f(1, 0);
    glVertex2f(surface->w - 1, 0);
    glTexCoord2f(1, 1);
    glVertex2f(surface->w - 1, surface->h - 1);
    glTexCoord2f(0, 1);
    glVertex2f(0, surface->h - 1);
    glEnd();
    glFlush();

    SDL_GL_SwapBuffers();
}

