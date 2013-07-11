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

SDLGfx::SDLGfx(void)
{
    _window = surface_ptr(SDL_SetVideoMode(
        640, 480, 32, SDL_HWSURFACE | SDL_DOUBLEBUF));
    if (_window == NULL)
        throw SDLException();
}

SDLGfx::~SDLGfx(void)
{
}

void
SDLGfx::render(RasterScreen *screen)
{
    /* Convert the screen into an SDL surface */
    surface_ptr surface = surface_ptr(SDL_CreateRGBSurface(SDL_SWSURFACE,
        screen->width, screen->height, 32, 0x000000ff,
        0x0000ff00, 0x00ff0000, 0x00000000));
    SDL_LockSurface(surface.get());
    byte_t *dest = reinterpret_cast<byte_t *>(surface->pixels);
    byte_t *src = reinterpret_cast<byte_t *>(screen->data.data());

    for (unsigned y = 0; y < screen->height; y++)
        memcpy(&dest[y * surface->pitch],
               &src[y * screen->pitch],
               screen->pitch);
    SDL_UnlockSurface(surface.get());

    SDL_Rect rect;
    rect.x = 64;
    rect.y = 64;

    SDL_BlitSurface(surface.get(), NULL, _window.get(), &rect);

    SDL_Flip(_window.get());
}
