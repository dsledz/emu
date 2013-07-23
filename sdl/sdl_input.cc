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

#include "input.h"
#include "sdl_input.h"

using namespace EMU;

SDLInput::SDLInput(void)
{
    _map[SDLK_LEFT]  = InputKey::Joy1Left;
    _map[SDLK_RIGHT] = InputKey::Joy1Right;
    _map[SDLK_UP]    = InputKey::Joy1Up;
    _map[SDLK_DOWN]  = InputKey::Joy1Down;
    _map[SDLK_SPACE] = InputKey::Joy1Btn1;
    _map[SDLK_x]     = InputKey::Joy1Btn2;
    _map[SDLK_1]     = InputKey::Start1;
    _map[SDLK_2]     = InputKey::Start2;
    _map[SDLK_5]     = InputKey::Coin1;
    _map[SDLK_6]     = InputKey::Coin2;
    _map[SDLK_7]     = InputKey::Service;
    _map[SDLK_s]     = InputKey::Select1;
}

SDLInput::~SDLInput(void)
{

}

void
SDLInput::handle_input(SDL_Event *event, InputMap *dev)
{
    switch (event->type) {
    case SDL_KEYUP: {
        auto it = _map.find(event->key.keysym.sym);
        if (it != _map.end())
            dev->release(it->second);
        break;
    }
    case SDL_KEYDOWN: {
        auto it = _map.find(event->key.keysym.sym);
        if (it != _map.end())
            dev->depress(it->second);
        break;
    }
    default:
        break;
    }
}
