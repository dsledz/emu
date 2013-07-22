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

#include "emu.h"
#include "galaga.h"
#include "gb.h"

#include "sdl_util.h"
#include "sdl_gfx.h"
#include "sdl_input.h"
#include "sdl_main.h"

#include "getopt.h"

class SDLMain {
public:
    SDLMain(void): stop(false) {
    }
    ~SDLMain(void) {
    }

    void load(Options *opts) {
        machine = loader.start(opts);

        gfx.init(machine->screen());

        /* Connect our graphics */
        machine->set_render([&](RasterScreen *screen) {
            gfx.render(screen);
        });

        machine->add_timer(Time(msec(1)),
            [&]() {
                SDL_Event event;
                while (SDL_PollEvent(&event))
                    on_event(&event);
            },
            Time(msec(1)));
    }

    void loop(void) {
        SDL_Event event;
        while (SDL_PollEvent(&event))
            on_event(&event);

        while (!stop) {
            machine->run();
        }
    }

    void on_event(SDL_Event *event) {
        switch (event->type) {
        case SDL_QUIT:
            stop = true;
            break;
        case SDL_KEYUP:
        case SDL_KEYDOWN:
            switch (event->key.keysym.sym) {
            case SDLK_q:
                stop = true;
                break;
            case SDLK_F1:
                if (event->type == SDL_KEYDOWN)
                    EMU::log.set_level(EMU::LogLevel::Trace);
                break;
            default:
                input.handle_input(event, machine->input());
            }
            break;
        default:
            break;
        }
    }

private:

    machine_ptr machine;
    SDLGfx   gfx;
    SDLInput input;
    bool     stop;
};

extern "C" int main(int argc, char **argv)
{
    if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
        throw SDLException();

    Options opts("galaga");
    opts.parse(argc, argv);

    {
        SDLMain main;

        main.load(&opts);
        main.loop();
    }

    SDL_Quit();
    return 0;
}

FORCE_UNDEFINED_SYMBOL(galaga);
FORCE_UNDEFINED_SYMBOL(gb);
FORCE_UNDEFINED_SYMBOL(nes);
