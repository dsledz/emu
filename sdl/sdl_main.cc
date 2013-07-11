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

#include "sdl_util.h"
#include "sdl_gfx.h"
#include "sdl_input.h"
#include "sdl_main.h"

#include "getopt.h"

class SDLMain {
public:
    SDLMain(void) {
    }
    ~SDLMain(void) {
    }

    void load(void) {
        machine = std::move(machine_ptr(new Driver::Galaga()));

        /* Connect our graphics */
        machine->set_render([&](RasterScreen *screen) {
            gfx.render(screen);
        });
    }

    void loop(void) {
        SDL_Event event;
        while (SDL_PollEvent(&event))
            on_event(&event);

        while (!stop) {
            while (SDL_PollEvent(&event))
                on_event(&event);

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

class Options {
public:
    Options(void) {
    }
    ~Options(void) {
    }

    void parse(int argc, char **argv) {
        static struct option opts[] = {
            {"log",   required_argument,  0, 'l'},
            {0, 0, 0, 0}
        };
        int idx = 0;
        char c;
        while ((c = getopt_long(argc, argv, "l:", opts, &idx)) != -1) {
            switch (c) {
            case 'l':
                EMU::log.set_level(optarg);
                break;
            default:
                break;
            }
        }
    }
private:
};

extern "C" int main(int argc, char **argv)
{
    if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
        throw SDLException();

    Options opts;
    opts.parse(argc, argv);

    {
        SDLMain main;

        main.load();
        main.loop();
    }

    SDL_Quit();
    return 0;
}


