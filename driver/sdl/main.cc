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

#include "emu/emu.h"

#include "driver/emulator.h"
#include "driver/opengl.h"

#include <getopt.h>

#include <SDL.h>

using namespace EMU;

class SDLException: public EMU::EmuException {
public:
    SDLException(): EmuException("SDL exception") { }
};

struct SDLKeyHash
{
    size_t operator ()(const SDLKey &key) const {
        return std::hash<int>()(static_cast<int>(key));
    }
};

class SDLInput {
public:
    SDLInput(void);
    ~SDLInput(void);

    /**
     * Process an sdl event.
     */
    void handle_input(SDL_Event *e, InputMap *dev);

private:
    std::unordered_map<SDLKey, InputKey, SDLKeyHash> _map;
};

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


class CLIOptions: public Options
{
public:
    CLIOptions(int argc, char **argv):
        Options()
    {
        static struct option opts[] = {
            {"log",    required_argument, 0, 'l'},
            {"driver", required_argument, 0, 'd'},
            {"rom",    required_argument, 0, 'r'},
            {0, 0, 0, 0}
        };
        int idx = 0;
        char c;
        while ((c = getopt_long(argc, argv, "l:d:r:", opts, &idx)) != -1) {
            switch (c) {
            case 'l':
                log_level = optarg;
                break;
            case 'd':
                driver = optarg;
                break;
            case 'r':
                rom = optarg;
                break;
            default:
                break;
            }
        }
    }
};

class SDLEmulator: public Emulator
{
public:
    SDLEmulator(const Options &options):
        Emulator(options)
    {
        SDL_Surface * window = SDL_SetVideoMode(100, 100, 32,
                SDL_HWSURFACE | SDL_OPENGL);
        if (window == NULL)
            throw SDLException();

        SDL_WM_SetCaption("Emulator", "Emulator");

        _screen = std::unique_ptr<GLRasterScreen>(new GLRasterScreen());
        machine()->set_screen(_screen.get());

        /* Resize our window to the correct size. */
        SDL_SetVideoMode(_screen->width(), _screen->height(), 32,
                         SDL_HWSURFACE | SDL_OPENGL);
        /* Reinit gl context */
        _screen->init();

        /* Connect our graphics */
        machine()->set_render([&](RasterScreen *screen) {
            screen->flip();
            screen->render();
            SDL_GL_SwapBuffers();
        });

        machine()->add_timer(Time(msec(1)),
            [&]() {
                SDL_Event event;
                while (SDL_PollEvent(&event))
                    on_event(&event);
            },
            Time(msec(1)));

        SDL_Event event;
        while (SDL_PollEvent(&event))
            on_event(&event);
    }

    virtual void start(void) {
        set_state(EmuState::Running);
        do_execute();
    }

    void on_event(SDL_Event *event) {
        switch (event->type) {
        case SDL_QUIT:
            stop();
            break;
        case SDL_KEYUP:
        case SDL_KEYDOWN:
            switch (event->key.keysym.sym) {
            case SDLK_q:
                stop();
                break;
            case SDLK_F1:
                if (event->type == SDL_KEYDOWN)
                    EMU::log.set_level(EMU::LogLevel::Trace);
                break;
            default:
                input.handle_input(event, machine()->input());
            }
            break;
        default:
            break;
        }
    }

private:

    std::unique_ptr<GLRasterScreen> _screen;
    SDLInput input;
};

extern "C" int main(int argc, char **argv)
{
    try {
        if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
            throw SDLException();

        CLIOptions opts(argc, argv);

        SDLEmulator emu(opts);

        emu.start();
    } catch (EmuException &e) {
        std::cout << "Exception: " << e.message() << std::endl;
    }

    SDL_Quit();
    return 0;
}

