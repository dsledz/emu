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
using namespace Core;

class SDLException: public Core::CoreException {
public:
    SDLException(): CoreException("SDL exception") { }
};

struct SDLKeyHash
{
    size_t operator ()(const SDLKey &key) const {
        return std::hash<int>()(static_cast<int>(key));
    }
};

std::unordered_map<SDLKey, InputKey, SDLKeyHash> key_map = {
    std::make_tuple(SDLK_LEFT, InputKey::Joy1Left),
    std::make_tuple(SDLK_RIGHT, InputKey::Joy1Right),
    std::make_tuple(SDLK_UP, InputKey::Joy1Up),
    std::make_tuple(SDLK_DOWN, InputKey::Joy1Down),
    std::make_tuple(SDLK_SPACE, InputKey::Joy1Btn1),
    std::make_tuple(SDLK_x, InputKey::Joy1Btn2),
    std::make_tuple(SDLK_1, InputKey::Start1),
    std::make_tuple(SDLK_2, InputKey::Start2),
    std::make_tuple(SDLK_5, InputKey::Coin1),
    std::make_tuple(SDLK_6, InputKey::Coin2),
    std::make_tuple(SDLK_7, InputKey::Service),
    std::make_tuple(SDLK_s, InputKey::Select1),
};

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

        _screen = std::unique_ptr<GLFrameBuffer>(new GLFrameBuffer());
        machine()->set_screen(_screen.get());

        /* Resize our window to the correct size. */
        SDL_SetVideoMode(_screen->width() * 2, _screen->height() * 2, 32,
                         SDL_HWSURFACE | SDL_OPENGL);
        /* Reinit gl context */
        _screen->init();

        SDL_Event event;
        while (SDL_PollEvent(&event))
            on_event(&event);
    }

    virtual void start(void) {
        machine()->load_rom(options()->rom);
        machine()->reset();

        set_state(EmuState::Running);
        task = std::async(std::launch::async, &Emulator::do_execute, this);

        while (get_state() == EmuState::Running) {
            SDL_Event event;
            while (SDL_PollEvent(&event))
                on_event(&event);

            _screen->render();
            SDL_GL_SwapBuffers();
        }

    }

    virtual void stop(void) {
        Emulator::stop();
        task.get();
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
                    Core::log.set_level(Core::LogLevel::Trace);
                break;
            default: {
                auto it = key_map.find(event->key.keysym.sym);
                if (it != key_map.end())
                    machine()->send_input(
                        it->second, event->type == SDL_KEYDOWN);
            }
            }
            break;
        default:
            break;
        }
    }

private:

    std::unique_ptr<GLFrameBuffer> _screen;

    std::future<void> task;
};

extern "C" int main(int argc, char **argv)
{
    try {
        if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
            throw SDLException();

        CLIOptions opts(argc, argv);

        SDLEmulator emu(opts);

        emu.start();
    } catch (CoreException &e) {
        std::cout << "Exception: " << e.message() << std::endl;
    }

    SDL_Quit();
    return 0;
}

