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
#include "driver/cli_opts.h"

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>

using namespace EMU;
using namespace Core;

class SDLException: public Core::CoreException {
public:
    SDLException(): CoreException("SDL exception") { }
};

struct SDL_KeycodeHash
{
    size_t operator ()(const SDL_Keycode &key) const {
        return std::hash<int>()(static_cast<int>(key));
    }
};

std::unordered_map<SDL_Keycode, InputKey, SDL_KeycodeHash> key_map = {
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

class SDLEmulator: public Emulator
{
public:
    SDLEmulator(const Options &options):
        Emulator(options)
    {
        SDL_GL_SetAttribute( SDL_GL_CONTEXT_MAJOR_VERSION, 2 );
        SDL_GL_SetAttribute( SDL_GL_CONTEXT_MINOR_VERSION, 1 );

        m_window = SDL_CreateWindow("Emulator",
                        SDL_WINDOWPOS_CENTERED,
                        SDL_WINDOWPOS_CENTERED,
                        machine()->get_screen_width() * 2,
                        machine()->get_screen_height() * 2,
                        SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
        if (m_window == NULL)
            throw SDLException();

        /* Reinit gl context */
        m_fbo = 0;

        m_glcontext = SDL_GL_CreateContext(m_window);
        glGetIntegerv(GL_FRAMEBUFFER_BINDING, &m_fbo);
        SDL_GL_MakeCurrent(m_window, m_glcontext);

        SDL_GL_SetSwapInterval( 1 );

        m_frame_buffer = std::unique_ptr<GLFrameBuffer>(new GLFrameBuffer());
        machine()->set_frame_buffer(m_frame_buffer.get());
        m_frame_buffer->init();

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

            glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
            m_frame_buffer->render();
            SDL_GL_SwapWindow(m_window);

            SDL_Event event;
            while (SDL_PollEvent(&event))
                on_event(&event);

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

    SDL_Window *m_window;
    SDL_GLContext m_glcontext;
    GLint m_fbo;
    std::unique_ptr<GLFrameBuffer> m_frame_buffer;

    std::future<void> task;
};

extern "C" int main(int argc, char **argv)
{
    try {
        if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
            throw SDLException();

        Driver::CLIOptions opts(argc, argv);

        SDLEmulator emu(opts);

        emu.start();
    } catch (CoreException &e) {
        std::cout << "Exception: " << e.message() << std::endl;
    }

    SDL_Quit();
    return 0;
}
