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

#include "driver/cli_opts.h"
#include "driver/emulator.h"
#include "driver/opengl.h"

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>

using namespace EMU;
using namespace Core;

class SDLException : public Core::CoreException {
 public:
  SDLException() : CoreException("SDL Exception: ") {
    msg += std::string(SDL_GetError());
  }

  std::string sdl_error;
};

struct SDL_KeycodeHash {
  size_t operator()(const SDL_Keycode &key) const {
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

std::unordered_map<SDL_Keycode, InputKey, SDL_KeycodeHash> keyboard_map = {
    std::make_tuple(SDLK_0, InputKey::Keyboard0),
    std::make_tuple(SDLK_1, InputKey::Keyboard1),
    std::make_tuple(SDLK_2, InputKey::Keyboard2),
    std::make_tuple(SDLK_3, InputKey::Keyboard3),
    std::make_tuple(SDLK_4, InputKey::Keyboard4),
    std::make_tuple(SDLK_5, InputKey::Keyboard5),
    std::make_tuple(SDLK_6, InputKey::Keyboard6),
    std::make_tuple(SDLK_7, InputKey::Keyboard7),
    std::make_tuple(SDLK_8, InputKey::Keyboard8),
    std::make_tuple(SDLK_9, InputKey::Keyboard9),
    std::make_tuple(SDLK_a, InputKey::KeyboardA),
    std::make_tuple(SDLK_b, InputKey::KeyboardB),
    std::make_tuple(SDLK_c, InputKey::KeyboardC),
    std::make_tuple(SDLK_d, InputKey::KeyboardD),
    std::make_tuple(SDLK_e, InputKey::KeyboardE),
    std::make_tuple(SDLK_f, InputKey::KeyboardF),
    std::make_tuple(SDLK_g, InputKey::KeyboardG),
    std::make_tuple(SDLK_h, InputKey::KeyboardH),
    std::make_tuple(SDLK_i, InputKey::KeyboardI),
    std::make_tuple(SDLK_j, InputKey::KeyboardJ),
    std::make_tuple(SDLK_k, InputKey::KeyboardK),
    std::make_tuple(SDLK_l, InputKey::KeyboardL),
    std::make_tuple(SDLK_m, InputKey::KeyboardM),
    std::make_tuple(SDLK_n, InputKey::KeyboardN),
    std::make_tuple(SDLK_o, InputKey::KeyboardO),
    std::make_tuple(SDLK_p, InputKey::KeyboardP),
    std::make_tuple(SDLK_q, InputKey::KeyboardQ),
    std::make_tuple(SDLK_r, InputKey::KeyboardR),
    std::make_tuple(SDLK_s, InputKey::KeyboardS),
    std::make_tuple(SDLK_t, InputKey::KeyboardT),
    std::make_tuple(SDLK_u, InputKey::KeyboardU),
    std::make_tuple(SDLK_v, InputKey::KeyboardV),
    std::make_tuple(SDLK_w, InputKey::KeyboardW),
    std::make_tuple(SDLK_x, InputKey::KeyboardX),
    std::make_tuple(SDLK_y, InputKey::KeyboardY),
    std::make_tuple(SDLK_z, InputKey::KeyboardZ),
};

class SDLEmulator : public Emulator {
 public:
  SDLEmulator(const Options &options) : Emulator(options) {
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);

    m_window = SDL_CreateWindow(
        "Emulator", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        machine()->fb()->width() * 2, machine()->fb()->height() * 2,
        SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    if (m_window == NULL) throw SDLException();

    /* Reinit gl context */
    m_fbo = 0;

    m_glcontext = SDL_GL_CreateContext(m_window);
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &m_fbo);
    SDL_GL_MakeCurrent(m_window, m_glcontext);

    SDL_GL_SetSwapInterval(1);

    SDL_Event event;
    while (SDL_PollEvent(&event)) on_event(&event);
  }

  virtual ~SDLEmulator() {
    if (m_task.valid())
      m_task.get();
  }

  virtual void start(void) {
    machine()->load_rom(options()->rom);

    m_fb = std::unique_ptr<GLRender>(new GLRender(machine()->screen()));
    m_fb->init();

    machine()->reset();

    set_state(EmuState::Running);

    machine()->poweron();

    while (get_state() == EmuState::Running) {
      auto future = frame_start();
      glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
      m_fb->render();
      SDL_GL_SwapWindow(m_window);
      Time left;
#if WAIT
      while ((left = frame_left()) > time_zero) {
        SDL_Event event;
        if (SDL_WaitEventTimeout(&event, 1))
          on_event(&event);
      }
#else
      SDL_Event event;
      while (SDL_PollEvent(&event)) on_event(&event);
#endif
      frame_end(future);
    }
    machine()->poweroff();
  }

  virtual void stop(void) {
    Emulator::stop();
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
            if (it == key_map.end())
                it = keyboard_map.find(event->key.keysym.sym);
            if (it != key_map.end())
              machine()->send_input(it->second, event->type == SDL_KEYDOWN);
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
  std::unique_ptr<GLRender> m_fb;

  std::future<void> m_task;
};

extern "C" int main(int argc, char **argv) {
  try {
    glutInit(&argc, argv);
    glutCreateWindow("GLEW Test");
    GLenum err = glewInit();
    if (GLEW_OK != err) abort();
    if (SDL_Init(SDL_INIT_EVERYTHING) < 0) throw SDLException();

    Driver::CLIOptions opts(argc, argv);

    SDLEmulator emu(opts);

    emu.start();
  } catch (CoreException &e) {
    std::cout << "Exception: " << e.message() << std::endl;
  }

  SDL_Quit();
  return 0;
}
