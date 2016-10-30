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

class SDLEmulator : public Emulator {
 public:
  SDLEmulator(const Options &options) : Emulator(options) {
	  key_map_init();

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

    glewExperimental=GL_TRUE;
    GLenum err = glewInit();
    if (GLEW_OK != err) throw EmulatorException("glewInit failure");

    SDL_GL_SetSwapInterval(1);

    SDL_Event event;
    while (SDL_PollEvent(&event)) on_event(&event);
  }

  virtual ~SDLEmulator() {
    if (m_task.valid())
      m_task.get();
  }

  void key_map_init(void) {
    m_key_map.insert(std::make_pair(SDLK_LEFT, InputKey::Joy1Left));
    m_key_map.insert(std::make_pair(SDLK_RIGHT, InputKey::Joy1Right));
    m_key_map.insert(std::make_pair(SDLK_UP, InputKey::Joy1Up));
    m_key_map.insert(std::make_pair(SDLK_DOWN, InputKey::Joy1Down));
    m_key_map.insert(std::make_pair(SDLK_SPACE, InputKey::Joy1Btn1));
    m_key_map.insert(std::make_pair(SDLK_x, InputKey::Joy1Btn2));
    m_key_map.insert(std::make_pair(SDLK_1, InputKey::Start1));
    m_key_map.insert(std::make_pair(SDLK_2, InputKey::Start2));
    m_key_map.insert(std::make_pair(SDLK_5, InputKey::Coin1));
    m_key_map.insert(std::make_pair(SDLK_6, InputKey::Coin2));
    m_key_map.insert(std::make_pair(SDLK_7, InputKey::Service));
    m_key_map.insert(std::make_pair(SDLK_s, InputKey::Select1));

    m_keyboard_map.insert(std::make_pair(SDLK_0, InputKey::Keyboard0));
    m_keyboard_map.insert(std::make_pair(SDLK_1, InputKey::Keyboard1));
    m_keyboard_map.insert(std::make_pair(SDLK_2, InputKey::Keyboard2));
    m_keyboard_map.insert(std::make_pair(SDLK_3, InputKey::Keyboard3));
    m_keyboard_map.insert(std::make_pair(SDLK_4, InputKey::Keyboard4));
    m_keyboard_map.insert(std::make_pair(SDLK_5, InputKey::Keyboard5));
    m_keyboard_map.insert(std::make_pair(SDLK_6, InputKey::Keyboard6));
    m_keyboard_map.insert(std::make_pair(SDLK_7, InputKey::Keyboard7));
    m_keyboard_map.insert(std::make_pair(SDLK_8, InputKey::Keyboard8));
    m_keyboard_map.insert(std::make_pair(SDLK_9, InputKey::Keyboard9));
    m_keyboard_map.insert(std::make_pair(SDLK_a, InputKey::KeyboardA));
    m_keyboard_map.insert(std::make_pair(SDLK_b, InputKey::KeyboardB));
    m_keyboard_map.insert(std::make_pair(SDLK_c, InputKey::KeyboardC));
    m_keyboard_map.insert(std::make_pair(SDLK_d, InputKey::KeyboardD));
    m_keyboard_map.insert(std::make_pair(SDLK_e, InputKey::KeyboardE));
    m_keyboard_map.insert(std::make_pair(SDLK_f, InputKey::KeyboardF));
    m_keyboard_map.insert(std::make_pair(SDLK_g, InputKey::KeyboardG));
    m_keyboard_map.insert(std::make_pair(SDLK_h, InputKey::KeyboardH));
    m_keyboard_map.insert(std::make_pair(SDLK_i, InputKey::KeyboardI));
    m_keyboard_map.insert(std::make_pair(SDLK_j, InputKey::KeyboardJ));
    m_keyboard_map.insert(std::make_pair(SDLK_k, InputKey::KeyboardK));
    m_keyboard_map.insert(std::make_pair(SDLK_l, InputKey::KeyboardL));
    m_keyboard_map.insert(std::make_pair(SDLK_m, InputKey::KeyboardM));
    m_keyboard_map.insert(std::make_pair(SDLK_n, InputKey::KeyboardN));
    m_keyboard_map.insert(std::make_pair(SDLK_o, InputKey::KeyboardO));
    m_keyboard_map.insert(std::make_pair(SDLK_p, InputKey::KeyboardP));
    m_keyboard_map.insert(std::make_pair(SDLK_q, InputKey::KeyboardQ));
    m_keyboard_map.insert(std::make_pair(SDLK_r, InputKey::KeyboardR));
    m_keyboard_map.insert(std::make_pair(SDLK_s, InputKey::KeyboardS));
    m_keyboard_map.insert(std::make_pair(SDLK_t, InputKey::KeyboardT));
    m_keyboard_map.insert(std::make_pair(SDLK_u, InputKey::KeyboardU));
    m_keyboard_map.insert(std::make_pair(SDLK_v, InputKey::KeyboardV));
    m_keyboard_map.insert(std::make_pair(SDLK_w, InputKey::KeyboardW));
    m_keyboard_map.insert(std::make_pair(SDLK_x, InputKey::KeyboardX));
    m_keyboard_map.insert(std::make_pair(SDLK_y, InputKey::KeyboardY));
    m_keyboard_map.insert(std::make_pair(SDLK_z, InputKey::KeyboardZ));
    m_keyboard_map.insert(
        std::make_pair(SDLK_RETURN, InputKey::KeyboardReturn));
    m_keyboard_map.insert(
        std::make_pair(SDLK_LSHIFT, InputKey::KeyboardLShift));
    m_keyboard_map.insert(
        std::make_pair(SDLK_RSHIFT, InputKey::KeyboardRShift));
    m_keyboard_map.insert(std::make_pair(SDLK_COMMA, InputKey::KeyboardComma));
    m_keyboard_map.insert(std::make_pair(SDLK_SLASH, InputKey::KeyboardSlash));
    m_keyboard_map.insert(std::make_pair(SDLK_STOP, InputKey::KeyboardStop));
    m_keyboard_map.insert(std::make_pair(SDLK_DOWN, InputKey::KeyboardDown));
    m_keyboard_map.insert(std::make_pair(SDLK_CARET, InputKey::KeyboardCaret));
    m_keyboard_map.insert(std::make_pair(SDLK_COLON, InputKey::KeyboardColon));
    m_keyboard_map.insert(
        std::make_pair(SDLK_SEMICOLON, InputKey::KeyboardSemicolon));
    m_keyboard_map.insert(std::make_pair(SDLK_AT, InputKey::KeyboardAt));
    m_keyboard_map.insert(std::make_pair(SDLK_PLUS, InputKey::KeyboardPlus));
    m_keyboard_map.insert(
        std::make_pair(SDLK_PERIOD, InputKey::KeyboardPeriod));
    m_keyboard_map.insert(std::make_pair(SDLK_DOLLAR, InputKey::KeyboardPound));
  }

  virtual void start(void) {
    machine()->load_rom(options()->rom);

    m_fb = std::unique_ptr<GLRender>(new GLRender(machine()->screen()));
    m_fb->init();

    machine()->reset();

    set_state(EmuState::Running);

    machine()->poweron();

    while (get_state() != EmuState::Stopped) {
      while (get_state() == EmuState::Paused) {
        SDL_Event event;
        if (SDL_WaitEventTimeout(&event, 50))
          on_event(&event);
      }
      auto future = frame_start();
      glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
      m_fb->render();
      SDL_GL_SwapWindow(m_window);
      SDL_Event event;
      Time left = frame_left();
      if (left > Time(msec(11))) {
        if (SDL_WaitEventTimeout(&event, 10))
          on_event(&event);
      }
      while (SDL_PollEvent(&event)) on_event(&event);
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
          case SDLK_F12:
            if (event->type == SDL_KEYDOWN) stop();
            break;
          case SDLK_F1:
            if (event->type == SDL_KEYDOWN) Core::log.increase_level();
            break;
          case SDLK_F2:
            if (event->type == SDL_KEYDOWN) Core::log.decrease_level();
            break;
          case SDLK_F5:
            if (event->type == SDL_KEYDOWN) {
              if (get_state() == EmuState::Paused)
                resume();
              else
                pause();
            }
            break;
          case SDLK_F3:
            if (event->type == SDL_KEYDOWN) reset();
            break;
          default: {
            auto it = m_key_map.find(event->key.keysym.sym);
            if (it == m_key_map.end())
                it = m_keyboard_map.find(event->key.keysym.sym);
            if (it != m_key_map.end())
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
  std::unordered_map<SDL_Keycode, InputKey, SDL_KeycodeHash> m_key_map;
  std::unordered_map<SDL_Keycode, InputKey, SDL_KeycodeHash> m_keyboard_map;
};

int main(int argc, char *argv[]) {
  try {
    glutInit(&argc, argv);
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

#ifdef WIN32
#include <SDKDDKVer.h>
#include <tchar.h>
int wmain(int wargc, wchar_t *wargv[]) {
  int argc = 1;
  const char *argv0 = "sdlEmu";
  char *argv[] = { argv0 };
  return main(argc, argv);
}
#endif
