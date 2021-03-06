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

#include <SFML/OpenGL.hpp>
#include <SFML/Window.hpp>

using namespace EMU;
using namespace Core;

class SFMLException : public Core::CoreException {
 public:
  SFMLException() : CoreException("SFML exception") {}
};

class SFMLEmulator : public Emulator {
 public:
  SFMLEmulator(const Options &options)
      : Emulator(options), m_window(nullptr), m_task() {
    sf::VideoMode vm(machine()->fb()->width() * 2,
                     machine()->fb()->height() * 2);

    m_window = std::unique_ptr<sf::Window>(new sf::Window(
        vm, "OpenGL", sf::Style::Default, sf::ContextSettings(32)));
    m_window->setVerticalSyncEnabled(true);
    poll_events();
  }

  virtual ~SFMLEmulator() { m_task.get(); }

  virtual void start(void) {
    machine()->load_rom(options()->rom);

    m_fb = std::unique_ptr<GLRender>(new GLRender(machine()->screen()));
    m_fb->init();

    machine()->reset();

    set_state(EmuState::Running);

    machine()->poweron();

    while (get_state() != EmuState::Stopped) {
      while (get_state() == EmuState::Paused) {
        poll_events();
      }
      auto future = frame_start();
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      m_fb->render();
      m_window->display();
      poll_events();
      frame_end(future);
    }
    machine()->poweroff();
  }

  virtual void stop(void) {
    Emulator::stop();
  }

 private:
  void poll_events(void) {
    sf::Event event;
    while (m_window->pollEvent(event)) on_event(event);
  }

  void on_event(const sf::Event &event) {
    if (event.type == sf::Event::Closed) {
      stop();
    } else if (event.type == sf::Event::Resized) {
      glViewport(0, 0, event.size.width, event.size.height);
    } else if (event.type == sf::Event::KeyPressed) {
      switch (event.key.code) {
        case sf::Keyboard::Key::Q:
          stop();
          break;
        default: { /* XXX: Find key */ }
      }
    }
  }

  std::unique_ptr<sf::Window> m_window;
  std::unique_ptr<GLRender> m_fb;

  std::future<void> m_task;
};

extern "C" int main(int argc, char **argv) {
  try {
    Driver::CLIOptions opts(argc, argv);

    SFMLEmulator emu(opts);

    emu.start();

  } catch (CoreException &e) {
    std::cout << "Exception: " << e.message() << std::endl;
  }

  return 0;
}
