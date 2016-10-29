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
#pragma once

#include "emu/emu.h"

namespace EMU {

class Emulator {
 public:
  enum class EmuState {
    Running,
    Paused,
    Stopped,
  };

  Emulator(const Options &options);
  virtual ~Emulator(void);

  virtual void start(void) = 0;
  virtual void stop(void);
  virtual void pause(void);
  virtual void reset(void);
  virtual void resume(void);
  virtual void render(void);
  virtual void key_event(InputKey key, bool pressed);

  Machine *machine(void);
  const Options *options(void);
  FrameBuffer *fb(void);

  std::future<void> frame_start(void) {
    m_frame_start = m_rtc.now();
    return m_machine->advance(m_frame_period);
  }

  Time frame_left(void) {
    const Time now = m_rtc.now();
    Time spent = now - m_frame_start;
    if (spent > m_frame_period)
      return Time(sec(0));
    else
      return m_frame_period - spent;
  }

  void frame_end(std::future<void> &future) {
    assert(future.valid());
    future.get();
  }

  void do_execute(void);

 protected:
  EmuState get_state(void);
  void set_state(EmuState state);

  RealTimeClock m_rtc;
  Time m_frame_period;
  Time m_frame_start;

 private:
  std::mutex mtx;
  std::condition_variable cv;

  TaskScheduler m_scheduler;
  Emulator::EmuState m_state;
  Options m_options;
  machine_ptr m_machine;
};
};
