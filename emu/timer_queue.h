/*
 * Copyright (c) 2016, Dan Sledz
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
#include "emu/timing.h"

using namespace Core;

namespace EMU {

struct WorkItem {
  WorkItem(callback_t callback) : _callback(callback) {}

  void operator()(void) { _callback(); }

 private:
  callback_t _callback;
};

struct TimerItem : public WorkItem {
  TimerItem(Time timeout, callback_t callback, bool periodic = false)
      : WorkItem(callback),
        _timeout(timeout),
        _periodic(periodic),
        _deadline(time_zero) {}

  bool expired(Time time) { return _deadline <= time; }

  const Time deadline(void) { return _deadline; }

  void schedule(Time abs) { _deadline = abs + _timeout; }

  bool periodic(void) { return _periodic; }

 private:
  Time _timeout;
  bool _periodic;
  Time _deadline;
};

typedef std::shared_ptr<TimerItem> TimerItem_ptr;

class TimerQueue {
 public:
  TimerQueue(void);
  ~TimerQueue(void);

  void stop(void);
  Time run(Time delta);

  TimerItem_ptr add_periodic(Time period, callback_t callback);
  TimerItem_ptr add_timeout(Time timeout, callback_t callback);
  bool remove(TimerItem_ptr timer);

 private:
  void add(TimerItem_ptr timer);

  TimerItem_ptr pop(Time deadline);
  void wait(void);

  Time _clock;

  std::mutex mtx;
  std::condition_variable cv;

  std::list<TimerItem_ptr> _timers;

  bool _quit;
};
};
