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

using namespace EMU;

Machine::Machine(unsigned hertz):
    _hertz(hertz),
    _quantum(6000),
    _ic(0),
    _rtc(_hertz)
{
}

Machine::~Machine(void)
{
}

void
Machine::add_device(Device *dev)
{
    _devs.push_back(dev);
}

void
Machine::remove_device(Device *dev)
{
    _devs.remove(dev);
}

Timer_ptr
Machine::add_timer(Time delay, callback_t callback, Time period)
{
    Cycles deadline = delay.to_cycles(_hertz) + _ic;
    Timer_ptr timer(new Timer(deadline, callback, period));
    add_timer(timer);
    return timer;
}

void
Machine::add_timer(Timer_ptr timer)
{
    auto it = _timers.begin();
    while (it != _timers.end()) {
        if ((*it)->deadline > timer->deadline)
            break;
        it++;
    }
    _timers.insert(it, timer);
}

void
Machine::remove_timer(Timer_ptr timer)
{
    _timers.remove(timer);
}

void
Machine::run(void)
{
    Cycles end = _ic + (_hertz / _quantum);
    Cycles interval(100);
    while (_ic < end) {
        // Trigger any expired events
        while (!_timers.empty() && _timers.front()->deadline < _ic) {
            Timer_ptr t = _timers.front();
            _timers.pop_front();
            t->callback();
            if (t->period != time_zero) {
                /* XXX: This isn't exact */
                t->deadline = _ic + t->period.to_cycles(_hertz);
                add_timer(t);
            }
        }

        for_each(_devs.begin(), _devs.end(), [=](Device *dev) {
                 dev->tick(interval.v);
                 });
        _ic += interval;
    }
}

Device *
Machine::dev(const std::string &name)
{
    for (auto it = _devs.begin(); it != _devs.end(); it++) {
        if ((*it)->name() == name)
            return *it;
    }
    throw DeviceException(name);
}

RasterScreen *
Machine::screen(void)
{
    return _screen.get();
}

void
Machine::set_render(render_cb cb)
{
    _render_cb = cb;
}

