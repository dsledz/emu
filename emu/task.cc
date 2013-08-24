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

#include "emu/bits.h"
#include "emu/exception.h"
#include "emu/timing.h"

using namespace EMU;

Clockable::Clockable(const std::string &name):
    _name(name),
    _current_time(time_zero),
    _avail_time(time_zero)
{
}

Clockable::~Clockable(void)
{
    do_set_state(State::Stopped);
    if (_task.valid())
        _task.get();
}

void
Clockable::power(void)
{
    _task = std::async(&Clockable::do_run, this);
}

void
Clockable::wait(void)
{
    std::unique_lock<std::mutex> lock(_mtx);
    while (_state == State::Waiting)
        _cv.wait(lock);
}

void
Clockable::wait_state(State state)
{
    std::unique_lock<std::mutex> lock(_mtx);
    while (_state != state)
        _cv.wait(lock);
}

void
Clockable::add_time(Time delta)
{
    lock_mtx lock(_mtx);
    _avail_time += delta;
    if (_state == State::Waiting) {
        _state = State::Running;
        _cv.notify_all();
    }
}

void
Clockable::do_set_state(State state)
{
    lock_mtx lock(_mtx);
    _state = state;
    _cv.notify_all();
}

Clockable::State
Clockable::do_get_state(void)
{
    lock_mtx lock(_mtx);
    return _state;
}

Clockable::State
Clockable::state(void)
{
    return do_get_state();
}

void
Clockable::do_run(void)
{
    do {
        {
            std::unique_lock<std::mutex> lock(_mtx);
            while (_state == State::Waiting)
                _cv.wait(lock);
            if (_state == State::Stopped)
                break;
        }
        try {
            execute();
        } catch (EmuException &e) {
            std::cout << "Exception: " << e.what() << std::endl;
            std::unique_lock<std::mutex> lock(_mtx);
            _state = State::Stopped;
        }
    } while (true);
}

Scheduler::Scheduler(void)
{
}

Scheduler::~Scheduler(void)
{
}

void
Scheduler::add(Clockable *dev)
{
}

TimerQueue::TimerQueue(void):
    _clock(time_zero),
    _quit(false)
{
}

TimerQueue::~TimerQueue(void)
{
}

void
TimerQueue::stop(void)
{
    lock_mtx lock(mtx);
    _quit = true;
    cv.notify_one();
}

Time
TimerQueue::run(Time delta)
{
    lock_mtx lock(mtx);
    Time deadline = _clock + delta;
    TimerItem_ptr item = NULL;
    while ((item = pop(deadline)) != NULL) {
        (*item.get())();
        _clock = item->deadline();
        if (item->periodic())
            add(item);
    }
    _clock = deadline;
    return _clock;
}

TimerItem_ptr
TimerQueue::add_periodic(Time period, callback_t callback)
{
    lock_mtx lock(mtx);
    TimerItem_ptr timer = TimerItem_ptr(new TimerItem(period, callback, true));
    add(timer);
    return timer;
}

TimerItem_ptr
TimerQueue::add_timeout(Time period, callback_t callback)
{
    lock_mtx lock(mtx);
    TimerItem_ptr timer = TimerItem_ptr(new TimerItem(period, callback));
    add(timer);
    return timer;
}

void
TimerQueue::add(TimerItem_ptr timer)
{
    _timers.remove(timer);
    timer->schedule(_clock);
    auto it = _timers.begin();
    while (it != _timers.end()) {
        if ((*it)->deadline() > timer->deadline())
            break;
        it++;
    }
    _timers.insert(it, timer);
    cv.notify_one();
}

bool
TimerQueue::remove(TimerItem_ptr timer)
{
    lock_mtx lock(mtx);
    _timers.remove(timer);
    cv.notify_one();
    return true;
}

TimerItem_ptr
TimerQueue::pop(Time deadline)
{
    TimerItem_ptr item = NULL;
    if (!_timers.empty() && _timers.front()->deadline() < deadline) {
        item = _timers.front();
        _timers.pop_front();
    }
    return item;
}

void
TimerQueue::wait(void)
{
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock);
}

