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

#include "core/bits.h"
#include "core/exception.h"

#include "emu/timing.h"
#include "emu/timer_queue.h"

using namespace EMU;

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
    lock_mtx lock(mtx);
    lock.wait(cv);
}

