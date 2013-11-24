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

#include <mach/mach.h>
#include <mach/mach_time.h>

namespace EMU {

struct Cycles {
    Cycles(void) = default;
    explicit Cycles(int64_t v): v(v) { }

    const struct Cycles operator +(const struct Cycles &rhs) const {
        Cycles res(*this);
        res += rhs;
        return res;
    }

    struct Cycles & operator +=(const struct Cycles &rhs) {
        v += rhs.v;
        return *this;
    }

    struct Cycles & operator +=(unsigned rhs) {
        v += rhs;
        return *this;
    }

    const struct Cycles operator -(const struct Cycles &rhs) const {
        return Cycles(v - rhs.v);
    }

    struct Cycles & operator -=(const struct Cycles &rhs) {
        v -= rhs.v;
        return *this;
    }

    const struct Cycles operator /(unsigned rhs) const {
        return Cycles(v / rhs);
    }
    const struct Cycles operator *(unsigned rhs) const {
        return Cycles(v * rhs);
    }

    bool operator >(int64_t rhs) const {
        return v > rhs;
    }
    bool operator >(const struct Cycles &rhs) const {
        return v > rhs.v;
    }
    bool operator <(const struct Cycles &rhs) const {
        return v < rhs.v;
    }
    bool operator ==(const struct Cycles &rhs) const {
        return v == rhs.v;
    }
    int64_t v;
};

struct nsec {
    explicit nsec(uint64_t v): v(v) { }
    uint64_t v;
};

struct usec {
    explicit usec(uint64_t v): v(v) { }
    uint64_t v;
};

struct msec {
    explicit msec(uint64_t v): v(v) { }
    uint64_t v;
};

struct sec {
    explicit sec(uint64_t v): v(v) { }
    uint64_t v;
};

struct Time {
    Time(void) = default;
    Time(sec s): ns(s.v * NSEC_PER_SEC) { }
    Time(msec ms): ns(ms.v * NSEC_PER_MSEC) { }
    Time(usec us): ns(us.v * NSEC_PER_USEC) { }
    Time(nsec ns): ns(ns.v) { }

    struct Cycles to_cycles(Cycles hertz) {
        return Cycles((ns * hertz.v) / NSEC_PER_SEC);
    }

    bool operator ==(const Time &rhs) const {
        return (ns == rhs.ns);
    }

    bool operator !=(const Time &rhs) const {
        return (ns != rhs.ns);
    }

    const Time operator /(unsigned rhs) const {
        return Time(nsec(ns / rhs));
    }

    Time & operator +=(const Time &rhs) {
        ns += rhs.ns;
        return *this;
    }

    Time & operator -=(const Time &rhs) {
        ns -= rhs.ns;
        return *this;
    }

    const Time operator -(const Time &rhs) const {
        Time res(*this);
        res -= rhs;
        return res;
    }

    const Time operator +(const Time &rhs) {
        Time res(*this);
        res += rhs;
        return res;
    }

    bool operator >(const Time &rhs) const {
        return ns > rhs.ns;
    }

    bool operator <(const Time &rhs) const {
        return ns < rhs.ns;
    }

    bool operator <=(const Time &rhs) const {
        return ns <= rhs.ns;
    }

    bool operator >=(const Time &rhs) const {
        return ns >= rhs.ns;
    }

    uint64_t ns;
};

extern const Time time_zero;

/**
 * Clock device. Regulates the running of a device based on real time.
 */
class RealTimeClock {
    public:
        RealTimeClock(void) {
            _clock = mach_absolute_time();
            mach_timebase_info_data_t timebase;
            mach_timebase_info(&timebase);
        }
        ~RealTimeClock(void) { }

        void reset(void) {
            _clock = mach_absolute_time();
        }

        /* Return the number of ticks since last call */
        Time get_delta(void) {
            uint64_t now = mach_absolute_time();
            uint64_t delta = (now - _clock);
            _clock = now;

            return Time(nsec(delta));
        }

        Time now(void) {
            return Time(nsec(mach_absolute_time()));
        }

    private:
        uint64_t _clock;
};

struct WorkItem {
    WorkItem(callback_t callback):
        _callback(callback) { }

    void operator()(void) {
        _callback();
    }

private:
    callback_t _callback;
};

struct TimerItem: public WorkItem {
    TimerItem(Time timeout, callback_t callback, bool periodic=false):
        WorkItem(callback), _timeout(timeout), _periodic(periodic),
        _deadline(time_zero) { }

    bool expired(Time time) {
        return _deadline <= time;
    }

    const Time deadline(void) {
        return _deadline;
    }

    void schedule(Time abs) {
        _deadline = abs + _timeout;
    }

    bool periodic(void) {
        return _periodic;
    }

private:

    Time _timeout;
    bool _periodic;
    Time _deadline;
};

template<typename mtx_type>
class unlock_guard {
public:
    unlock_guard(mtx_type & m): _mtx(m) {
        _mtx.unlock();
    }
    ~unlock_guard(void) {
        _mtx.lock();
    }

private:
    mtx_type & _mtx;
};
typedef std::lock_guard<std::mutex> lock_mtx;
typedef unlock_guard<std::mutex> unlock_mtx;

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

/**
 * Create a clockable device.
 * Each device operates on a notion of virtual time, this is usually translated
 * to hertz befor being used.
 */
class Clockable {
public:
    enum class State {
        Running, /* Executing */
        Waiting, /* Waiting for an event */
        Paused,  /* Paused, events won't wake us. */
        Stopped, /* Dead */
    };

    Clockable(const std::string &name);
    virtual ~Clockable(void);

    virtual void power(void);
    virtual void execute(void) = 0;

    const std::string &name(void) {
        return _name;
    }

    const Time now(void) {
        return _current_time;
    }

    const Time left(void) {
        return _avail_time;
    }

    Clockable::State state(void);

    void add_time(Time interval);

    void wait_state(State state);

    void wait(void);

protected:
    void do_advance(Time interval) {
        assert(_avail_time >= interval);
        _current_time += interval;
        _avail_time -= interval;
    }

    void do_run(void);

    void do_set_state(State state);
    Clockable::State do_get_state(void);

    std::future<void> _task;
private:
    std::mutex _mtx;
    std::condition_variable _cv;

    State _state;
    std::string _name;
    Time _current_time;
    Time _avail_time;
};

class Scheduler {
public:
    Scheduler(void);
    ~Scheduler(void);

    void add(Clockable *dev);

private:
    std::list<Clockable *> _devices;
};

};
