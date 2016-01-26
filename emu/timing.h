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

#include "core/channel.h"
#include "core/task.h"

using namespace Core;

namespace EMU {

struct Cycles {
    Cycles(void) = default;
    explicit Cycles(int64_t v): v(v) { }

    inline const struct Cycles operator +(const struct Cycles &rhs) const {
        Cycles res(*this);
        res += rhs;
        return res;
    }

    inline const struct Cycles operator *(const struct Cycles &rhs) {
        return Cycles(v * rhs.v);
    }

    inline struct Cycles &operator *=(const struct Cycles &rhs) {
        v *= rhs.v;
        return *this;
    }

    inline struct Cycles & operator +=(const struct Cycles &rhs) {
        v = v + rhs.v;
        return *this;
    }

    inline struct Cycles & operator +=(unsigned rhs) {
        v += rhs;
        return *this;
    }

    inline const struct Cycles operator -(const struct Cycles &rhs) const {
        return Cycles(v - rhs.v);
    }

    inline struct Cycles & operator -=(const struct Cycles &rhs) {
        v -= rhs.v;
        return *this;
    }

    inline const struct Cycles operator /(unsigned rhs) const {
        return Cycles(v / rhs);
    }

    inline const struct Cycles operator *(unsigned rhs) const {
        return Cycles(v * rhs);
    }

    inline bool operator >(int64_t rhs) const {
        return v > rhs;
    }

    inline bool operator >(const struct Cycles &rhs) const {
        return v > rhs.v;
    }

    inline bool operator <(const struct Cycles &rhs) const {
        return v < rhs.v;
    }

    inline bool operator ==(const struct Cycles &rhs) const {
        return v == rhs.v;
    }

    int64_t v = 0;
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

    void add_cycles(Cycles used, Cycles hertz) {
        /* XXX: Overflow */
        ns += (used.v * NSEC_PER_SEC)/hertz.v;
    }

    int64_t ns = 0;
};

static inline std::ostream & operator << (std::ostream &os, const Time &obj)
{
    int64_t ns = obj.ns;
    if (obj.ns < 0) {
        os << "-";
        ns = -ns;
    }
    if (ns < NSEC_PER_USEC)
        os << ns << "ns";
    else if (ns < NSEC_PER_MSEC)
        os << ns / NSEC_PER_USEC << "."
           << (ns % NSEC_PER_USEC)/(NSEC_PER_USEC/10) << "us";
    else if (ns < NSEC_PER_SEC)
        os << ns / NSEC_PER_MSEC << "."
           << (ns % NSEC_PER_MSEC)/(NSEC_PER_MSEC/10) << "ms";
    else
        os << ns / NSEC_PER_SEC << "."
           << (ns % NSEC_PER_SEC)/(NSEC_PER_SEC/10) << "s";
    return os;
}

typedef Time EmuTime;

extern const Time time_zero;

/**
 * Clock device. Regulates the running of a device based on real time.
 */
class RealTimeClock {
    public:
        RealTimeClock(void) {
            _start = mach_absolute_time();
            _clock = _start;
            mach_timebase_info_data_t timebase;
            mach_timebase_info(&timebase);
        }
        ~RealTimeClock(void) { }

        void reset(void) {
            _start = mach_absolute_time();
            _clock = _start;
        }

        void pause(void) {
            _pause = mach_absolute_time();
        }

        void resume(void) {
            uint64_t now = mach_absolute_time();
            _start -= (now - _pause);
        }

        Time runtime(void) {
            uint64_t now = mach_absolute_time();

            return Time(nsec(now - _start));
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
        uint64_t _start;
        uint64_t _pause;
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

struct EmuClockUpdate {
    EmuClockUpdate(void) = default;
    EmuClockUpdate(EmuTime now): now(now) {}

    EmuTime      now;
};

typedef Channel<EmuClockUpdate> EmuTimeChannel;
typedef std::shared_ptr<EmuTimeChannel> EmuTimeChannel_ptr;

class EmuClockBase
{
public:
    EmuClockBase(void):m_target(), m_current() { }
    virtual ~EmuClockBase(void) { }
    EmuClockBase(const EmuClockBase &clock) = delete;

    virtual void time_set(EmuTime now) = 0;

    inline const EmuTime time_now(void) const {
        return m_current;
    }

protected:
    EmuTime    m_target;
    EmuTime    m_current;
};

/**
 * Global clock.
 */
class EmuSimClock
{
public:
    EmuSimClock(void):m_now(), m_current(), m_newest(), m_oldest() { }
    ~EmuSimClock(void) { }
    EmuSimClock(const EmuSimClock &sim_clock) = delete;

    void add_clock(EmuClockBase *clock) {
        m_clocks.push_back(clock);
    }

    void remove_clock(EmuClockBase *clock) {
        m_clocks.remove(clock);
    }

    const EmuTime now(void) const {
        return m_now;
    }

    /**
     * The newest device clock.
     */
    const EmuTime current(void) const {
        return m_current;
    }

    /**
     * The oldest device clock.
     */
    const EmuTime oldest(void) const {
        return m_oldest;
    }

    void set(EmuTime now) {
        /* Actual Time */
        EmuTime skew(usec(100));
        m_now = now;
        update_stats();
        if (m_oldest + skew > m_current) {
            m_current = m_now;
            for (auto it = m_clocks.begin(); it != m_clocks.end(); it++)
                (*it)->time_set(m_current);
        } else {
            EmuTime diff = m_current - m_oldest;
            LOG_DEBUG("Running ", diff, " behind, ", m_oldest, " ", m_current);
        }
    }

private:

    void update_stats(void) {
        EmuTime oldest = time_zero;
        EmuTime newest = time_zero;
        for (auto it = m_clocks.begin(); it != m_clocks.end(); it++) {
            EmuTime fb = (*it)->time_now();
            LOG_DEBUG("Found clock: ", fb);
            if (oldest == time_zero || fb < oldest)
                oldest = fb;
            if (newest == time_zero || fb > newest)
                newest = fb;
        }
        m_oldest = oldest;
        m_newest = newest;
    }

    EmuTime m_now;        /**< Wall time */
    EmuTime m_current;    /**< Simulation target time */

    EmuTime m_newest;
    EmuTime m_oldest;

    std::list<EmuClockBase *> m_clocks;
};

};
