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

    uint64_t ns = 0;
};

static inline std::ostream & operator << (std::ostream &os, const Time &obj)
{
    if (obj.ns < NSEC_PER_USEC)
        os << obj.ns << "ns";
    else if (obj.ns < NSEC_PER_MSEC)
        os << obj.ns / NSEC_PER_USEC << "."
           << (obj.ns % NSEC_PER_USEC)/(NSEC_PER_USEC/10) << "us";
    else if (obj.ns < NSEC_PER_SEC)
        os << obj.ns / NSEC_PER_MSEC << "."
           << (obj.ns % NSEC_PER_MSEC)/(NSEC_PER_MSEC/10) << "ms";
    else
        os << obj.ns / NSEC_PER_SEC << "."
           << (obj.ns % NSEC_PER_SEC)/(NSEC_PER_SEC/10) << "s";
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
    bool         stop;
    EmuTime      now;
};

typedef Channel<EmuClockUpdate> EmuTimeChannel;
typedef std::shared_ptr<EmuTimeChannel> EmuTimeChannel_ptr;

class EmuClockBase
{
public:
    EmuClockBase(void):m_stopped(false), m_now(), m_current() { }
    virtual ~EmuClockBase(void) { }
    EmuClockBase(const EmuClockBase &clock) = delete;

    virtual EmuTime time_wait(EmuTime interval) = 0;
    virtual void time_advance(void) = 0;
    virtual void time_set(EmuTime now) = 0;
    virtual void time_stop(void) = 0;
    virtual void time_update(const EmuClockUpdate &update) = 0;

    inline const EmuTime time_now(void) const {
        return m_current;
    }

protected:
    bool       m_stopped;
    EmuTime    m_now;
    EmuTime    m_current;
};

/**
 * Sleepable virtual clock for a single simulated device
 */
class EmuClock: public EmuClockBase
{
public:
    EmuClock(void): EmuClockBase(), m_channel() { }
    virtual ~EmuClock(void) { }
    EmuClock(const EmuClock &clock) = delete;

    /**
     * Wait until we have at least enough time requested.
     */
    virtual EmuTime time_wait(EmuTime interval) {
        EmuTime avail = m_now - m_current;
        while (!m_stopped && (avail == time_zero || avail < interval)) {
            EmuClockUpdate update = m_channel.get();
            if (update.stop) {
                LOG_DEBUG("Stopped!");
                m_stopped = true;
                break;
            }
            m_now = update.now;
            avail = m_now - m_current;
        }
        if (m_stopped)
            throw TaskCanceled("Off");
        return avail;
    }

    virtual void time_advance(void) {
        m_current = m_now;
    }

    virtual void time_set(EmuTime now) {
        EmuClockUpdate update {
            .stop = false,
            .now = now
        };
        m_channel.put(update);
    }

    virtual void time_stop(void) {
        EmuClockUpdate update {
            .stop = false,
            .now = time_zero
        };
        m_channel.put(update);
    }

    virtual void time_update(const EmuClockUpdate &update) {
        m_channel.put(update);
    }

private:

    EmuTimeChannel m_channel;
};

/**
 * Global clock.
 */
class EmuSimClock
{
public:
    EmuSimClock(void) { }
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

    void set(EmuTime now, EmuTime interval) {
        EmuTime skew(usec(10));
        m_now = now;
        update_stats();
        if (m_oldest + skew < m_newest) {
            m_current = std::max(m_oldest + skew, m_newest);
            LOG_DEBUG("Time skew Oldest: ", m_oldest, " Newest: ", m_newest);
        } else {
            m_current = m_now;
        }
        m_current = std::min(m_current, m_now);
        for (auto it = m_clocks.begin(); it != m_clocks.end(); it++)
            (*it)->time_set(m_current);
    }

    void set(EmuTime now) {
        set(now, Time(usec(100)));
    }
private:

    void update_stats(void) {
        EmuTime oldest = time_zero;
        EmuTime newest = time_zero;
        for (auto it = m_clocks.begin(); it != m_clocks.end(); it++) {
            EmuTime fb = (*it)->time_now();
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
