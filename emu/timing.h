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
    explicit nsec(unsigned v): v(v) { }
    uint64_t v;
};

struct usec {
    explicit usec(unsigned v): v(v) { }
    uint64_t v;
};

struct msec {
    explicit msec(unsigned v): v(v) { }
    uint64_t v;
};

struct sec {
    explicit sec(unsigned v): v(v) { }
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

    uint64_t ns;
};

extern const Time time_zero;

/**
 * Clock device. Regulates the running of a device based on real time.
 */
class RealTimeClock {
    public:
        RealTimeClock(const Cycles &hz) {
            _clock = mach_absolute_time();
            mach_timebase_info_data_t timebase;
            mach_timebase_info(&timebase);
            _hz_per_nano = ((double)(hz.v) * timebase.numer) /
                (1000000000 * timebase.denom);
        }
        ~RealTimeClock(void) { }

        void reset(void) {
            _clock = mach_absolute_time();
        }

        /* Return the number of ticks since last call */
        struct Cycles get_ticks(void) {

            uint64_t now = mach_absolute_time();
            uint64_t delta = (now - _clock);

            _clock = now;
            return Cycles(delta * _hz_per_nano);
        }

    private:
        double _hz_per_nano;
        uint64_t _clock;
};

};
