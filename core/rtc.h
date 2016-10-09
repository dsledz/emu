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

#include "core/bits.h"

#ifdef WIN32
#define NSEC_PER_SEC 1000000000ll
#define NSEC_PER_MSEC 1000000ll
#define NSEC_PER_USEC 1000ll
#elif __APPLE__
#include <mach/mach.h>
#include <mach/mach_time.h>
#else
#define NSEC_PER_SEC 1000000000ll
#define NSEC_PER_MSEC 1000000ll
#define NSEC_PER_USEC 1000ll
#include <time.h>
#endif

namespace Core {

struct nsec {
  explicit nsec(uint64_t v) : v(v) {}
  explicit nsec(const struct timespec &t)
      : v(t.tv_sec * NSEC_PER_SEC + t.tv_nsec) {}
  uint64_t v;
};

struct usec {
  explicit usec(uint64_t v) : v(v) {}
  uint64_t v;
};

struct msec {
  explicit msec(uint64_t v) : v(v) {}
  uint64_t v;
};

struct sec {
  explicit sec(uint64_t v) : v(v) {}
  uint64_t v;
};

struct Time {
  Time(void) = default;
  Time(sec s) : ns(s.v * NSEC_PER_SEC) {}
  Time(msec ms) : ns(ms.v * NSEC_PER_MSEC) {}
  Time(usec us) : ns(us.v * NSEC_PER_USEC) {}
  Time(nsec ns) : ns(ns.v) {}

  inline explicit operator double() const {
    return double(ns) / NSEC_PER_SEC;
  }

  inline explicit operator msec() const {
    return msec(ns / NSEC_PER_MSEC);
  }

  inline bool operator==(const Time &rhs) const { return (ns == rhs.ns); }

  inline bool operator!=(const Time &rhs) const { return (ns != rhs.ns); }

  inline const Time operator/(unsigned rhs) const { return Time(nsec(ns / rhs)); }

  inline Time &operator+=(const Time &rhs) {
    ns += rhs.ns;
    return *this;
  }

  inline Time &operator-=(const Time &rhs) {
    ns -= rhs.ns;
    return *this;
  }

  inline const Time operator-(const Time &rhs) const {
    Time res(*this);
    res -= rhs;
    return res;
  }

  inline const Time operator+(const Time &rhs) const {
    Time res(*this);
    res += rhs;
    return res;
  }

  inline bool operator>(const Time &rhs) const { return ns > rhs.ns; }

  inline bool operator<(const Time &rhs) const { return ns < rhs.ns; }

  inline bool operator<=(const Time &rhs) const { return ns <= rhs.ns; }

  inline bool operator>=(const Time &rhs) const { return ns >= rhs.ns; }

  int64_t ns = 0;
};

extern const Time time_zero;

static inline std::ostream &operator<<(std::ostream &os, const Time &obj) {
  int64_t ns = obj.ns;
  if (obj.ns < 0) {
    os << "-";
    ns = -ns;
  }
  if (ns < NSEC_PER_USEC)
    os << ns << "ns";
  else if (ns < NSEC_PER_MSEC)
    os << ns / NSEC_PER_USEC << "."
       << (ns % NSEC_PER_USEC) / (NSEC_PER_USEC / 1000) << "us";
  else if (ns < NSEC_PER_SEC)
    os << ns / NSEC_PER_MSEC << "."
       << (ns % NSEC_PER_MSEC) / (NSEC_PER_MSEC / 1000) << "ms";
  else
    os << ns / NSEC_PER_SEC << "."
       << (ns % NSEC_PER_SEC) / (NSEC_PER_SEC / 1000) << "s";
  return os;
}

#ifdef WIN32
#include <Windows.h>
class RealTimeClock {
 public:
  RealTimeClock(void) {
    QueryPerformanceCounter(&m_start);
    QueryPerformanceFrequency(&m_frequency);
  }
  ~RealTimeClock(void) {}

  void reset(void) {
    QueryPerformanceCounter(&m_start);
    m_clock = m_start;
  }

  void pause(void) { QueryPerformanceCounter(&m_pause); }

  void resume(void) {
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    m_start.QuadPart -= now.QuadPart - m_pause.QuadPart;
  }

  Time runtime(void) {
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    now.QuadPart -= m_start.QuadPart;

    return Time(to_nsec(now));
  }

  Time get_delta(void) {
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    LARGE_INTEGER delta;
    delta.QuadPart = now.QuadPart - m_clock.QuadPart;
    m_clock = now;

    return Time(to_nsec(delta));
  }

  Time now(void) {
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return Time(to_nsec(now));
  }

 private:
  nsec to_nsec(LARGE_INTEGER val) { return nsec(val.QuadPart); }

  LARGE_INTEGER m_clock;
  LARGE_INTEGER m_start;
  LARGE_INTEGER m_pause;
  LARGE_INTEGER m_frequency;
};
#elif __APPLE__
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
  ~RealTimeClock(void) {}

  void reset(void) {
    _start = mach_absolute_time();
    _clock = _start;
  }

  void pause(void) { _pause = mach_absolute_time(); }

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

  Time now(void) { return Time(nsec(mach_absolute_time())); }

 private:
  uint64_t _clock;
  uint64_t _start;
  uint64_t _pause;
};
#else

static inline void timespec_diff(struct timespec *dest, struct timespec first,
                                 struct timespec second) {
  if (second.tv_nsec < first.tv_nsec) {
    second.tv_sec -= 1;
    second.tv_nsec += NSEC_PER_SEC;
  }
  dest->tv_sec = first.tv_sec - second.tv_sec;
  dest->tv_nsec = first.tv_nsec - second.tv_nsec;
}

/**
 * Clock device. Regulates the running of a device based on real time.
 */
class RealTimeClock {
 public:
  RealTimeClock(void) {
    clock_gettime(CLOCK_MONOTONIC, &m_start);
    m_clock = m_start;
  }
  ~RealTimeClock(void) {}

  void reset(void) {
    clock_gettime(CLOCK_MONOTONIC, &m_start);
    m_clock = m_start;
  }

  void pause(void) { clock_gettime(CLOCK_MONOTONIC, &m_pause); }

  void resume(void) {
    clock_gettime(CLOCK_MONOTONIC, &m_now);
    timespec_diff(&m_start, m_now, m_pause);
  }

  Time runtime(void) {
    clock_gettime(CLOCK_MONOTONIC, &m_now);
    timespec_diff(&m_runtime, m_now, m_start);

    return Time(nsec(m_runtime));
  }

  /* Return the number of ticks since last call */
  Time get_delta(void) {
    clock_gettime(CLOCK_MONOTONIC, &m_now);
    timespec_diff(&m_delta, m_now, m_clock);
    m_clock = m_now;

    return Time(nsec(m_delta));
  }

  Time now(void) {
    clock_gettime(CLOCK_MONOTONIC, &m_now);
    return Time(nsec(m_now));
  }

 private:
  struct timespec m_clock;
  struct timespec m_start;
  struct timespec m_pause;
  struct timespec m_runtime;
  struct timespec m_now;
  struct timespec m_delta;
};

#endif
};
