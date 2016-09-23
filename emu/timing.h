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

#include "core/debug.h"
#include "core/rtc.h"

using namespace Core;

namespace EMU {

struct Hertz {
  Hertz(void) = default;
  explicit Hertz(unsigned v): v(v) {}

  unsigned v;
};

inline std::ostream &operator<<(std::ostream &os, const Hertz &h) {
  os << h.v / 1000000 << "Mhz";
  return os;
}

struct ClockDivider {
  ClockDivider(void) = default;
  explicit ClockDivider(int v): v(v) {}

  int64_t v = 0;
};

inline std::ostream &operator<<(std::ostream &os, const ClockDivider &d) {
  os << d.v;
  return os;
}

struct Cycles {
  Cycles(void) = default;
  explicit Cycles(int64_t v) : v(v) {}
  Cycles(const Time t, unsigned hertz)
      : v(((t.ns / NSEC_PER_SEC) * hertz) +
          ((t.ns % NSEC_PER_SEC) * hertz / NSEC_PER_SEC)) {
    assert(v > 0);
  }
  Cycles(const Time t, Hertz hertz)
      : v(((t.ns / NSEC_PER_SEC) * hertz.v) +
          ((t.ns % NSEC_PER_SEC) * hertz.v / NSEC_PER_SEC)) {
    assert(v> 0);
  }

  inline Time to_time(unsigned hertz) const {
    return Time(nsec((v * NSEC_PER_SEC) / hertz));
  }

  inline Time to_time(Hertz hertz) const {
    return Time(nsec((v * NSEC_PER_SEC) / hertz.v));
  }

  inline Hertz to_hertz(Time t) const {
    const Hertz hertz(v / (t.ns / NSEC_PER_SEC) +
                      (v % (t.ns / NSEC_PER_SEC) * NSEC_PER_SEC) / t.ns);
    return hertz;
  }

  inline const Cycles operator+(const struct Cycles &rhs) const {
    Cycles res(*this);
    res += rhs;
    return res;
  }

  inline const struct Cycles operator*(const struct Cycles &rhs) {
    return Cycles(v * rhs.v);
  }

  inline struct Cycles &operator*=(const struct Cycles &rhs) {
    v *= rhs.v;
    return *this;
  }

  inline struct Cycles &operator+=(const struct Cycles &rhs) {
    v = v + rhs.v;
    return *this;
  }

  inline struct Cycles &operator+=(unsigned rhs) {
    v += rhs;
    return *this;
  }

  inline const struct Cycles operator-(const struct Cycles &rhs) const {
    return Cycles(v - rhs.v);
  }

  inline struct Cycles &operator-=(const struct Cycles &rhs) {
    v -= rhs.v;
    return *this;
  }

  inline const struct Cycles operator/(unsigned rhs) const {
    return Cycles(v / rhs);
  }

  inline const struct Cycles operator*(unsigned rhs) const {
    return Cycles(v * rhs);
  }

  inline bool operator>(int64_t rhs) const { return v > rhs; }

  inline bool operator>(const struct Cycles &rhs) const { return v > rhs.v; }

  inline bool operator<(const struct Cycles &rhs) const { return v < rhs.v; }

  inline bool operator<=(const struct Cycles &rhs) const { return v <= rhs.v; }

  inline bool operator<=(int rhs) const { return v <= rhs; }

  inline bool operator==(const struct Cycles &rhs) const { return v == rhs.v; }

  inline const struct Cycles subtract(const struct Cycles &rhs,
                                      const struct ClockDivider divider) {
    v -= (rhs.v * divider.v);
    return *this;
  }

  inline const struct Cycles add(const struct Cycles &rhs,
                                      const struct ClockDivider divider) {
    v += (rhs.v * divider.v);
    return *this;
  }

  inline const struct Cycles multiple(const struct ClockDivider divider) const {
    return Cycles(v * divider.v);
  }

  inline const struct Cycles divide(const struct ClockDivider divider) const {
    return Cycles(v / divider.v);
  }

  int64_t v = 0;
};

inline std::ostream &operator<<(std::ostream &os, const Cycles &c) {
  os << c.v;
  return os;
}

typedef Time EmuTime;

struct EmuClockUpdate {
  EmuClockUpdate(void) = default;
  EmuClockUpdate(EmuTime now) : EmuClockUpdate(now, false) { }
  EmuClockUpdate(EmuTime now, bool stop) : now(now), stop(stop) {}

  EmuTime now;
  bool stop;
};

};
