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

struct Cycles {
  Cycles(void) = default;
  explicit Cycles(int64_t v) : v(v) {}
  Cycles(const Time t, unsigned hertz): v((t.ns * hertz) / NSEC_PER_SEC) {}
  Cycles(const Time t, Hertz hertz): v((t.ns * hertz.v) / NSEC_PER_SEC) {}

  inline Time to_time(unsigned hertz) {
    return Time(nsec((v * NSEC_PER_SEC) / hertz));
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

  inline bool operator==(const struct Cycles &rhs) const { return v == rhs.v; }

  int64_t v = 0;
};

typedef Time EmuTime;

struct EmuClockUpdate {
  EmuClockUpdate(void) = default;
  EmuClockUpdate(EmuTime now) : now(now) {}

  EmuTime now;
};

class EmuClockBase {
 public:
  EmuClockBase(const std::string &name)
      : m_name(name), m_target(), m_current() {}
  virtual ~EmuClockBase(void) {}
  EmuClockBase(const EmuClockBase &clock) = delete;

  virtual void time_set(EmuTime now) = 0;

  inline const EmuTime time_now(void) const { return m_current; }

 protected:
  friend inline std::ostream &operator<<(std::ostream &os,
                                         const EmuClockBase *obj);
  const std::string &m_name;
  EmuTime m_target;
  EmuTime m_current;
};

inline std::ostream &operator<<(std::ostream &os, const EmuClockBase *obj) {
  os << "Name: " << obj->m_name << " Target: " << obj->m_target
     << " Current: " << obj->m_current;
  return os;
}

/**
 * Global clock.
 */
class EmuSimClock {
 public:
  EmuSimClock(void) : m_now(), m_current(), m_newest(), m_oldest() {}
  ~EmuSimClock(void) {}
  EmuSimClock(const EmuSimClock &sim_clock) = delete;

  void add_clock(EmuClockBase *clock) { m_clocks.push_back(clock); }

  void remove_clock(EmuClockBase *clock) { m_clocks.remove(clock); }

  const EmuTime now(void) const { return m_now; }

  /**
   * The newest device clock.
   */
  const EmuTime current(void) const { return m_current; }

  /**
   * The oldest device clock.
   */
  const EmuTime oldest(void) const { return m_oldest; }

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
      LOG_TRACE("Running ", diff, " behind, (", m_oldest, " - ", m_current,
                ")");
    }
  }

 private:
  void update_stats(void) {
    EmuTime oldest = time_zero;
    EmuTime newest = time_zero;
    for (auto it = m_clocks.begin(); it != m_clocks.end(); it++) {
      EmuTime fb = (*it)->time_now();
      LOG_TRACE("Found clock: ", (*it));
      if (oldest == time_zero || fb < oldest) oldest = fb;
      if (newest == time_zero || fb > newest) newest = fb;
    }
    m_oldest = oldest;
    m_newest = newest;
  }

  EmuTime m_now;     /**< Wall time */
  EmuTime m_current; /**< Simulation target time */

  EmuTime m_newest;
  EmuTime m_oldest;

  std::list<EmuClockBase *> m_clocks;
};
};
