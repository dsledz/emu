/*
 * Copyright (c) 2013, Dan Sledz
 * All rights reserved.
 *
 */

#include "emu/clock.h"
#include "core/bits.h"
#include "core/exception.h"

using namespace EMU;
using namespace Core;

Clock::Clock(void)
    : m_scheduler(),
      m_devs(),
      m_task(&m_scheduler, std::bind(&Clock::task_fn, this), "crystal"),
      m_cv(),
      m_mtx(),
      m_channel(),
      m_now(),
      m_current(),
      m_newest(),
      m_oldest() {}

Clock::~Clock(void) {}

void Clock::attach_clocked(ClockedDevice *dev) {
  LOG_INFO("Attaching ", dev->name());
  m_devs.push_back(dev);
}

void Clock::detach_clocked(ClockedDevice *dev) {
  LOG_INFO("Detaching ", dev->name());
  m_devs.remove(dev);
}

void Clock::task_fn(void) {
  LOG_INFO("Starting clock");
  while (true) {
    while (m_oldest < m_now) {
      /* Update our time */
      update_stats();
      publish();
      Task::yield();
    }
    EmuClockUpdate u = m_channel.get();
    if (u.now == time_zero) {
      LOG_INFO("Shutting down clock");
      break;
    }
    m_now = u.now;
  }
}

void Clock::start_clocked(void) {
  m_scheduler.run_task(&m_task);
  for (auto it = m_devs.begin(); it != m_devs.end(); it++) {
    m_scheduler.run_task((*it)->task());
  }
}

void Clock::stop_clocked(void) {
  LOG_TRACE("Stopping clocks");
  // TODO: Cancel outstanding clocks
  m_task.force();
  m_scheduler.wait_for_idle();
}

void Clock::wait_for_target(EmuTime t) {
  EmuClockUpdate u(t);
  m_channel.put(u);
  m_scheduler.wait_for_idle();
}

void Clock::update_stats(void) {
  EmuTime oldest = time_zero;
  EmuTime newest = time_zero;
  for (auto it = m_devs.begin(); it != m_devs.end(); it++) {
    EmuTime fb = (*it)->time_now();
    LOG_TRACE("Found clock: ", (*it));
    if (oldest == time_zero || fb < oldest) oldest = fb;
    if (newest == time_zero || fb > newest) newest = fb;
  }
  m_oldest = oldest;
  m_newest = newest;
}

void Clock::publish(void) {
  /* Actual Time */
  EmuTime skew(usec(100));
  if (m_oldest + skew > m_current) {
    m_current = m_now;
    for (auto it = m_devs.begin(); it != m_devs.end(); it++)
      (*it)->time_set(m_current);
  } else {
    EmuTime diff = m_current - m_oldest;
    LOG_TRACE("Running ", diff, " behind, (", m_oldest, " - ", m_current, ")");
  }
}
