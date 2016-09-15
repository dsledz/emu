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
      m_task(&m_scheduler, std::bind(&Clock::task_fn, this), "clock"),
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
      if (publish())
        Task::yield();
    }
    EmuClockUpdate u = m_channel.get();
    if (u.stop) {
      LOG_INFO("Shutting down clock");
      break;
    }
    m_now = u.now;
    LOG_INFO("Updated time to: ", m_now);
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
  m_channel.put(EmuClockUpdate(time_zero, true));
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
    EmuTime fb = (*it)->time_current();
    LOG_DEBUG("Found clock: ", (*it)->name(), ", current: ", fb, ", target: ",
              (*it)->time_target());
    if (oldest == time_zero || fb < oldest) oldest = fb;
    if (newest == time_zero || fb > newest) newest = fb;
  }
  m_oldest = oldest;
  m_newest = newest;
}

bool Clock::publish(void) {
  /* Actual Time */
  bool yield = false;
  EmuTime skew(usec(100));
  EmuTime step(msec(1));
  if (m_oldest + skew > m_current) {
    if (m_now > m_current + step)
      m_current += step;
    else
      m_current = m_now;
  } else {
    EmuTime diff = m_current - m_oldest;
    LOG_DEBUG("Running ", diff, " behind, (", m_oldest, " - ", m_current, ")");
  }
  for (auto it = m_devs.begin(); it != m_devs.end(); it++)
    yield = (*it)->time_set(m_current) || yield;
  return yield;
}
