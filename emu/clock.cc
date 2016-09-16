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

ClockedDevice::ClockedDevice(Machine *machine, Clock *clock,
                             const std::string &name, unsigned hertz)
    : Device(machine, name),
      m_clock(clock),
      m_hertz(hertz),
      m_used(0),
      m_avail(0),
      m_our_ctx(reinterpret_cast<uint64_t>(&ClockedDevice::run_context),
                reinterpret_cast<uint64_t>(this)),
      m_their_ctx(0) {
  m_clock->attach_clocked(this);
}

ClockedDevice::~ClockedDevice(void) { m_clock->detach_clocked(this); }

void ClockedDevice::update(DeviceUpdate &update) {
  switch (update.type) {
    case DeviceUpdateType::Clock: {
      time_forward(update.clock.now);
      break;
    }
    default:
      Device::update(update);
      break;
  }
}

bool ClockedDevice::time_forward(EmuTime now) {
  if (now <= m_current)
    return false;

  m_target = now;
  m_avail = Cycles(m_target - m_current, m_hertz);
  return true;
}

void ClockedDevice::run_context(ClockedDevice *dev) {
  dev->run_internal();
}

void ClockedDevice::run_internal(void) {
  yield();
  DEVICE_DEBUG("Execute");
  try {
    execute();
  } catch (CoreException &e) {
    DEVICE_INFO("Exception escaped: ", e.what());
  }
  DEVICE_DEBUG("Finished");
  {
    lock_mtx lock(m_mtx);
    m_status = DeviceStatus::Off;
    m_cv.notify_all();
  }
  m_their_ctx.switch_context(&m_our_ctx);
}

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
  for (auto *dev : m_devs) {
    LOG_INFO("Starting device: ", dev->name(), " ",
             Hex(reinterpret_cast<uintptr_t>(dev)));
    dev->start();
  }

  while (true) {
    while (m_oldest < m_now) {
      /* Update our time */
      update_stats();
      if (!publish())
        Task::yield();
      for (auto *dev: m_runnable) {
        dev->resume();
      }
      m_runnable.clear();
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
  bool runnable = false;
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
  for (auto it = m_devs.begin(); it != m_devs.end(); it++) {
    if ((*it)->time_forward(m_current)) {
      LOG_DEBUG("Adding: ", (*it)->name());
      m_runnable.push_back(*it);
      runnable = true;
    }
  }
  return runnable;
}
