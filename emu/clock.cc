/*
 * Copyright (c) 2013, Dan Sledz
 * All rights reserved.
 *
 */

#include "emu/clock.h"
#include "emu/machine.h"
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

void ClockedDevice::time_forward(EmuTime now) {
  if (now > m_current) {
    m_target = now;
    m_avail = Cycles(m_target - m_current, m_hertz);
    /* XXX: Hack to work around rounding */
    if (m_avail == Cycles(time_zero, m_hertz))
      m_avail += 1;
    resume();
  }
}

void ClockedDevice::run_context(ClockedDevice *dev) {
    dev->run_internal();
}

void ClockedDevice::run_internal(void) {
  try {
    wait(DeviceStatus::Running);
    yield();
    DEVICE_DEBUG("Execute");
    while (m_target_status != DeviceStatus::Off) execute();
  } catch (CoreException &e) {
    DEVICE_INFO("Device exception: ", e.what());
  }
  update_status(DeviceStatus::Off);
  m_their_ctx.switch_context(&m_our_ctx);
}

Clock::Clock(Machine *machine, Hertz hertz)
    : Device(machine, "clock"),
      m_hertz(hertz),
      m_devs(),
      m_task(machine->get_scheduler(), std::bind(&Clock::task_fn, this),
             "clock"),
      m_now(),
      m_current() {}

Clock::~Clock(void) {}

void Clock::attach_clocked(ClockedDevice *dev) {
  LOG_INFO("Attaching ", dev->name());
  m_devs.push_back(dev);
}

void Clock::detach_clocked(ClockedDevice *dev) {
  LOG_INFO("Detaching ", dev->name());
  m_devs.remove(dev);
}

void Clock::update(DeviceUpdate &update) {
  switch (update.type) {
    case DeviceUpdateType::Status:
      m_target_status = update.status;
      break;
    case DeviceUpdateType::None:
      break;
    case DeviceUpdateType::Clock: {
      m_now = update.clock.now;
      break;
    }
  }
}

void Clock::task_fn(void) {
  try {
    initialize();
    wait(DeviceStatus::Running);
    while (m_target_status != DeviceStatus::Off) execute();
  } catch (CoreException &e) {
    DEVICE_INFO("Device exception: ", e.what());
  }
  DEVICE_INFO("Stopping children devices");
  for (auto *dev : m_devs) dev->resume();
  {
    lock_mtx lock(m_mtx);
    m_status = DeviceStatus::Off;
    m_cv.notify_all();
  }
}

void Clock::initialize(void) {
  for (auto *dev : m_devs) {
    LOG_INFO("Starting device: ", dev->name(), " ",
             Hex(reinterpret_cast<uintptr_t>(dev)));
    dev->set_status(DeviceStatus::Running);
    dev->start();
  }
}

void Clock::execute(void) {
  wait(DeviceStatus::Running);
  LOG_INFO("Starting clock");
  while (m_target_status == DeviceStatus::Running) {
    run_loop();
    wait_for_update();
  }
}

void Clock::start(void) {
  m_task.start();
  set_status(DeviceStatus::Running);
  wait_status(DeviceStatus::Running);
}

void Clock::stop(void) {
  for (auto *dev: m_devs) {
    dev->set_status(DeviceStatus::Off);
  }
  set_status(DeviceStatus::Off);
  for (auto *dev: m_devs) {
    dev->wait_status(DeviceStatus::Off);
  }
  m_task.force();
}

void Clock::wait_for_target(EmuTime t) {
  m_channel.put(EmuClockUpdate(t));
  {
    lock_mtx lock(m_mtx);
    while (m_current < t)
      lock.wait(m_cv);
  }
}

void Clock::run_loop(void) {
  /* Actual Time */
  EmuTime skew(usec(100));
  EmuTime step(msec(1));
  EmuTime oldest = time_zero;
  while (oldest < m_now) {
    oldest = time_zero;
    for (auto *dev : m_devs) {
      EmuTime fb = dev->time_current();
      LOG_DEBUG("Found clock: ", dev->name(), ", current: ", fb, ", target: ",
                dev->time_target());
      if (oldest == time_zero || fb < oldest) oldest = fb;
    }
    if (oldest + skew > m_current) {
      if (m_now > m_current + step)
        m_current += step;
      else
        m_current = m_now;
    } else {
      EmuTime diff = m_current - oldest;
      LOG_DEBUG("Running ", diff, " behind, (", oldest, " - ", m_current, ")");
    }
    for (auto *dev : m_devs) {
      dev->time_forward(m_current);
    }
  }
  {
    lock_mtx lock(m_mtx);
    m_cv.notify_all();
  }
}
