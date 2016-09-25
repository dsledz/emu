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
                             const std::string &name)
    : ClockedDevice(machine, clock, name, ClockDivider(1)) {}

ClockedDevice::ClockedDevice(Machine *machine, Clock *clock,
                             const std::string &name, ClockDivider divider)
    : Device(machine, name),
      m_clock(clock),
      m_divider(divider),
      m_our_ctx(reinterpret_cast<uint64_t>(&ClockedDevice::run_context),
                reinterpret_cast<uint64_t>(this)),
      m_their_ctx(0),
      m_avail(0) {
  m_clock->attach_clocked(this);
}

ClockedDevice::~ClockedDevice(void) { m_clock->detach_clocked(this); }

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

void ClockedDevice::test_start(void) {
  m_target_status = DeviceStatus::Running;
  m_status = DeviceStatus::Running;
  m_our_ctx.initial_switch(&m_their_ctx);
}

void ClockedDevice::test_step(void) {
  m_avail = Cycles(1).multiple(m_divider);
  resume();
}

Clock::Clock(Machine *machine, Hertz hertz)
    : Device(machine, "clock"),
      m_rtc(),
      m_hertz(hertz),
      m_devs(),
      m_task(machine->get_scheduler(), std::bind(&Clock::task_fn, this),
             "clock"),
      m_now(),
      m_target(),
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
      m_target = Cycles(m_now, m_hertz);
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
  Cycles current;
  Cycles target;
  DEVICE_INFO("Starting clock");
  m_rtc.reset();
  while (m_target_status == DeviceStatus::Running) {
    {
      lock_mtx lock(m_mtx);
      current = m_current;
      target = m_target;
    }
    current = run_loop(current, target);
    {
      lock_mtx lock(m_mtx);
      m_current = current;
      m_cv.notify_all();
    }
    wait_for_update();
  }
  DEVICE_INFO("Effective hertz: ", m_current.to_hertz(m_rtc.get_delta()));
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
  {
    lock_mtx lock(m_mtx);
    m_channel.put(EmuClockUpdate(t));
    Cycles target(t, m_hertz);
    // XXX: We should be able to use m_target here
    while (m_current < target)
      lock.wait(m_cv);
  }
}

Cycles Clock::run_loop(Cycles current, Cycles target) {
  /* Actual Time */
  const Cycles skew(40);
  Cycles oldest(0);
  while (current < target) {
    /* Run any old devices */
    current += skew;
    for (auto *dev : m_devs) {
      dev->cycles_forward(skew);
    }
    bool once;
    do {
      once = false;
      for (auto *dev : m_devs) {
        Cycles fb = dev->cycles_left();
        LOG_DEBUG("Found clock: ", dev->name(), ", available: ", fb);
        if (fb > Cycles(0)) {
          dev->resume();
          once = true;
        }
      }
    } while (once);
  }
  return current;
}

