/*
 * Copyright (c) 2013, Dan Sledz
 * All rights reserved.
 */
#pragma once

#include "core/channel.h"
#include "core/task.h"
#include "core/rtc.h"

#include  "emu/timing.h"
#include  "emu/device.h"

using namespace Core;

namespace EMU {

class Clock;
class Machine;

class ClockedDevice : public Device {
 public:
  ClockedDevice(Machine *machine, Clock *clock, const std::string &name,
                ClockDivider divider);
  ClockedDevice(Machine *machine, Clock *clock, const std::string &name);
  virtual ~ClockedDevice(void);

  inline void add_icycles(Cycles cycles) {
    m_avail.subtract(cycles, m_divider);
    while (m_avail <= 0) {
      yield();
    }
  }

  void test_start(void);
  void test_step(void);

  inline void start(void) { m_our_ctx.initial_switch(&m_their_ctx); }

  inline void resume(void) { m_our_ctx.switch_context(&m_their_ctx); }

  inline void yield(void) {
    m_their_ctx.switch_context(&m_our_ctx);
    if (unlikely(m_channel.available() > 0)) wait_for_update();
  }

  const Cycles cycles_left(void) const { return m_avail.multiple(m_divider); }
  void cycles_forward(Cycles avail) {
    m_avail += avail;
    resume();
  }

 private:

  void run_internal(void);
  static void run_context(ClockedDevice *dev);

  Clock *m_clock;
  ClockDivider m_divider;

  ThreadContext m_our_ctx;
  ThreadContext m_their_ctx;

  Cycles m_avail;
};

class Clock : public Device {
 public:
  Clock(Machine *machine, Hertz hertz);
  virtual ~Clock(void);

  void attach_clocked(ClockedDevice *dev);
  void detach_clocked(ClockedDevice *dev);

  void start(void);
  void stop(void);

  const Hertz hertz(void) const { return m_hertz; }
  void wait_for_target(EmuTime t);
  void wait_for_delta(EmuTime t) { wait_for_target(m_now + t); }

  void device_yield(ClockedDevice *dev);
  void device_resume(ClockedDevice *dev);
  void device_start(ClockedDevice *dev);

  virtual void initialize(void);
  virtual void execute(void);
  virtual void update(DeviceUpdate &update);

  const EmuTime now(void) const { return m_now; }
  const EmuTime current(void) const { return m_current.to_time(m_hertz); }

 private:
  void task_fn(void);
  Cycles run_loop(Cycles current, Cycles target);

  RealTimeClock m_rtc;
  Hertz m_hertz;
  std::list<ClockedDevice *> m_devs;
  FiberTask m_task;

  EmuTime m_now;     /**< Wall time */
  Cycles m_target;   /**< Target cycles */
  Cycles m_current;  /**< Current simulation cycles */
};

};
