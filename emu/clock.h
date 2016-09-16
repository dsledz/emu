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

class ClockedDevice : public Device {
 public:
  ClockedDevice(Machine *machine, Clock *clock, const std::string &name,
                unsigned hertz);
  virtual ~ClockedDevice(void);

  inline void add_icycles(unsigned cycles) { add_icycles(Cycles(cycles)); }

  inline void add_icycles(Cycles cycles) {
    m_avail -= cycles;
    m_used += cycles;
    while (m_avail <= 0) {
      yield();
    }
  }

  inline void start(void) { m_our_ctx.initial_switch(&m_their_ctx); }

  inline void resume(void) { m_our_ctx.switch_context(&m_their_ctx); }

  inline void yield(void) {
    m_current += m_used.to_time(m_hertz);
    m_used = Cycles(0);
    m_their_ctx.switch_context(&m_our_ctx);
  }

  bool time_forward(EmuTime now);
  inline const EmuTime time_current(void) const { return m_current; }
  inline const EmuTime time_target(void) const { return m_target; }

 protected:
  virtual void update(DeviceUpdate &update);

 private:

  void run_internal(void);
  static void run_context(ClockedDevice *dev);

  Clock *m_clock;
  unsigned m_hertz;
  Cycles m_used;
  Cycles m_avail;

  ThreadContext m_our_ctx;
  ThreadContext m_their_ctx;

  EmuTime m_target;
  EmuTime m_current;
};


typedef Channel<EmuClockUpdate> ClockChannel;
typedef std::shared_ptr<ClockChannel> ClockChannel_ptr;

class Clock {
 public:
  Clock(void);
  virtual ~Clock(void);

  void attach_clocked(ClockedDevice *dev);
  void detach_clocked(ClockedDevice *dev);

  void start_clocked(void);
  void stop_clocked(void);

  TaskScheduler *sched(void) { return &m_scheduler; }
  void wait_for_target(EmuTime t);
  void wait_for_delta(EmuTime t) { wait_for_target(m_now + t); }

  void device_yield(ClockedDevice *dev);
  void device_resume(ClockedDevice *dev);
  void device_start(ClockedDevice *dev);

  const EmuTime now(void) const { return m_now; }
  const EmuTime current(void) const { return m_current; }

 private:
  void task_fn(void);
  void update_stats(void);
  bool publish(void);

  TaskScheduler m_scheduler;
  std::list<ClockedDevice *> m_devs;
  std::list<ClockedDevice *> m_runnable;
  FiberTask m_task;

  std::condition_variable m_cv; /* ? */
  std::mutex m_mtx;             /* ? */
  ClockChannel m_channel;   /* ? */

  EmuTime m_now;     /**< Wall time */
  EmuTime m_current; /**< Simulation target time */

  EmuTime m_newest;
  EmuTime m_oldest;
};
};
