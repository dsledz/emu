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

  const EmuTime now(void) const { return m_now; }
  const EmuTime current(void) const { return m_current; }

 private:
  void task_fn(void);
  void update_stats(void);
  bool publish(void);

  TaskScheduler m_scheduler;
  std::list<ClockedDevice *> m_devs;
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
