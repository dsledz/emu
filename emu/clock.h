/*
 * Copyright (c) 2013, Dan Sledz
 * All rights reserved.
 */
#pragma once

#include "core/channel.h"
#include "core/task.h"

using namespace Core;

namespace EMU {

class Crystal : public Device {
 public:
  Crystal(void);
  virtual ~Crystal(void);

  void add_device_clock(EmuClockBase *clock);
  void remove_device_clock(EmuClockBase *clock);

  const EmuTime now(void) const;
  const EmuTime current(void) const;
  EmuClockBase *oldest(void);
  EmuClockBase *newest(void);

  virtual void line(Line line, LineState state);
  virtual void reset(void);
  virtual void execute(void);
  virtual void update(DeviceUpdate &update);

 private:
  void update_stats(void);
  void publish(void);

  std::vector<EmuClockBase *> m_clocks;
  std::unique_ptr<Task> m_task;

  EmuTime m_now;     /**< Wall time */
  EmuTime m_current; /**< Simulation target time */

  EmuTime m_newest;
  EmuTime m_oldest;
};
};
