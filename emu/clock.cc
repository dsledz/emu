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

Crystal::Crystal(void)
    : m_clocks(),
      m_task(new FiberTask(std::bind(&Crystal::task_fn, *this), "crystal")) {}

Crystal::~Crystal(void) {}

Crystal::task_fn(void) {}

void Crystal::execute(void) {
  while (true) {
    while (m_oldest < m_current) {
      /* Update our time */
      publish();
      yield();
    }
    /* Wait for a global time update */
    idle();
  }
}

void Crystal::update(DeviceUpdate &update) {
  switch (update.type) {
    case DeviceUpdateType::Clock: {
      EmuTime target = update.clock.now;
      /* XXX: push time out to the others */
      break;
    }
    default:
      Device::update(update);
      break;
  }
}

void Crystal::update_stats(void) {
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

void Crystal::publish(void) {
  /* Actual Time */
  EmuTime skew(usec(100));
  m_now = now;
  if (m_oldest + skew > m_current) {
    m_current = m_now;
    for (auto it = m_clocks.begin(); it != m_clocks.end(); it++)
      (*it)->time_set(m_current);
  } else {
    EmuTime diff = m_current - m_oldest;
    LOG_TRACE("Running ", diff, " behind, (", m_oldest, " - ", m_current, ")");
  }
}
