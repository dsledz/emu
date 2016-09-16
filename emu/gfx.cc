/*
 * Copyright (c) 2013, Dan Sledz
 * All rights reserved.
 *
 */

#include "emu/gfx.h"
#include "emu/device.h"
#include "emu/machine.h"

using namespace EMU;

GfxDevice::GfxDevice(Machine *machine, Clock *clock, const std::string &name,
                     unsigned hertz)
    : ClockedDevice(machine, clock, name, hertz), m_scanline(0) {}

GfxDevice::~GfxDevice(void) {}

void GfxDevice::execute(void) {
  static const Cycles m_cycles_per_scanline(384);
  while (true) {
    add_icycles(m_cycles_per_scanline);
    m_scanline = (m_scanline + 1) % 264;
    auto it = m_callbacks.find(m_scanline);
    if (it != m_callbacks.end()) it->second();
  }
}

void GfxDevice::register_callback(unsigned scanline, scanline_fn fn) {
  m_callbacks.insert(make_pair(scanline, fn));
}
