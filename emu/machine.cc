/*
 * Copyright (c) 2013, Dan Sledz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "emu.h"

using namespace EMU;

Machine::Machine(Hertz hertz)
    : m_clock(nullptr),
      m_scheduler(),
      m_input(),
      m_devs(),
      m_switches(),
      m_ports(),
      m_debugger(),
      m_fb(nullptr) {
  add_clock(hertz);
}

Machine::~Machine(void) {
  /* Make sure all devices are disconnected */
  m_clock.reset(nullptr);
  assert(m_devs.empty());
}

void Machine::load_rom(const std::string &rom) {}

void Machine::poweron(void) {
  m_clock->start();
}

void Machine::poweroff(void) {
  m_clock->stop();
}

void Machine::add_clock(Hertz hertz) {
  m_clock = std::unique_ptr<Clock>(new Clock(this, hertz));
}

void Machine::add_device(Device *dev) { m_devs.push_back(dev); }

void Machine::remove_device(Device *dev) {
  /* XXX: This is broken */
  if (m_debugger) m_debugger->remove_debuggable(dev);
  m_devs.remove(dev);
}

EmuTime Machine::now(void) { return m_clock->now(); }

void Machine::run_forward(EmuTime delta) {
  auto future = m_clock->set_delta(delta);
  future.get();
}

std::future<void> Machine::advance(EmuTime delta) {
  return m_clock->set_delta(delta);
}

Device *Machine::dev(const std::string &name) {
  for (auto it = m_devs.begin(); it != m_devs.end(); it++) {
    if ((*it)->name() == name) return *it;
  }
  throw KeyError(name);
}

FrameBuffer *Machine::screen(void) { return m_fb.get(); }

void Machine::add_ioport(const std::string &name) {
  m_ports.insert(make_pair(name, IOPort()));
}

IOPort *Machine::ioport(const std::string &name) {
  auto it = m_ports.find(name);
  if (it == m_ports.end()) throw KeyError(name);
  return &it->second;
}

byte_t Machine::read_ioport(const std::string &name) {
  IOPort *port = ioport(name);
  return read_ioport(port);
}

void Machine::write_ioport(const std::string &name, byte_t value) {
  IOPort *port = ioport(name);
  write_ioport(port, value);
}

byte_t Machine::read_ioport(IOPort *port) { return port->value; }

void Machine::write_ioport(IOPort *port, byte_t value) { port->value = value; }

void Machine::add_input(const InputSignal &signal) { m_input.add(signal); }

void Machine::add_input(InputKey key, input_fn fn) {
  m_input.add_input(key, fn);
}

void Machine::send_input(InputKey key, bool pressed) {
  if (pressed)
    m_input.depress(key);
  else
    m_input.release(key);
}

dipswitch_ptr Machine::add_switch(const std::string &name,
                                  const std::string &port, byte_t mask,
                                  byte_t def) {
  auto ptr = dipswitch_ptr(new Dipswitch(name, port, mask, def));
  m_switches.insert(make_pair(name, ptr));
  return ptr;
}

void Machine::set_switch(const std::string &name, const std::string &value) {
  auto sw = m_switches.find(name);
  if (sw == m_switches.end()) throw KeyError(name);
  sw->second->select(this, value);
}

void Machine::reset_switches(void) {
  for (auto it = m_switches.begin(); it != m_switches.end(); it++)
    it->second->set_default(this);
}

void Machine::reset(void) {
  for (auto it = m_devs.begin(); it != m_devs.end(); it++)
    set_line(*it, Line::RESET, LineState::Pulse);
}

void Machine::set_line(const std::string &name, Line line, LineState state) {
  LOG_TRACE("DEV(", name, "): ", line, " ", state);
  set_line(dev(name), line, state);
}

void Machine::set_line(Device *dev, Line line, LineState state) {
  /* XXX: Handle pulse */
  dev->line(line, state);
}

void Machine::add_screen(short width, short height, GfxScale scale,
                         FrameBuffer::Rotation rotation) {
  m_fb = FrameBuffer_ptr(new FrameBuffer(width, height, GfxScale::None, rotation));
}

void Machine::set_debugger(Debugger *debugger) {
  m_debugger = debugger;
  for (auto it = m_devs.begin(); it != m_devs.end(); it++) {
    m_debugger->add_debuggable(*it);
  }
}

void Machine::log(LogLevel level, const std::string fmt, ...) {
  va_list args;
  va_start(args, fmt);
  Core::log.log(level, fmt, args);
  va_end(args);
}

MachineLoader *EMU::loader(void) {
  static MachineLoader loader;
  return &loader;
}

MachineDefinition::MachineDefinition(const std::string &name,
                                     const MachineInformation &info,
                                     machine_create_fn fn)
    : name(name), info(info), fn(fn) {
  loader()->add_machine(this);
}

MachineDefinition::~MachineDefinition(void) {
  /* XXX: Should we remove the entry? */
}

void MachineDefinition::add(void) { loader()->add_machine(this); }

MachineLoader::MachineLoader(void) {}

MachineLoader::~MachineLoader(void) {}

void MachineLoader::add_machine(struct MachineDefinition *definition) {
  m_machines.push_back(definition);
}

machine_ptr MachineLoader::load(Options *opts) {
  machine_ptr machine;
  for (auto it = m_machines.begin(); it != m_machines.end(); it++) {
    if ((*it)->name == opts->driver) return (*it)->fn(opts);
  }
  throw DriverError(opts->driver);
}

const struct MachineDefinition *MachineLoader::find(const std::string &name) {
  for (auto it = m_machines.begin(); it != m_machines.end(); it++) {
    if ((*it)->name == name) return *it;
  }
  throw DriverError(name);
}

std::list<MachineDefinition *>::const_iterator MachineLoader::start(void) {
  return m_machines.begin();
}

std::list<MachineDefinition *>::const_iterator MachineLoader::end(void) {
  return m_machines.end();
}
