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
#pragma once

#include "core/bits.h"
#include "core/exception.h"
#include "emu/device.h"
#include "emu/dipswitch.h"
#include "emu/options.h"
#include "emu/timing.h"
#include "emu/video.h"

using namespace Core;

namespace EMU {

struct KeyError : public CoreException {
  KeyError(const std::string &key) : CoreException("Missing Key: "), key(key) {
    msg += key;
  }

  std::string key;
};

/**
 * Encapsulates a physical machine.
 * Each machine consists of:
 * Devices
 * Dipswitches
 * IOPorts
 */
class Machine {
 public:
  /**
   * Initialize the machine with a master clock rate of @a hertz
   */
  Machine(void);
  virtual ~Machine(void);

  void poweron(void);
  void poweroff(void);

  virtual void load_rom(const std::string &rom);

  /* Public functions */
  void set_switch(const std::string &name, const std::string &value);
  void send_input(InputKey key, bool pressed);
  void reset(void);
  void run(void);
  void run_forward(EmuTime delta);
  void run_until(EmuTime target);

  /* FrameBuffer */
  void set_frame_buffer(FrameBuffer *screen);
  /* Debugger */
  void set_debugger(Debugger *debugger);
  Debugger *get_debugger(void) { return m_debugger; }

  void add_device(Device *dev);
  void remove_device(Device *dev);

  void add_clock(EmuClockBase *clock) { m_sim_clock.add_clock(clock); }
  void remove_clock(EmuClockBase *clock) { m_sim_clock.remove_clock(clock); }

  /**
   * Declare an IO port with the name of @a name.
   * Ports
   * IO lines can be used to communicate between
   * different devices in a mailbox manner.
   */
  void add_ioport(const std::string &name);
  IOPort *ioport(const std::string &name);
  byte_t read_ioport(const std::string &name);
  byte_t read_ioport(IOPort *port);
  void write_ioport(const std::string &name, byte_t value);
  void write_ioport(IOPort *port, byte_t value);

  /**
   * Manage dipswitches
   */
  dipswitch_ptr add_switch(const std::string &name, const std::string &port,
                           byte_t mask, byte_t def);
  void reset_switches(void);

  void add_input(const InputSignal &signal);

  /**
   * Set an line on the device @a name.
   */
  void set_line(const std::string &name, Line line, LineState state);
  void set_line(Device *dev, Line line, LineState state);

  void add_screen(short width, short height,
                  FrameBuffer::Rotation rotation = FrameBuffer::ROT0);
  FrameBuffer *screen(void);

  short get_screen_width(void) { return m_screen_width; }

  short get_screen_height(void) { return m_screen_height; }

  void set_time(EmuTime now);
  EmuTime now(void);

  TaskScheduler *get_scheduler(void) { return &m_scheduler; }

 protected:
  void log(LogLevel level, const std::string fmt, ...);

 private:
  Device *dev(const std::string &name);

  TaskScheduler m_scheduler;
  EmuSimClock m_sim_clock;
  InputMap m_input;
  std::list<Device *> m_devs;
  std::map<std::string, dipswitch_ptr> m_switches;
  std::map<std::string, IOPort> m_ports;

  Debugger *m_debugger;
  FrameBuffer *m_screen;
  short m_screen_width;
  short m_screen_height;
  FrameBuffer::Rotation m_screen_rot;
};

typedef std::unique_ptr<Machine> machine_ptr;

typedef std::function<machine_ptr(Options *opts)> machine_create_fn;

struct MachineInformation {
  std::string name;
  std::string year;
  std::string extension;
  bool cartridge;
};

struct MachineDefinition {
  MachineDefinition(const std::string &name, const MachineInformation &info,
                    machine_create_fn fn);
  ~MachineDefinition(void);

  void add(void);

  std::string name;
  MachineInformation info;
  machine_create_fn fn;
};

class MachineLoader {
 public:
  MachineLoader();
  ~MachineLoader();

  void add_machine(struct MachineDefinition *definition);

  const struct MachineDefinition *find(const std::string &name);
  std::list<MachineDefinition *>::const_iterator start();
  std::list<MachineDefinition *>::const_iterator end();

  machine_ptr load(Options *opts);

 private:
  std::list<MachineDefinition *> m_machines;
};

MachineLoader *loader(void);

#define FORCE_UNDEFINED_SYMBOL(x) \
  extern MachineDefinition x;     \
  void *__##x##_fp = (void *)&x;
};
