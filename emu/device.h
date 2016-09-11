/*
 * Copyright (c) 2013, Dan Sledz * All rights reserved.
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
#include "core/channel.h"
#include "core/exception.h"
#include "core/fiber.h"
#include "core/task.h"
#include "emu/debugger.h"
#include "emu/input.h"
#include "emu/io.h"
#include "emu/state.h"
#include "emu/timing.h"

using namespace Core;

namespace EMU {

enum class DeviceStatus {
  Off = 0,
  Halted = 1,
  Running = 2,
  Stopped = 3,
  Fault = 4,
};

class Machine;

enum class DeviceUpdateType {
  None,
  Clock,
  Status,
};

struct DeviceUpdate {
  DeviceUpdate(void) : type(DeviceUpdateType::None) {}
  DeviceUpdate(const EmuClockUpdate &clock)
      : type(DeviceUpdateType::Clock), clock(clock) {}
  DeviceUpdate(const DeviceStatus status)
      : type(DeviceUpdateType::Status), status(status) {}

  DeviceUpdateType type;
  union {
    EmuClockUpdate clock;
    DeviceStatus status;
  };
};

struct DeviceFault : public CoreException {
  DeviceFault(const std::string &device, const std::string &details = "")
      : CoreException(""), device(device) {
    std::stringstream ss;
    ss << "(" << device << "): Fault";
    if (details != "") ss << " " << details;
    msg += ss.str();
  }

  std::string device;
};

typedef Channel<DeviceUpdate> EmuDeviceChannel;
typedef std::shared_ptr<EmuDeviceChannel> EmuDeviceChannel_ptr;

#define DEFAULT_HERTZ (1000 * 1000)

/**
 * Emulation device. Specific chips implement the device class.
 */
class Device : public Debuggable {
 public:
  Device(Machine *machine, const std::string &name);
  virtual ~Device(void);

  virtual EmuClockBase *clock(void) { return NULL; }
  virtual void line(Line line, LineState state);
  virtual void reset(void);
  virtual void execute(void);
  virtual void update(DeviceUpdate &update);

  void set_status(DeviceStatus status);
  void wait_status(DeviceStatus status);
  DeviceStatus get_status(void);

  /** XXX: These should be protected */
  const std::string &name(void);
  Machine *machine(void);

  Task *task(void) { return &m_task; }

 protected:
  void log(LogLevel level, const std::string fmt, ...);
  void wait(DeviceStatus status);
  void idle(void);

 protected:
  void task_fn(void);

  std::string m_name;           /* ro */
  Machine *m_machine;           /* ro */
  DeviceStatus m_status;        /* (m) */
  DeviceStatus m_target_status; /* (p) */
  std::condition_variable m_cv; /* ? */
  std::mutex m_mtx;             /* ? */
  EmuDeviceChannel m_channel;   /* ? */
  FiberTask m_task;
};

typedef std::unique_ptr<Device> Device_ptr;

class IODevice : public Device {
 public:
  IODevice(Machine *machine, const std::string &name, size_t size);
  virtual ~IODevice(void);

  size_t size(void);

  virtual void write8(offset_t offset, byte_t arg) = 0;
  virtual byte_t read8(offset_t offset) = 0;
  virtual byte_t *direct(offset_t offset) = 0;

 protected:
  size_t m_size;
};

class ClockedDevice : public Device, public EmuClockBase {
 public:
  ClockedDevice(Machine *machine, const std::string &name, unsigned hertz);
  virtual ~ClockedDevice(void);

  virtual EmuClockBase *clock(void) { return this; }

  inline void add_icycles(unsigned cycles) { add_icycles(Cycles(cycles)); }

  inline void add_icycles(Cycles cycles) {
    if (__builtin_expect((m_channel.available() > 0), 0)) idle();
    if (__builtin_expect((m_avail < cycles), 0)) wait_icycles(cycles);
    m_avail -= cycles;
    m_used += cycles;
  }

  inline void yield(void) {
    time_advance();
    Task::yield();
  }

  virtual void time_set(EmuTime now);

 protected:
  virtual void update(DeviceUpdate &update);

 private:
  void wait_icycles(Cycles cycles);
  void time_advance(void) {
    m_current += m_used.to_time(m_hertz);
    m_used = Cycles(0);
  }

  unsigned m_hertz;
  Cycles m_used;
  Cycles m_avail;
};

class GfxDevice : public ClockedDevice {
 public:
  GfxDevice(Machine *machine, const std::string &name, unsigned hertz);
  virtual ~GfxDevice(void);

  virtual void execute(void);
  typedef std::function<void(void)> scanline_fn;

  void register_callback(unsigned scanline, scanline_fn fn);

 protected:
  unsigned m_scanline;
  std::unordered_map<unsigned, scanline_fn> m_callbacks;
};

std::ostream &operator<<(std::ostream &os, const DeviceStatus status);

std::ostream &operator<<(std::ostream &os, const EMU::DeviceUpdate &update);
};
