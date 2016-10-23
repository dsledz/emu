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
#include "emu/input.h"

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

  virtual void line(Line line, LineState state);
  virtual void reset(void);
  virtual void initialize(void);
  virtual void execute(void);
  virtual void update(DeviceUpdate &update);

  void set_status(DeviceStatus status);
  virtual void wait_status(DeviceStatus status);
  DeviceStatus get_status(void);

  /** XXX: These should be protected */
  const std::string &name(void);
  Machine *machine(void);

 protected:
  void log(LogLevel level, const std::string fmt, ...);
  void wait(DeviceStatus status);
  void wait_for_update(void);
  void update_status(DeviceStatus status) {
    lock_mtx lock(m_mtx);
    m_status = status;
    m_cv.notify_all();
  }

 protected:
  std::string m_name;           /* ro */
  Machine *m_machine;           /* ro */
  DeviceStatus m_status;        /* (m) */
  DeviceStatus m_target_status; /* (p) */
  std::condition_variable m_cv; /* ? */
  std::mutex m_mtx;             /* ? */
  EmuDeviceChannel m_channel;   /* ? */
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

/**
 * Class to manage an external input device such as a
 * joystick or keyboard
 */
class InputDevice : public Device {
 public:
  InputDevice(Machine *machine, const std::string &name);
  virtual ~InputDevice(void);

 protected:
  InputMap m_map;
};

std::ostream &operator<<(std::ostream &os, const DeviceStatus status);

std::ostream &operator<<(std::ostream &os, const EMU::DeviceUpdate &update);
};
