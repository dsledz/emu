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

#include "emu/bits.h"
#include "emu/state.h"
#include "emu/io.h"
#include "emu/input.h"
#include "emu/timing.h"

namespace EMU {

enum class DeviceStatus {
    Off      = 0,
    Halted   = 1,
    Running  = 2,
    Stopped  = 3,
    Fault    = 4,
};

class Machine;

/**
 * Emulation device. Specific chips implement the device class.
 */
class Device {
public:
    Device(Machine *machine, const std::string &name);
    virtual ~Device(void);

    Machine *machine(void);
    const std::string &name(void);

    void task(void);

    /**
     * Return the local clock.
     */
    virtual EmuClock *clock(void);

    /**
     * Signal one of the external lines.
     */
    virtual void line(Line line, LineState state);

    virtual void reset(void);

    virtual void task_loop(void);

    virtual void set_status(DeviceStatus status);
    virtual void wait_status(DeviceStatus status);
    virtual DeviceStatus get_status(void);

protected:

    void log(LogLevel level, const std::string &fmt, ...);

    void wait(DeviceStatus status);

    std::string              m_name;
    Machine                 *m_machine;
    DeviceStatus             m_status;
    DeviceStatus             m_target_status;
    std::condition_variable  m_cv;
    std::mutex               m_mtx;
};

typedef std::unique_ptr<Device> device_ptr;

class IODevice: public Device {
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

class ClockedDevice: public Device {
public:
    ClockedDevice(Machine *machine, const std::string &name, unsigned hertz);
    virtual ~ClockedDevice(void);

    virtual void execute(void) = 0;

    virtual EmuClock *clock(void);

    virtual void task_loop(void);

    void add_icycles(unsigned cycles) {
        const Cycles c(cycles);
        if (__builtin_expect((m_avail < c), 0))
            wait_icycles(c);
        m_avail -= c;
    }

    void add_icycles(Cycles cycles) {
        if (__builtin_expect((m_avail < cycles), 0))
            wait_icycles(cycles);
        m_avail -= cycles;
    }

protected:
    void wait_icycles(Cycles cycles);

    EmuClock m_clock;
    unsigned m_hertz;
    Cycles m_avail;
    Cycles m_icycles;
};

class GfxDevice: public ClockedDevice {
public:
    GfxDevice(Machine *machine, const std::string &name, unsigned hertz);
    virtual ~GfxDevice(void);

    virtual void execute(void);
    typedef std::function<void (void)> scanline_fn;

    void register_callback(unsigned scanline, scanline_fn fn);

protected:

    unsigned m_scanline;
    std::unordered_map<unsigned, scanline_fn> m_callbacks;
};

};

