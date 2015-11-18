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

#include "emu/emu.h"

using namespace Core;
using namespace EMU;

Device::Device(Machine *machine, const std::string &name):
    Device(machine, name, DEFAULT_HERTZ)
{
}

Device::Device(Machine *machine, const std::string &name, unsigned hertz):
    m_name(name),
    m_machine(machine),
    m_status(DeviceStatus::Off),
    m_target_status(DeviceStatus::Off),
    m_hertz(hertz),
    m_avail(0),
    m_cv(),
    m_mtx(),
    m_clock(),
    m_channel()
{
    m_machine->add_device(this);
}

Device::~Device(void)
{
    m_machine->remove_device(this);
}

Machine *
Device::machine(void)
{
    return m_machine;
}

const std::string &
Device::name(void)
{
    return m_name;
}

EmuClock *
Device::clock(void)
{
    return &m_clock;
}

void
Device::task(void)
{
    wait(DeviceStatus::Running);
    DeviceStatus new_status = DeviceStatus::Running;
    try {
        task_loop();
    } catch (CoreException &e) {
        new_status = DeviceStatus::Fault;
    }
    {
        lock_mtx lock(m_mtx);
        m_status = DeviceStatus::Off;
    }
}

void
Device::line(Line line, LineState state)
{
}

void
Device::reset(void)
{
}

void
Device::task_loop(void)
{
    while (true)
        execute();
    // wait(DeviceStatus::Off);
}

void
Device::execute(void)
{
    add_icycles(5000);
}

void
Device::set_status(DeviceStatus status)
{
    DeviceUpdate update = {
        .type = DeviceUpdateType::Status,
        .status = status
    };
    m_channel.put(update);
}

DeviceStatus
Device::get_status(void)
{
    lock_mtx lock(m_mtx);
    return m_status;
}

void
Device::wait_status(DeviceStatus status)
{
    lock_mtx lock(m_mtx);
    while (status != m_status)
    {
        m_cv.wait(lock);
    }
}

void
Device::wait(DeviceStatus status)
{
    if (status == m_status)
        return;
    while (status != m_target_status)
    {
        idle();
    }
    {
        lock_mtx lock(m_mtx);
        m_status = status;
        m_cv.notify_all();
    }
}

void
Device::log(LogLevel level, const std::string fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    std::stringstream os;
    os << m_clock.now() << ":" << name() << ":" << fmt;
    Core::log.log(level, os.str(), args);
    va_end(args);
}

void
Device::idle(void)
{
    DeviceUpdate update = m_channel.get();
    switch (update.type) {
        case DeviceUpdateType::Clock:
            m_clock.update(update.clock);
            break;
        case DeviceUpdateType::Status:
            m_target_status = update.status;
            break;
    }
}

void
Device::wait_icycles(Cycles cycles)
{
    while (m_avail < cycles) {
        clock()->advance();
        EmuTime avail = clock()->wait(time_zero);
        m_avail += avail.to_cycles(Cycles(m_hertz));
    }
}

IODevice::IODevice(Machine *machine, const std::string &name, size_t size):
    Device(machine, name, DEFAULT_HERTZ),
    m_size(size)
{
}

IODevice::~IODevice(void)
{
}

size_t
IODevice::size(void)
{
    return m_size;
}

ClockedDevice::ClockedDevice(Machine *machine, const std::string &name, unsigned hertz):
    Device(machine, name, hertz)
{
}

ClockedDevice::~ClockedDevice(void)
{
}

GfxDevice::GfxDevice(Machine *machine, const std::string &name, unsigned hertz):
    ClockedDevice(machine, name, hertz), m_scanline(0)
{
}

GfxDevice::~GfxDevice(void)
{
}

void
GfxDevice::execute(void)
{
    static const Cycles m_cycles_per_scanline(384);
    while (true) {
        add_icycles(m_cycles_per_scanline);
        m_scanline = (m_scanline + 1) % 264;
        auto it = m_callbacks.find(m_scanline);
        if (it != m_callbacks.end())
            it->second();
    }
}

void
GfxDevice::register_callback(unsigned scanline, scanline_fn fn)
{
    m_callbacks.insert(make_pair(scanline, fn));
}
