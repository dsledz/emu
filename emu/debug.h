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

#include "emu/bits.h"
#include <iostream>
#include <iomanip>
#include <string>

namespace EMU {

enum class LogLevel {
    Trace,
    Debug,
    Info,
    Critical,
    Error,
    Fatal
};

struct DebugException: public EmuException {
    DebugException(const std::string &msg):
        EmuException(msg) { }
};

/**
 * XXX: I'd love to not have to write my own logging, but that
 * doesn't seem possible right now.
 */
class Debug {
public:
    Debug(std::ostream &os): m_os(os), m_level(LogLevel::Info) {
    }
    ~Debug(void) { }

    void set_level(LogLevel level) {
        m_level = level;
    }
    void set_level(const std::string &level) {
        if (level == "trace") {
            m_level = LogLevel::Trace;
        } else if (level == "debug") {
            m_level = LogLevel::Debug;
        } else if (level == "info") {
            m_level = LogLevel::Info;
        } else if (level == "error") {
            m_level = LogLevel::Error;
        } else {
            throw DebugException("Unknown level " + level);
        }
    }
    inline bool enabled(LogLevel level) {
        return (m_level <= level);
    }

    void trace(const std::string &fmt) {
        if (m_level <= LogLevel::Trace) {
            std::lock_guard<std::mutex> lock(m_mtx);
            m_os << fmt << "\n";
        }
    }
    void debug(const std::string &fmt) {
        if (m_level <= LogLevel::Debug) {
            std::lock_guard<std::mutex> lock(m_mtx);
            m_os << fmt << "\n";
        }
    }
    void info(const std::string &fmt) {
        if (m_level <= LogLevel::Info) {
            std::lock_guard<std::mutex> lock(m_mtx);
            m_os << fmt << "\n";
        }
    }
    void error(const std::string &fmt) {
        if (m_level <= LogLevel::Error) {
            std::lock_guard<std::mutex> lock(m_mtx);
            m_os << fmt << "\n";
        }
    }
    void log(LogLevel level, const std::string &fmt, va_list args)
    {
        if (m_level <= level) {
            std::lock_guard<std::mutex> lock(m_mtx);
            m_os << fmt << "\n";
        }
    }

private:
    std::mutex m_mtx;
    std::ostream &m_os;
    LogLevel m_level;
};

extern Debug log;

#define IF_LOG(lvl) \
    for (bool once=true; once && EMU::log.enabled(LogLevel::lvl); once=false)

#define LOG_TRACE(fmt, args...) \
    for (bool once=true; once && EMU::log.enabled(EMU::LogLevel::Trace); once=false) \
        EMU::log.trace(fmt);

#define LOG_DEBUG(fmt, args...) \
    for (bool once=true; once && EMU::log.enabled(EMU::LogLevel::Debug); once=false) \
        EMU::log.debug(fmt);

#define LOG_INFO(fmt, args...) \
    for (bool once=true; once && EMU::log.enabled(EMU::LogLevel::Info); once=false) \
        EMU::log.info(fmt);

#define LOG_ERROR(fmt, args...) \
    for (bool once=true; once && EMU::log.enabled(EMU::LogLevel::Error); once=false) \
        EMU::log.error(fmt);

#define DEVICE_TRACE(fmt, ...) \
    for (bool once=true; once && EMU::log.enabled(EMU::LogLevel::Trace); once=false) \
        EMU::Device::log(EMU::LogLevel::Trace, fmt, ##__VA_ARGS__)

#define DEVICE_DEBUG(fmt, ...) \
    for (bool once=true; once && EMU::log.enabled(EMU::LogLevel::Debug); once=false) \
        EMU::Device::log(EMU::LogLevel::Debug, fmt, ##__VA_ARGS__)

#define DEVICE_INFO(fmt, ...) \
    for (bool once=true; once && EMU::log.enabled(EMU::LogLevel::Info); once=false) \
        EMU::Device::log(EMU::LogLevel::Info, fmt, ##__VA_ARGS__)

#define DEVICE_ERROR(fmt, ...) \
    for (bool once=true; once && EMU::log.enabled(EMU::LogLevel::Error); once=false) \
        EMU::Device::log(EMU::LogLevel::Error, fmt, ##__VA_ARGS__)

#define MACHINE_TRACE(fmt, ...) \
    for (bool once=true; once && EMU::log.enabled(EMU::LogLevel::Trace); once=false) \
        EMU::Machine::log(EMU::LogLevel::Trace, fmt, ##__VA_ARGS__)

#define MACHINE_DEBUG(fmt, ...) \
    for (bool once=true; once && EMU::log.enabled(EMU::LogLevel::Debug); once=false) \
        EMU::Machine::log(EMU::LogLevel::Debug, fmt, ##__VA_ARGS__)

#define MACHINE_INFO(fmt, ...) \
    for (bool once=true; once && EMU::log.enabled(EMU::LogLevel::Info); once=false) \
        EMU::Machine::log(EMU::LogLevel::Info, fmt, ##__VA_ARGS__)

#define MACHINE_ERROR(fmt, ...) \
    for (bool once=true; once && EMU::log.enabled(EMU::LogLevel::Error); once=false) \
        EMU::Machine::log(EMU::LogLevel::Error, fmt, ##__VA_ARGS__)

};
