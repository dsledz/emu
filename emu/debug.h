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
    Debug(std::ostream &os): _os(os), _level(LogLevel::Info) {
    }
    ~Debug(void) { }

    void set_level(LogLevel level) {
        _level = level;
    }
    void set_level(const std::string &level) {
        if (level == "trace") {
            _level = LogLevel::Trace;
        } else if (level == "debug") {
            _level = LogLevel::Debug;
        } else if (level == "info") {
            _level = LogLevel::Info;
        } else if (level == "error") {
            _level = LogLevel::Error;
        } else {
            throw DebugException("Unknown level " + level);
        }
    }
    inline bool enabled(LogLevel level) {
        return (_level <= level);
    }

    void trace(const std::string &fmt) {
        if (_level <= LogLevel::Trace)
            _os << fmt << "\n";
    }
    void debug(const std::string &fmt) {
        if (_level <= LogLevel::Debug)
            _os << fmt << "\n";
    }
    void info(const std::string &fmt) {
        if (_level <= LogLevel::Info)
            _os << fmt << "\n";
    }
    void error(const std::string &fmt) {
        if (_level <= LogLevel::Error)
            _os << fmt << "\n";
    }

private:
    std::ostream &_os;
    LogLevel _level;
};

extern Debug log;

#define IF_LOG(lvl) \
    for (bool once=true; once && EMU::log.enabled(LogLevel::lvl); once=false)

#define TRACE(fmt, args...) \
    EMU::log.trace(fmt);

#define DBG(fmt, args...) \
    EMU::log.debug(fmt);

#define INFO(fmt, args...) \
    EMU::log.info(fmt);

#define ERROR(fmt, args...) \
    EMU::log.error(fmt);

};
