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
#include <stdexcept>
#include <sstream>

namespace Core {

/*  _____                    _   _
 * | ____|_  _____ ___ _ __ | |_(_) ___  _ __
 * |  _| \ \/ / __/ _ \ '_ \| __| |/ _ \| '_ \
 * | |___ >  < (_|  __/ |_) | |_| | (_) | | | |
 * |_____/_/\_\___\___| .__/ \__|_|\___/|_| |_|
 *                    |_|
 */

/**
 * General exception
 */
struct CoreException: public std::exception {
    const std::string &message() { return msg; }

    virtual const char *what(void) const noexcept { return msg.c_str(); }

protected:
    CoreException(const std::string &msg): msg(msg) { }

    std::string msg;
};

/**
 * Bus Error
 */
struct BusError: public CoreException {
    BusError(uint32_t addr): CoreException("Bus fault: "), address(addr)
    {
        std::stringstream ss;
        ss << Hex(addr);
        msg += ss.str();
    }
    uint32_t address;
};

};
