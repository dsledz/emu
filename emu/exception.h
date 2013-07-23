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

#include "bits.h"
#include <stdexcept>
#include <sstream>

namespace EMU {

/*  _____                    _   _
 * | ____|_  _____ ___ _ __ | |_(_) ___  _ __
 * |  _| \ \/ / __/ _ \ '_ \| __| |/ _ \| '_ \
 * | |___ >  < (_|  __/ |_) | |_| | (_) | | | |
 * |_____/_/\_\___\___| .__/ \__|_|\___/|_| |_|
 *                    |_|
 */

/**
 * General Emulator exception
 */
struct EmuException: public std::exception {
    const std::string &message() { return msg; }

protected:
    EmuException(const std::string &msg): msg(msg) { }

    std::string msg;
};

struct DeviceFault: public EmuException {
    DeviceFault(const std::string &device, const std::string &details=""):
        EmuException(""),
        device(device)
    {
        std::stringstream ss;
        ss << "(" << device << "): Fault";
        if (details != "")
            ss << " " << details;
        msg += ss.str();
    }

    std::string device;
};

/**
 * CPU Fault
 */
struct CpuFault: public DeviceFault {
    CpuFault(const std::string &cpu, const std::string &msg):
        DeviceFault(cpu, msg) { }
};

/**
 * CPU Opcode error
 */
struct CpuOpcodeFault: public CpuFault {
    CpuOpcodeFault(const std::string &cpu, unsigned opcode, unsigned addr):
        CpuFault(cpu, "invalid opcode"),
        op(opcode),
        pc(addr)
    {
        std::stringstream ss;
        ss << " " << Hex(opcode);
        if (addr != 0)
            ss << " at address " << Hex(addr);
        msg += ss.str();
    }
    unsigned op;
    unsigned pc;
};

struct CpuRegisterFault: public CpuFault {
    CpuRegisterFault(const std::string &cpu, unsigned index):
        CpuFault(cpu, "invalid regsiter")
    {
        std::stringstream ss;
        ss << " " << Hex(index);
        msg += ss.str();
    }
};

struct CpuFeatureFault: public CpuFault {
    CpuFeatureFault(const std::string &cpu, const std::string &feature=""):
        CpuFault(cpu, "unsupported feature")
    {
        std::stringstream ss;
        if (feature != "") {
            ss << " " << feature;
            msg += ss.str();
        }
    }
};

/**
 * Bus Error
 */
struct BusError: public EmuException {
    BusError(addr_t addr): EmuException("Bus fault"), address(addr)
    {
        std::stringstream ss;
        ss << ": " << Hex(addr);
        msg = ss.str();
    }
    addr_t address;
};

};
