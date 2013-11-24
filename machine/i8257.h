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

#include "emu/emu.h"

using namespace EMU;

/* XXX: This seems wrong */
namespace EMU {

class I8257: public ClockedDevice
{
public:
    I8257(Machine *machine, const std::string &name, unsigned hertz, AddressBus16 *bus);
    virtual ~I8257(void);

    virtual void execute(void);

    void write_cb(offset_t offset, byte_t value);
    byte_t read_cb(offset_t offset);

    void dma(void);

private:
    struct Channel {
        reg16_t    address;
        reg16_t    count;
    } _channels[4];

    union {
        struct {
            uint8_t autoload:1;
            uint8_t tcstop:1;
            uint8_t extwr:1;
            uint8_t rprio:1;
            uint8_t dma3:1;
            uint8_t dma2:1;
            uint8_t dma1:1;
            uint8_t dma0:1;
        } mode;
        uint8_t val;
    } _mode;

    int _last_channel;
    int _flip_flop;
    byte_t _status;
    AddressBus16 *_bus;
};

typedef std::unique_ptr<I8257> I8257_ptr;
};
