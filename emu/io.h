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

#include <cassert>
#include "bits.h"
#include "radix.h"
#include "input.h"
#include "device.h"

namespace EMU {

typedef std::function<byte_t (addr_t)> io_read;
typedef std::function<void (addr_t, byte_t)> io_write;
struct DefaultRead {
    byte_t operator ()(addr_t addr) { return 0; }
};
struct DefaultWrite {
    void operator ()(addr_t addr, byte_t arg) { return; }
};

class IODevice
{
public:
    virtual ~IODevice(void) { }
    virtual void write8(addr_t addr, byte_t arg) = 0;
    virtual byte_t read8(addr_t addr) = 0;
    virtual addr_t size(void) = 0;
    virtual byte_t *direct(addr_t addr) = 0;
};

/**
 * I/O Port. Used to communicate to devices.
 */
struct IOPort
{
    IOPort(void):
        read(DefaultRead()),
        write(DefaultWrite()) { }
    IOPort(io_read read, io_write write):
        read(read),
        write(write) { }
    IOPort(byte_t *val):
        read([=](addr_t addr) throw() -> byte_t { return *val; }),
        write([=](addr_t addr, byte_t v) { *val = v; }) { }
    io_read read;
    io_write write;
};

/**
 * Address Bus.
 * Devices are connected via an address bus by wiring various
 * IOPorts together.
 * XXX: Using a template seems like an abuse here.
 */
template<int width=16>
class AddressBus
{
    public:
        AddressBus(void) { }
        ~AddressBus(void) { }
        AddressBus(const AddressBus &bus) = delete;

        void write8(addr_t addr, byte_t arg) {
            _map.find(addr).write(addr, arg);
        }
        byte_t read8(addr_t addr) {
            return _map.find(addr).read(addr);
        }
        addr_t size(void) {
            size_t s = (1 << width) - 1;
            return s;
        }

        void write(addr_t addr, byte_t arg) {
            _map.find(addr).write(addr, arg);
        }
        byte_t read(addr_t addr) {
            return _map.find(addr).read(addr);
        }

        /**
         * Add a single address port. Useful for registers.
         */
        void add_port(addr_t addr, const IOPort &port) {
            _map.add(addr, port);
        }

        /**
         * Add a masked port. Useful for ranges. Mask must be
         * continous. (IE 0xf0f0 is NOT a valid mask.)
         */
        void add_port(addr_t addr, addr_t mask, const IOPort &port) {
            _map.add(addr, mask, port);
        }

        void add_port(addr_t base, bvec *data) {
            IOPort port(
                [=](addr_t addr) -> byte_t {
                    assert(addr >= base);
                    addr -= base;
                    return (*data)[addr];
                },
                [=](addr_t addr, byte_t v) {
                    assert(addr >= base);
                    addr -= base;
                    (*data)[addr] = v;
                });
            addr_t mask = 0xFFFF & ~(data->size() - 1);
            _map.add(base, mask, port);
        }

        void add_port(addr_t base, IODevice *dev) {
            IOPort port(
                [=](addr_t addr) -> byte_t {
                assert(addr >= base);
                addr -= base;
                return dev->read8(addr);
                },
                [=](addr_t addr, byte_t v) {
                assert(addr >= base);
                addr -= base;
                dev->write8(addr, v);
                });
            addr_t mask = 0xFFFF & ~(dev->size() - 1);
            _map.add(base, mask, port);
        }


    private:
        RadixTree<IOPort, width> _map;
};

typedef AddressBus<16> AddressBus16;
typedef std::unique_ptr<AddressBus16> AddressBus16_ptr;

};
