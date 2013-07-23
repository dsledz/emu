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

namespace EMU {

/**
 * Device lines. Each line represents a possible connection between two
 * devices. Lines have no fixed source, but are always sent to a single
 * device. Shared lines (like reset) must be handled manually.
 *
 * The Lines are laid out to make it easy to map into one of the pre-defined
 * interrupt lines.
 */
enum class Line {
    INT0  =  0,
    INT1  =  1,
    INT2  =  2,
    INT3  =  3,
    INT4  =  4,
    INT5  =  5,
    INT6  =  6,
    INT7  =  7,

    RESET =  8,
    WAIT  =  9,
    NMI   = 10,

    /* Address lines */
    A00   = 100,
    A01   = 101,
    A02   = 102,
    A03   = 103,
    A04   = 104,
    A05   = 105,
    A06   = 106,
    A07   = 107,
    A08   = 108,
    A09   = 109,
    A10   = 110,
    A11   = 111,
    A12   = 112,
    A13   = 113,
    A14   = 114,
    A15   = 115,

    /* Data lines */
    D00   = 200,
    D01   = 201,
    D02   = 202,
    D03   = 203,
    D04   = 204,
    D05   = 205,
    D06   = 206,
    D07   = 207,
    D08   = 208,
    D09   = 209,
    D10   = 210,
    D11   = 211,
    D12   = 212,
    D13   = 213,
    D14   = 214,
    D15   = 215,
};

static inline Line make_irq_line(unsigned i) {
    return Line(i);
}

static inline Line make_addr_line(unsigned i) {
    return Line(100 + i);
}

static inline Line make_data_line(unsigned i) {
    return Line(200 + i);
}

/**
 * New state of a line. The state usually represents the logical level
 * of the line.
 * For example, if the irq is active low, you'll still Assert when
 * you want to trigger an interrupt.
 */
enum class LineState {
    Clear  = 0,  /**< Line should be cleared. */
    Assert = 1,  /**< Line should be asserted. */
    Pulse  = 2,  /**< Line should be pulsed. */
};

/**
 * An external port, capable of holding data. Ports are used to communicate
 * between the machine and the outside world.
 * XXX: Sometimes ports are abused for inter-device communication.
 */
struct IOPort {
    IOPort(void) = default;

    byte_t value;
};

class IODevice
{
public:
    virtual ~IODevice(void) { }
    virtual void write8(offset_t offset, byte_t arg) = 0;
    virtual byte_t read8(offset_t offset) = 0;
    virtual size_t size(void) = 0;
    virtual byte_t *direct(offset_t offset) = 0;
};

template<class Val, typename addr_type>
class MemoryMap
{
private:
    struct Entry {
        Entry(addr_type start, addr_type end, const Val &value):
            start(start), end(end), value(value) { }

        bool match(addr_type addr) {
            return (start <= addr && addr <= end);
        }

        addr_type start;
        addr_type end;
        Val value;
    };

    typedef std::vector<Entry> entry_list;

public:
    MemoryMap(void)
    {
        _entries.resize(16);
    }
    ~MemoryMap(void)
    {
    }

    void add(addr_type key, addr_type keymask, const Val &val)
    {
        Entry entry(key, key | ~keymask, val);

        const int start = (entry.start & 0xF000) >> 12;
        const int end = (entry.end & 0xF000) >> 12;
        for (int i = start; i <= end; i++)
            _entries[i].push_back(entry);
    }

    Val &find(addr_type key)
    {
        const int bucket = (key & 0xF000) >> 12;
        entry_list &list = _entries[bucket];
        for (auto it = list.begin(); it != list.end(); it++)
            if (it->match(key))
                return it->value;
        throw EMU::BusError(key);
    }

    void add(addr_type key, const Val &val) {
        add(key, 0xffff, val);
    }

private:

    std::vector<entry_list> _entries;
};

/**
 * Data bus.
 *
 * Supports a configurable address and data size.
 * It's recommended to use on of the pre-defined typedefs
 *
 */
template<typename addr_type, int addr_width, typename data_type>
class DataBus
{
public:
    typedef std::function<data_type (offset_t)> read_fn;
    typedef std::function<void (offset_t, data_type)> write_fn;
    struct DefaultRead {
        data_type operator ()(offset_t offset) { return 0; }
    };
    struct DefaultWrite {
        void operator ()(offset_t offset, data_type data) { return; }
    };
    struct DataRead {
        DataRead(data_type *d): data(d) { }
        data_type operator ()(offset_t offset) { return *data; }
        data_type *data;
    };
    struct DataWrite {
        DataWrite(data_type *d): data(d) { }
        void operator ()(offset_t offset, data_type d) { *data = d; }
        data_type *data;
    };

    struct IOPort
    {
        IOPort(addr_type base, addr_type mask):
            base(base), mask(mask),
            read(DefaultRead()), write(DefaultWrite()) { }
        IOPort(addr_type base, addr_type mask, data_type *data):
            base(base), mask(mask),
            read(DataRead(data)), write(DataWrite(data)) { }
        IOPort(addr_type base, addr_type mask, read_fn read, write_fn write):
            base(base), mask(mask), read(read), write(write) { }

        addr_type base;
        addr_type mask;
        read_fn read;
        write_fn write;
    };

    DataBus(void): _map()
    {
    }
    ~DataBus(void)
    {
    }

    void write(addr_type addr, data_type data)
    {
        auto it = _map.find(addr);
        addr -= it.base;
        it.write(addr, data);
    }
    data_type read(addr_type addr)
    {
        auto it = _map.find(addr);
        addr -= it.base;
        return it.read(addr);
    }

    void add(const IOPort &dev)
    {
        _map.add(dev.base, dev.mask, dev);
    }

    void add(addr_type base, addr_type mask)
    {
        IOPort port(base, mask);
        add(port);
    }

    void add(addr_type base, addr_type mask, read_fn read, write_fn write)
    {
        IOPort port(base, mask, read, write);
        add(port);
    }

    /* XXX: IODevice is hardcoded with an 16 bit addr, 8 bit bus. */
    void add(addr_type base, addr_type mask, IODevice *dev)
    {
        throw DeviceFault("DataBus");
    }

    void add(addr_type base, data_type *data)
    {
        IOPort port(base, 0xFFFF, data);
        add(port);
    }

private:
    MemoryMap<IOPort, addr_type> _map;

    /* XXX: The radix tree is slow */
    // RadixTree<IOPort, addr_type, addr_width> _map;
};

typedef DataBus<addr_t, 16, byte_t> AddressBus16;
typedef DataBus<addr_t, 8, byte_t>  AddressBus8;
typedef std::unique_ptr<AddressBus16> AddressBus16_ptr;

template<> inline void
DataBus<addr_t, 16, byte_t>::add(addr_t base, addr_t mask, IODevice *dev)
{
    IOPort port(base, mask,
        [=](offset_t offset) -> byte_t {
            return dev->read8(offset);
        },
        [=](offset_t offset, byte_t data) {
            dev->write8(offset, data);
        });
    add(port);
}

};
