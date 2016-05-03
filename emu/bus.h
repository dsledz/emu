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
#include "core/bits.h"
#include "emu/io.h"
#include "core/radix.h"

namespace EMU {

template<class Val, typename addr_type, int addr_width, int shift>
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
    MemoryMap(void):
        m_entries(1 << shift)
    {
    }
    ~MemoryMap(void)
    {
    }

    void add(addr_type key, addr_type keyend, const Val &val)
    {
        Entry entry(key, keyend, val);

        auto start = bucket(entry.start);
        auto end = bucket(entry.end);
        for (auto i = start; i <= end; i++) {
            entry_list &list = m_entries[i];
            /* Check for duplicates */
            for (auto it = list.begin(); it != list.end(); it++)
                if (it->match(key))
                    throw Core::BusError(key);
            list.push_back(entry);
        }
    }

    Val &find(addr_type key)
    {
        entry_list &list = m_entries[bucket(key)];
        for (auto it = list.begin(); it != list.end(); it++)
            if (it->match(key))
                return it->value;
        assert(false);
    }

    const size_t bucket(addr_type key) {
        size_t tmp = (key >> (addr_width - shift)) & ((1 << shift) - 1);
        return tmp;
    }

private:

    std::vector<entry_list> m_entries;
};

/**
 * Data bus.
 *
 * Supports a configurable address and data size.
 * It's recommended to use one of the pre-defined typedefs
 *
 */
template<typename _addr_type, int addr_width, typename _data_type>
class DataBus
{
public:
    typedef _addr_type addr_type;
    typedef _data_type data_type;

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
    struct BufferRead {
        BufferRead(bvec *d): data(d) { }
        data_type operator ()(offset_t offset) { return (*data)[offset]; }
        bvec *data;
    };
    struct BufferWrite {
        BufferWrite(bvec *d): data(d) { }
        void operator ()(offset_t offset, data_type d) {
            (*data)[offset] = d; }
        bvec *data;
    };

    struct IOPort
    {
        IOPort(addr_type base, addr_type end):
            base(base), end(end),
            read(DefaultRead()), write(DefaultWrite()) { }
        IOPort(addr_type base, addr_type end, data_type *data):
            base(base), end(end),
            read(DataRead(data)), write(DataWrite(data)) { }
        IOPort(addr_type base, addr_type end, read_fn read, write_fn write):
            base(base), end(end), read(read), write(write) { }

        data_type io(offset_t offset, data_type data, bool wr) {
            if (wr)
                write(offset, data);
            else
                data = read(offset);
            return data;
        }

        addr_type base;
        addr_type end;
        read_fn read;
        write_fn write;
    };

    DataBus(void): _map()
    {
    }
    ~DataBus(void)
    {
    }

    data_type io(addr_type addr, data_type data, bool write)
    {
        IOPort & it = _map.find(addr);
        addr -= it.base;
        return it.io(addr, data, write);
    }
    void write(addr_type addr, data_type data)
    {
        IOPort & it = _map.find(addr);
        addr -= it.base;
        it.write(addr, data);
    }
    data_type read(addr_type addr)
    {
        IOPort & it = _map.find(addr);
        addr -= it.base;
        return it.read(addr);
    }

    void add(const IOPort &dev)
    {
        _map.add(dev.base, dev.end, dev);
    }

    void add(addr_type base, addr_type end)
    {
        IOPort port(base, end);
        add(port);
    }

    void add(addr_type base, addr_type end, read_fn read, write_fn write)
    {
        IOPort port(base, end, read, write);
        add(port);
    }

    void add(addr_type base, read_fn read, write_fn write)
    {
        IOPort port(base, base, read, write);
        add(port);
    }

    void add(addr_type base, bvec *buf)
    {
        add(base, base + buf->size() - 1, BufferRead(buf), BufferWrite(buf));
    }

    void add(addr_type base, data_type *data)
    {
        IOPort port(base, base, DataRead(data), DataWrite(data));
        add(port);
    }

private:
    MemoryMap<IOPort, addr_type, addr_width, 5> _map;

    /* XXX: The radix tree is slow */
    // RadixTree<IOPort, addr_type, addr_width> _map;
};

typedef DataBus<uint32_t, 21, byte_t> AddressBus21;
typedef DataBus<uint16_t, 16, byte_t> AddressBus16;
typedef DataBus<uint16_t, 8, byte_t>  AddressBus8;
typedef DataBus<uint16_t, 16, uint16_t> AddressBus16x16;
typedef DataBus<uint8_t, 8, byte_t> DataBus8x8;
typedef std::unique_ptr<AddressBus16> AddressBus16_ptr;

#define WRITE_CB(cb, ...) std::bind(&cb, __VA_ARGS__, std::placeholders::_1, \
                               std::placeholders::_2)
#define READ_CB(cb, ...) std::bind(&cb, __VA_ARGS__, std::placeholders::_1)


};

