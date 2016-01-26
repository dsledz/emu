#pragma once

#include <cassert>
#include "core/bits.h"
#include "emu/io.h"
#include "core/radix.h"

namespace EMU {

template<typename _addr_type, int addr_width, typename _data_type>
class ClockedBus
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

    ClockedBus(void): _map()
    {
    }
    ~ClockedBus(void)
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
    MemoryMap<IOPort, addr_type, addr_width> _map;

    /* XXX: The radix tree is slow */
    // RadixTree<IOPort, addr_type, addr_width> _map;
};

};
