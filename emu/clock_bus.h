#pragma once

#include <cassert>
#include "core/bits.h"
#include "emu/bus.h"
#include "emu/io.h"

namespace EMU {

enum class bus_mode { Read, Write };

template <typename _addr_type, int addr_width, typename _data_type>
class ClockedBus {
 public:
  typedef _addr_type addr_type;
  typedef _data_type data_type;

  ClockedBus(void) {}

  ~ClockedBus(void) {}

  inline void schedule_write(addr_type addr, data_type data) {
    m_addr = addr;
    m_mode = bus_mode::Write;
    m_data = data;
  }

  inline void schedule_read(addr_type addr, data_type *data_ptr) {
    m_addr = addr;
    m_mode = bus_mode::Read;
    m_data_ptr = data_ptr;
  }

  typedef std::function<bool(bus_mode, offset_t, data_type *)> io_fn;

  struct IOPort {
    IOPort(addr_type base, addr_type end, io_fn io)
        : base(base), end(end), io(io), read_only(false), raw(nullptr) {}
    IOPort(addr_type base, addr_type end, data_type *raw, bool read_only)
        : base(base), end(end), io(), read_only(read_only), raw(raw) {}

    addr_type base;
    addr_type end;
    io_fn io;
    bool read_only;
    data_type *raw;
  };

  void add(const IOPort &dev) { m_map.add(dev.base, dev.end, dev); }

  void add(addr_type base, RamDevice *dev) {
    IOPort port(base, dev->size() - 1, dev->direct(0), false);
  }

  bool io_read(addr_type addr, data_type *data) {
    schedule_read(addr, data);
    io_execute();
    return false;
  }

  bool io_write(addr_type addr, data_type data) {
    schedule_write(addr, data);
    io_execute();
    return false;
  }

  bool io_execute(void) {
    bool yield = false;
    addr_type addr = m_addr;
    IOPort &it = m_map.find(m_addr);
    addr -= it.base;
    assert(addr % sizeof(data_type) == 0);
    addr /= sizeof(data_type);
    if (it.raw != nullptr) {
      switch (m_mode) {
        case bus_mode::Write:
          if (it.read_only) throw BusError(m_addr);
          it.raw[addr] = m_data;
          break;
        case bus_mode::Read:
          *m_data_ptr = it.raw[addr];
          break;
      }
    } else {
      yield = it.io(m_mode, addr, m_data_ptr);
    }
    return yield;
  }

 private:
  MemoryMap<IOPort, addr_type, addr_width, 5> m_map;
  addr_type m_addr;
  data_type m_data;
  data_type *m_data_ptr;
  bus_mode m_mode;
};

typedef ClockedBus<uint16_t, 16, uint8_t> ClockedBus16;
};
