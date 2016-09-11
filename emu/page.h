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
#include "core/exception.h"
#include "core/enum.h"

using namespace Core;

namespace EMU {

enum class PageFlags {
  None = 0x00,
  Write = 0x01,
  Read = 0x02,
  Auto = 0x04,
};

struct PageException : public CoreException {
  PageException(off_t off) : CoreException("Page Fault: "), offset(off) {
    std::stringstream ss;
    ss << Hex(off);
    msg += ss.str();
  }
  off_t offset;
};

template <typename _addr_type, typename _data_type>
void NoopWrite(_addr_type offset, _data_type data) {}

template <typename _addr_type, typename _data_type>
_data_type ReadWrite(_addr_type offset) {
  return _data_type();
}

template <typename _addr_type, typename _data_type, unsigned page_size>
class Page {
 public:
  typedef _addr_type addr_type;
  typedef _data_type data_type;

  typedef std::function<data_type(offset_t)> read_fn;
  typedef std::function<void(offset_t, data_type)> write_fn;
  typedef std::function<void(offset_t, data_type)> page_fault_fn;
  struct DefaultPageFault {
    void operator()(offset_t offset, data_type data) {
      throw PageException(offset);
    }
  };
  struct DefaultRead {
    data_type operator()(offset_t offset) { return 0; }
  };
  struct DefaultWrite {
    void operator()(offset_t offset, data_type data) { return; }
  };
  struct DataRead {
    DataRead(data_type *d) : data(d) {}
    data_type operator()(offset_t offset) { return *data; }
    data_type *data;
  };
  struct DataWrite {
    DataWrite(data_type *d) : data(d) {}
    void operator()(offset_t offset, data_type d) { *data = d; }
    data_type *data;
  };

  struct IOPort {
    IOPort(addr_type base, addr_type end)
        : base(base % page_size),
          end(end % page_size),
          read(DefaultRead()),
          write(DefaultWrite()) {}
    IOPort(addr_type base, addr_type end, data_type *data)
        : base(base % page_size),
          end(end % page_size),
          read(DataRead(data)),
          write(DataWrite(data)) {}
    IOPort(addr_type base, addr_type end, read_fn read, write_fn write)
        : base(base % page_size),
          end(end % page_size),
          read(read),
          write(write) {}

    bool match(addr_type addr) { return (base <= addr && addr <= end); }

    addr_type base;
    addr_type end;
    read_fn read;
    write_fn write;
  };

 public:
  Page(void)
      : m_page_fault(DefaultPageFault()),
        m_port_list(),
        m_page(nullptr),
        m_page_data(nullptr),
        m_flags({PageFlags::Write, PageFlags::Read}) {}

  Page(std::initializer_list<PageFlags> flags)
      : m_page_fault(DefaultPageFault()),
        m_port_list(),
        m_page(nullptr),
        m_page_data(nullptr),
        m_flags(flags) {
    if (m_flags.is_set(PageFlags::Auto)) {
      m_page_data = new data_type[page_size];
      m_page = m_page_data;
    }
  }
  Page(const Page &rhs) = delete;
  ~Page(void) { delete m_page_data; }

  void add(addr_type start, data_type *data) {
    IOPort port(start, start, data);
    for (auto it = m_port_list.begin(); it != m_port_list.end(); it++) {
      assert(m_page == NULL);
      if (it->match(start)) throw Core::BusError(start);
    }
    m_port_list.push_back(port);
  }

  void add(addr_type start, addr_type end, read_fn read, write_fn write) {
    IOPort port(start, end, read, write);
    for (auto it = m_port_list.begin(); it != m_port_list.end(); it++)
      if (it->match(start)) throw Core::BusError(start);
    m_port_list.push_back(port);
  }

  inline void set(data_type *ptr, page_fault_fn handler) {
    m_page = ptr;
    m_page_fault = handler;
    m_flags.clear(PageFlags::Write);
  }

  inline void set(data_type *ptr) { m_page = ptr; }

  inline void write(addr_type addr, data_type data) {
    const offset_t offset = addr % page_size;
    auto it = m_port_list.begin();
    for (; it != m_port_list.end() && !it->match(offset); it++) {
    }
    if (it != m_port_list.end())
      it->write(offset, data);
    else if (unlikely(m_flags.is_clear(PageFlags::Write)))
      m_page_fault(addr, data);
    else
      m_page[offset] = data;
  }

  inline data_type read(addr_type addr) {
    const offset_t offset = addr % page_size;
    auto it = m_port_list.begin();
    for (; it != m_port_list.end() && !it->match(offset); it++) {
    }
    if (it != m_port_list.end())
      return it->read(offset);
    else
      return m_page[offset];
  }

  page_fault_fn m_page_fault;
  std::vector<IOPort> m_port_list;
  data_type *m_page;
  data_type *m_page_data;
  BitField<PageFlags> m_flags;
};

template <typename _addr_type, typename _data_type, int addr_width, int shift>
class PageTable {
 public:
  typedef _addr_type addr_type;
  typedef _data_type data_type;
  static const int page_size = 1 << (addr_width - shift);
  static const int page_count = 1 << shift;
  static const int bucket_shift = (addr_width - shift);
  static const int bucket_mask = (1 << shift) - 1;
  typedef Page<addr_type, data_type, page_size> page_type;

  PageTable() : m_pages() {}
  ~PageTable() {}
  PageTable(const PageTable &rhs) = delete;

  inline page_type *page(offset_t offset) {
    assert(((offset >> bucket_shift) & ~bucket_mask) == 0);
    return &m_pages[offset >> bucket_shift];
  }

  page_type m_pages[page_count];
};
};
