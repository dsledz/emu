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
#include "core/radix.h"
#include "emu/io.h"
#include "emu/page.h"
#include "emu/ram.h"
#include "emu/rom.h"

namespace EMU {

#define WRITE_CB(cb, ...) \
  std::bind(&cb, __VA_ARGS__, std::placeholders::_1, std::placeholders::_2)
#define READ_CB(cb, ...) std::bind(&cb, __VA_ARGS__, std::placeholders::_1)

#define PAGE_FAULT_CB(cb, ...) \
  std::bind(&cb, __VA_ARGS__, std::placeholders::_1, std::placeholders::_2)

template <class Val, typename addr_type, int addr_width, int shift>
class MemoryMap {
 private:
  struct Entry {
    Entry(addr_type start, addr_type end, const Val &value)
        : start(start), end(end), value(value) {}

    bool match(addr_type addr) { return (start <= addr && addr <= end); }

    addr_type start;
    addr_type end;
    Val value;
  };

  typedef std::vector<Entry> entry_list;

 public:
  MemoryMap(void) : m_entries(1 << shift) {}
  ~MemoryMap(void) {}

  void add(addr_type key_begin, addr_type key_end, const Val &val) {
    Entry entry(key_begin, key_end, val);

    auto start = bucket(entry.start);
    auto end = bucket(entry.end);
    for (auto i = start; i <= end; i++) {
      entry_list &list = m_entries[i];
      /* Check for duplicates */
      for (auto it = list.begin(); it != list.end(); it++)
        if (it->match(key_begin)) throw Core::BusError(key_begin);
      list.push_back(entry);
    }
  }

  Val &find(addr_type key) {
    entry_list &list = m_entries[bucket(key)];
    for (auto it = list.begin(); it != list.end(); it++)
      if (it->match(key)) return it->value;
    throw Core::BusError(key);
  }

  const size_t bucket(addr_type key) {
    size_t tmp = (key >> (addr_width - shift)) & ((1 << shift) - 1);
    return tmp;
  }

 private:
  std::vector<entry_list> m_entries;
};

/**
 * Page-based bus
 */
template <typename _addr_type, typename _data_type, int addr_width, int page_shift>
class IOBus {
 public:
  typedef _addr_type addr_type;
  typedef _data_type data_type;
  typedef PageTable<addr_type, data_type, addr_width, addr_width - page_shift> page_table_type;
  typedef typename page_table_type::page_type page_type;
  typedef typename page_type::read_fn read_fn;
  typedef typename page_type::write_fn write_fn;
  typedef typename page_type::DefaultRead DefaultRead;
  typedef typename page_type::DefaultWrite DefaultWrite;
  typedef typename page_type::DataRead DataRead;
  typedef typename page_type::DataWrite DataWrite;

  IOBus() : m_page_table() {}
  ~IOBus() {}
  IOBus(const IOBus &rhs) = delete;

  inline data_type read(addr_type addr) {
    auto *page = m_page_table.page(addr);
    return page->read(addr);
  }

  inline void write(addr_type addr, data_type data) {
    auto *page = m_page_table.page(addr);
    page->write(addr, data);
  }

  void add(addr_type start, addr_type end, read_fn read, write_fn write) {
    for (page_type *p = m_page_table.page(start);
         p != m_page_table.page(end) + 1; p++) {
      p->add(start, end, read, write);
    }
  }
  void add(addr_type start, read_fn read, write_fn write) {
    add(start, start, read, write);
  }

  void add(addr_type start, data_type *ptr, size_t size, bool read_only) {
    assert(size - 1 <= (std::numeric_limits<addr_type>::max)());
    addr_type end = start + (addr_type)(size - 1);
    assert((start & (m_page_table.page_size - 1)) == 0);
    for (page_type *p = m_page_table.page(start);
         p != m_page_table.page(end) + 1; p++) {
      p->m_page = ptr;
      ptr += m_page_table.page_size;
    }
  }

  void add(addr_type start, data_type *ptr, size_t size,
           typename page_table_type::page_type::page_fault_fn handler) {
    assert(size - 1 <= (std::numeric_limits<addr_type>::max)());
    addr_type end = start + (addr_type)(size - 1);
    assert((start & (m_page_table.page_size - 1)) == 0);
    for (page_type *p = m_page_table.page(start);
         p != m_page_table.page(end) + 1; p++) {
      p->set(ptr, handler);
      ptr += m_page_table.page_size;
    }
  }
  void add(addr_type start, bvec &buf) {
    add(start, &buf.front(), buf.size(), false);
  }

  void add(addr_type start, data_type *data) {
    m_page_table.page(start)->add(start, data);
  }

  void add(addr_type start, addr_type end) {
    add(start, end, DefaultRead(), DefaultWrite());
  }

  void add(addr_type start, RamDevice *ram) {
    add(start, ram->direct(0), ram->size(), false);
  }

  void add(addr_type start, Rom *rom) {
    add(start, rom->direct(0), rom->size(), true);
  }

  void remove(addr_type start, addr_type end) {
    assert((start & (m_page_table.page_size - 1)) == 0);
    for (page_type *p = m_page_table.page(start);
         p != m_page_table.page(end) + 1; p++) {
      p->set(nullptr);
    }
  }

 private:
  page_table_type m_page_table;
};

typedef IOBus<uint16_t, uint8_t, 16, 10> AddressBus16x8;
typedef IOBus<uint32_t, byte_t, 21, 10> AddressBus21x8;
typedef IOBus<uint16_t, uint16_t, 16, 10> AddressBus16x16;
typedef IOBus<uint8_t, byte_t, 8, 4> AddressBus8x8;
typedef std::unique_ptr<AddressBus16x8> AddressBus16x8_ptr;

};
