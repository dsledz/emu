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

#include  <type_traits>

#include "core/bits.h"

namespace Core {

template <typename T, typename I>
static inline bool enum_isset(I arg, T bit) {
  I n = static_cast<typename std::underlying_type<T>::type>(bit);
  return (arg & (1 << n));
}

template <typename T>
class Enum {
 public:
  class Iterator {
   public:
    Iterator(int value) : m_value(value) {}

    T operator*(void)const { return (T)m_value; }

    void operator++(void) { ++m_value; }

    bool operator!=(Iterator rhs) { return m_value != rhs.m_value; }

   private:
    int m_value;
  };
};

template <typename T>
typename Enum<T>::Iterator begin(Enum<T>) {
  return typename Enum<T>::Iterator((int)T::First);
}

template <typename T>
typename Enum<T>::Iterator end(Enum<T>) {
  return typename Enum<T>::Iterator(((int)T::Last) + 1);
}

template <typename T>
class BitField {
 public:
  typedef typename std::underlying_type<T>::type field_type;

  BitField() : m_value(0) {}

  BitField(std::initializer_list<T> l) : m_value(0) {
    for (auto b : l) m_value |= static_cast<field_type>(b);
  }

  ~BitField() {}

  bool is_set(T t) { return (m_value & static_cast<field_type>(t)) != 0; }

  bool is_clear(T t) { return !is_set(t); }

  void clear(T t) { m_value &= ~static_cast<field_type>(t); }

  void set(T t) { m_value |= static_cast<field_type>(t); }

 private:
  field_type m_value;
};
}
