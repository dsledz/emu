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

#ifndef WIN32
#include <sys/cdefs.h>
#include <sys/mman.h>
#else
#define _CRT_SECURE_NO_WARNINGS
#include <windows.h>
#define __builtin_expect(expr, val) expr
#endif

#include <string.h>

#include <stdint.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <functional>
#include <future>
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <sstream>
#include <type_traits>
#include <unordered_map>
#include <vector>

#ifdef WIN32
#define a_used
#define a_unused
#define a_const
#define likely(x) x
#define unlikely(x) x
#else
#define a_used __attribute__((used))
#define a_unused __attribute((unused))
#define a_const __attribute__((const))
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#endif

/*  _____                     _       __
 * |_   _|   _ _ __   ___  __| | ___ / _|___
 *   | || | | | '_ \ / _ \/ _` |/ _ \ |_/ __|
 *   | || |_| | |_) |  __/ (_| |  __/  _\__ \
 *   |_| \__, | .__/ \___|\__,_|\___|_| |___/
 *       |___/|_|
 */
typedef unsigned char byte_t;
typedef unsigned short addr_t; /* XXX: Allow different address types. */
typedef unsigned offset_t;
typedef std::vector<byte_t> bvec;
typedef std::vector<uint16_t> u16vec;
typedef unsigned char reg8_t;

union reg16_t {
  reg16_t(void) = default;
  reg16_t(uint16_t d) : d(d) {}
  reg16_t(reg8_t h, reg8_t l) : b(h, l) {}

  struct Bits {
    Bits(void) = default;
    Bits(reg8_t h, reg8_t l) : l(l), h(h) {}
    reg8_t l;
    reg8_t h;
  } b;
  uint16_t d = 0;
};
union reg32_t {
  reg32_t(void) = default;
  reg32_t(uint32_t d) : d(d) {}

  struct Bits {
    reg16_t l;
    reg16_t h;
  } b;
  uint16_t d = 0;
};

/*  ____  _ _      ___
 * | __ )(_) |_   / _ \ _ __  ___
 * |  _ \| | __| | | | | '_ \/ __|
 * | |_) | | |_  | |_| | |_) \__ \
 * |____/|_|\__|  \___/| .__/|___/
 *                     |_|
 */
template <typename A, typename T>
static inline void bit_set(A &arg, T bit, bool val) {
  auto n = static_cast<typename std::underlying_type<T>::type>(bit);
  arg &= ~(1 << n);
  arg |= (val ? (1 << n) : 0);
}

template <typename A>
static inline void bit_set(A &arg, int n, bool val) {
  arg &= ~(1 << n);
  arg |= (val ? (1 << n) : 0);
}

template <typename A>
static inline void bit_set(A &arg, unsigned n, bool val) {
  arg &= ~(1 << n);
  arg |= (val ? (1 << n) : 0);
}

static inline void bit_setmask(byte_t &arg, byte_t mask, byte_t val) {
  arg = (arg & ~mask) | val;
}
#if 0
template<typename T>
static inline bool bit_isset(uint16_t arg, T bit)
{
    auto n = static_cast<typename std::underlying_type<T>::type>(bit);
    return (arg & (1 << n));
}

static inline bool bit_isset(uint16_t arg, unsigned n)
{
    return (arg & (1 << n));
}

static inline bool bit_isset(uint16_t arg, int n)
{
    return (arg & (1 << n));
}
#else
#define bit_isset(arg, bit) (((arg) & (1 << (bit))) != 0)
#endif

#define bit_toggle(arg1, arg2, bit)                         \
  ((bit_isset((arg1), (bit)) ^ bit_isset((arg2), (bit))) && \
   bit_isset((arg1), (bit)))

typedef std::function<void()> callback_t;

template <typename T>
static inline uint8_t a_const val(T t) {
  return static_cast<typename std::underlying_type<T>::type>(t);
}

/* XXX: Find a better place for this */

#include <iomanip>
#include <iostream>
class Hex {
 public:
  Hex(bool arg) : v(arg), w(2) {}
  Hex(byte_t arg) : v(arg), w(2) {}
  Hex(char arg) : v(arg), w(2) {}
  Hex(uint16_t arg) : v(arg), w(4) {}
  Hex(unsigned arg) : v(arg), w(4) {}
  Hex(size_t arg) : v((unsigned)arg), w(8) {}
  Hex(int arg) : v(arg), w(2) {}
  Hex(off_t arg) : v((unsigned)arg), w(8) {}
  Hex(reg16_t arg) : v(arg.d), w(4) {}
  Hex(reg32_t arg) : v(arg.d), w(2) {}
  unsigned v;
  unsigned w;
};

static inline std::ostream &operator<<(std::ostream &os, const Hex &obj) {
  os << "0x" << std::hex << std::setfill('0') << std::right << std::setw(obj.w)
     << obj.v;
  return os;
}

static inline void set_byte(reg16_t &reg, int b, byte_t value) {
  if (b == 0)
    reg.b.l = value;
  else
    reg.b.h = value;
}

static inline byte_t get_byte(reg16_t &reg, int b) {
  if (b == 0)
    return reg.b.l;
  else
    return reg.b.h;
}
