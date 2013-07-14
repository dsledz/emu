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

#include <string.h>

#include <string>
#include <algorithm>
#include <future>
#include <string>
#include <type_traits>
#include <vector>
#include <list>
#include <memory>
#include <cassert>

#define a_unused __attribute((unused))

/*  _____                     _       __
 * |_   _|   _ _ __   ___  __| | ___ / _|___
 *   | || | | | '_ \ / _ \/ _` |/ _ \ |_/ __|
 *   | || |_| | |_) |  __/ (_| |  __/  _\__ \
 *   |_| \__, | .__/ \___|\__,_|\___|_| |___/
 *       |___/|_|
 */
typedef unsigned char byte_t;
typedef unsigned short word_t;
typedef unsigned short addr_t; /* XXX: Allow different address types. */
typedef std::vector<byte_t> bvec;
struct Bytes {
    byte_t l;
    byte_t h;
};
union Word {
    Word(void) = default;
    Word(word_t w): w(w) { }

    Bytes b;
    word_t w;
};

/*  ____  _ _      ___
 * | __ )(_) |_   / _ \ _ __  ___
 * |  _ \| | __| | | | | '_ \/ __|
 * | |_) | | |_  | |_| | |_) \__ \
 * |____/|_|\__|  \___/| .__/|___/
 *                     |_|
 */
template<typename T>
static inline void bit_set(byte_t &arg, T bit, bool val)
{
    auto n = static_cast<typename std::underlying_type<T>::type>(bit);
    arg &= ~(1 << n);
    arg |= (val ? (1 << n) : 0);
}

static inline void bit_set(byte_t &arg, int n, bool val)
{
    arg &= ~(1 << n);
    arg |= (val ? (1 << n) : 0);
}

static inline void bit_set(byte_t &arg, unsigned n, bool val)
{
    arg &= ~(1 << n);
    arg |= (val ? (1 << n) : 0);
}

static inline void bit_setmask(byte_t &arg, byte_t mask, byte_t val)
{
    arg = (arg & ~mask) | val;
}
#if 0
template<typename T>
static inline bool bit_isset(word_t arg, T bit)
{
    auto n = static_cast<typename std::underlying_type<T>::type>(bit);
    return (arg & (1 << n));
}

static inline bool bit_isset(word_t arg, unsigned n)
{
    return (arg & (1 << n));
}

static inline bool bit_isset(word_t arg, int n)
{
    return (arg & (1 << n));
}
#else
#define bit_isset(arg, bit) \
    (((arg) & (1 << (bit))) != 0)
#endif

typedef std::function<void ()> callback_t;
