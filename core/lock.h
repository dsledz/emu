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
/*
 * Locking primatives
 */
#pragma once

#include "core/bits.h"

#ifdef LINUX
#include "sched.h"
#endif

namespace Core {

class spin_lock {
 public:
  spin_lock() : m_lock(0) {}
  ~spin_lock() { assert(m_lock == 0); }

  inline void lock() {
    // TODO: thread for debugging
    uint64_t unlocked = UNLOCKED;
    while (!std::atomic_compare_exchange_strong(&m_lock, &unlocked, LOCKED)) {
#ifdef LINUX
      sched_yield();
#endif
    }
  }

  inline void unlock() {
    uint64_t locked = LOCKED;
    std::atomic_compare_exchange_strong(&m_lock, &locked, UNLOCKED);
  }

 private:
  static const uint64_t UNLOCKED = 0;
  static const uint64_t LOCKED = 1;
  std::atomic<uint64_t> m_lock;
};

template <typename mtx_type>
class unlock_guard {
 public:
  unlock_guard(mtx_type& m) : m_mtx(m) { m_mtx.unlock(); }
  ~unlock_guard(void) { m_mtx.lock(); }

 private:
  mtx_type& m_mtx;
};

template <typename mtx_type>
class lock_guard {
 public:
  lock_guard(mtx_type& m) : m_mtx(m) { m_mtx.lock(); }

  ~lock_guard(void) { m_mtx.unlock(); }

  void wait(std::condition_variable& cv) {
    std::unique_lock<mtx_type> lock(m_mtx, std::adopt_lock);
    cv.wait(lock);
    lock.release();
  }

 private:
  mtx_type& m_mtx;
};

typedef lock_guard<std::mutex> lock_mtx;
typedef unlock_guard<std::mutex> unlock_mtx;
};
