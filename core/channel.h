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

#include "core/bits.h"
#include "core/debug.h"
#include "core/exception.h"
#include "core/lock.h"
#include "core/ring.h"
#include "core/task.h"

namespace Core {

/**
 * Type-safe object channel.
 */
template <class object_t>
class Channel {
 public:
  typedef Core::spin_lock lock_t;

  Channel(void) : m_mtx(), m_waiting(), m_objects() {}
  ~Channel(void) {}
  Channel(const Channel &ch) = delete;

  /**
   * Place an object in the channel.
   */
  void put(object_t obj) {
    lock_guard<lock_t> lock(m_mtx);
    m_objects.push_back(obj);
    if (m_waiting) {
      m_waiting->wake();
    }
  }

  /**
   * Remove an object from the channel, blocking if no object
   * exists.
   */
  object_t get(void) {
    std::unique_lock<lock_t> lock(m_mtx);
    assert(m_waiting == NULL);
    Task *task = Thread::cur_task();
    while (m_objects.empty()) {
      m_waiting = task;
      lock.unlock();
      task->suspend();
      lock.lock();
    }
    object_t obj = m_objects.front();
    m_objects.pop_front();
    m_waiting = NULL;
    return obj;
  }

  /**
   * Remove an object from the channel, returning an empty object
   * if none exist.
   */
  object_t try_get(void) {
    object_t obj = object_t();
    std::unique_lock<lock_t> lock(m_mtx);
    if (!m_objects.empty()) {
      obj = m_objects.front();
      m_objects.pop();
    }
    return obj;
  }

  size_t available(void) { return m_objects.size(); }

 private:
  lock_t m_mtx;
  Task *m_waiting;
  RingBuffer<object_t, 64> m_objects;
};

/**
 * Type-safe object channel.
 */
template <class object_t>
class WaitChannel {
 public:
  WaitChannel(void) : m_mtx(), m_cv(), m_objects(), m_closed(false) {}
  ~WaitChannel(void) {}
  WaitChannel(const WaitChannel &ch) = delete;

  /**
   * Place an object in the channel.
   */
  void put(object_t obj) {
    lock_mtx lock(m_mtx);
    m_objects.push_back(obj);
    m_cv.notify_all();
  }

  void close(void) {
    lock_mtx lock(m_mtx);
    m_closed = true;
    m_cv.notify_all();
  }

  /**
   * Remove an object from the channel, blocking if no object
   * exists.
   */
  object_t get(void) {
    std::unique_lock<std::mutex> lock(m_mtx);
    while (m_objects.empty()) {
      if (m_closed) throw TaskCanceled("Channel closed");
      m_cv.wait(lock);
    }
    object_t obj = m_objects.front();
    m_objects.pop_front();
    return obj;
  }

  /**
   * Remove an object from the channel, returning an empty object
   * if none exist.
   */
  object_t try_get(void) {
    object_t obj = object_t();
    std::unique_lock<std::mutex> lock(m_mtx);
    if (!m_objects.empty()) {
      obj = m_objects.front();
      m_objects.pop_front();
    }
    return obj;
  }

  size_t available(void) { return m_objects.size(); }

 private:
  std::mutex m_mtx;
  std::condition_variable m_cv;
  std::deque<object_t> m_objects;
  bool m_closed;
};
};
