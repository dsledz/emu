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
#include "core/lock.h"
#include "core/exception.h"
#include "core/debug.h"
#include "core/task.h"

namespace Core {

/**
 * Type-safe object channel.
 */
template<class object_t>
class Channel
{
public:
    Channel(void): m_mtx(), m_waiting(), m_objects()
    {
    }
    ~Channel(void)
    {
    }
    Channel(const Channel &ch) = delete;

    /**
     * Place an object in the channel.
     */
    void put(object_t obj) {
        lock_mtx lock(m_mtx);
        m_objects.push(obj);
        if (!m_waiting.empty()) {
            LOG_DEBUG("Waking task.");
            Task_ptr task = m_waiting.front();
            m_waiting.pop();
            resume_task(task);
        } else {
            LOG_DEBUG("No waiters.");
        }
    }

    /**
     * Remove an object from the channel, blocking if no object
     * exists.
     */
    object_t get(void) {
        std::unique_lock<std::mutex> lock(m_mtx);
        while (m_objects.empty()) {
            LOG_DEBUG("Waiting in channel.");
            Task_ptr task = Task::cur_task();
            m_waiting.push(task);
            {
                unlock_mtx unlock(m_mtx);
                suspend_task(task);
            }
        }
        LOG_DEBUG("Got object.");
        object_t obj = m_objects.front();
        m_objects.pop();
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
            m_objects.pop();
        }
        return obj;
    }

private:
    std::mutex              m_mtx;
    std::queue<Task_ptr>    m_waiting;
    std::queue<object_t>    m_objects;
};

};
