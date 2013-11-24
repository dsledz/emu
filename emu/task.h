/*
 * Copyright (c) 2013, Dan Sledz * All rights reserved.
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

#include "emu/bits.h"
#include "emu/mem.h"
#include "emu/exception.h"
#include "emu/debug.h"

namespace EMU {

struct EmuChannelCanceled: public EmuException {
    EmuChannelCanceled(void):
        EmuException("Channel canceled") { }
};

struct EmuTaskCanceled: public EmuException {
    EmuTaskCanceled(const std::string &reason): EmuException("Canceled")
    {
        msg += ": " + reason;
    }

    std::string reason;
};

template<typename mtx_type>
class unlock_guard {
public:
    unlock_guard(mtx_type & m): _mtx(m) {
        _mtx.unlock();
    }
    ~unlock_guard(void) {
        _mtx.lock();
    }

private:
    mtx_type & _mtx;
};

typedef std::unique_lock<std::mutex> lock_mtx;
typedef unlock_guard<std::mutex> unlock_mtx;

class EmuCancelable
{
public:
    virtual void cancel(void) = 0;
};

class EmuCancel
{
public:
    EmuCancel(EmuCancelable *cancel);
    ~EmuCancel(void);
};

class EmuTask
{
public:
    enum class State {
        Created,
        Queued,
        Running,
        Finished,
        Dead };
    typedef std::function<void (void)> task_fn;

    EmuTask(task_fn fn);
    ~EmuTask(void);
    EmuTask(const EmuTask &task) = delete;

    void run(void) {
        {
            lock_mtx lock(m_mtx);
            m_state = State::Running;
        }
        State new_state = State::Finished;
        try {
            m_func();
        } catch (...) {
            new_state = State::Dead;
        }
        {
            lock_mtx lock(m_mtx);
            m_state = new_state;
            m_cv.notify_all();
        }
    }

    bool wait(void) {
        std::unique_lock<std::mutex> lock(m_mtx);
        while (m_state != State::Finished && m_state != State::Dead)
            m_cv.wait(lock);
        return (m_state == State::Finished);
    }

private:

    State m_state;
    std::mutex m_mtx;
    std::condition_variable m_cv;
    std::function<void (void)> m_func;
};

typedef std::shared_ptr<EmuTask> EmuTask_ptr;

template<class object_t>
class EmuChannel: public EmuCancelable
{
public:
    EmuChannel(void): m_canceled(false), m_mtx(), m_cv(), m_objects()
    {
    }
    ~EmuChannel(void)
    {
    }
    EmuChannel(const EmuChannel &ch) = delete;

    void put(object_t obj) {
        lock_mtx lock(m_mtx);
        m_objects.push(obj);
        m_cv.notify_all();
    }

    object_t get(void) {
        EmuCancel cancel(this);
        std::unique_lock<std::mutex> lock(m_mtx);
        while (!m_canceled && m_objects.empty()) {
            m_cv.wait(lock);
            if (m_canceled)
                break;
        }
        if (m_canceled)
            throw EmuTaskCanceled("Canceled");
        object_t obj = m_objects.front();
        m_objects.pop();
        return obj;
    }

    object_t try_get(void) {
        object_t obj = object_t();
        std::unique_lock<std::mutex> lock(m_mtx);
        if (!m_objects.empty()) {
            obj = m_objects.front();
            m_objects.pop();
        }
        return obj;
    }

    virtual void cancel(void) {
        lock_mtx lock(m_mtx);
        m_canceled = true;
        m_cv.notify_all();
    }

private:
    bool m_canceled;
    std::mutex m_mtx;
    std::condition_variable m_cv;
    std::queue<object_t> m_objects;
};

typedef EmuChannel<EmuTask_ptr> EmuTaskChannel;
typedef std::shared_ptr<EmuTaskChannel> EmuTaskChannel_ptr;

class EmuThread: public std::thread
{
public:
    enum class ThreadState {
        Dead,
        Idle,
        Running,
        Exiting
    };

    EmuThread(EmuTaskChannel_ptr channel);
    virtual ~EmuThread(void);

    virtual void run(void);

    EmuThread(const EmuThread &thread) = delete;

    void set_cancel(EmuCancelable *cancel);

    void cancel(void);

private:
    void thread_loop(void);

    EmuCancelable *m_cancel_cb;
    bool m_canceled;
    std::future<void> m_task;
    std::mutex m_mtx;
    ThreadState m_state;
    EmuTaskChannel_ptr m_channel;
};

typedef std::unique_ptr<EmuThread> EmuThread_ptr;

class EmuScheduler
{
public:
    EmuScheduler(void);
    ~EmuScheduler(void);

    void add(EmuTask_ptr task);

    EmuTask_ptr create(EmuTask::task_fn fn);

private:
    std::list<EmuThread_ptr> m_threads;
    EmuTaskChannel_ptr m_channel;
};

};
