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

namespace Core {

struct CanceledException: public CoreException {
    CanceledException(const std::string &reason):
        CoreException(reason) { }
};

struct TaskCanceled: public CanceledException {
    TaskCanceled(const std::string &reason): CanceledException("Task")
    {
        msg += ": " + reason;
    }
};

struct TaskException: public CoreException {
    TaskException(const std::string &reason):
        CoreException(reason) { }
};

class Task;
typedef std::shared_ptr<Task> Task_ptr;
typedef std::queue<Task_ptr> TaskQueue;
template<class object_t> class Channel;
typedef Channel<Task_ptr> TaskChannel;
typedef std::shared_ptr<TaskChannel> TaskChannel_ptr;

class Task
{
public:
    enum class State {
        Created,
        Queued,
        Running,
        Suspended,
        Finished,
        Canceled,
        Dead };
    typedef std::function<void (void)> task_fn;

    Task(task_fn fn);
    virtual ~Task(void);
    Task(const Task &task) = delete;

    uint64_t id(void) const {
        return m_id;
    }

    /**
     * Task is nonblocking
     */
    virtual bool nonblocking(void) = 0;

    /**
     * Execute the task.
     */
    virtual State run(void) = 0;

    /**
     * Cancel the task.
     */
    virtual void cancel(void) = 0;

    /**
     * Suspend the task until it can be woken up.
     */
    virtual void suspend(void) = 0;

    /**
     * Resume a suspended task.
     */
    virtual void resume(Task_ptr task) = 0;

    /**
     * Force a task to execute.
     */
    virtual State force(void) = 0;

    static bool finished(State state) {
        return (state == State::Finished ||
                state == State::Canceled ||
                state == State::Dead);
    }

    static bool runnable(State state) {
        return (state == State::Running ||
                state == State::Queued);
    }

    static Task_ptr cur_task(void);

protected:

    uint64_t                    m_id;
    State                       m_state;
    std::mutex                  m_mtx;
    std::condition_variable     m_cv;
    std::function<void (void)>  m_func;

private:
    static uint64_t next_id(void);
};

class ThreadTask: public Task
{
public:
    typedef std::function<void (void)> task_fn;

    ThreadTask(task_fn fn);
    ~ThreadTask(void);
    ThreadTask(const ThreadTask &task) = delete;

    virtual bool nonblocking(void);
    virtual State run(void);
    virtual void cancel(void);
    virtual void suspend(void);
    virtual void resume(Task_ptr task);
    virtual State force(void);
};

void resume_task(Task_ptr task);
void suspend_task(Task_ptr task);

class Thread: public std::thread
{
public:
    enum class ThreadState {
        Dead,
        Idle,
        Running,
        Exiting
    };

    Thread(TaskChannel_ptr channel);
    virtual ~Thread(void);
    Thread(const Thread &thread) = delete;

    void cancel(void);

    Task_ptr cur_task(void);

    void schedule(Task_ptr task);

    static Thread *cur_thread(void);

private:
    void thread_main(void);
    void thread_task(void);

    Task_ptr        m_idle_task;   /**< Idle task */
    Task_ptr        m_task;        /**< Current task */
    bool            m_canceled;    /**< Thread was canceled */
    std::mutex      m_mtx;         /**< Mutex */
    ThreadState     m_state;       /**< Thread state */
    TaskChannel_ptr m_channel;     /**< Channel to receive new tasks */
};

typedef std::unique_ptr<Thread> Thread_ptr;

class TaskScheduler
{
public:
    TaskScheduler(void);
    ~TaskScheduler(void);

    /**
     * Create a new task
     */
    Task_ptr create_task(Task::task_fn fn);
    /**
     * Create a co-routine task
     */
    Task_ptr create_fiber_task(Task::task_fn fn);
    /**
     * Add a new task to the scheduler.
     */
    void add_task(Task_ptr task);
    /**
     * Start the execution of the task.
     */
    void run_task(Task_ptr task);
    /**
     * Cancel the task.
     */
    void cancel_task(Task_ptr task);

private:
    std::mutex            m_mtx;
    TaskChannel_ptr       m_event_channel;
    Thread_ptr            m_event_thread;
    TaskChannel_ptr       m_work_channel;
    std::list<Thread_ptr> m_work_threads;
    std::list<Task_ptr>   m_tasks;
};

};
