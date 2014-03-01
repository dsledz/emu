#include "core/task.h"
#include "core/debug.h"
#include "core/fiber.h"

using namespace Core;

static __thread Thread * curthread = NULL;

Task::Task(task_fn fn):
    m_state(State::Created),
    m_mtx(),
    m_cv(),
    m_func(fn)
{

}

Task::~Task(void)
{

}

Task_ptr
Task::cur_task(void)
{
    if (curthread)
        return curthread->cur_task();
    else
        throw TaskException("Invalid Task!");
}

Thread::Thread(TaskChannel_ptr channel):
    thread(std::bind(&Thread::thread_main, this)),
    m_idle_task(),
    m_task(),
    m_mtx(),
    m_state(ThreadState::Dead),
    m_channel(channel)
{
    m_idle_task = Task_ptr(new ThreadTask(std::bind(&Thread::thread_task, this)));
    m_task = m_idle_task;
}

Thread::~Thread(void)
{
    try {
        cancel();
    } catch (...) {
    }
    join();
}

void
Core::suspend_task(Task_ptr task)
{
    task->suspend();
}

void
Core::resume_task(Task_ptr task)
{
    task->resume(task);
}

void
Thread::schedule(Task_ptr task)
{
    m_channel->put(task);
}

void
Thread::cancel(void)
{
    lock_mtx lock(m_mtx);
    if (m_task)
        m_task->cancel();
    m_canceled = true;
}

void
Thread::thread_main(void)
{
    try {
        m_task->run();
    } catch (CoreException &e) {
        LOG_DEBUG("Task Exception");
    }
}

void
Thread::thread_task(void)
{
    curthread = this;
    std::unique_lock<std::mutex> lock(m_mtx);
    m_state = ThreadState::Idle;
    while (!m_canceled) {
        // Grab the first task off the runnable queue
        m_state = ThreadState::Idle;
        m_task = m_idle_task;
        Task_ptr task;
        try {
            unlock_mtx unlock(m_mtx);
            LOG_DEBUG("Getting next runnable task");
            task = m_channel->get();
        } catch (CanceledException &e) {
            LOG_DEBUG("Loop canceled");
            m_canceled = true;
            break;
        }
        Task::State task_state;
        m_state = ThreadState::Running;
        m_task = task;
        try {
            unlock_mtx unlock(m_mtx);
            task_state = task->run();
        } catch (CoreException &e) {
            LOG_DEBUG("Task exception");
            throw e;
        }
    }
    m_state = ThreadState::Dead;
    m_task = m_idle_task;
    curthread = NULL;
}

Thread *
Thread::cur_thread(void)
{
    return curthread;
}

Task_ptr
Thread::cur_task(void)
{
    Task_ptr task;
    {
        lock_mtx lock(m_mtx);
        task = m_task;
    }
    assert(m_task.get() != NULL);
    return task;
}

/*  _____                _____ _                        _ _____         _
 * | ____|_ __ ___  _   |_   _| |__  _ __ ___  __ _  __| |_   _|_ _ ___| | __
 * |  _| | '_ ` _ \| | | || | | '_ \| '__/ _ \/ _` |/ _` | | |/ _` / __| |/ /
 * | |___| | | | | | |_| || | | | | | | |  __/ (_| | (_| | | | (_| \__ \   <
 * |_____|_| |_| |_|\__,_||_| |_| |_|_|  \___|\__,_|\__,_| |_|\__,_|___/_|\_\
 */

ThreadTask::ThreadTask(task_fn fn):
    Task(fn)
{
}

ThreadTask::~ThreadTask(void)
{
}

bool
ThreadTask::nonblocking(void)
{
    return false;
}

Task::State
ThreadTask::run(void)
{
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
    return new_state;
}

void
ThreadTask::cancel(void)
{
    lock_mtx lock(m_mtx);
    m_state = State::Canceled;
    m_cv.notify_all();
}

void
ThreadTask::suspend(void)
{
    std::unique_lock<std::mutex> lock(m_mtx);
    LOG_DEBUG("Suspending thread task.");
    if (m_state == State::Running)
        m_state = State::Suspended;
    while (m_state != State::Queued && !finished(m_state))
        m_cv.wait(lock);
    if (m_state == State::Canceled)
        throw TaskCanceled("Canceled");
    m_state = State::Running;
}

void
ThreadTask::resume(Task_ptr task)
{
    lock_mtx lock(m_mtx);
    LOG_DEBUG("Resuming thread task.");
    if (m_state == State::Suspended) {
        m_state = State::Queued;
        m_cv.notify_all();
    }
}

Task::State
ThreadTask::force(void)
{
    std::unique_lock<std::mutex> lock(m_mtx);
    while (!Task::finished(m_state))
        m_cv.wait(lock);
    return m_state;
}

/**
 * Scheduler
 */
TaskScheduler::TaskScheduler(void):
    m_event_channel(new TaskChannel()),
    m_event_thread(new Thread(m_event_channel)),
    m_work_channel(new TaskChannel()),
    m_work_threads(),
    m_tasks()
{
}

TaskScheduler::~TaskScheduler(void)
{
    /* Cancel all outstanding tasks. */
    for (auto it = m_tasks.begin(); it != m_tasks.end(); it++)
        (*it)->cancel();

    /* Now make sure they've had a chance to comeplete. */
    for (auto it = m_tasks.begin(); it != m_tasks.end(); it++)
        (*it)->force();

    /* Cancel our threads. */
    m_event_thread->cancel();
    for (auto it = m_work_threads.begin(); it != m_work_threads.end(); it++) {
        (*it)->cancel();
    }
}

void
TaskScheduler::add_task(Task_ptr task)
{
    m_tasks.push_back(task);
    if (task->nonblocking()) {
        m_event_channel->put(task);
    } else {
        m_work_threads.push_back(Thread_ptr(new Thread(m_work_channel)));
        m_work_channel->put(task);
    }
}

Task_ptr
TaskScheduler::create_task(Task::task_fn fn)
{
    Task_ptr ptr = Task_ptr(new ThreadTask(fn));
    add_task(ptr);
    return ptr;
}

Task_ptr
TaskScheduler::create_fiber_task(Task::task_fn fn)
{
    Task_ptr ptr = Task_ptr(new FiberTask(fn));
    add_task(ptr);
    return ptr;
}

void
TaskScheduler::run_task(Task_ptr task)
{

}

void
TaskScheduler::cancel_task(Task_ptr task)
{

}
