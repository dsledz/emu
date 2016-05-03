#include "core/task.h"
#include "core/channel.h"
#include "core/debug.h"
#include "core/fiber.h"

using namespace Core;

#ifdef WIN32
__declspec(thread) Thread *curthread = NULL;
#else
static __thread Thread * curthread = NULL;
#endif

uint64_t
Task::next_id(void)
{
    static std::atomic<uint64_t> next_id= ATOMIC_VAR_INIT(1);
    return std::atomic_fetch_add(&next_id, (uint64_t)1);
}

Task::Task(TaskScheduler *Scheduler, task_fn fn):
    m_id(Task::next_id()),
    m_state(State::Created),
    m_mtx(),
    m_cv(),
    m_func(fn),
    m_name("(Null)"),
    m_sched(Scheduler)
{

}

Task::Task(TaskScheduler *Scheduler, task_fn fn, const std::string &name):
    m_id(Task::next_id()),
    m_state(State::Created),
    m_mtx(),
    m_cv(),
    m_func(fn),
    m_name(name),
    m_sched(Scheduler)
{
    m_sched->add_task(this);
}

Task::~Task(void)
{
    //assert(finished(m_state));
    m_sched->remove_task(this);
}

void
Task::yield(void)
{
    Task *cur = Thread::cur_task();
    assert(cur != NULL);
    cur->yield_internal();
}

Thread::Thread(TaskChannel_ptr channel):
    thread(std::bind(&Thread::thread_main, this)),
    m_task(NULL),
    m_mtx(),
    m_state(ThreadState::Dead),
    m_channel(channel)
{
}

Thread::~Thread(void)
{
    assert(m_state == ThreadState::Dead);
}

std::ostream& Core::operator<< (std::ostream &os, const Task &t)
{
    os << t.name() << "(" << t.id() << ")";
    return os;
}

void
Thread::schedule(Task *task)
{
    m_channel->put(task);
}

void
Thread::thread_main(void)
{
    curthread = this;
    try {
        thread_task();
    } catch (...) {
        LOG_ERROR("Unclean thread exit");
        m_state = ThreadState::Dead;
    }
}

void
Thread::thread_task(void)
{
    lock_mtx lock(m_mtx);
    m_state = ThreadState::Idle;
    for (;;) {
        // Grab the first task off the runnable queue
        m_state = ThreadState::Idle;
        Task * task;
        try {
            unlock_mtx unlock(m_mtx);
            LOG_DEBUG("ThreadTask waiting");
            task = m_channel->get();
        } catch (CanceledException &e) {
			LOG_DEBUG("Loop canceled: ", e.message());
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
    curthread = NULL;
}

Thread *
Thread::cur_thread(void)
{
    return curthread;
}

Task *
Thread::cur_task(void)
{
    return cur_thread()->m_task;
}

/*  _____                _____ _                        _ _____         _
 * | ____|_ __ ___  _   |_   _| |__  _ __ ___  __ _  __| |_   _|_ _ ___| | __
 * |  _| | '_ ` _ \| | | || | | '_ \| '__/ _ \/ _` |/ _` | | |/ _` / __| |/ /
 * | |___| | | | | | |_| || | | | | | | |  __/ (_| | (_| | | | (_| \__ \   <
 * |_____|_| |_| |_|\__,_||_| |_| |_|_|  \___|\__,_|\__,_| |_|\__,_|___/_|\_\
 */

ThreadTask::ThreadTask(TaskScheduler *Scheduler, task_fn fn):
    Task(Scheduler, fn)
{
}

ThreadTask::ThreadTask(TaskScheduler *Scheduler, task_fn fn,
                       const std::string &name):
    Task(Scheduler, fn, name)
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
    lock_mtx lock(m_mtx);
    LOG_DEBUG("ThreadTask Suspend: ", *this);
    if (m_state == State::Running)
        m_state = State::Suspended;
    while (m_state != State::Queued && !finished(m_state))
        lock.wait(m_cv);
    LOG_DEBUG("ThreadTask Resumed: ", *this);
    if (m_state == State::Canceled)
        throw TaskCanceled("Canceled");
    m_state = State::Running;
}

void
ThreadTask::yield_internal(void)
{

}

void
ThreadTask::wake(void)
{
    lock_mtx lock(m_mtx);
    LOG_DEBUG("ThreadTask wake: ", *this);
    if (m_state == State::Suspended) {
        m_state = State::Queued;
        m_cv.notify_all();
    }
}

Task::State
ThreadTask::force(void)
{
    lock_mtx lock(m_mtx);
    while (!Task::finished(m_state))
        lock.wait(m_cv);
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
    m_event_channel->close();
    for (auto it = m_tasks.begin(); it != m_tasks.end(); it++)
        (*it)->cancel();
    for (auto it = m_tasks.begin(); it != m_tasks.end(); it++)
        (*it)->force();

    /* Now cancel the work threads */
    m_work_channel->close();
    for (auto it = m_work_threads.begin(); it != m_work_threads.end(); it++) {
        (*it)->join();
    }
    m_event_thread->join();
}

void
TaskScheduler::add_task(Task * task)
{
    m_tasks.push_back(task);
}

void
TaskScheduler::run_task(Task * task)
{
    if (task->nonblocking()) {
        LOG_DEBUG("RunTask nonblocking: ", *task);
        m_event_channel->put(task);
    } else {
        LOG_DEBUG("RunTask blocking: ", *task);
        m_work_threads.push_back(Thread_ptr(new Thread(m_work_channel)));
        m_work_channel->put(task);
    }
}

void
TaskScheduler::cancel_task(Task * task)
{

}

void
TaskScheduler::shutdown(void)
{
    /* Cancel all outstanding tasks. */
    m_event_channel->close();
    for (auto it = m_tasks.begin(); it != m_tasks.end(); it++)
        (*it)->cancel();
    for (auto it = m_tasks.begin(); it != m_tasks.end(); it++)
        (*it)->force();
}

void
TaskScheduler::remove_task(Task *task)
{
    m_tasks.remove(task);
}
