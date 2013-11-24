#include "emu/task.h"
#include "emu/debug.h"

using namespace EMU;

static __thread EmuThread * curthread = NULL;

EmuTask::EmuTask(task_fn fn):
    m_state(State::Created),
    m_mtx(),
    m_cv(),
    m_func(fn)
{

}

EmuTask::~EmuTask(void)
{

}

EmuCancel::EmuCancel(EmuCancelable *cancel)
{
    if (curthread)
        curthread->set_cancel(cancel);
}

EmuCancel::~EmuCancel(void)
{
    if (curthread)
        curthread->set_cancel(NULL);
}

EmuThread::EmuThread(EmuTaskChannel_ptr channel):
    thread(std::bind(&EmuThread::thread_loop, this)),
    m_cancel_cb(NULL),
    m_canceled(false),
    m_task(),
    m_mtx(),
    m_state(ThreadState::Dead),
    m_channel(channel)
{
}

EmuThread::~EmuThread(void)
{
    try {
        cancel();
    } catch (...) {
    }
}

void
EmuThread::run(void)
{

}

void
EmuThread::set_cancel(EmuCancelable *cancel)
{
    lock_mtx lock(m_mtx);
    if (m_canceled && cancel)
        cancel->cancel();
    m_cancel_cb = cancel;
}

void
EmuThread::cancel(void)
{
    lock_mtx lock(m_mtx);
    m_canceled = true;
    if (m_cancel_cb != NULL)
        m_cancel_cb->cancel();
}

void
EmuThread::thread_loop(void)
{
    curthread = this;
    std::unique_lock<std::mutex> lock(m_mtx);
    while (m_state != ThreadState::Exiting) {
        m_state = ThreadState::Idle;
        try {
            unlock_mtx unlock(m_mtx);
            EmuTask_ptr obj = m_channel->get();
            obj->run();
        } catch (EmuException &e) {
            break;
        }
        m_state = ThreadState::Exiting;
    }
    m_state = ThreadState::Dead;
    curthread = NULL;
}

/**
 * Scheduler
 */
EmuScheduler::EmuScheduler(void):
    m_threads(),
    m_channel()
{
    m_channel = EmuTaskChannel_ptr(new EmuTaskChannel());
}

EmuScheduler::~EmuScheduler(void)
{
    m_channel->cancel();
    for (auto it = m_threads.begin(); it != m_threads.end(); it++) {
        (*it)->cancel();
    }
}

void
EmuScheduler::add(EmuTask_ptr task)
{
    m_threads.push_back(EmuThread_ptr(new EmuThread(m_channel)));
    m_channel->put(task);
}

EmuTask_ptr
EmuScheduler::create(EmuTask::task_fn fn)
{
    EmuTask_ptr ptr = EmuTask_ptr(new EmuTask(fn));
    add(ptr);
    return ptr;
}

