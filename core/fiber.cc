#include "core/bits.h"
#include "core/debug.h"
#include "core/task.h"
#include "core/fiber.h"

using namespace Core;

extern "C" {
void
SwitchContext(ThreadRegisters *old_ctx, ThreadRegisters *new_ctx);
};

__thread int Core::held_locks = 0;

ThreadContext::ThreadContext(uint64_t rip):
    m_stack(64*1024),
    m_registers()
{
    uint8_t *stack = &(*m_stack.end());
    // 128 byte red zone
    stack -= 128;
    // 8 bytes for our hazard ebp
    stack -= 8;
    *reinterpret_cast<uint64_t *>(stack) = 0xFFFFFFFFFFFFFFFFul;
    // 8 bytes for our hazard ip
    stack -= 8;
    *reinterpret_cast<uint64_t *>(stack) = 0xFFFFFFFFFFFFFFFFul;
    // 8 bytes for our real ebp
    stack -= 8;
    *reinterpret_cast<uint64_t *>(stack) = reinterpret_cast<uintptr_t>(stack);
    // 8 bytes for our real eip
    stack -= 8;
    *reinterpret_cast<uint64_t *>(stack) = rip;
    m_registers.rsp = reinterpret_cast<uintptr_t>(stack);
}

ThreadContext::~ThreadContext(void)
{
    /* XXX: Assert we're not on that stack */
}

void
ThreadContext::switch_context(
    ThreadContext *saved_ctx)
{
    assert(held_locks == 0);
    // Swap our registers On return, we'll be in the other context
    // and our return stack will change.
    SwitchContext(&saved_ctx->m_registers, &m_registers);
    assert(held_locks == 0);
}

/**
 * Main entry point for a fiber. All fibers has this for their stack
 * frame.
 */
void
FiberTask::run_context(void)
{
    FiberTask *task = reinterpret_cast<FiberTask *>(Thread::cur_task());

    task->run_internal();
    /* Switch to the thread context. */
    task->m_thread_ctx.switch_context(&task->m_our_ctx);
    /* Return from the thread context. */
}

FiberTask::FiberTask(TaskScheduler *Scheduler, task_fn fn):
    Task(Scheduler, fn),
    m_our_ctx(reinterpret_cast<uint64_t>(&FiberTask::run_context)),
    m_thread_ctx(0)
{
}

FiberTask::FiberTask(TaskScheduler *Scheduler, task_fn fn,
                     const std::string &name):
    Task(Scheduler, fn, name),
    m_our_ctx(reinterpret_cast<uint64_t>(&FiberTask::run_context)),
    m_thread_ctx(0)
{
}

FiberTask::~FiberTask()
{
}

void
FiberTask::run_internal(void)
{
    {
        lock_mtx lock(m_mtx);
        m_state = State::Running;
    }
    LOG_DEBUG("FiberTask execute: ", *this);
    State new_state = State::Finished;
        m_func();
    {
        lock_mtx lock(m_mtx);
        m_state = new_state;
        m_cv.notify_all();
    }
    LOG_DEBUG("FiberTask : ", *this);
}

bool
FiberTask::nonblocking(void)
{
    return true;
}

Task::State
FiberTask::run(void)
{
    // Check if we're runnable
    {
        lock_mtx lock(m_mtx);
        switch (m_state) {
        case State::Created:
            m_thread = Thread::cur_thread();
            break;
        case State::Queued:
            break;
        default:
            assert(false);
            // XXX: Not runnable
            return m_state;
        }
    }
    LOG_DEBUG("FiberTask switch: ", *this);
    // Switch to our context and run.
    m_our_ctx.switch_context(&m_thread_ctx);
    LOG_DEBUG("FiberTask swapped: ", *this);
    // We've returned from our context.
    {
        lock_mtx lock(m_mtx);
        return m_state;
    }
}

void
FiberTask::cancel(void)
{
    lock_mtx lock(m_mtx);
    LOG_DEBUG("Canceling fiber task: ", *this);
    if (m_state == State::Suspended) {
        m_state = State::Canceled;
        m_thread->schedule(this);
    } else {
        m_state = State::Canceled;
    }
}

void
FiberTask::suspend(void)
{
    lock_mtx lock(m_mtx);
    LOG_DEBUG("FiberTask waiting: ", *this);
    if (m_state == State::Running)
        m_state = State::Suspended;
    while (m_state != State::Queued && !finished(m_state)) {
        unlock_mtx unlock(m_mtx);
        /* Switch to the thread context. */
        m_thread_ctx.switch_context(&m_our_ctx);
        /* Return from the thread context. */
    }
    LOG_DEBUG("FiberTask resumed: ", *this);
    if (m_state == State::Queued)
        m_state = State::Running;
    if (finished(m_state))
        throw TaskCanceled("Finished");
}

void
FiberTask::yield_internal(void)
{
    /* Put us on the runnable list */
    {
        lock_mtx lock(m_mtx);
        LOG_DEBUG("FiberTask yield: ", *this);
        m_state = State::Queued;
        m_thread->schedule(this);
    }
    m_thread_ctx.switch_context(&m_our_ctx);
}

void
FiberTask::wake(void)
{
    /* Put us on the runnable list */
    lock_mtx lock(m_mtx);
    LOG_DEBUG("FiberTask wake: ", *this);
    if (m_state == State::Suspended) {
        m_state = State::Queued;
        m_thread->schedule(this);
    }
}

Task::State
FiberTask::force(void)
{
    lock_mtx lock(m_mtx);
    while (!Task::finished(m_state))
        lock.wait(m_cv);
    return m_state;
}

