#include "core/bits.h"
#include "core/debug.h"
#include "core/task.h"
#include "core/fiber.h"

using namespace Core;

extern "C" {
void
SwitchContext(ThreadRegisters *old_ctx, ThreadRegisters *new_ctx);
};

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
    // Swap our registers On return, we'll be in the other context
    // and our return stack will change.
    SwitchContext(&saved_ctx->m_registers, &m_registers);
}

/**
 * Main entry point for a fiber. All fibers has this for their stack
 * frame.
 */
void
FiberTask::run_context(void)
{
    Thread *thread = Thread::cur_thread();
    FiberTask *task = reinterpret_cast<FiberTask *>(
        thread->cur_task().get());

    task->run_internal();
    /* Switch to the thread context. */
    task->m_thread_ctx.switch_context(&task->m_our_ctx);
    /* Return from the thread context. */
}

FiberTask::FiberTask(task_fn fn):
    Task(fn),
    m_our_ctx(reinterpret_cast<uint64_t>(&FiberTask::run_context)),
    m_thread_ctx(0)
{
}

FiberTask::FiberTask(task_fn fn, const std::string &name):
    Task(fn, name),
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
    LOG_DEBUG("Task started");
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
    LOG_DEBUG("Task finished");
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
    LOG_DEBUG("Running fiber task: ", *this);
    // Switch to our context and run.
    m_our_ctx.switch_context(&m_thread_ctx);
    // We've returned from our context.
    {
        lock_mtx lock(m_mtx);
        return m_state;
    }
}

void
FiberTask::cancel(void)
{
    std::unique_lock<std::mutex> lock(m_mtx);
    LOG_DEBUG("Canceling fiber task: ", *this);
    if (m_state == State::Suspended) {
        m_state = State::Canceled;
        /* XXX: How do we trigger ourselves? */
    } else {
        m_state = State::Canceled;
    }
}

void
FiberTask::suspend(void)
{
    std::unique_lock<std::mutex> lock(m_mtx);
    LOG_DEBUG("Suspending fiber task: ", *this);
    if (m_state == State::Running)
        m_state = State::Suspended;
    while (m_state != State::Queued && !finished(m_state)) {
        unlock_mtx unlock(m_mtx);
        /* Switch to the thread context. */
        m_thread_ctx.switch_context(&m_our_ctx);
        /* Return from the thread context. */
    }
    if (m_state == State::Queued)
        m_state = State::Running;
    if (finished(m_state))
        throw TaskCanceled("Finished");
}

void
FiberTask::resume(Task_ptr task)
{
    /* Put us on the runnable list */
    lock_mtx lock(m_mtx);
    LOG_DEBUG("Resuming fiber task: ", *this);
    if (m_state == State::Suspended) {
        m_state = State::Queued;
        m_thread->schedule(task);
    }
}

Task::State
FiberTask::force(void)
{
    std::unique_lock<std::mutex> lock(m_mtx);
    while (!Task::finished(m_state))
        m_cv.wait(lock);
    return m_state;
}

