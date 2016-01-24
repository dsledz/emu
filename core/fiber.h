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
#include "core/exception.h"
#include "core/debug.h"
#include "core/task.h"

namespace Core {

struct ThreadRegisters {
    uint64_t r8;
    uint64_t r9;
    uint64_t r10;
    uint64_t r11;
    uint64_t r12;
    uint64_t r13;
    uint64_t r14;
    uint64_t r15;
    uint64_t rsp;
    uint64_t rbp;
};

class ThreadContext
{
public:
    ThreadContext(uint64_t rip);
    ~ThreadContext(void);

    void switch_context(ThreadContext *new_context);

private:
    std::vector<uint8_t> m_stack;
    ThreadRegisters m_registers;
};

class FiberTask: public Task
{
public:
    FiberTask(task_fn fn);
    FiberTask(task_fn fn, const std::string &name);
    virtual ~FiberTask(void);
    FiberTask(const FiberTask &rhs) = delete;

    virtual bool nonblocking(void);
    virtual State run(void);
    virtual void cancel(void);
    virtual void suspend(void);
    virtual void resume(Task_ptr task);
    virtual State force(void);

private:

    void run_internal(void);
    static void run_context(void);

    Thread       *m_thread;
    ThreadContext m_our_ctx;
    ThreadContext m_thread_ctx;
};

};
