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

#include "gtest/gtest.h"

#include "emu/emu.h"
#include "emu/timing.h"

using namespace EMU;

#if 0
class TestClockable: public Clockable {
public:
    TestClockable(void): Clockable("test")
    {
    }

    virtual ~TestClockable(void)
    {
    }

    virtual void execute(void)
    {
        Time interval = Time(usec(200));
        while (left() >= interval) {
            value++;
            do_advance(interval);
        }
        /* XXX: Should this be somewhere different? */
        do_set_state(State::Waiting);
    }

    int value = 0;
};

TEST(ClockableTest, busy)
{
    TestClockable test1;

    test1.power();

    EXPECT_EQ(0, test1.value);

    test1.add_time(Time(usec(200)));

    test1.wait_state(Clockable::State::Waiting);

    EXPECT_EQ(1, test1.value);
    test1.add_time(Time(usec(100)));

    test1.wait_state(Clockable::State::Waiting);
    EXPECT_EQ(1, test1.value);

    test1.add_time(Time(usec(1000)));

    test1.wait_state(Clockable::State::Waiting);
    EXPECT_EQ(6, test1.value);
}

TEST(ClockableTest, real_time)
{
    TestClockable test;

    test.power();

    EXPECT_EQ(0, test.value);

    Time total = Time(sec(0));
    RealTimeClock clock;
    while (total < Time(msec(100))) {
        Time interval = clock.get_delta();
        test.add_time(interval);
        struct timespec t = { 0, 1000000 };
        nanosleep(&t, NULL);
        total += interval;
    }

    test.wait_state(Clockable::State::Waiting);
    EXPECT_GT(500 + 10, test.value);
}

TEST(ClockableTest, task)
{
    TestClockable test;

    EmuScheduler sched;
    EmuTask_ptr task = test.create_task();

    sched.add(task);

    EXPECT_EQ(0, test.value);

    Time total = Time(sec(0));
    RealTimeClock clock;
    while (total < Time(msec(100))) {
        Time interval = clock.get_delta();
        test.add_time(interval);
        struct timespec t = { 0, 1000000 };
        nanosleep(&t, NULL);
        total += interval;
    }

    test.wait_state(Clockable::State::Waiting);
    test.set_state(Clockable::State::Stopped);
    EXPECT_GT(500 + 10, test.value);
}
#endif

TEST(TimerQueueTest, periodic)
{
    TimerQueue queue;
    int foo = 0;

    queue.add_periodic(Time(usec(200)), [&]() { foo++; });

    queue.run(Time(usec(150)));
    EXPECT_EQ(0, foo);

    queue.run(Time(usec(150)));
    EXPECT_EQ(1, foo);

    queue.run(Time(usec(150)));
    EXPECT_EQ(2, foo);

    queue.run(Time(usec(2000)));
    EXPECT_EQ(12, foo);
}

TEST(TimerQueueTest, single)
{
    TimerQueue queue;
    int foo = 0;

    queue.add_timeout(Time(usec(200)), [&]() { foo++; });

    queue.run(Time(usec(150)));
    EXPECT_EQ(0, foo);

    queue.run(Time(usec(150)));
    EXPECT_EQ(1, foo);

    queue.run(Time(usec(150)));
    EXPECT_EQ(1, foo);
}


