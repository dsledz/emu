#include "gtest/gtest.h"

#include "core/task.h"
#include "core/thread.h"

using namespace Core;

TEST(TaskTest, simple)
{
    int count = 0;
    TaskScheduler sched;

    Task_ptr task = sched.create_task([&](void) { count++; });

    task->force();

    EXPECT_EQ(1, count);
}

TEST(TaskTest, hang)
{
    TaskScheduler sched;

    Task_ptr task = sched.create_task([&](void) { sleep(1); });
}

static void
consume(Channel<int> *channel)
{
    while (true) {
        int d = channel->get();
        d = 0;
    }
}

static void
produce_n(Channel<int> *channel, int max)
{
    for (int i = 0; i < max; i++) {
        LOG_ERROR("Putting value");
        channel->put(i);
    }
}

static void
consume_n(Channel<int> *channel, int max, int *total)
{
    for (int i = 0; i < max; i++) {
        int d = channel->get();
        *total += d;
    }
}

TEST(TaskTest, cancel)
{
    Channel<int> channel;
    {
        TaskScheduler sched;

        Task_ptr task = sched.create_task(std::bind(&consume, &channel));

        /* Force the scheduler to cancel the task */
    }
}

TEST(TaskTest, Producer)
{
    TaskScheduler sched;
    Channel<int> channel;
    int total = 0;

    Task_ptr ctask = sched.create_task(
        std::bind(&consume_n, &channel, 10, &total));

    struct timespec t = { 0, 1000000 };
    nanosleep(&t, NULL);
    Task_ptr ptask = sched.create_task(
        std::bind(&produce_n, &channel, 10));

    ptask->force();
    ctask->force();
    EXPECT_EQ(45, total);
}
