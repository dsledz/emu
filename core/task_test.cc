#include "gtest/gtest.h"

#include "core/task.h"
#include "core/channel.h"

using namespace Core;

TEST(TaskTest, simple)
{
    int count = 0;
    TaskScheduler sched;

    ThreadTask task(&sched, [&](void) { count++; });

    sched.run_task(&task);

    task.force();

    EXPECT_EQ(1, count);
}

TEST(TaskTest, hang)
{
    TaskScheduler sched;
    ThreadTask task(&sched, [&](void) { sleep(1); });

    sched.run_task(&task);
    task.force();
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

        ThreadTask task(&sched, std::bind(&consume, &channel));

        sched.run_task(&task);

        task.force();
    }
}

TEST(TaskTest, Producer)
{
    TaskScheduler sched;
    Channel<int> channel;
    int total = 0;

    ThreadTask ctask(&sched, std::bind(&consume_n, &channel, 10, &total));
    sched.run_task(&ctask);

    struct timespec t = { 0, 1000000 };
    nanosleep(&t, NULL);
    ThreadTask ptask(&sched, std::bind(&produce_n, &channel, 10));
    sched.run_task(&ptask);

    ptask.force();
    ctask.force();
    EXPECT_EQ(45, total);
}
