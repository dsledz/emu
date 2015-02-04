#include "gtest/gtest.h"

#include "core/task.h"
#include "core/channel.h"
#include "core/fiber.h"

using namespace Core;

TEST(FiberTest, simple)
{
    int count = 0;
    TaskScheduler sched;

    Task_ptr task = sched.create_fiber_task([&](void) { count++; });

    sched.run_task(task);

    task->force();

    EXPECT_EQ(1, count);
}

static void
consume(Channel<int> *channel, int max, int *total)
{
    for (int i = 0; i < max; i++) {
        int d = channel->get();
        LOG_ERROR("Running consume task.");
        *total += d;
    }
}

TEST(FiberTest, sleeping)
{
    TaskScheduler sched;
    Channel<int> channel;
    int total = 0;

    Task_ptr ctask = sched.create_fiber_task(
        std::bind(&consume, &channel, 1, &total));

    channel.put(1);

    ctask->force();

    EXPECT_EQ(1, total);
}

static void
produce(Channel<int> *channel, int max)
{
    for (int i = 0; i < max; i++) {
        channel->put(i);
    }
}

TEST(FiberTest, Producer)
{
    TaskScheduler sched;
    Channel<int> channel;
    int total = 0;

    Task_ptr ctask = sched.create_fiber_task(
        std::bind(&consume, &channel, 10, &total));
    Task_ptr ptask = sched.create_fiber_task(
        std::bind(&produce, &channel, 10));

    ptask->force();
    ctask->force();
    EXPECT_EQ(45, total);
}
