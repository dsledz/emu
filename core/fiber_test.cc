#include "gtest/gtest.h"

#include "core/task.h"
#include "core/channel.h"
#include "core/fiber.h"

using namespace Core;

TEST(FiberTest, simple)
{
    int count = 0;
    TaskScheduler sched;

    FiberTask task(&sched, [&](void) { count++; });

    sched.run_task(&task);
    task.force();

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

    FiberTask ctask(&sched, std::bind(&consume, &channel, 1, &total));

    sched.run_task(&ctask);
    channel.put(1);

    ctask.force();

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

    FiberTask ctask(&sched, std::bind(&consume, &channel, 10, &total));
    sched.run_task(&ctask);
    FiberTask ptask(&sched, std::bind(&produce, &channel, 10));
    sched.run_task(&ptask);

    ptask.force();
    ctask.force();
    EXPECT_EQ(45, total);
}
