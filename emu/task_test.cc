#include "gtest/gtest.h"

#include "emu/emu.h"

using namespace EMU;

TEST(TaskTest, simple)
{
    int count = 0;
    EmuScheduler sched;

    EmuTask_ptr task = sched.create([&](void) { count++; });

    task->wait();

    EXPECT_EQ(1, count);
}

TEST(TaskTest, hang)
{
    EmuScheduler sched;

    EmuTask_ptr task = sched.create([&](void) { sleep(1); });
}

static void
consume(EmuChannel<int> *channel)
{
    while (true) {
        int d = channel->get();
        d = 0;
    }
}

TEST(TaskTest, cancel)
{
    EmuScheduler sched;
    EmuChannel<int> channel;

    EmuTask_ptr task = sched.create(std::bind(&consume, &channel));

}

