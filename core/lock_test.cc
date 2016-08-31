#include "gtest/gtest.h"

#include "core/lock.h"

using namespace Core;

TEST(LockTest, lock_unlock_lock)
{
    spin_lock sl;

    sl.lock();
    sl.unlock();
    sl.lock();
    sl.unlock();
}
