/*
 * Copyright (c) 2013, Dan Sledz * All rights reserved.
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

#include "emu.h"

#include "galaga.h"

using namespace EMU;

TEST(GalagaTest, run10)
{
    Driver::Galaga machine;
    bool press = false;
    machine.add_timer(Time(msec(500)), []() {
        INFO("Mark");
        },
        Time(msec(500)));
    /* 10 seconds of runtime */
    unsigned steps = 6000 * 10;
    for (unsigned i = 0; i < steps; i++) {
        machine.run();
    }
    machine.add_timer(Time(msec(100)), [&]() {
        press = !press;
        if (press) {
            std::cout << "Press" << std::endl;
            machine.input()->depress(InputKey::Start1);
        } else {
            std::cout << "Release" << std::endl;
            machine.input()->release(InputKey::Start1);
        }
        },
        Time(msec(100)));
    EMU::log.set_level(LogLevel::Trace);
    for (unsigned i = 0; i < steps*2; i++) {
        machine.run();
    }
}

TEST(GalagaTest, run100)
{
    Driver::Galaga machine;
    /* 10 seconds of runtime */
    unsigned steps = 6000 * 10;
    for (unsigned i = 0; i < steps; i++) {
        machine.run();
    }
}

TEST(GalagaTest, input)
{
    Driver::Galaga machine;
#if 0
    machine.add_timer(Time(msec(8000)), [&]() {
            machine.input()->depress(InputKey::Coin1);
        }, time_zero);
    machine.add_timer(Time(msec(8400)), [&]() {
            machine.input()->release(InputKey::Coin1);
        }, time_zero);
#endif

    EMU::log.set_level(LogLevel::Trace);
    unsigned steps = 6000 * 6;
    for (unsigned i = 0; i < steps; i++) {
        machine.run();
    }
    machine.input()->depress(InputKey::Coin1);
    for (unsigned i = 0; i < 1000; i++) {
        machine.run();
    }
    machine.input()->release(InputKey::Coin1);
    for (unsigned i = 0; i < 1000; i++) {
        machine.run();
    }
}

TEST(GalagaTest, load)
{
    Driver::Galaga machine;
}
