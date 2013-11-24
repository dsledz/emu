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

#include "emu/emu.h"

#include "machine/galaga.h"

using namespace EMU;

#if 0
TEST(GalagaTest, run10)
{
    Arcade::Galaga machine("");
    bool press = false;
    /* 10 seconds of runtime */
    unsigned steps = 6000 * 10;
    for (unsigned i = 0; i < steps; i++) {
        machine.run();
    }
    machine.add_timer(Time(msec(100)), [&]() {
            press = !press;
            machine.send_input(InputKey::Start1, press);
        },
        true);
    for (unsigned i = 0; i < steps*2; i++) {
        machine.run();
    }
}
#endif

TEST(GalagaTest, run100)
{
    Arcade::Galaga machine("");
    /* 10 seconds of runtime */
    unsigned steps = 6000 * 10;
    for (unsigned i = 0; i < steps; i++) {
        machine.run();
    }
}

TEST(GalagaTest, input)
{
    Arcade::Galaga machine("");

    unsigned steps = 6000 * 6;
    for (unsigned i = 0; i < steps; i++) {
        machine.run();
    }
    machine.send_input(InputKey::Coin1, true);
    for (unsigned i = 0; i < 1000; i++) {
        machine.run();
    }
    machine.send_input(InputKey::Coin1, false);
    for (unsigned i = 0; i < 1000; i++) {
        machine.run();
    }
}

TEST(GalagaTest, load)
{
    Arcade::Galaga machine("");
}
