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
using namespace EMU;

TEST(PageTest, simple_page)
{
    Page<uint16_t, uint8_t, 256> page;
    uint8_t data1 = 30;
    uint8_t data2 = 30;
    uint16_t offset1 = 5;

    page.write(offset1, data1);
    data2 = page.read(offset1);

    EXPECT_EQ(data1, data2);
}

TEST(PageTest, bounds_checking)
{
    Page<uint16_t, uint8_t, 256> page;
    uint8_t data1 = 30;
    uint8_t data2 = 30;
    uint16_t offset1 = 256;

    page.write(offset1, data1);
    data2 = page.read(offset1);

    EXPECT_EQ(data1, data2);
}

TEST(PageTest, prot_wr)
{
    Page<uint16_t, uint8_t, 256> page({PageFlags::None});
    uint8_t data1 = 30;
    uint16_t offset1 = 256;

    EXPECT_THROW(page.write(offset1, data1), EMU::PageException);
}

