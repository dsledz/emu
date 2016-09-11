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

typedef DataBus<uint16_t, 16, uint8_t> Bus16x8;

TEST(BusTest, Bus16by8) {
  Bus16x8 bus;

  byte_t foo = 0;
  bus.add(Bus16x8::IOPort(0x1000, 0x1FFF));
  // bus.add(Bus16x8::IOPort(0x2000, 0x2FFF, &foo));
  bus.add(0x2000, &foo);

  bus.read(0x1000);
  bus.write(0x1000, 0x00);

  bus.write(0x2000, 0x66);
  bus.add(0x3000, &foo);
  bus.write(0x2000, 0x66);
  bus.write(0x2000, 0x66);
  bus.write(0x2000, 0x66);

  EXPECT_EQ(0x66, foo);
  EXPECT_EQ(0x66, bus.read(0x2000));

  EXPECT_THROW(bus.read(0x0000), BusError);
}

typedef DataBus<uint16_t, 16, uint16_t> Bus16x16;

TEST(BusTest, Bus16by16) {
  Bus16x16 bus;

  uint16_t foo = 0;
  bus.add(Bus16x16::IOPort(0x1000, 0x1FFF));
  bus.add(Bus16x16::IOPort(0x2000, 0x2FFF, Bus16x16::DataRead(&foo),
                           Bus16x16::DataWrite(&foo)));

  bus.read(0x1000);
  bus.write(0x1000, 0x0000);

  bus.write(0x2000, 0x6666);
  EXPECT_EQ(0x6666, foo);
  EXPECT_EQ(0x6666, bus.read(0x2000));

  EXPECT_THROW(bus.read(0x0000), BusError);
}

typedef DataBus<uint32_t, 21, uint8_t> Bus21x8;

TEST(BusTest, Bus21by8) {
  Bus21x8 bus;

  uint8_t bar = 0;
  bus.add(0x1F0000, 0x1F03FF, Bus21x8::DataRead(&bar),
          Bus21x8::DataWrite(&bar));
  uint8_t foo = 0;
  bus.add(0x1FE000, 0x1FE3FF, Bus21x8::DataRead(&foo),
          Bus21x8::DataWrite(&foo));

  EXPECT_EQ(0, bus.read(0x1FE048));
  bus.write(0x1FE048, 0x66);
  EXPECT_EQ(0x66, bus.read(0x1FE000));
  EXPECT_EQ(0x66, foo);
}

typedef DataBus<uint32_t, 24, uint16_t> Bus24x16;

TEST(BusTest, Bus24x16) {
  Bus24x16 bus;

  uint16_t foo = 0x2211;
  bus.add(Bus24x16::IOPort(0x1000, 0x1000, &foo, false));
  bus.add(Bus24x16::IOPort(0x100000, 0x100000, &foo, false));

  EXPECT_EQ(foo, bus.read(0x100000));
  EXPECT_EQ(bus.read(0x1000), bus.read(0x100000));
}
