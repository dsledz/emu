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
#pragma once

#include <cassert>
#include "bits.h"

namespace EMU {

/**
 * Device lines. Each line represents a possible connection between two
 * devices. Lines have no fixed source, but are always sent to a single
 * device. Shared lines (like reset) must be handled manually.
 *
 * The Lines are laid out to make it easy to map into one of the pre-defined
 * interrupt lines.
 */
enum class Line {
    INT0  =  0,
    INT1  =  1,
    INT2  =  2,
    INT3  =  3,
    INT4  =  4,
    INT5  =  5,
    INT6  =  6,
    INT7  =  7,

    POWER =  8,
    RESET =  9,
    WAIT  = 10,
    NMI   = 11,

    /* Address lines */
    A00   = 100,
    A01   = 101,
    A02   = 102,
    A03   = 103,
    A04   = 104,
    A05   = 105,
    A06   = 106,
    A07   = 107,
    A08   = 108,
    A09   = 109,
    A10   = 110,
    A11   = 111,
    A12   = 112,
    A13   = 113,
    A14   = 114,
    A15   = 115,

    /* Data lines */
    D00   = 200,
    D01   = 201,
    D02   = 202,
    D03   = 203,
    D04   = 204,
    D05   = 205,
    D06   = 206,
    D07   = 207,
    D08   = 208,
    D09   = 209,
    D10   = 210,
    D11   = 211,
    D12   = 212,
    D13   = 213,
    D14   = 214,
    D15   = 215,
};

static inline Line make_irq_line(unsigned i) {
    return Line(i);
}

static inline Line make_addr_line(unsigned i) {
    return Line(100 + i);
}

static inline Line make_data_line(unsigned i) {
    return Line(200 + i);
}

/**
 * New state of a line. The state usually represents the logical level
 * of the line.
 * For example, if the irq is active low, you'll still Assert when
 * you want to trigger an interrupt.
 */
enum class LineState {
    Clear  = 0,  /**< Line should be cleared. */
    Assert = 1,  /**< Line should be asserted. */
    Pulse  = 2,  /**< Line should be pulsed. */
};

/**
 * An external port, capable of holding data. Ports are used to communicate
 * between the machine and the outside world.
 * XXX: Sometimes ports are abused for inter-device communication.
 */
struct IOPort {
    IOPort(void) = default;

    byte_t value = 0;
};

};
