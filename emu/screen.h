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

#include "emu/bits.h"
#include "emu/device.h"

namespace EMU {

class ScreenDevice: public ClockedDevice
{
public:
    ScreenDevice(Machine *machine, const std::string &name, unsigned hertz,
        unsigned width, unsigned height, unsigned hbstart, unsigned hbend,
        unsigned vbstart, unsigned vbend);

    virtual void execute(void);

    unsigned visible_width(void) const;
    unsigned visible_height(void) const;

protected:
    enum class HState {
        HStart,
        HDraw,
        HBlank,
        HEnd,
    };

    enum class VState {
        VStart,
        VDraw,
        VBlank,
        VEnd,
    };

    virtual void do_hstart(void) { }
    virtual void do_hdraw(void) { }
    virtual void do_hblank(void) { }
    virtual void do_hend(void) { }
    virtual void do_vstart(void) { }
    virtual void do_vdraw(void) { }
    virtual void do_vblank(void) { }
    virtual void do_vend(void) { }

private:

    HState next_hstate(unsigned *cycles_out);
    VState next_vstate(void);
    void set_hstate(HState state);
    void set_vstate(VState state);

    HState m_hstate;
    unsigned m_hpos;
    VState m_vstate;
    unsigned m_vpos;

    unsigned m_width;
    unsigned m_height;
    unsigned m_hbstart;
    unsigned m_hbend;
    unsigned m_vbstart;
    unsigned m_vbend;
    unsigned m_hvisible;
    unsigned m_vvisible;
};

};

