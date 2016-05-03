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

#include "emu/screen.h"

using namespace EMU;

ScreenDevice::ScreenDevice(Machine *machine, const std::string &name, unsigned hertz,
    unsigned width, unsigned height, unsigned hbstart, unsigned hbend,
    unsigned vbstart, unsigned vbend):
    ClockedDevice(machine, name, hertz),
    m_hstate(HState::HStart), m_hpos(0),
    m_vstate(VState::VStart), m_vpos(0),
    m_width(width), m_height(height),
    m_hbstart(hbstart), m_hbend(hbend),
    m_vbstart(vbstart), m_vbend(vbend)
{
    m_hvisible = m_hbstart - m_hbend;
    m_vvisible = m_vbstart - m_vbend;
}

unsigned
ScreenDevice::visible_width(void) const
{
    return m_hvisible;
}

unsigned
ScreenDevice::visible_height(void) const
{
    return m_vvisible;
}

ScreenDevice::HState
ScreenDevice::next_hstate(unsigned *cycles_out)
{
    switch (m_hstate) {
    case HState::HStart:
        *cycles_out = m_hbend;
        return HState::HDraw;
    case HState::HDraw:
        *cycles_out = m_hbstart;
        return HState::HBlank;
    case HState::HBlank:
        *cycles_out = m_width;
        return HState::HEnd;
    case HState::HEnd:
        *cycles_out = 0;
        return HState::HStart;
    }
	assert(false);
}

ScreenDevice::VState
ScreenDevice::next_vstate(void)
{
    m_vpos++;

    auto it = m_callbacks.find(m_vpos);
    if (it != m_callbacks.end())
        it->second();

    switch (m_vstate) {
    case VState::VStart:
        if (m_vpos >= m_vbend) {
            m_vstate = VState::VDraw;
            do_vdraw();
        }
        break;
    case VState::VDraw:
        if (m_vpos >= m_vbstart) {
            m_vstate = VState::VBlank;
            do_vblank();
        }
        break;
    case VState::VBlank:
        if (m_vpos >= m_height) {
            m_vstate = VState::VEnd;
            do_vend();
        }
        break;
    case VState::VEnd:
        m_vstate = VState::VStart;
        m_vpos = 0;
        do_vstart();
        break;
    }

    return m_vstate;
}

void
ScreenDevice::set_hstate(HState state)
{
    m_hstate = state;
    switch (state) {
    case HState::HStart:
        m_hpos = 0;
        next_vstate();
        do_hstart();
        break;
    case HState::HDraw:
        m_hpos = m_hbend;
        do_hdraw();
        break;
    case HState::HBlank:
        m_hpos = m_hbstart;
        do_hblank();
        break;
    case HState::HEnd:
        m_hpos = 0;
        do_hend();
        break;
    }
}

void
ScreenDevice::execute(void)
{
    Cycles cycles_per_scanline(m_width);
    while (true) {
        unsigned cycles = 0;
        HState next = next_hstate(&cycles);
        add_icycles(cycles);
        set_hstate(next);
        Task::yield();
    }
}

void
ScreenDevice::register_callback(unsigned scanline, scanline_fn fn)
{
    m_callbacks.insert(make_pair(scanline, fn));
}
