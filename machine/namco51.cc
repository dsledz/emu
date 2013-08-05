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

#include "emu/emu.h"

#include "machine/namco51.h"

using namespace EMU;

Namco51::Namco51(Machine *machine):
    _mode(Mode::Unknown),
    _remap(false),
    _credits(0),
    _coinage_bytes(0),
    _read_count(0),
    _last_coin(0),
    _last_joy(0),
    _machine(machine)
{
}

Namco51::~Namco51(void)
{

}

void
Namco51::write8(offset_t offset, byte_t value)
{
    if (_coinage_bytes) {
        DBG("51xx coinage write");
        _coinage_bytes--;
        return;
    }

    switch (Command(value)) {
    case Command::Coinage:
        DBG("Coinage bytes");
        _coinage_bytes = 4;
        break;
    case Command::Credit:
        DBG("51xx Credit Mode");
        _mode = Mode::Credit;
        _read_count = 0;
        break;
    case Command::DisableRemap:
        _remap = false;
        break;
    case Command::EnableRemap:
        _remap = true;
        break;
    case Command::SwitchMode:
        DBG("51xx Switch Mode");
        _mode = Mode::Switch;
        _read_count = 0;
        break;
    default:
        break;
    }
}

byte_t
Namco51::read8(offset_t offset)
{
    if (_mode == Mode::Switch) {
        switch (_read_count++ % 3) {
        case 0: /* buttons & coins */
            DBG("51xx Buttons");
            return (read_port("IN0") | (read_port("IN1") << 4));
        case 1: /* joysticks */
            DBG("51xx Joystick");
            return (read_port("IN2") | (read_port("IN3") << 4));
        case 2: /* unusued */
            DBG("51xx Unused");
            return 0x00;
        }

    } else if (_mode == Mode::Credit) {
        switch (_read_count++ % 3) {
        case 0: {
            /* Credits */
            /* Read in the new keys */
            /* d = XXCC21XX C = coin, 1 = start1, 2 = start2 */
            int in = (~read_port("IN0") & 0xC) | (~read_port("IN1") << 4);
            /* See if we got credits. */
            if (bit_toggle(in, _last_coin, 5))
                _credits++;
            if (bit_toggle(in, _last_coin, 4))
                _credits++;

            /* XXX: This could miss credits if the keys are pressed at
             * the same time.
             */
            if (bit_toggle(in, _last_coin, 3) && _credits > 2)
                _credits -= 2;

            if (bit_toggle(in, _last_coin, 2) && _credits > 1)
                _credits -= 1;

            _last_coin = in;
            /* Credits in BCD */
            return (_credits % 10) | (_credits / 10) << 4;
        }
        case 1: {
            if (_remap)
                throw CpuFeatureFault("namco51", "remap");

            int joy = read_port("IN2") & 0x0f;
            int in = ~read_port("IN0") & 0x01;
            int toggle = in ^ _last_joy;

            /* One button press = one fire */
            _last_joy = (_last_joy & 0x02) | in;
            joy |= ((toggle & in) ^ 0x01) << 4;
            joy |= (in ^ 0x01) << 5;

            return joy;
        }
        case 2: {
            if (_remap)
                throw CpuFeatureFault("namco51", "remap");

            int joy = read_port("IN3") & 0x0f;
            int in = ~read_port("IN0") & 0x02;
            int toggle = in ^ _last_joy;
            _last_joy = (_last_joy & 0x01) | in;
            joy |= ((toggle & in) ^ 0x02) << 3;
            joy |= (in ^ 0x02) << 4;

            return joy;
        }
        }
    }

    DBG("51xx Unknown READ");

    return 0;
}

