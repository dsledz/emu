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

#include "emu.h"

#include "namco51.h"

using namespace EMU;

Namco51::Namco51(Machine *machine, Device *parent):
    Device(machine, "51xx"),
    _mode(Namco51Mode::Unknown),
    _remap(false),
    _coinage_bytes(0),
    _read_count(0),
    _in()
{
    _in[0] = 0x0F;
    _in[1] = 0x0F;
    _in[2] = 0x0F;
    _in[3] = 0x0F;
    InputDevice *input = machine->input();
    input->add_input(InputKey::Joy1Btn1, [&](LineState state) {
        bit_set(_in[0], 0, LineState::Clear == state); });
    input->add_input(InputKey::Joy2Btn1, [&](LineState state) {
        bit_set(_in[0], 1, LineState::Clear == state); });
    input->add_input(InputKey::Start1, [&](LineState state) {
        bit_set(_in[0], 2, LineState::Clear == state); });
    input->add_input(InputKey::Start2, [&](LineState state) {
        bit_set(_in[0], 3, LineState::Clear == state); });

    input->add_input(InputKey::Coin1, [&](LineState state) {
        bit_set(_in[1], 0, LineState::Clear == state); });
    input->add_input(InputKey::Coin2, [&](LineState state) {
        bit_set(_in[1], 1, LineState::Clear == state); });
    input->add_input(InputKey::Service, [&](LineState state) {
        bit_set(_in[1], 2, LineState::Clear == state); });

    input->add_input(InputKey::Joy1Right, [&](LineState state) {
        bit_set(_in[2], 1, LineState::Clear == state); });
    input->add_input(InputKey::Joy1Left, [&](LineState state) {
        bit_set(_in[2], 3, LineState::Clear == state); });

    input->add_input(InputKey::Joy2Right, [&](LineState state) {
        bit_set(_in[3], 1, LineState::Clear == state); });
    input->add_input(InputKey::Joy2Left, [&](LineState state) {
        bit_set(_in[3], 3, LineState::Clear == state); });
}

Namco51::~Namco51(void)
{

}

void
Namco51::save(SaveState &state)
{
}

void
Namco51::load(LoadState &state)
{
}

void
Namco51::tick(unsigned cycles)
{
}

void
Namco51::set_line(InputLine line, LineState state)
{
    switch (line) {
    case InputLine::RESET:
        DEBUG("51xx RESET");
        break;
    default:
        break;
    }
}

void
Namco51::write(addr_t addr, byte_t value)
{
    if (_coinage_bytes) {
        DEBUG("51xx coinage write");
        _coinage_bytes--;
    }

    switch (Namco51Command(value)) {
    case Namco51Command::Coinage:
        DEBUG("Coinage bytes");
        _coinage_bytes = 4;
        break;
    case Namco51Command::Credit:
        DEBUG("51xx Credit Mode");
        _mode = Namco51Mode::Credit;
        _read_count = 0;
        break;
    case Namco51Command::DisableRemap:
        _remap = false;
        break;
    case Namco51Command::EnableRemap:
        _remap = true;
        break;
    case Namco51Command::SwitchMode:
        DEBUG("51xx Switch Mode");
        _mode = Namco51Mode::Switch;
        _read_count = 0;
        break;
    default:
        break;
    }
}

byte_t
Namco51::read(addr_t addr)
{
    if (_mode == Namco51Mode::Switch) {
        switch (_read_count++ % 3) {
        case 0: /* buttons & coins */
            DEBUG("51xx Buttons");
            return (_in[0] | (_in[1] << 4));
        case 1: /* joysticks */
            DEBUG("51xx Joystick");
            return (_in[2] | (_in[3] << 4));
        case 2: /* unusued */
            DEBUG("51xx Unused");
            return 0x00;
        }

    } else if (_mode == Namco51Mode::Credit) {
        switch (_read_count++ % 3) {
        case 0: /* Credits */
            /* XXX: Lie about credits for now. */
            return 0x09;
        case 1: /* buttons and joy */
            return 0x00;
        case 2: /* coins and cocktail joy*/
            return 0x00;
        }
    }

    DEBUG("51xx Unknown READ");

    return 0;
}
