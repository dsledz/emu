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

#include "input.h"

using namespace EMU;

InputMap::InputMap(void)
{
}

InputMap::~InputMap(void)
{
}

void
InputMap::add_input(InputKey in, input_fn fn)
{
    std::pair<InputKey, input_fn> value (in, fn);
    if (_input_map.find(in) != _input_map.end())
        throw DuplicateInput(in);
    _input_map.insert(value);
}

void
InputMap::add(const InputSignal &signal)
{
    if (signal.active_high) {
        add_input(signal.key, [=](LineState state) {
            IOPort *port = signal.port;
            bit_set(port->value, signal.bit, state == LineState::Assert);
        });
    } else {
        add_input(signal.key, [=](LineState state) {
            IOPort *port = signal.port;
            bit_set(port->value, signal.bit, state == LineState::Clear);
        });
    }
}

void
InputMap::depress(InputKey in)
{
    auto it = _input_map.find(in);
    if (it != _input_map.end())
        it->second(LineState::Assert);
}

void
InputMap::release(InputKey in)
{
    auto it = _input_map.find(in);
    if (it != _input_map.end())
        it->second(LineState::Clear);
}

void
InputMap::pulse(InputKey in)
{
    auto it = _input_map.find(in);
    if (it != _input_map.end())
        it->second(LineState::Pulse);
}
