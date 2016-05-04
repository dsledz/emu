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

#include "driver/emulator.h"

using namespace EMU;

Emulator::Emulator(const Options &options):
    _clock(),
    _state(EmuState::Stopped),
    _options(options),
    _machine()
{
    Core::log.set_level(_options.log_level);
    /* XXX: Fix constness */
    _machine = loader()->load(const_cast<Options *>(&_options));
}

Emulator::~Emulator(void)
{
}

void
Emulator::start(void)
{
    set_state(EmuState::Running);
    _clock.resume();
}

void
Emulator::stop(void)
{
    set_state(EmuState::Stopped);
    /* XXX: task? */
    _clock.reset();
}

void
Emulator::pause(void)
{
    set_state(EmuState::Paused);
    _clock.pause();
}

void
Emulator::reset(void)
{
    set_state(EmuState::Paused);
    _machine->reset();
    set_state(EmuState::Running);
}

void
Emulator::render(void)
{
}

void
Emulator::do_execute(void)
{
    _machine->poweron();

    do {
        {
            lock_mtx lock(mtx);
            while (_state == EmuState::Paused)
                lock.wait(cv);
            if (_state == EmuState::Stopped)
                break;
        }

        _machine->run_forward(_clock.get_delta());
    } while (true);

    /* XXX: workaround */
    exit(0);
    //_machine->poweroff();
}

Emulator::EmuState
Emulator::get_state(void)
{
    std::lock_guard<std::mutex> lock(mtx);
    return _state;
}

void
Emulator::set_state(EmuState state)
{
    std::lock_guard<std::mutex> lock(mtx);
    _state = state;
    _clock.reset();
    cv.notify_one();
}

Machine *
Emulator::machine(void)
{
    return _machine.get();
}

const Options *
Emulator::options(void)
{
    return &_options;
}

void
Emulator::key_event(InputKey key, bool pressed)
{
    machine()->send_input(key, pressed);
}

FORCE_UNDEFINED_SYMBOL(galaga);
FORCE_UNDEFINED_SYMBOL(gb);
FORCE_UNDEFINED_SYMBOL(nes);
FORCE_UNDEFINED_SYMBOL(tg16);
FORCE_UNDEFINED_SYMBOL(pacman);
FORCE_UNDEFINED_SYMBOL(dkong);
