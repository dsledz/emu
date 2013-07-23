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

using namespace EMU;

Machine::Machine(void):
    _clock(time_zero),
    _quantum(6000)
{
}

Machine::~Machine(void)
{
    /* Make sure all devices are disconnected */
    assert(_devs.empty());
}

void
Machine::add_device(Device *dev)
{
    _devs.push_back(dev);
}

void
Machine::remove_device(Device *dev)
{
    _devs.remove(dev);
}

Timer_ptr
Machine::add_timer(Time deadline, callback_t callback, Time period)
{
    deadline += _clock;
    Timer_ptr timer(new Timer(deadline, callback, period));
    add_timer(timer);
    return timer;
}

void
Machine::_schedule_timer(Timer_ptr timer)
{
    auto it = _timers.begin();
    while (it != _timers.end()) {
        if ((*it)->deadline > timer->deadline)
            break;
        it++;
    }
    _timers.insert(it, timer);
}

void
Machine::add_timer(Timer_ptr timer)
{
    timer->deadline = _clock + timer->period;
    _schedule_timer(timer);
}

void
Machine::remove_timer(Timer_ptr timer)
{
    _timers.remove(timer);
}

void
Machine::run(void)
{
    Time end = _clock + quantum();
    Time interval(usec(20));
    while (_clock < end) {
        // Trigger any expired events
        while (!_timers.empty() && _timers.front()->deadline < _clock) {
            Timer_ptr t = _timers.front();
            _timers.pop_front();
            t->callback();
            if (t->period != time_zero) {
                /* XXX: This isn't exact */
                t->deadline += t->period;
                add_timer(t);
            }
        }

        execute(interval);
        for_each(_devs.begin(), _devs.end(), [=](Device *dev) {
                 dev->execute(interval);
                 });
        _clock += interval;
    }
}

Device *
Machine::dev(const std::string &name)
{
    for (auto it = _devs.begin(); it != _devs.end(); it++) {
        if ((*it)->name() == name)
            return *it;
    }
    throw KeyError(name);
}

RasterScreen *
Machine::screen(void)
{
    return _screen.get();
}

void
Machine::set_render(render_cb cb)
{
    _render_cb = cb;
}

InputPort *
Machine::add_input_port(const std::string &name)
{
    _ports.insert(make_pair(name, InputPort()));
    return &_ports[name];
}

InputPort *
Machine::input_port(const std::string &name)
{
    auto it = _ports.find(name);
    if (it == _ports.end())
        throw KeyError(name);
    return &it->second;
}

dipswitch_ptr
Machine::add_switch(const std::string &name, const std::string &port,
    byte_t mask, byte_t def)
{
    auto ptr = dipswitch_ptr(new Dipswitch(name, port, mask, def));
    _switches.insert(make_pair(name, ptr));
    return ptr;
}

void
Machine::set_switch(const std::string &name, const std::string &value)
{
    auto sw = _switches.find(name);
    if (sw == _switches.end())
        throw KeyError(name);
    sw->second->select(this, value);
}

void
Machine::reset_switches(void)
{
    for (auto it = _switches.begin(); it != _switches.end(); it++)
        it->second->set_default(this);
}

void
Machine::set_line(const std::string &name, Line line, LineState state)
{
    set_line(dev(name), line, state);
}

void
Machine::set_line(Device *dev, Line line, LineState state)
{
    /* XXX: Handle pulse */
    dev->line(line, state);
}

MachineLoader EMU::loader __used;

MachineDefinition::MachineDefinition(
    const std::string &name,
    machine_create_fn fn):
    name(name),
    fn(fn)
{
    loader.add_machine(this);
}

MachineDefinition::~MachineDefinition(void)
{
    /* XXX: Should we remove the entry? */
}

void
MachineDefinition::add(void)
{
    loader.add_machine(this);
}

MachineLoader::MachineLoader(void)
{
}

MachineLoader::~MachineLoader(void)
{
}

void
MachineLoader::add_machine(struct MachineDefinition *definition)
{
    _machines.push_back(definition);
}

machine_ptr
MachineLoader::start(Options *opts)
{
    machine_ptr machine;
    for (auto it = _machines.begin(); it != _machines.end(); it++) {
        if ((*it)->name == opts->driver)
            return (*it)->fn(opts);
    }
    throw KeyError(opts->driver);
}

