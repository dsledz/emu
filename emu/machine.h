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

#include "bits.h"
#include "timing.h"
#include "exception.h"
#include "device.h"
#include "video.h"
#include "dipswitch.h"
#include "options.h"

#include <map>

namespace EMU {

/**
 * A sing shot machine event.
 */
struct Timer {
    Timer(Time deadline, callback_t callback):
        deadline(deadline), callback(callback), period(sec(0)) { }
    Timer(Time deadline, callback_t callback, Time period):
        deadline(deadline), callback(callback), period(period) { }
    Timer(callback_t callback, Time period):
        deadline(time_zero), callback(callback), period(period) { }

    Time deadline;
    callback_t callback;
    Time period;
};

typedef std::shared_ptr<Timer> Timer_ptr;

struct KeyError: public EmuException {
    KeyError(const std::string &key):
        EmuException("Missing Key: "),
        key(key)
    {
        msg += key;
    }

    std::string key;
};

/**
 * Encapsulates a physical machine.
 * Each machine consists of:
 * Devices
 * Dipswitches
 * Timers
 * InputPorts
 */
class Machine {
public:
    /**
     * Initialize the machine with a master clock rate of @a hertz
     */
    Machine(void);
    virtual ~Machine(void);

    /**
     * Add a device to the machine.  This device is synchronized
     * by the machine, ticking based on @a freq
     */
    void add_device(Device *dev);
    void remove_device(Device *dev);

    /**
     * Add a timer. The timer will fire at the start of the next tick after
     * the deadline.
     */
    Timer_ptr add_timer(Time deadline, callback_t callback, Time period);
    void add_timer(Timer_ptr timer);
    void remove_timer(Timer_ptr timer);

    /**
     * Declare an IO line.  IO lines can be used to communicate between
     * different devices in a mailbox manner.
     */
    InputPort *add_input_port(const std::string &name);
    InputPort *input_port(const std::string &name);

    /**
     * Manage dipswitches
     */
    dipswitch_ptr add_switch(const std::string &name,
        const std::string &port, byte_t mask, byte_t def);
    void set_switch(const std::string &name, const std::string &value);
    void reset_switches(void);

    /**
     * Run for a single time quantum.
     */
    void run(void);
    virtual void execute(Time interval) {
    }

    Time quantum(void) {
        return Time(Time(sec(1)) / _quantum);
    }

    InputDevice *input(void) {
        return &_input;
    }

    /**
     * Set an line on the device @a name.
     */
    void set_line(const std::string &name, Line line, LineState state);
    /**
     * Set the line on @a dev.
     */
    void set_line(Device *dev, Line line, LineState state);

    RasterScreen *screen(void);
    void set_render(render_cb cb);
    void render(void) {
        if (_render_cb)
            _render_cb(_screen.get());
    }

protected:
    std::list<Device *> _devs;
    std::list<Timer_ptr> _timers;
    Time _clock;
    unsigned _quantum;
    InputDevice _input;
    std::unique_ptr<RasterScreen> _screen;
    render_cb _render_cb;
    std::map<std::string, dipswitch_ptr> _switches;
    std::map<std::string, InputPort> _ports;

private:

    Device *dev(const std::string &name);

    void _schedule_timer(Timer_ptr timer);
};

typedef std::unique_ptr<Machine> machine_ptr;

typedef std::function<machine_ptr (Options *opts)> machine_create_fn;

struct MachineDefinition {
    MachineDefinition(const std::string &name, machine_create_fn fn);
    ~MachineDefinition(void);

    void add(void);

    std::string name;
    machine_create_fn fn;
};

class MachineLoader {
public:
    MachineLoader();
    ~MachineLoader();

    void add_machine(struct MachineDefinition *definition);

    machine_ptr start(Options *opts);

private:
    std::list<MachineDefinition *> _machines;
};

extern MachineLoader loader;

#define FORCE_UNDEFINED_SYMBOL(x) \
    extern MachineDefinition x; \
    void * __ ## x ## _fp = (void *)&x;

};
