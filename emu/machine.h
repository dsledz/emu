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
#include "emu/timing.h"
#include "emu/exception.h"
#include "emu/device.h"
#include "emu/video.h"
#include "emu/dipswitch.h"
#include "emu/options.h"

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
 * IOPorts
 */
class Machine {
public:
    /**
     * Initialize the machine with a master clock rate of @a hertz
     */
    Machine(void);
    virtual ~Machine(void);

    virtual void load_rom(const std::string &rom);
    virtual void execute(Time interval);

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
     * Declare an IO port with the name of @a name.
     * Ports 
     * IO lines can be used to communicate between
     * different devices in a mailbox manner.
     */
    void add_ioport(const std::string &name);
    IOPort *ioport(const std::string &name);
    byte_t read_ioport(const std::string &name);
    byte_t read_ioport(IOPort *port);
    void write_ioport(const std::string &name, byte_t value);
    void write_ioport(IOPort *port, byte_t value);

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

    Time quantum(void) {
        return Time(Time(sec(1)) / _quantum);
    }

    InputMap *input(void) {
        return &_input;
    }

    /**
     * Reset all devices.
     */
    void reset(void);

    /**
     * Set an line on the device @a name.
     */
    void set_line(const std::string &name, Line line, LineState state);

    /**
     * Set the line on @a dev.
     */
    void set_line(Device *dev, Line line, LineState state);

    void add_screen(short width, short height,
        RasterScreen::Rotation rotation = RasterScreen::ROT0);
    void set_screen(RasterScreen *screen);

    RasterScreen *screen(void);
    void set_render(render_cb cb);
    void render(void) {
        if (_render_cb && _screen)
            _render_cb(_screen);
    }

protected:
    std::list<Device *> _devs;
    std::list<Timer_ptr> _timers;
    Time _clock;
    unsigned _quantum;
    InputMap _input;
    render_cb _render_cb;
    std::map<std::string, dipswitch_ptr> _switches;
    std::map<std::string, IOPort> _ports;

private:

    Device *dev(const std::string &name);
    void _schedule_timer(Timer_ptr timer);

    RasterScreen *_screen;
    short _screen_width;
    short _screen_height;
    RasterScreen::Rotation _screen_rot;
};

typedef std::unique_ptr<Machine> machine_ptr;

typedef std::function<machine_ptr (Options *opts)> machine_create_fn;

struct MachineInformation {
    std::string name;
    std::string year;
    std::string extension;
    bool cartridge;
};

struct MachineDefinition {
    MachineDefinition(
        const std::string &name,
        const MachineInformation &info,
        machine_create_fn fn);
    ~MachineDefinition(void);

    void add(void);

    std::string name;
    MachineInformation info;
    machine_create_fn fn;
};

class MachineLoader {
public:
    MachineLoader();
    ~MachineLoader();

    void add_machine(struct MachineDefinition *definition);

    const struct MachineDefinition *find(const std::string &name);
    std::list<MachineDefinition *>::const_iterator start();
    std::list<MachineDefinition *>::const_iterator end();

    machine_ptr load(Options *opts);

private:
    std::list<MachineDefinition *> _machines;
};

MachineLoader * loader(void);

#define FORCE_UNDEFINED_SYMBOL(x) \
    extern MachineDefinition x; \
    void * __ ## x ## _fp = (void *)&x;

};
