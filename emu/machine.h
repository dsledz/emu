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
class Machine: public Clockable {
public:
    /**
     * Initialize the machine with a master clock rate of @a hertz
     */
    Machine(void);
    virtual ~Machine(void);

    //virtual void power(void);
    virtual void execute(void);

    virtual void load_rom(const std::string &rom);
    virtual void execute(Time interval);

    /* Public functions */
    void set_switch(const std::string &name, const std::string &value);
    void send_input(InputKey key, bool pressed);
    void reset(void);
    void run(void);
    void set_screen(RasterScreen *screen);

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
    TimerItem_ptr add_timer(Time timeout, callback_t callback, bool periodic);
    void remove_timer(TimerItem_ptr timer);

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
    void reset_switches(void);

    void add_input(const InputSignal &signal);

    /**
     * Set an line on the device @a name.
     */
    void set_line(const std::string &name, Line line, LineState state);
    void set_line(Device *dev, Line line, LineState state);

    void add_screen(short width, short height,
        RasterScreen::Rotation rotation = RasterScreen::ROT0);
    RasterScreen *screen(void);

private:

    Device *dev(const std::string &name);
    void schedule_timer(TimerItem_ptr timer);

    InputMap _input;
    render_cb _render_cb;
    std::list<Device *> _devs;
    TimerQueue _timers;
    std::map<std::string, dipswitch_ptr> _switches;
    std::map<std::string, IOPort> _ports;

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
