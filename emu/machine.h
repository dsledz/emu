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

namespace EMU {

/**
 * A sing shot machine event.
 */
struct Timer {
    Timer(Cycles deadline, callback_t callback):
        deadline(deadline), callback(callback), period(sec(0)) { }
    Timer(Cycles deadline, callback_t callback, Time period):
        deadline(deadline), callback(callback), period(period) { }
    Timer(callback_t callback, Time period):
        deadline(Cycles(0)), callback(callback), period(period) { }

    Cycles deadline;
    callback_t callback;
    Time period;
};

typedef std::shared_ptr<Timer> Timer_ptr;

class DeviceException: public EmuException {
public:
    DeviceException(const std::string &name): name(name) {}
    std::string name;
};

/**
 * Encapsulates a physical machine.
 */
class Machine {
public:
    /**
     * Initialize the machine with a master clock rate of @a hertz
     */
    Machine(unsigned hertz);
    ~Machine(void);

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
    Timer_ptr add_timer(Time delay, callback_t callback, Time period);
    void add_timer(Timer_ptr timer);
    void remove_timer(Timer_ptr timer);

    /**
     * Run for a single time quantum.
     */
    void run(void);

    Time quantum(void) {
        return Time(Time(sec(1)) / _quantum);
    }

    InputDevice *input(void) {
        return &_input;
    }

    /**
     * Return the device named @a name.
     */
    Device *dev(const std::string &name);

    RasterScreen *screen(void);
    void set_render(render_cb cb);

protected:
    std::list<Device *> _devs;
    std::list<Timer_ptr> _timers;
    Cycles _hertz;
    unsigned _quantum;
    Cycles _ic;
    InputDevice _input;
    RealTimeClock _rtc;
    std::unique_ptr<RasterScreen> _screen;
    render_cb _render_cb;
};

typedef std::unique_ptr<Machine> machine_ptr;

};
