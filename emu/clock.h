/*
 * Copyright (c) 2013, Dan Sledz
 * All rights reserved.
 */
#pragma once

#include "core/channel.h"
#include "core/task.h"

using namespace Core;

namespace EMU {

class Crystal: public Device{
public:
    Crystal(void);
    virtual ~Crystal(void);

    void add_device_clock(EmuClockBase *clock);
    void remove_device_clock(EmuClockBase *clock);

    virtual void line(Line line, LineState state);
    virtual void reset(void);
    virtual void execute(void);
    virtual void update(DeviceUpdate &update);

private:

    std::vector<EmuClockBase *> m_clocks;
    std::unique_ptr<Task> m_task;
};

};

