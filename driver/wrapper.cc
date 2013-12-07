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

#include "driver/wrapper.h"
#include "driver/emulator.h"
#include "driver/opengl.h"

using namespace EMU;

#if OPENGL_LEGACY
#define GL_SCREEN GLFrameBuffer
#else
#define GL_SCREEN GLSLFrameBuffer
#endif

class OSXEmulator: public Emulator
{
public:
    OSXEmulator(const Options &options):
        Emulator(options)
    {
        _screen = std::unique_ptr<GL_SCREEN>(new GL_SCREEN());
        machine()->set_screen(_screen.get());

        _screen->init();
    }

    virtual ~OSXEmulator(void)
    {
    }

    void load(void)
    {
        machine()->load_rom(options()->rom);
        machine()->reset();

        set_state(EmuState::Paused);
        task = std::async(std::launch::async, &Emulator::do_execute, this);
    }

    void start(void)
    {
        set_state(EmuState::Running);
    }

    void render(void)
    {
        _screen->render();
    }

private:

    std::future<void> task;

    std::unique_ptr<GL_SCREEN> _screen;
};


struct emu
{
    std::string error;
    OSXEmulator *emulator;
};

struct emu *
emu_new(void)
{
    struct emu *state = new emu;
    state->emulator = NULL;
    return state;
}

void
emu_free(struct emu *state)
{
    if (state->emulator)
        delete state->emulator;
    delete state;
}

const char *
emu_error(struct emu *state)
{
    return state->error.c_str();
}

int
emu_machine_load(struct emu *state, const char *machine, const char *rom)
{
    try {
        state->error = "";
        Options opts;
        if (machine != NULL)
            opts.driver = machine;
        if (rom != NULL)
            opts.rom = rom;

        state->emulator = new OSXEmulator(opts);
        state->emulator->load();
    } catch (std::exception &e) {
        state->error = e.what();
        return 1;
    }
    return 0;
}

struct emu_machine *
emu_list_enum(struct emu *state)
{
    struct emu_machine *head = NULL;
    try {
        state->error = "";
        struct emu_machine **cur = &head;
        for (auto it = loader()->start(); it != loader()->end(); it++) {
            (*cur) = new emu_machine;
            (*cur)->driver = (*it)->name.c_str();
            (*cur)->name = (*it)->info.name.c_str();
            (*cur)->extension = (*it)->info.extension.c_str();
            (*cur)->next = NULL;
            cur = &(*cur)->next;
        }
    } catch (std::exception & e) {
        state->error = e.what();
        emu_list_free(head);
        return NULL;
    }
    return head;
}

void
emu_list_free(struct emu_machine *head)
{
    struct emu_machine *next;
    while (head != NULL) {
        next = head->next;
        delete head;
        head = next;
    }
}

int
emu_machine_start(struct emu *state)
{
    try {
        state->error = "";
        state->emulator->start();
    } catch (std::exception &e) {
        state->error = e.what();
        return 1;
    }
    return 0;
}

int
emu_machine_stop(struct emu *state)
{
    try {
        state->error = "";
        if (state->emulator != NULL)
            state->emulator->stop();
    } catch (std::exception &e) {
        state->error = e.what();
        return 1;
    }
    return 0;
}

int
emu_machine_pause(struct emu *state)
{
    try {
        state->error = "";
        state->emulator->pause();
    } catch (std::exception &e) {
        state->error = e.what();
        return 1;
    }
    return 0;
}

int
emu_machine_reset(struct emu *state)
{
    try {
        state->error = "";
        state->emulator->reset();
    } catch (std::exception &e) {
        state->error = e.what();
        return 1;
    }
    return 0;
}

int
emu_machine_render(struct emu *state)
{
    try {
        state->error = "";
        if (state->emulator != NULL)
            state->emulator->render();
    } catch (std::exception &e) {
        state->error = e.what();
        return 1;
    }
    return 0;
}

typedef std::tuple<std::string, enum emu_key, InputKey> key_tuple;

#define KEY(name, a, b) \
    std::make_tuple(name, a, b)

static std::vector<key_tuple> keys = {
    KEY("Joystick 1 Up", EMU_KEY_JOY1UP, InputKey::Joy1Up),
    KEY("Joystick 1 Down", EMU_KEY_JOY1DOWN, InputKey::Joy1Down),
    KEY("Joystick 1 Left", EMU_KEY_JOY1LEFT, InputKey::Joy1Left),
    KEY("Joystick 1 Right", EMU_KEY_JOY1RIGHT, InputKey::Joy1Right),
    KEY("Joystick 1 Btn1", EMU_KEY_JOY1BTN1, InputKey::Joy1Btn1),
    KEY("Joystick 1 Btn2", EMU_KEY_JOY1BTN2, InputKey::Joy1Btn2),
    KEY("Joystick 1 Select", EMU_KEY_JOY1SELECT, InputKey::Select1),
    KEY("Joystick 1 Start", EMU_KEY_JOY1START, InputKey::Start1),
};

int
emu_send_event(struct emu *state, const struct emu_event *event)
{
    for (auto it = keys.begin(); it != keys.end(); it++) {
        if (std::get<1>(*it) == event->key) {
            state->emulator->key_event(std::get<2>(*it), event->pressed);
            break;
        }
    }
    return 0;
}
