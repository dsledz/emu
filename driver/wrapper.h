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

#include <stdint.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

enum emu_key {
  EMU_KEY_NONE = 0,
  EMU_KEY_JOY1UP,
  EMU_KEY_JOY1DOWN,
  EMU_KEY_JOY1LEFT,
  EMU_KEY_JOY1RIGHT,
  EMU_KEY_JOY1BTN1,
  EMU_KEY_JOY1BTN2,
  EMU_KEY_JOY1SELECT,
  EMU_KEY_JOY1START,
  EMU_KEY_COIN1,
};

struct emu_key_info {
  enum emu_key key;
  const char *name;
};

struct emu_event {
  enum emu_key key;
  bool pressed;
};

struct emu;

struct emu *emu_new(void);

void emu_free(struct emu *state);

const char *emu_error(struct emu *state);

struct emu_machine {
  const char *name;
  const char *driver;
  const char *extension;
  struct emu_machine *next;
};

struct emu_machine *emu_list_enum(struct emu *state);

void emu_list_free(struct emu_machine *m);

int emu_machine_load(struct emu *state, const char *machine, const char *rom);

int emu_machine_start(struct emu *state);

int emu_machine_pause(struct emu *state);

int emu_machine_stop(struct emu *state);

int emu_machine_reset(struct emu *state);

int emu_machine_render(struct emu *state);

int emu_send_event(struct emu *state, const struct emu_event *event);

#ifdef __cplusplus
};
#endif
