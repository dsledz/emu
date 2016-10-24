/**
 * Copyright (c) 2016, Dan Sledz
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

#include "emu/emu.h"

#include "machine/c64/c64.h"
#include "machine/c64/m6510.h"

using namespace EMU;
using namespace C64Machine;

enum C64Keys {
  DELETE = 000,
  KEY_3 = 001,
  KEY_5 = 002,
  KEY_7 = 003,
  KEY_9 = 004,
  KEY_PLUS = 005,
  KEY_POUND = 006,
  KEY_1 = 007,
  RETURN = 010,
  KEY_W = 011,
  KEY_R = 012,
  KEY_Y = 013,
  KEY_I = 014,
  KEY_P = 015,
  KEY_STAR = 016,
  KEY_BSPC = 017,
  CRSR_RT = 020,
  KEY_A = 021,
  KEY_D = 022,
  KEY_G = 023,
  KEY_J = 024,
  KEY_L = 025,
  SEMI = 026,
  CTRL = 027,
  F7 = 030,
  KEY_4 = 031,
  KEY_6 = 032,
  KEY_8 = 033,
  KEY_0 = 034,
  DASH = 035,
  HOME = 036,
  KEY_2 = 037,
  F1 = 040,
  KEY_Z = 041,
  KEY_C = 042,
  KEY_B = 043,
  KEY_M = 044,
  DOT = 045,
  RSHIFT = 046,
  SPACE = 047,
  F3 = 050,
  KEY_S = 051,
  KEY_F = 052,
  KEY_H = 053,
  KEY_K = 054,
  COLON = 055,
  EQUALS = 056,
  CEQUALS = 057,
  F5 = 060,
  KEY_E = 061,
  KEY_T = 062,
  KEY_U = 063,
  KEY_O = 064,
  AT = 065,
  CARROT = 066,
  KEY_Q = 067,
  CRSR_DN = 070,
  LSHIFT = 071,
  KEY_X = 072,
  KEY_V = 073,
  KEY_N = 074,
  COMMA = 075,
  SLASH = 076,
  STOP = 077,
};

C64Keyboard::C64Keyboard(C64 *c64)
    : InputDevice(c64, "keyboard"),
      m_c64(c64),
      m_mask_row(0),
      m_mask_col(0),
      m_matrix{0} {
  c64->add_ioport("ROW1");
  IOPort *port = c64->ioport("ROW1");

  c64->add_input(InputSignal(InputKey::Keyboard3, port, 1, true));
}

C64Keyboard::~C64Keyboard(void) { }

byte_t
C64Keyboard::read_rows(void) {
  DEVICE_DEBUG("read_rows");
  byte_t result = 0;

  return result;
}

void
C64Keyboard::write_rows(byte_t value) {

}

byte_t
C64Keyboard::read_columns(void) {
  DEVICE_DEBUG("read_columns");
  byte_t result = 0;

  return result;
}

void
C64Keyboard::write_columns(byte_t value) {

}
