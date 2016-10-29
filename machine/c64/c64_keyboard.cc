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
  KEY_DEL = 000,
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

std::vector<std::pair<C64Keys, InputKey> > keys = {
std::make_pair(C64Keys::KEY_DEL, InputKey::KeyboardDelete),
std::make_pair(C64Keys::KEY_3, InputKey::Keyboard3),
std::make_pair(C64Keys::KEY_5, InputKey::Keyboard5),
std::make_pair(C64Keys::KEY_7, InputKey::Keyboard7),
std::make_pair(C64Keys::KEY_9, InputKey::Keyboard9),
std::make_pair(C64Keys::KEY_PLUS, InputKey::KeyboardPlus),
std::make_pair(C64Keys::KEY_POUND, InputKey::KeyboardPound),
std::make_pair(C64Keys::KEY_1, InputKey::Keyboard1),
std::make_pair(C64Keys::RETURN, InputKey::KeyboardReturn),
std::make_pair(C64Keys::KEY_W, InputKey::KeyboardW),
std::make_pair(C64Keys::KEY_R, InputKey::KeyboardR),
std::make_pair(C64Keys::KEY_Y, InputKey::KeyboardY),
std::make_pair(C64Keys::KEY_I, InputKey::KeyboardI),
std::make_pair(C64Keys::KEY_P, InputKey::KeyboardP),
std::make_pair(C64Keys::KEY_STAR, InputKey::KeyboardStar),
std::make_pair(C64Keys::KEY_BSPC, InputKey::KeyboardBspc),
std::make_pair(C64Keys::CRSR_RT, InputKey::KeyboardRight),
std::make_pair(C64Keys::KEY_A, InputKey::KeyboardA),
std::make_pair(C64Keys::KEY_D, InputKey::KeyboardD),
std::make_pair(C64Keys::KEY_G, InputKey::KeyboardG),
std::make_pair(C64Keys::KEY_J, InputKey::KeyboardJ),
std::make_pair(C64Keys::KEY_L, InputKey::KeyboardL),
std::make_pair(C64Keys::SEMI, InputKey::KeyboardSemicolon),
std::make_pair(C64Keys::CTRL, InputKey::KeyboardCtrl),
std::make_pair(C64Keys::F7, InputKey::KeyboardF7),
std::make_pair(C64Keys::KEY_4, InputKey::Keyboard4),
std::make_pair(C64Keys::KEY_6, InputKey::Keyboard6),
std::make_pair(C64Keys::KEY_8, InputKey::Keyboard8),
std::make_pair(C64Keys::KEY_0, InputKey::Keyboard0),
std::make_pair(C64Keys::DASH, InputKey::KeyboardDash),
std::make_pair(C64Keys::HOME, InputKey::KeyboardHome),
std::make_pair(C64Keys::KEY_2, InputKey::Keyboard2),
std::make_pair(C64Keys::F1, InputKey::KeyboardF1),
std::make_pair(C64Keys::KEY_Z, InputKey::KeyboardZ),
std::make_pair(C64Keys::KEY_C, InputKey::KeyboardC),
std::make_pair(C64Keys::KEY_B, InputKey::KeyboardB),
std::make_pair(C64Keys::KEY_M, InputKey::KeyboardM),
std::make_pair(C64Keys::DOT, InputKey::KeyboardPeriod),
std::make_pair(C64Keys::RSHIFT, InputKey::KeyboardRShift),
std::make_pair(C64Keys::SPACE, InputKey::KeyboardSpace),
std::make_pair(C64Keys::F3, InputKey::KeyboardF3),
std::make_pair(C64Keys::KEY_S, InputKey::KeyboardS),
std::make_pair(C64Keys::KEY_F, InputKey::KeyboardF),
std::make_pair(C64Keys::KEY_H, InputKey::KeyboardH),
std::make_pair(C64Keys::KEY_K, InputKey::KeyboardK),
std::make_pair(C64Keys::COLON, InputKey::KeyboardColon),
std::make_pair(C64Keys::EQUALS, InputKey::KeyboardEquals),
std::make_pair(C64Keys::CEQUALS, InputKey::KeyboardCEquals),
std::make_pair(C64Keys::F5, InputKey::KeyboardF5),
std::make_pair(C64Keys::KEY_E, InputKey::KeyboardE),
std::make_pair(C64Keys::KEY_T, InputKey::KeyboardT),
std::make_pair(C64Keys::KEY_U, InputKey::KeyboardU),
std::make_pair(C64Keys::KEY_O, InputKey::KeyboardO),
std::make_pair(C64Keys::AT, InputKey::KeyboardAt),
std::make_pair(C64Keys::CARROT, InputKey::KeyboardCaret),
std::make_pair(C64Keys::KEY_Q, InputKey::KeyboardQ),
std::make_pair(C64Keys::CRSR_DN, InputKey::KeyboardDown),
std::make_pair(C64Keys::LSHIFT, InputKey::KeyboardLShift),
std::make_pair(C64Keys::KEY_X, InputKey::KeyboardX),
std::make_pair(C64Keys::KEY_V, InputKey::KeyboardV),
std::make_pair(C64Keys::KEY_N, InputKey::KeyboardN),
std::make_pair(C64Keys::COMMA, InputKey::KeyboardComma),
std::make_pair(C64Keys::SLASH, InputKey::KeyboardSlash),
std::make_pair(C64Keys::STOP, InputKey::KeyboardStop)
};

C64Keyboard::C64Keyboard(C64 *c64)
    : InputDevice(c64, "keyboard"),
      m_c64(c64),
      m_row_mask(0),
      m_col_mask(0),
      m_matrix{LineState::Clear} {
  for (auto it = keys.begin(); it != keys.end(); it++) {
    c64->add_input(it->second, [=](LineState s) { on_key(it->first, s); });
  }
}

C64Keyboard::~C64Keyboard(void) { }

void
C64Keyboard::on_key(int key, LineState s) {
  m_matrix[key] = s;
}

byte_t
C64Keyboard::read_rows(void) {
  byte_t result = 0;

  return result;
}

void
C64Keyboard::write_rows(byte_t value) {
  m_row_mask = value;
}

byte_t
C64Keyboard::read_columns(void) {
  byte_t result = 0;
  for (int r = 7; r >= 0; r--) {
    if (bit_isset(m_row_mask, r))
      continue;
    for (int c = 0; c < 8; c++) {
      if (m_matrix[r << 3 | c] == LineState::Assert)
        bit_set(result, c, true);
    }
  }
  DEVICE_DEBUG("read_columns: ", Hex(result));

  return result;
}

void
C64Keyboard::write_columns(byte_t value) {
  m_col_mask = value;
}
