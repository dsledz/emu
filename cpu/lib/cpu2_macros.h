/*
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

#define OPCODE(opnum, cycles, bytes, name, code)     \
  template <typename CpuState>                       \
  static inline void func_##opnum(CpuState *state);  \
  static const char *name_##opnum = name;            \
  static const Cycles cycles_##opnum(cycles);        \
  static const uint8_t bytes_##opnum = bytes;        \
  template <typename CpuState>                       \
  static inline void func_##opnum(CpuState *state) { \
    code;                                            \
  }

#define OPCODE_DEF(prefix, num)                            \
  {                                                        \
    0x##num, name_0x##prefix##num, cycles_0x##prefix##num, \
        bytes_0x##prefix##num, func_0x##prefix##num        \
  }

#define OPCODE16(prefix, num)                                 \
  OPCODE_DEF(prefix, num##0)                                  \
  , OPCODE_DEF(prefix, num##1), OPCODE_DEF(prefix, num##2),   \
      OPCODE_DEF(prefix, num##3), OPCODE_DEF(prefix, num##4), \
      OPCODE_DEF(prefix, num##5), OPCODE_DEF(prefix, num##6), \
      OPCODE_DEF(prefix, num##7), OPCODE_DEF(prefix, num##8), \
      OPCODE_DEF(prefix, num##9), OPCODE_DEF(prefix, num##A), \
      OPCODE_DEF(prefix, num##B), OPCODE_DEF(prefix, num##C), \
      OPCODE_DEF(prefix, num##D), OPCODE_DEF(prefix, num##E), \
      OPCODE_DEF(prefix, num##F)

#define OPCODE_SWITCH(prefix, op, state) \
  case 0x##op: {                     \
    func_##prefix##op(state);            \
    break;                       \
  }

#define OPCODE_SWITCH16(prefix, op, state) \
  OPCODE_SWITCH(prefix, op##0, state)      \
  OPCODE_SWITCH(prefix, op##1, state)      \
  OPCODE_SWITCH(prefix, op##2, state)      \
  OPCODE_SWITCH(prefix, op##3, state)      \
  OPCODE_SWITCH(prefix, op##4, state)      \
  OPCODE_SWITCH(prefix, op##5, state)      \
  OPCODE_SWITCH(prefix, op##6, state)      \
  OPCODE_SWITCH(prefix, op##7, state)      \
  OPCODE_SWITCH(prefix, op##8, state)      \
  OPCODE_SWITCH(prefix, op##9, state)      \
  OPCODE_SWITCH(prefix, op##A, state)      \
  OPCODE_SWITCH(prefix, op##B, state)      \
  OPCODE_SWITCH(prefix, op##C, state)      \
  OPCODE_SWITCH(prefix, op##D, state)      \
  OPCODE_SWITCH(prefix, op##E, state)      \
  OPCODE_SWITCH(prefix, op##F, state)

#define OPCODE_SWITCH_BLOCK(prefix, state) \
  switch (state->latch_op) {               \
    OPCODE_SWITCH16(prefix, 0, state)  \
    OPCODE_SWITCH16(prefix, 1, state)  \
    OPCODE_SWITCH16(prefix, 2, state)  \
    OPCODE_SWITCH16(prefix, 3, state)  \
    OPCODE_SWITCH16(prefix, 4, state)  \
    OPCODE_SWITCH16(prefix, 5, state)  \
    OPCODE_SWITCH16(prefix, 6, state)  \
    OPCODE_SWITCH16(prefix, 7, state)  \
    OPCODE_SWITCH16(prefix, 8, state)  \
    OPCODE_SWITCH16(prefix, 9, state)  \
    OPCODE_SWITCH16(prefix, A, state)  \
    OPCODE_SWITCH16(prefix, B, state)  \
    OPCODE_SWITCH16(prefix, C, state)  \
    OPCODE_SWITCH16(prefix, D, state)  \
    OPCODE_SWITCH16(prefix, E, state)  \
    OPCODE_SWITCH16(prefix, F, state)  \
  }


