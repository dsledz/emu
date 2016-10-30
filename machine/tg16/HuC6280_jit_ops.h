#pragma once

#include "cpu/m6502/m6502_jit_ops.h"

#define JIT_OP(op, ...) \
  static inline bool op##_jit(JITEmitter *_jit, uint16_t pc, ##__VA_ARGS__)

#define JIT_ADDR(addr)                                                     \
  static inline void addr##_jit(JITEmitter *_jit,                          \
                                std::function<uint8_t(uint16_t)> bus_read, \
                                uint16_t pc)

#define JIT_OP_NYI(op, ...)                                                   \
  static inline bool op##_jit(JITEmitter *_jit, uint16_t pc, ##__VA_ARGS__) { \
    throw JITError(#op);                                                      \
  }

namespace HuC6280 {
using namespace M6502;
using namespace M65C02;

JIT_OP_NYI(CLA);

JIT_OP_NYI(CSH);

JIT_OP_NYI(SXY);

JIT_OP_NYI(SAX);

JIT_OP_NYI(SAY);

JIT_OP_NYI(SET);

JIT_OP_NYI(TDD);

JIT_OP_NYI(TII);

JIT_OP_NYI(TIN);

JIT_OP_NYI(TAI);

JIT_OP_NYI(TIA);

JIT_OP_NYI(CSL);

JIT_OP_NYI(CLX);

JIT_OP_NYI(CLY);

JIT_OP_NYI(BSR);

JIT_OP_NYI(TST_ZPG);

JIT_OP_NYI(TST_ZPGX);

JIT_OP_NYI(TST_ABS);

JIT_OP_NYI(TST_ABSX);

JIT_OP_NYI(TAM);

JIT_OP_NYI(TMA);

JIT_OP_NYI(ST0);

JIT_OP_NYI(ST1);

JIT_OP_NYI(ST2);
}
