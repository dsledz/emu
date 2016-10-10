#pragma once

#define JIT_OP(op, ...) \
  static inline bool op##_jit(JITEmitter *_jit, uint16_t pc, ##__VA_ARGS__)

#define JIT_ADDR(addr)                                                     \
  static inline void addr##_jit(JITEmitter *_jit,                          \
                                std::function<uint8_t(uint16_t)> bus_read, \
                                uint16_t pc)

/* M6502 Registers */
#define RegA RegIdx8::RegBL
#define RegSP RegIdx8::RegBH
#define RegX RegIdx8::RegCL
#define RegY RegIdx8::RegCH

/* Temporary Register */
#define RegTMP RegIdx16::RegAX
#define RegTMPl RegIdx8::RegAL
#define RegTMPh RegIdx8::RegAH

/* Effective address */
#define RegEA RegIdx16::RegDX
#define RegEAh RegIdx8::RegDH
#define RegEAl RegIdx8::RegDL
#define RegState RegIdx64L::RegR15

namespace M6502 {
static inline void set_flag(JITEmitter *_jit, int bit, bool value) {
  _jit->xMOV16(RegEA, RegState, 4);
  _jit->xPUSHF();
  if (value)
    _jit->xBTS(RegEA, bit);
  else
    _jit->xBTR(RegEA, bit);
  _jit->xPOPF();
  _jit->xMOV16(RegState, 4, RegEA);
}

static inline void pop(JITEmitter *_jit, RegIdx8 dst) {
  assert(dst != RegEAh && dst != RegEAl);
  /* XXX: zero page */
  uint8_t ZPG = 0;
  uint8_t tmp = ZPG + 0x01;
  _jit->xPUSHF();
  _jit->xPUSH(RegEA);
  _jit->xINC(RegSP);
  _jit->xMOV8(RegEAh, tmp);
  _jit->xMOV8(RegEAl, RegSP);
  _jit->xLOAD(dst, RegEA);
  _jit->xPOP(RegEA);
  _jit->xPOPF();
}

static inline void push(JITEmitter *_jit, RegIdx8 src) {
  assert(src != RegEAl && src != RegEAh);
  uint8_t ZPG = 0;
  uint8_t tmp = ZPG + 0x01;
  _jit->xPUSHF();
  _jit->xPUSH(RegEA);
  _jit->xMOV8(RegEAh, tmp);
  _jit->xMOV8(RegEAl, RegSP);
  _jit->xSTORE(RegEA, src);
  _jit->xDEC(RegSP);
  _jit->xPOP(RegEA);
  _jit->xPOPF();
}

static inline void pop_flags(JITEmitter *_jit) {
  _jit->xPUSHF16();
  _jit->xPOP(RegEA);

  _jit->xMOV8(RegTMPh, 0);
  pop(_jit, RegTMPl);

  _jit->xPUSH(RegTMP);

  _jit->xBTR(RegEA, Flags::SF);
  _jit->xBT(RegTMP, 7);
  _jit->xSETCC(RegTMPl, Condition::CFSet);
  _jit->xROL(RegTMP, Flags::SF);
  _jit->xOR(RegEA, RegTMP);

  _jit->xPOP(RegTMP);
  _jit->xPUSH(RegTMP);

  _jit->xBTR(RegEA, Flags::ZF);
  _jit->xBT(RegTMP, 1);
  _jit->xSETCC(RegTMPl, Condition::CFSet);
  _jit->xROL(RegTMP, Flags::ZF);
  _jit->xOR(RegEA, RegTMP);

  _jit->xPOP(RegTMP);
  _jit->xPUSH(RegTMP);

  _jit->xBTR(RegEA, Flags::OF);
  _jit->xBT(RegTMP, 6);
  _jit->xSETCC(RegTMPl, Condition::CFSet);
  _jit->xROL(RegTMP, Flags::OF);
  _jit->xOR(RegEA, RegTMP);

  _jit->xPOP(RegTMP);

  _jit->xBTR(RegEA, Flags::CF);
  _jit->xBT(RegTMP, 0);
  _jit->xSETCC(RegTMPl, Condition::CFSet);
  _jit->xROL(RegTMP, Flags::CF);
  _jit->xOR(RegEA, RegTMP);

  _jit->xPUSH(RegEA);
  _jit->xPOPF16();
}

static inline void set_sz(JITEmitter *_jit, RegIdx8 src) {
  _jit->xPUSHF16();
  _jit->xPOP(RegEA);

  _jit->xBTR(RegEA, Flags::SF);
  _jit->xMOV8(RegTMPh, 0);
  _jit->xMOV8(RegTMPl, src);
  _jit->xBT(RegTMP, 7);
  _jit->xSETCC(RegTMPl, Condition::CFSet);
  _jit->xROL(RegTMP, Flags::SF);
  _jit->xOR(RegEA, RegTMP);

  _jit->xBTR(RegEA, Flags::ZF);
  _jit->xCMP(src, 0);
  _jit->xMOV8(RegTMPh, 0);
  _jit->xSETCC(RegTMPl, Condition::ZFSet);
  _jit->xROL(RegTMP, Flags::ZF);
  _jit->xOR(RegEA, RegTMP);

  _jit->xPUSH(RegEA);
  _jit->xPOPF16();
}

JIT_ADDR(Inherent) {}

JIT_ADDR(Absolute) {
  uint16_t tmp = bus_read(pc + 1) | (bus_read(pc + 2) << 8);
  _jit->xMOV16(RegEA, tmp);
}

JIT_ADDR(AbsoluteX) {
  uint16_t tmp = bus_read(pc + 1) | (bus_read(pc + 2) << 8);
  _jit->xAbsolute(RegEA, RegX, tmp);
}

JIT_ADDR(AbsoluteY) {
  uint16_t tmp = bus_read(pc + 1) | (bus_read(pc + 2) << 8);
  _jit->xAbsolute(RegEA, RegY, tmp);
}

JIT_ADDR(Immediate) {
  uint16_t tmp = pc + 1;
  _jit->xMOV16(RegEA, tmp);
}

JIT_ADDR(Indirect) {
  uint16_t tmp = bus_read(pc + 1) | (bus_read(pc + 2) << 8);
  _jit->xPUSHF();
  _jit->xMOV16(RegEA, tmp);
  _jit->xLOAD(RegTMPl, RegEA);
  _jit->xINC(RegEAl);
  _jit->xLOAD(RegTMPh, RegEA);
  _jit->xMOV16(RegEA, RegTMP);
  _jit->xPOPF();
}

JIT_ADDR(Indirect_Fixed) {
  uint16_t tmp = bus_read(pc + 1) | (bus_read(pc + 2) << 8);
  _jit->xPUSHF();
  _jit->xMOV16(RegEA, tmp);
  _jit->xLOAD(RegTMPl, RegEA);
  _jit->xINC16(RegEA);
  _jit->xLOAD(RegTMPh, RegEA);
  _jit->xMOV16(RegEA, RegTMP);
  _jit->xPOPF();
}

JIT_ADDR(ZeroIndirect) {
  uint8_t ZPG = 0;
  uint16_t tmp = bus_read(pc + 1) | (ZPG << 8);
  _jit->xPUSHF();
  _jit->xMOV16(RegEA, tmp);
  _jit->xLOAD(RegTMPl, RegEA);
  _jit->xINC(RegEAl);
  _jit->xLOAD(RegTMPh, RegEA);
  _jit->xMOV16(RegEA, RegTMP);
  _jit->xPOPF();
}

JIT_ADDR(IndirectY) {
  uint8_t ZPG = 0;
  uint16_t tmp = bus_read(pc + 1) + (ZPG << 8);
  _jit->xPUSHF();
  _jit->xMOV16(RegEA, tmp);
  _jit->xLOAD(RegTMPl, RegEA);
  _jit->xINC(RegEAl);
  _jit->xLOAD(RegTMPh, RegEA);
  _jit->xMOV8(RegEAh, 0x00);
  _jit->xMOV8(RegEAl, RegY);
  _jit->xADD(RegTMP, RegEA);
  _jit->xMOV16(RegEA, RegTMP);
  _jit->xPOPF();
}

JIT_ADDR(Relative) {
  char tmp = bus_read(pc + 1);
  uint16_t addr = pc + 2 + tmp;
  _jit->xMOV16(RegEA, addr);
}

JIT_ADDR(XIndirect) {
  uint8_t ZPG = 0;
  uint16_t tmp = bus_read(pc + 1) + (ZPG << 8);
  _jit->xPUSHF();
  _jit->xMOV16(RegEA, tmp);
  _jit->xMOV8(RegTMPh, 0);
  _jit->xMOV8(RegTMPl, RegX);
  _jit->xADD(RegEA, RegTMP);
  _jit->xLOAD(RegTMPl, RegEA);
  _jit->xADD(RegEAl, 1);
  _jit->xLOAD(RegTMPh, RegEA);
  _jit->xMOV16(RegEA, RegTMP);
  _jit->xPOPF();
}

JIT_ADDR(ZeroPage) {
  /* XXX: zero page */
  uint8_t ZPG = 0;
  uint8_t tmp = bus_read(pc + 1);
  _jit->xMOV8(RegEAl, tmp);
  _jit->xMOV8(RegEAh, ZPG);
}

JIT_ADDR(ZeroPageX) {
  /* XXX: zero page */
  uint8_t ZPG = 0;
  uint16_t tmp = bus_read(pc + 1);
  _jit->xAbsolute(RegEA, RegX, tmp);
  _jit->xMOV8(RegEAh, ZPG);
}

JIT_ADDR(ZeroPageY) {
  /* XXX: zero page */
  uint8_t ZPG = 0;
  uint16_t tmp = bus_read(pc + 1);
  _jit->xAbsolute(RegEA, RegY, tmp);
  _jit->xMOV8(RegEAh, ZPG);
}

JIT_OP(ADC) {
  _jit->xADC(RegA, RegTMPl, RegEA);
  set_sz(_jit, RegA);
  return true;
}

JIT_OP(AND) {
  _jit->xAND(RegA, RegTMPl, RegEA);
  set_sz(_jit, RegA);
  return true;
}

JIT_OP(ASL) {
  _jit->xROL(RegTMPl, RegEA);
  return true;
}

JIT_OP(ASLA) {
  _jit->xROL(RegA);
  return true;
}

JIT_OP(BCC) {
  _jit->xBR(Condition::CFClear, RegEA);
  return false;
}

JIT_OP(BCS) {
  _jit->xBR(Condition::CFSet, RegEA);
  return false;
}

JIT_OP(BEQ) {
  _jit->xBR(Condition::ZFSet, RegEA);
  return false;
}

JIT_OP(BIT) {
  _jit->xPUSHF16();
  _jit->xMOV8(RegTMPh, RegA);
  _jit->xAND(RegTMPh, RegTMPl, RegEA);

  /* Update Z, V, and N */
  _jit->xPOP(RegEA);
  _jit->xPUSH(RegTMP);

  _jit->xBTR(RegEA, Flags::OF);
  _jit->xBT(RegTMP, 6);
  _jit->xSETCC(RegTMPl, Condition::CFSet);
  _jit->xMOV8(RegTMPh, 0);
  _jit->xROL(RegTMP, Flags::OF);
  _jit->xOR(RegEA, RegTMP);

  _jit->xPOP(RegTMP);
  _jit->xPUSH(RegTMP);

  _jit->xBTR(RegEA, Flags::ZF);
  _jit->xCMP(RegTMPh, 0);
  _jit->xSETCC(RegTMPl, Condition::ZFSet);
  _jit->xMOV8(RegTMPh, 0);
  _jit->xROL(RegTMP, Flags::ZF);
  _jit->xOR(RegEA, RegTMP);

  _jit->xPOP(RegTMP);

  _jit->xBTR(RegEA, Flags::SF);
  _jit->xBT(RegTMP, 7);
  _jit->xSETCC(RegTMPl, Condition::CFSet);
  _jit->xMOV8(RegTMPh, 0);
  _jit->xROL(RegTMP, Flags::SF);
  _jit->xOR(RegEA, RegTMP);

  _jit->xPUSH(RegEA);
  _jit->xPOPF16();

  return true;
}

JIT_OP(BMI) {
  _jit->xBR(Condition::SFSet, RegEA);
  return false;
}

JIT_OP(BNE) {
  _jit->xBR(Condition::ZFClear, RegEA);
  return false;
}

JIT_OP(BPL) {
  _jit->xBR(Condition::SFClear, RegEA);
  return false;
}

JIT_OP(BRK) {
  /* XXX: BRK */
  return false;
}

JIT_OP(BVC) {
  _jit->xBR(Condition::OFClear, RegEA);
  return false;
}

JIT_OP(BVS) {
  _jit->xBR(Condition::OFSet, RegEA);
  return false;
}

JIT_OP(CLC) {
  _jit->xCLC();
  return true;
}

JIT_OP(CLD) {
  set_flag(_jit, 3, false);
  return true;
}

JIT_OP(CLI) {
  set_flag(_jit, 4, false);
  return true;
}

JIT_OP(CLV) {
  _jit->xPUSHF16();
  _jit->xPOP(RegTMP);
  _jit->xBTR(RegTMP, Flags::OF);
  _jit->xPUSH(RegTMP);
  _jit->xPOPF16();
  return true;
}

JIT_OP(CMP) {
  _jit->xCMP(RegA, RegTMPl, RegEA);
  return true;
}

JIT_OP(CPX) {
  _jit->xCMP(RegX, RegTMPl, RegEA);
  return true;
}

JIT_OP(CPY) {
  _jit->xCMP(RegY, RegTMPl, RegEA);
  return true;
}

JIT_OP(DEC) {
  _jit->xDEC(RegTMPl, RegEA);
  // set_sz(_jit, RegTMPl);
  return true;
}

JIT_OP(DEX) {
  _jit->xDEC(RegX);
  set_sz(_jit, RegX);
  return true;
}

JIT_OP(DEY) {
  _jit->xDEC(RegY);
  set_sz(_jit, RegY);
  return true;
}

JIT_OP(EOR) {
  _jit->xXOR(RegA, RegTMPl, RegEA);
  set_sz(_jit, RegA);
  return true;
}

JIT_OP(INC) {
  _jit->xINC(RegTMPl, RegEA);
  set_sz(_jit, RegTMPl);
  return true;
}

JIT_OP(INX) {
  _jit->xINC(RegX);
  set_sz(_jit, RegX);
  return true;
}

JIT_OP(INY) {
  _jit->xINC(RegY);
  set_sz(_jit, RegY);
  return true;
}

JIT_OP(JMP) {
  _jit->xSETPC(RegEA);
  return false;
}

JIT_OP(JSR) {
  pc += 3;
  pc -= 1;
  _jit->xMOV8(RegTMPh, (pc & 0xFF00) >> 8);
  _jit->xMOV8(RegTMPl, (pc & 0xFF));
  push(_jit, RegTMPh);
  push(_jit, RegTMPl);
  _jit->xSETPC(RegEA);
  return false;
}

JIT_OP(LDA) {
  _jit->xLOAD(RegA, RegEA);
  set_sz(_jit, RegA);
  return true;
}

JIT_OP(LDX) {
  _jit->xLOAD(RegX, RegEA);
  set_sz(_jit, RegX);
  return true;
}

JIT_OP(LDY) {
  _jit->xLOAD(RegY, RegEA);
  set_sz(_jit, RegY);
  return true;
}

JIT_OP(LSR) {
  _jit->xROR(RegTMPl, RegEA);
  set_sz(_jit, RegTMPl);
  return true;
}

JIT_OP(LSRA) {
  _jit->xROR(RegA);
  set_sz(_jit, RegA);
  return true;
}

JIT_OP(NOP) {
  /* XXX: NOP */
  return true;
}

JIT_OP(ORA) {
  _jit->xOR(RegA, RegTMPl, RegEA);
  set_sz(_jit, RegA);
  return true;
}

JIT_OP(PHA) {
  push(_jit, RegA);
  return true;
}

JIT_OP(PHP) {
  _jit->xPUSHF();
  _jit->xPUSH(RegEA);
  _jit->xMOV16(RegEA, 0);
  _jit->xPUSHF16();

  _jit->xPOP(RegTMP);
  _jit->xPUSH(RegTMP);
  _jit->xAND(RegTMP, 1 << Flags::CF);
  /* CF and C are both 0 */
  _jit->xOR(RegEA, RegTMP);

  _jit->xPOP(RegTMP);
  _jit->xPUSH(RegTMP);
  _jit->xAND(RegTMP, 1 << Flags::ZF);
  _jit->xROR(RegTMP, Flags::ZF);
  _jit->xROL(RegTMP, 1);
  _jit->xOR(RegEA, RegTMP);

  _jit->xPOP(RegTMP);
  _jit->xPUSH(RegTMP);
  _jit->xAND(RegTMP, 1 << Flags::OF);
  _jit->xROR(RegTMP, Flags::OF);
  _jit->xROL(RegTMP, 6);
  _jit->xOR(RegEA, RegTMP);

  _jit->xPOP(RegTMP);
  _jit->xPUSH(RegTMP);
  _jit->xAND(RegTMP, 1 << Flags::SF);
  _jit->xROR(RegTMP, Flags::SF);
  _jit->xROL(RegTMP, 7);
  _jit->xOR(RegEA, RegTMP);

  /* RegEAl contains the new 4 bits */
  _jit->xMOV16(RegTMP, RegState, 4);
  _jit->xAND(RegTMPl, 0x3C);
  _jit->xOR(RegTMPl, RegEAl);
  _jit->xMOV16(RegState, 4, RegTMP);

  /* XXX: zero page */
  push(_jit, RegTMPl);
  _jit->xPOPF16();
  _jit->xPOP(RegEA);
  _jit->xPOPF();
  return true;
}

JIT_OP(PLA) {
  pop(_jit, RegA);
  set_sz(_jit, RegA);
  return true;
}

JIT_OP(PLP) {
  pop_flags(_jit);
  /* XXX: flags */
  return true;
}

JIT_OP(ROL) {
  _jit->xRCL(RegTMPl, RegEA);
  return true;
}

JIT_OP(ROLA) {
  _jit->xRCL(RegA);
  set_sz(_jit, RegA);
  return true;
}

JIT_OP(ROR) {
  _jit->xRCR(RegTMPl, RegEA);
  set_sz(_jit, RegTMPl);
  return true;
}

JIT_OP(RORA) {
  _jit->xRCR(RegA);
  set_sz(_jit, RegA);
  return true;
}

JIT_OP(RTI) {
  pop_flags(_jit);
  /* Set E */
  set_flag(_jit, 5, true);
  pop(_jit, RegTMPl);
  pop(_jit, RegTMPh);
  _jit->xSETPC(RegTMP);
  return false;
}

JIT_OP(RTS) {
  pop(_jit, RegTMPl);
  pop(_jit, RegTMPh);
  _jit->xPUSHF();
  _jit->xADD(RegTMP, 1);
  _jit->xPOPF();
  _jit->xSETPC(RegTMP);
  return false;
}

JIT_OP(SBC) {
  _jit->xSBC(RegA, RegTMPl, RegEA);
  set_sz(_jit, RegA);
  return true;
}

JIT_OP(SEC) {
  _jit->xSTC();
  return true;
}

JIT_OP(SED) {
  set_flag(_jit, 3, true);
  return true;
}

JIT_OP(SEI) {
  set_flag(_jit, 2, true);
  return true;
}

JIT_OP(STA) {
  _jit->xSTORE(RegEA, RegA);
  return true;
}

JIT_OP(STX) {
  _jit->xSTORE(RegEA, RegX);
  return true;
}

JIT_OP(STY) {
  _jit->xSTORE(RegEA, RegY);
  return true;
}

JIT_OP(TAX) {
  /* XXX: flags */
  _jit->xMOV8(RegX, RegA);
  return true;
}

JIT_OP(TAY) {
  /* XXX: flags */
  _jit->xMOV8(RegY, RegA);
  return true;
}

JIT_OP(TSX) {
  _jit->xMOV8(RegX, RegSP);
  set_sz(_jit, RegX);
  return true;
}

JIT_OP(TXA) {
  _jit->xMOV8(RegA, RegX);
  set_sz(_jit, RegA);
  return true;
}

JIT_OP(TYA) {
  _jit->xMOV8(RegA, RegY);
  set_sz(_jit, RegA);
  return true;
}

JIT_OP(TXS) {
  _jit->xMOV8(RegSP, RegX);
  return true;
}
};

namespace M65C02 {
using namespace M6502;

JIT_OP(TSB) { return true; }

JIT_OP(TRB) { return true; }

JIT_OP(BITIMM) { return true; }

JIT_OP(INA) { return true; }

JIT_OP(DEA) { return true; }

JIT_OP(PHX) { return true; }

JIT_OP(PHY) { return true; }

JIT_OP(PLX) { return true; }

JIT_OP(PLY) { return true; }

JIT_OP(SMB, uint8_t bit) { return true; }

JIT_OP(RMB, uint8_t bit) { return true; }

JIT_OP(BBR, uint8_t bit) { return true; }

JIT_OP(BBS, uint8_t bit) { return true; }

JIT_OP(STZ) { return true; }

JIT_OP(BRA) { return false; }
};
