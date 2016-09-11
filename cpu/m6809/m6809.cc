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

#include "cpu/m6809/m6809.h"
#include "core/enum.h"
#include "cpu/lib/cpu.h"

using namespace CPU;
using namespace M6809;

M6809Cpu::M6809Cpu(Machine *machine, const std::string &name, unsigned hertz,
                   AddressBus16x8 *bus)
    : Cpu(machine, name, hertz, bus) {
  build_table();
}

M6809Cpu::~M6809Cpu(void) {}

void M6809Cpu::build_table(void) {
#define OPCODE(op, name, addr, oper)            \
  {                                             \
    op, name, std::bind(&M6809Cpu::addr, this), \
        std::bind(&M6809Cpu::oper, this)        \
  }

  M6809Opcode opcodes[] = {
      OPCODE(0x00, "NEG", Direct, NEG), OPCODE(0x03, "COM", Direct, COM),
      OPCODE(0x04, "LSR", Direct, LSR), OPCODE(0x06, "ROR", Direct, ROR),
      OPCODE(0x07, "ASR", Direct, ASR),
      OPCODE(0x08, "LSL/ASL", Direct, LSL),  // XXX
      OPCODE(0x09, "ROL", Direct, ROL), OPCODE(0x0A, "DEC", Direct, DEC),
      OPCODE(0x0C, "INC", Direct, INC), OPCODE(0x0D, "TST", Direct, TST),
      OPCODE(0x0E, "JMP", Direct, JMP), OPCODE(0x0F, "CLR", Direct, CLR),
      OPCODE(0x12, "NOP", Inherent, NOP), OPCODE(0x13, "SYNC", Inherent, SYNC),
      OPCODE(0x16, "LBRA", Relative, LBRA),
      OPCODE(0x17, "LBSR", Relative, LBSR), OPCODE(0x19, "DAA", Inherent, DAA),
      OPCODE(0x1A, "ORCC", Immediate, ORCC),
      OPCODE(0x1C, "ANDCC", Immediate, ANDCC),
      OPCODE(0x1D, "SEX", Inherent, SEX), OPCODE(0x1E, "EXG", Inherent, EXG),
      OPCODE(0x1F, "TFR", Inherent, TFR),
      /* Branch Instructions */
      OPCODE(0x20, "BRA", Relative, BRA), OPCODE(0x21, "BRN", Relative, BRN),
      OPCODE(0x22, "BHI", Relative, BHI), OPCODE(0x23, "BLS", Relative, BLS),
      OPCODE(0x24, "BHS/BCC", Relative, BHS),  // XXX
      OPCODE(0x25, "BLO/BCS", Relative, BLO),  // XXX
      OPCODE(0x26, "BNE", Relative, BNE), OPCODE(0x27, "BEQ", Relative, BEQ),
      OPCODE(0x28, "BVC", Relative, BVC), OPCODE(0x29, "BVS", Relative, BVS),
      OPCODE(0x2A, "BPL", Relative, BPL), OPCODE(0x2B, "BMI", Relative, BMI),
      OPCODE(0x2C, "BGE", Relative, BGE), OPCODE(0x2D, "BLT", Relative, BLT),
      OPCODE(0x2E, "BGT", Relative, BGT), OPCODE(0x2F, "BLE", Relative, BLE),

      OPCODE(0x30, "LEAX", Indexed, LEAX), OPCODE(0x31, "LEAY", Indexed, LEAY),
      OPCODE(0x32, "LEAS", Indexed, LEAS), OPCODE(0x33, "LEAU", Indexed, LEAU),
      OPCODE(0x34, "PSHS", Inherent, PSHS),
      OPCODE(0x35, "PULS", Inherent, PULS),
      OPCODE(0x36, "PSHU", Inherent, PSHU),
      OPCODE(0x37, "PULU", Inherent, PULU), OPCODE(0x39, "RTS", Inherent, RTS),
      OPCODE(0x3A, "ABX", Inherent, ABX), OPCODE(0x3B, "RTI", Inherent, RTI),
      OPCODE(0x3C, "CWAI", Inherent, CWAI), OPCODE(0x3D, "MUL", Inherent, MUL),
      OPCODE(0x3E, "RESET", Inherent, RESET),
      OPCODE(0x3F, "SWI", Inherent, SWI),

      OPCODE(0x40, "NEGA", Inherent, NEGA),
      OPCODE(0x43, "COMA", Inherent, COMA),
      OPCODE(0x44, "LSRA", Inherent, LSRA),
      OPCODE(0x46, "RORA", Inherent, RORA),
      OPCODE(0x47, "ASRA", Inherent, ASRA),
      OPCODE(0x48, "LSLA/ASLA", Inherent, LSLA),  // XXX
      OPCODE(0x49, "ROLA", Inherent, ROLA),
      OPCODE(0x4A, "DECA", Inherent, DECA),
      OPCODE(0x4C, "INCA", Inherent, INCA),
      OPCODE(0x4D, "TSTA", Inherent, TSTA),
      OPCODE(0x4F, "CLRA", Inherent, CLRA),

      OPCODE(0x50, "NEGB", Inherent, NEGB),
      OPCODE(0x53, "COMB", Inherent, COMB),
      OPCODE(0x54, "LSRB", Inherent, LSRB),
      OPCODE(0x56, "RORB", Inherent, RORB),
      OPCODE(0x57, "ASRB", Inherent, ASRB),
      OPCODE(0x58, "LSLB/ASLB", Inherent, LSLB),  // XXX
      OPCODE(0x59, "ROLB", Inherent, ROLB),
      OPCODE(0x5A, "DECB", Inherent, DECB),
      OPCODE(0x5C, "INCB", Inherent, INCB),
      OPCODE(0x5D, "TSTB", Inherent, TSTB),
      OPCODE(0x5F, "CLRB", Inherent, CLRB),

      OPCODE(0x60, "NEG", Indexed, NEG), OPCODE(0x63, "COM", Indexed, COM),
      OPCODE(0x64, "LSR", Indexed, LSR), OPCODE(0x66, "ROR", Indexed, ROR),
      OPCODE(0x67, "ASR", Indexed, ASR),
      OPCODE(0x68, "LSL/ASL", Indexed, LSL),  // XXX
      OPCODE(0x69, "ROL", Indexed, ROL), OPCODE(0x6A, "DEC", Indexed, DEC),
      OPCODE(0x6C, "INC", Indexed, INC), OPCODE(0x6D, "TST", Indexed, TST),
      OPCODE(0x6E, "JMP", Indexed, JMP), OPCODE(0x6F, "CLR", Indexed, CLR),

      OPCODE(0x70, "NEG", Extended, NEG), OPCODE(0x73, "COM", Extended, COM),
      OPCODE(0x74, "LSR", Extended, LSR), OPCODE(0x76, "ROR", Extended, ROR),
      OPCODE(0x77, "ASR", Extended, ASR),
      OPCODE(0x78, "LSL/ASL", Extended, LSL),  // XXX
      OPCODE(0x79, "ROL", Extended, ROL), OPCODE(0x7A, "DEC", Extended, DEC),
      OPCODE(0x7C, "INC", Extended, INC), OPCODE(0x7D, "TST", Extended, TST),
      OPCODE(0x7E, "JMP", Extended, JMP), OPCODE(0x7F, "CLR", Extended, CLR),

      OPCODE(0x80, "SUBA", Immediate, SUBA),
      OPCODE(0x81, "CMPA", Immediate, CMPA),
      OPCODE(0x82, "SBCA", Immediate, SBCA),
      OPCODE(0x83, "SUBD", Immediate, SUBD),
      OPCODE(0x84, "ANDA", Immediate, ANDA),
      OPCODE(0x85, "BITA", Immediate, BITA),
      OPCODE(0x86, "LDA", Immediate, LDA),
      OPCODE(0x88, "EORA", Immediate, EORA),
      OPCODE(0x89, "ADCA", Immediate, ADCA),
      OPCODE(0x8A, "ORA", Immediate, ORA),
      OPCODE(0x8B, "ADDA", Immediate, ADDA),
      OPCODE(0x8C, "CMPX", Immediate, CMPX), OPCODE(0x8D, "BSR", Relative, BSR),
      OPCODE(0x8E, "LDX", Immediate, LDX),

      OPCODE(0x90, "SUBA", Direct, SUBA), OPCODE(0x91, "CMPA", Direct, CMPA),
      OPCODE(0x92, "SBCA", Direct, SBCA), OPCODE(0x93, "SUBD", Direct, SUBD),
      OPCODE(0x94, "ANDA", Direct, ANDA), OPCODE(0x95, "BITA", Direct, BITA),
      OPCODE(0x96, "LDA", Direct, LDA), OPCODE(0x98, "EORA", Direct, EORA),
      OPCODE(0x99, "ADCA", Direct, ADCA), OPCODE(0x9A, "ORA", Direct, ORA),
      OPCODE(0x9B, "ADDA", Direct, ADDA), OPCODE(0x9C, "CMPX", Direct, CMPX),
      OPCODE(0x9D, "JSR", Direct, JSR), OPCODE(0x9E, "LDX", Direct, LDX),
      OPCODE(0x9F, "STX", Direct, STX),

      OPCODE(0xA0, "SUBA", Indexed, SUBA), OPCODE(0xA1, "CMPA", Indexed, CMPA),
      OPCODE(0xA2, "SBCA", Indexed, SBCA), OPCODE(0xA3, "SUBD", Indexed, SUBD),
      OPCODE(0xA4, "ANDA", Indexed, ANDA), OPCODE(0xA5, "BITA", Indexed, BITA),
      OPCODE(0xA6, "LDA", Indexed, LDA), OPCODE(0xA8, "EORA", Indexed, EORA),
      OPCODE(0xA9, "ADCA", Indexed, ADCA), OPCODE(0xAA, "ORA", Indexed, ORA),
      OPCODE(0xAB, "ADDA", Indexed, ADDA), OPCODE(0xAC, "CMPX", Indexed, CMPX),
      OPCODE(0xAD, "JSR", Indexed, JSR), OPCODE(0xAE, "LDX", Indexed, LDX),
      OPCODE(0xAF, "STX", Indexed, STX),

      OPCODE(0xB0, "SUBA", Extended, SUBA),
      OPCODE(0xB1, "CMPA", Extended, CMPA),
      OPCODE(0xB2, "SBCA", Extended, SBCA),
      OPCODE(0xB3, "SUBD", Extended, SUBD),
      OPCODE(0xB4, "ANDA", Extended, ANDA),
      OPCODE(0xB5, "BITA", Extended, BITA), OPCODE(0xB6, "LDA", Extended, LDA),
      OPCODE(0xB8, "EORA", Extended, EORA),
      OPCODE(0xB9, "ADCA", Extended, ADCA), OPCODE(0xBA, "ORA", Extended, ORA),
      OPCODE(0xBB, "ADDA", Extended, ADDA),
      OPCODE(0xBC, "CMPX", Extended, CMPX), OPCODE(0xBD, "JSR", Extended, JSR),
      OPCODE(0xBE, "LDX", Extended, LDX), OPCODE(0xBF, "STX", Extended, STX),

      OPCODE(0xC0, "SUBB", Immediate, SUBB),
      OPCODE(0xC1, "CMPB", Immediate, CMPB),
      OPCODE(0xC2, "SBCB", Immediate, SBCB),
      OPCODE(0xC3, "SUBD", Immediate, SUBD),
      OPCODE(0xC4, "ANDB", Immediate, ANDB),
      OPCODE(0xC5, "BITB", Immediate, BITB),
      OPCODE(0xC6, "LDB", Immediate, LDB),
      OPCODE(0xC8, "EORB", Immediate, EORB),
      OPCODE(0xC9, "ADCB", Immediate, ADCB),
      OPCODE(0xCA, "ORB", Immediate, ORB),
      OPCODE(0xCB, "ADDB", Immediate, ADDB),
      OPCODE(0xCC, "LDD", Immediate, LDD), OPCODE(0xCE, "LDU", Immediate, LDU),

      OPCODE(0xD0, "SUBB", Direct, SUBB), OPCODE(0xD1, "CMPB", Direct, CMPB),
      OPCODE(0xD2, "SBCB", Direct, SBCB), OPCODE(0xD3, "SUBD", Direct, SUBD),
      OPCODE(0xD4, "ANDB", Direct, ANDB), OPCODE(0xD5, "BITB", Direct, BITB),
      OPCODE(0xD6, "LDB", Direct, LDB), OPCODE(0xD8, "EORB", Direct, EORB),
      OPCODE(0xD9, "ADCB", Direct, ADCB), OPCODE(0xDA, "ORB", Direct, ORB),
      OPCODE(0xDB, "ADDB", Direct, ADDB), OPCODE(0xDC, "LDD", Direct, LDD),
      OPCODE(0xDD, "STD", Direct, STD), OPCODE(0xDE, "LDU", Direct, LDU),
      OPCODE(0xDF, "STU", Direct, STU),

      OPCODE(0xE0, "SUBB", Indexed, SUBB), OPCODE(0xE1, "CMPB", Indexed, CMPB),
      OPCODE(0xE2, "SBCB", Indexed, SBCB), OPCODE(0xE3, "SUBD", Indexed, SUBD),
      OPCODE(0xE4, "ANDB", Indexed, ANDB), OPCODE(0xE5, "BITB", Indexed, BITB),
      OPCODE(0xE6, "LDB", Indexed, LDB), OPCODE(0xE8, "EORB", Indexed, EORB),
      OPCODE(0xE9, "ADCB", Indexed, ADCB), OPCODE(0xEA, "ORB", Indexed, ORB),
      OPCODE(0xEB, "ADDB", Indexed, ADDB), OPCODE(0xEC, "LDD", Indexed, LDD),
      OPCODE(0xED, "STD", Indexed, STD), OPCODE(0xEE, "LDU", Indexed, LDU),
      OPCODE(0xEF, "STU", Indexed, STU),

      OPCODE(0xF0, "SUBB", Extended, SUBB),
      OPCODE(0xF1, "CMPB", Extended, CMPB),
      OPCODE(0xF2, "SBCB", Extended, SBCB),
      OPCODE(0xF3, "SUBD", Extended, SUBD),
      OPCODE(0xF4, "ANDB", Extended, ANDB),
      OPCODE(0xF5, "BITB", Extended, BITB), OPCODE(0xF6, "LDB", Extended, LDB),
      OPCODE(0xF8, "EORB", Extended, EORB),
      OPCODE(0xF9, "ADCB", Extended, ADCB), OPCODE(0xFA, "ORB", Extended, ORB),
      OPCODE(0xFB, "ADDB", Extended, ADDB), OPCODE(0xFC, "LDD", Extended, LDD),
      OPCODE(0xFD, "STD", Extended, STD), OPCODE(0xFE, "LDU", Extended, LDU),
      OPCODE(0xFF, "STU", Extended, STU),
  };

  for (int i = 0; i < sizeof(opcodes) / sizeof(opcodes[0]); i++) {
    _opcodes[opcodes[i].code] = opcodes[i];
  }
}

void M6809Cpu::log_op(uint16_t pc, const M6809Opcode *op) {
  std::stringstream os;
  os << std::setw(8) << name() << ":" << Hex(pc) << ":" << Hex(op->code) << ":"
     << op->name << " = >";
  const std::string &str = op->name;
  const std::string &delimiters = " ";
  auto lastPos = str.find_first_not_of(delimiters, 0);
  auto pos = str.find_first_of(delimiters, lastPos);
  while (std::string::npos != pos || std::string::npos != lastPos) {
    std::string it = str.substr(lastPos, pos - lastPos);
    os << " ";
    os << it;
    lastPos = str.find_first_not_of(delimiters, pos);
    pos = str.find_first_of(delimiters, lastPos);
  }
  DEVICE_TRACE(os.str());
}

std::string M6809Cpu::dasm(addr_type addr) { return "NOP"; }

void M6809Cpu::line(Line line, LineState state) {
  switch (line) {
    case Line::RESET:
      _reset();
      break;
    case Line::NMI:
      break;
    case Line::INT0:
      break;
    default:
      break;
  }
}

void M6809Cpu::_reset(void) {
  _rEA = 0xFFFE;
  _rPC = bus_read16(_rEA);
}

void M6809Cpu::step(void) { dispatch(); }

void M6809Cpu::dispatch(void) {
  uint16_t pc = _rPC;
  uint16_t code = pc_read();

  if (code == 0x10 || code == 0x11) {
    code = code << 8 | pc_read();
  }

  auto it = _opcodes.find(code);
  if (it == _opcodes.end()) {
    std::cout << "Unknown opcode: " << Hex(code) << std::endl;
    throw CpuOpcodeFault(name(), code, pc);
  }

  M6809Opcode op = it->second;

  op.addr_mode();
  op.operation();

  IF_LOG(Trace) { log_op(pc, &op); }

  /* XXX: Some addressing modes have postfix beahvior */
}

void M6809Cpu::check_interrupt(void) {
  M6809Irq irq = M6809Irq::Reset;
  for (auto i : Core::Enum<M6809Irq>()) {
    if (m_state.Irq[i] != LineState::Clear) {
      irq = i;
      break;
    }
  }
  switch (irq) {
    case M6809Irq::Reset:
      /* XXX: Should we handle this here? */
      break;
    case M6809Irq::NMI:
      _rF.E = 1;
      push(_rS, 0xFF);
      _rF.F = 1;
      _rF.I = 1;
      _rPC = bus_read16(0xFFFC);
      /* 19 cycles */
      break;
    case M6809Irq::SWI:
      _rF.E = 1;
      push(_rS, 0xFF);
      _rF.F = 1;
      _rF.I = 1;
      _rPC = bus_read16(0xFFFA);
      /* 19 cycles */
      break;
    case M6809Irq::IRQ:
      _rF.E = 1;
      push(_rS, 0xFF);
      _rF.F = 1;
      _rF.I = 1;
      _rPC = bus_read16(0xFFF8);
      /* 19 cycles */
      break;
    case M6809Irq::FIRQ:
      _rF.E = 0;
      push(_rS, 0x81);
      _rF.F = 1;
      _rF.I = 1;
      _rPC = bus_read16(0xFFF6);
      /* 10 cycles */
      break;
    case M6809Irq::SWI2:
      _rF.E = 1;
      push(_rS, 0xFF);
      _rPC = bus_read16(0xFFF4);
      /* 20 cycles */
      break;
    case M6809Irq::SWI3:
      _rF.E = 1;
      push(_rS, 0xFF);
      _rPC = bus_read16(0xFFF2);
      /* 20 cycles */
      break;
    default:
      break;
  }
}
