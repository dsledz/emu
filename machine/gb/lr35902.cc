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
/*
 * Gameboy CPU. Similar, to a stripped down Z80
 */

#include "emu/emu.h"

#include "cpu/lib/cpu.h"
#include "machine/gb/lr35902.h"

using namespace EMU;
using namespace CPU;
using namespace GBMachine;

Registers::Registers(void)
    : _AF({}), _BC({}), _DE({}), _HL({}), _SP({}), _PC({}) {}

bool Registers::operator==(const Registers &rhs) const {
#define EQ(a, b) \
  if (a.d != b.d) return false;
  EQ(_AF, rhs._AF);
  EQ(_BC, rhs._BC);
  EQ(_DE, rhs._DE);
  EQ(_HL, rhs._HL);
  EQ(_SP, rhs._SP);
  EQ(_PC, rhs._PC);
#undef EQ
  return true;
}

void Registers::set(Register r, uint16_t arg) {
  switch (r) {
    case Register::A:
      _AF.b.h = arg;
      break;
    case Register::F:
      _AF.b.l = arg;
      break;
    case Register::B:
      _BC.b.h = arg;
      break;
    case Register::C:
      _BC.b.l = arg;
      break;
    case Register::D:
      _DE.b.h = arg;
      break;
    case Register::E:
      _DE.b.l = arg;
      break;
    case Register::H:
      _HL.b.h = arg;
      break;
    case Register::L:
      _HL.b.l = arg;
      break;
    case Register::SP:
      _SP.d = arg;
      break;
    case Register::PC:
      _PC.d = arg;
      break;
    case Register::AF:
      _AF.d = arg;
      break;
    case Register::BC:
      _BC.d = arg;
      break;
    case Register::DE:
      _DE.d = arg;
      break;
    case Register::HL:
      _HL.d = arg;
      break;
  }
}

uint16_t Registers::get(Register r) {
  switch (r) {
    case Register::A:
      return _AF.b.h;
      break;
    case Register::F:
      return _AF.b.l;
      break;
    case Register::B:
      return _BC.b.h;
      break;
    case Register::C:
      return _BC.b.l;
      break;
    case Register::D:
      return _DE.b.h;
      break;
    case Register::E:
      return _DE.b.l;
      break;
    case Register::H:
      return _HL.b.h;
      break;
    case Register::L:
      return _HL.b.l;
      break;
    case Register::SP:
      return _SP.d;
      break;
    case Register::PC:
      return _PC.d;
      break;
    case Register::AF:
      return _AF.d;
      break;
    case Register::BC:
      return _BC.d;
      break;
    case Register::DE:
      return _DE.d;
      break;
    case Register::HL:
      return _HL.d;
      break;
  }
  throw CpuRegisterFault("lr35902", 0);
}

LR35902Cpu::LR35902Cpu(Machine *machine, const std::string &name,
                       unsigned hertz, AddressBus16x8 *bus)
    : ClockedDevice(machine, machine->clock(), name, hertz),
      _bus(bus),
      _R(),
      _ime(IME::Disabled),
      _state(State::Running),
      _IF(0),
      _reset_line(LineState::Clear),
      _icycles(0) {
  _bus->add(0xFF0F, &_IF);

  reset();
}

LR35902Cpu::~LR35902Cpu(void) { /* XXX: Remove */ }

void LR35902Cpu::line(Line line, LineState state) {
  switch (line) {
    case Line::RESET:
      _reset_line = state;
      break;
    case Line::INT0: /* Interrupt::VBlank */
      bit_set(_IF, 0, true);
      break;
    case Line::INT1: /* Interrupt::LCDStat */
      bit_set(_IF, 1, true);
      break;
    case Line::INT2: /* Interrupt::Timeout */
      bit_set(_IF, 2, true);
      break;
    case Line::INT3: /* Interrupt::Serial */
      bit_set(_IF, 3, true);
      break;
    case Line::INT4: /* Interrupt::Joypad */
      bit_set(_IF, 4, true);
      break;
    default:
      break;
  };
}

void LR35902Cpu::reset(void) {
  _rA = 0x01;
  _rF = 0xB0;
  _rB = 0x00;
  _rC = 0x13;
  _rD = 0x00;
  _rE = 0xD8;
  _rH = 0x01;
  _rL = 0x4D;
  _rSP = 0xFFFE;
  _rPC = 0x0100;
  _ime = IME::Disabled;
  _state = State::Running;
  _IF = 0x00;
}

/*
 *  ___                       _   _
 * / _ \ _ __   ___ _ __ __ _| |_(_) ___  _ __  ___
 *| | | | '_ \ / _ \ '__/ _` | __| |/ _ \| '_ \/ __|
 *| |_| | |_) |  __/ | | (_| | |_| | (_) | | | \__ \
 * \___/| .__/ \___|_|  \__,_|\__|_|\___/|_| |_|___/
 *      |_|
 */

byte_t LR35902Cpu::fetch(Register reg) {
  switch (reg) {
    case Register::A:
      return _rA;
    case Register::B:
      return _rB;
    case Register::C:
      return _rC;
    case Register::D:
      return _rD;
    case Register::E:
      return _rE;
    case Register::H:
      return _rH;
    case Register::L:
      return _rL;
    case Register::HL:
      return _read(_rHL);
    default:
      throw CpuRegisterFault(name(), 0);
  }
}

void LR35902Cpu::store(Register reg, byte_t value) {
  switch (reg) {
    case Register::A:
      _rA = value;
      break;
    case Register::B:
      _rB = value;
      break;
    case Register::C:
      _rC = value;
      break;
    case Register::D:
      _rD = value;
      break;
    case Register::E:
      _rE = value;
      break;
    case Register::H:
      _rH = value;
      break;
    case Register::L:
      _rL = value;
      break;
    case Register::HL:
      _write(_rHL, value);
      break;
    default:
      throw CpuRegisterFault(name(), 0);
  }
}

void LR35902Cpu::_add(byte_t &dest, byte_t arg) {
  uint16_t result = dest + arg;

  _set_hflag(dest, arg, result);
  _set_cflag(dest, arg, result);
  _set_zflag(result);
  _set_nflag(false);

  dest = (byte_t)result;
}

void LR35902Cpu::_inc(byte_t &dest) {
  uint16_t result = dest + 1;

  _set_hflag(dest, 1, result);
  _set_zflag(result);
  _set_nflag(false);

  dest = (byte_t)result;
}

void LR35902Cpu::_inci(addr_t addr) {
  byte_t dest = _read(addr);

  _inc(dest);

  _write(addr, dest);
}

void LR35902Cpu::_adc(byte_t &dest, byte_t arg) {
  uint16_t result = dest + arg + _flags.C;

  _set_hflag(dest, arg, result);
  _set_cflag(dest, arg, result);
  _set_zflag(result);
  _set_nflag(false);

  dest = result;
}

void LR35902Cpu::_sub(byte_t &dest, byte_t arg) {
  uint16_t result = dest - arg;

  _set_hflag(dest, arg, result);
  _set_cflag(dest, arg, result);
  _set_zflag(result);
  _set_nflag(true);

  dest = result;
}

void LR35902Cpu::_dec(byte_t &dest) {
  byte_t result = dest - 1;

  _set_hflag(dest, 1, result);
  _set_zflag(result);
  _set_nflag(true);

  dest = result;
}

void LR35902Cpu::_deci(addr_t addr) {
  byte_t dest = _read(addr);

  _dec(dest);

  _write(addr, dest);
}

void LR35902Cpu::_sbc(byte_t &dest, byte_t arg) {
  byte_t carry = _flags.C;
  byte_t result = dest - arg - carry;

  _flags.C = (arg > dest - carry) ? true : false;
  _flags.H = ((arg & 0x0f) + carry > (dest & 0x0f)) ? true : false;
  _set_zflag(result);
  _set_nflag(true);

  dest = result;
}

void LR35902Cpu::_cp(byte_t dest, byte_t arg) {
  _flags.Z = (dest == arg);
  _flags.C = (arg > dest);
  _flags.H = ((arg & 0x0f) > (dest & 0x0f)) ? true : false;
  _flags.N = 1;
}

void LR35902Cpu::_and(byte_t &dest, byte_t arg) {
  byte_t result = dest & arg;

  _flags.C = 0;
  _flags.H = 1;
  _set_zflag(result);
  _set_nflag(false);

  dest = result;
}

void LR35902Cpu::_xor(byte_t &dest, byte_t arg) {
  byte_t result = dest ^ arg;

  _flags.C = 0;
  _flags.H = 0;
  _set_zflag(result);
  _set_nflag(false);

  dest = result;
}

void LR35902Cpu::_or(byte_t &dest, byte_t arg) {
  byte_t result = dest | arg;

  _flags.C = 0;
  _flags.H = 0;
  _set_zflag(result);
  _set_nflag(false);

  dest = result;
}

void LR35902Cpu::_ld(byte_t &dest, byte_t arg) {
  byte_t result = arg;

  dest = result;
}

void LR35902Cpu::_bit(byte_t dest, int bit) {
  byte_t result = dest & (1 << bit);

  _flags.H = 1;
  _set_zflag(result);
  _set_nflag(false);
}

void LR35902Cpu::_reset(byte_t &dest, int bit) {
  byte_t result = dest & ~(1 << bit);

  dest = result;
}

void LR35902Cpu::_set(byte_t &dest, int bit) {
  byte_t result = dest | (1 << bit);

  dest = result;
}

void LR35902Cpu::_rl(byte_t &dest) {
  byte_t result = (dest << 1) | _flags.C;

  _flags.C = (dest & 0x80) != 0;
  _flags.H = 0;
  _set_zflag(result);
  _set_nflag(false);

  dest = result;
}

void LR35902Cpu::_rla(void) {
  _rl(_rA);
  _flags.Z = 0;
}

void LR35902Cpu::_rlc(byte_t &dest) {
  byte_t result = (dest << 1) | ((dest & 0x80) >> 7);

  _flags.C = (dest & 0x80) != 0;
  _flags.H = 0;
  _set_zflag(result);
  _set_nflag(false);

  dest = result;
}

void LR35902Cpu::_rlca(void) {
  _rlc(_rA);
  _flags.Z = 0;
}

void LR35902Cpu::_rr(byte_t &dest) {
  byte_t result = (dest >> 1) | (_flags.C ? 0x80 : 0x00);

  _flags.C = (dest & 0x01) != 0;
  _flags.H = 0;
  _set_zflag(result);
  _set_nflag(false);

  dest = result;
}

void LR35902Cpu::_rra(void) {
  _rr(_rA);
  _flags.Z = 0;
}

void LR35902Cpu::_rrc(byte_t &dest) {
  byte_t result = (dest >> 1) | (dest & 0x01 ? 0x80 : 0x00);

  _flags.C = (dest & 0x01) != 0;
  _flags.H = 0;
  _set_zflag(result);
  _set_nflag(false);

  dest = result;
}

void LR35902Cpu::_rrca(void) {
  _rrc(_rA);
  _flags.Z = 0;
}

void LR35902Cpu::_sla(byte_t &dest) {
  byte_t result = (dest << 1);

  _flags.C = (dest & 0x80) != 0;
  _flags.H = 0;
  _set_zflag(result);
  _set_nflag(false);

  dest = result;
}

void LR35902Cpu::_sra(byte_t &dest) {
  byte_t result = (dest >> 1) | (dest & 0x80);

  _flags.C = (dest & 0x01) != 0;
  _flags.H = 0;
  _set_zflag(result);
  _set_nflag(false);

  dest = result;
}

void LR35902Cpu::_srl(byte_t &dest) {
  byte_t result = (dest >> 1);

  _flags.C = (dest & 0x01) != 0;
  _flags.H = 0;
  _set_zflag(result);
  _set_nflag(false);

  dest = result;
}

void LR35902Cpu::_swap(byte_t &dest) {
  byte_t result = (dest >> 4) | (dest << 4);

  _flags.C = 0;
  _flags.H = 0;
  _set_zflag(result);
  _set_nflag(false);

  dest = result;
}

void LR35902Cpu::prefix_cb(void) {
  byte_t op = _d8();
  int bit = (op & 0x38) >> 3;
  byte_t dest = fetch(Register(op & 0x07));
  if ((op & 0xC0) == 0x00) {
    if ((op & 0xF8) == 0x00) {
      _rlc(dest);
    } else if ((op & 0xF8) == 0x08) {
      _rrc(dest);
    } else if ((op & 0xF8) == 0x10) {
      _rl(dest);
    } else if ((op & 0xF8) == 0x18) {
      _rr(dest);
    } else if ((op & 0xF8) == 0x20) {
      _sla(dest);
    } else if ((op & 0xF8) == 0x28) {
      _sra(dest);
    } else if ((op & 0xF8) == 0x30) {
      _swap(dest);
    } else if ((op & 0xF8) == 0x38) {
      _srl(dest);
    }
    store(Register(op & 0x07), dest);
  } else if ((op & 0xC0) == 0x40) {
    _bit(dest, bit);
  } else if ((op & 0xC0) == 0x80) {
    _reset(dest, bit);
    store(Register(op & 0x07), dest);
  } else if ((op & 0xC0) == 0xC0) {
    _set(dest, bit);
    store(Register(op & 0x07), dest);
  }
}

void LR35902Cpu::_rst(byte_t arg) {
  _push(_rPCh, _rPCl);
  _rPC = arg;
}

void LR35902Cpu::_jr(bool jump, byte_t arg) {
  if (jump) {
    _rPC += (char)arg;
    _add_icycles(4);
  }
}

void LR35902Cpu::_jp(bool jump, uint16_t arg) {
  if (jump) {
    _rPC = arg;
    _add_icycles(4);
  }
}

void LR35902Cpu::_call(bool jump, uint16_t addr) {
  if (jump) {
    _push(_rPCh, _rPCl);

    _rPC = addr;
    _add_icycles(12);
  }
}

void LR35902Cpu::_ret(bool jump) {
  if (jump) {
    _pop(_rPCh, _rPCl);
    _add_icycles(16);
  }
}

void LR35902Cpu::_push(byte_t high, byte_t low) {
  _write(--_rSP, high);
  _write(--_rSP, low);
}

void LR35902Cpu::_pop(byte_t &high, byte_t &low) {
  low = _read(_rSP++);
  high = _read(_rSP++);
}

void LR35902Cpu::_halt() {
  // Cpu is halted until the next interrupt
  _state = State::Halted;
}

void LR35902Cpu::_stop() {
  _d8();
  _state = State::Stopped;
}

void LR35902Cpu::_addw(uint16_t &wdest, uint16_t arg) {
  uint16_t result = wdest + arg;

  _flags.C = (result < arg) ? true : false;
  _flags.H = ((result & 0xfff) < (arg & 0xfff)) ? true : false;
  _set_nflag(false);

  wdest = result;
}

void LR35902Cpu::_addsp(byte_t arg) {
  uint16_t &wdest = _rSP;
  uint16_t result = wdest + arg;

  _set_cflag(_rSP, arg, result);
  _set_hflag(_rSP, arg, result);
  _flags.Z = 0;
  _set_nflag(false);

  wdest = result;
}

void LR35902Cpu::_ldhlsp(byte_t arg) {
  uint16_t &wdest = _rHL;
  uint16_t result = _rSP + arg;

  _set_cflag(_rSP, arg, result);
  _set_hflag(_rSP, arg, result);
  _flags.Z = 0;
  _set_nflag(false);

  wdest = result;
}

void LR35902Cpu::_incw(uint16_t &wdest) {
  uint16_t result = wdest + 1;

  wdest = result;
}

void LR35902Cpu::_subw(uint16_t &wdest, uint16_t arg) {
  uint16_t result = wdest - arg;

  wdest = result;
}

void LR35902Cpu::_decw(uint16_t &wdest) {
  uint16_t result = wdest - 1;

  wdest = result;
}

void LR35902Cpu::_ldw(uint16_t &wdest, uint16_t arg) { wdest = arg; }

void LR35902Cpu::_ldi(addr_t addr, byte_t arg) { _write(addr, arg); }

void LR35902Cpu::_ldwi(addr_t addr, uint16_t arg) {
  _write(addr, arg & 0xff);
  _write(addr + 1, arg >> 8);
}

void LR35902Cpu::_cpl(void) {
  byte_t &dest = _rA;
  byte_t result = ~dest;

  _flags.H = 1;
  _flags.N = 1;

  dest = result;
}

void LR35902Cpu::_ccf(void) {
  _flags.C = !_flags.C;
  _flags.H = 0;
  _flags.N = 0;
}

void LR35902Cpu::_scf(void) {
  _flags.H = 0;
  _flags.C = 1;
  _flags.N = 0;
}

void LR35902Cpu::_daa(void) {
  byte_t &dest = _rA;
  uint16_t arg = 0;
  uint16_t result = dest;

  if (!_flags.N) {
    if (_flags.H || (dest & 0x0f) > 9) arg += 0x06;
    if (_flags.C || dest > 0x99) arg += 0x60;
    result += arg;
  } else {
    if (_flags.H) arg += 0x6;
    if (_flags.C) arg += 0x60;
    result -= arg;
  }

  _set_cflag(dest, arg, result);
  _flags.H = 0;
  _set_zflag(result);

  dest = (byte_t)result;
}

#define OPCODE(op, cycles, bytes, name, func)                               \
  case op: {                                                                \
    func;                                                                   \
    IF_LOG(Trace) {                                                         \
      std::cout << Hex(_rPC) << ":" << Hex(op) << ":" << name << std::endl; \
    }                                                                       \
    _add_icycles(cycles);                                                   \
    break;                                                                  \
  }

Cycles LR35902Cpu::dispatch(void) {
  _icycles = Cycles(0);

  interrupt();

  const addr_t pc = _rPC;
  byte_t op = _read(_rPC++);

  if (_state == State::Halted) {
    op = 0x00;
    _rPC--;
  }

  switch (op) {
    OPCODE(0x00, 4, 1, "NOP", );
    OPCODE(0x01, 10, 3, "LD BC,d16", _ldw(_rBC, _d16()));
    OPCODE(0x02, 8, 1, "LD (BC),A", _ldi(_rBC, _rA));
    OPCODE(0x03, 8, 1, "INC BC", _incw(_rBC));
    OPCODE(0x04, 4, 1, "INC B", _inc(_rB));
    OPCODE(0x05, 4, 1, "DEC B", _dec(_rB));
    OPCODE(0x06, 8, 2, "LD B,d8", _ld(_rB, _d8()));
    OPCODE(0x07, 4, 1, "RLCA", _rlca());
    OPCODE(0x08, 20, 3, "LD (a16), SP", _ldwi(_d16(), _rSP));
    OPCODE(0x09, 8, 1, "ADD HL,BC", _addw(_rHL, _rBC));
    OPCODE(0x0A, 8, 1, "LD A,(BC)", _ld(_rA, _read(_rBC)));
    OPCODE(0x0B, 8, 1, "DEC BC", _decw(_rBC));
    OPCODE(0x0C, 4, 1, "INC C", _inc(_rC));
    OPCODE(0x0D, 4, 1, "DEC C", _dec(_rC));
    OPCODE(0x0E, 8, 2, "LD C,d8", _ld(_rC, _d8()));
    OPCODE(0x0F, 4, 1, "RRCA", _rrca());
    OPCODE(0x10, 4, 2, "STOP", _stop());
    OPCODE(0x11, 10, 3, "LD DE,d16", _ldw(_rDE, _d16()));
    OPCODE(0x12, 8, 1, "LD (DE),A", _ldi(_rDE, _rA));
    OPCODE(0x13, 8, 1, "INC DE", _incw(_rDE));
    OPCODE(0x14, 4, 1, "INC D", _inc(_rD));
    OPCODE(0x15, 4, 1, "DEC D", _dec(_rD));
    OPCODE(0x16, 8, 2, "LD D,d8", _ld(_rD, _d8()));
    OPCODE(0x17, 4, 1, "RLA", _rla());
    OPCODE(0x18, 12, 2, "JR r8", _jr(true, _r8()));
    OPCODE(0x19, 8, 1, "ADD HL,DE", _addw(_rHL, _rDE));
    OPCODE(0x1A, 8, 1, "LD A,(DE)", _ld(_rA, _read(_rDE)));
    OPCODE(0x1B, 8, 1, "DEC DE", _decw(_rDE));
    OPCODE(0x1C, 4, 1, "INC E", _inc(_rE));
    OPCODE(0x1D, 4, 1, "DEC E", _dec(_rE));
    OPCODE(0x1E, 8, 2, "LD E,d8", _ld(_rE, _d8()));
    OPCODE(0x1F, 4, 1, "RRA", _rra());
    OPCODE(0x20, 8, 2, "JR NZ,r8", _jr(!_flags.Z, _r8()));
    OPCODE(0x21, 10, 3, "LD HL,d16", _ldw(_rHL, _d16()));
    OPCODE(0x22, 8, 1, "LDI (HL), A", _ldi(_rHL++, _rA));
    OPCODE(0x23, 8, 1, "INC HL", _incw(_rHL));
    OPCODE(0x24, 4, 1, "INC H", _inc(_rH));
    OPCODE(0x25, 4, 1, "DEC H", _dec(_rH));
    OPCODE(0x26, 8, 2, "LD H,d8", _ld(_rH, _d8()));
    OPCODE(0x27, 4, 1, "DAA", _daa());
    OPCODE(0x28, 8, 2, "JR Z,r8", _jr(_flags.Z, _r8()));
    OPCODE(0x29, 8, 1, "ADD HL,HL", _addw(_rHL, _rHL));
    OPCODE(0x2A, 8, 1, "LDI a, (HL)", _ld(_rA, _read(_rHL++)));
    OPCODE(0x2B, 8, 1, "DEC HL", _decw(_rHL));
    OPCODE(0x2C, 4, 1, "INC L", _inc(_rL));
    OPCODE(0x2D, 4, 1, "DEC L", _dec(_rL));
    OPCODE(0x2E, 8, 2, "LD L,d8", _ld(_rL, _d8()));
    OPCODE(0x2F, 8, 2, "CPL", _cpl());
    OPCODE(0x30, 8, 2, "JR NC,r8", _jr(!_flags.C, _r8()));
    OPCODE(0x31, 12, 3, "LD SP,d16", _ldw(_rSP, _d16()));
    OPCODE(0x32, 8, 1, "LDD (HL), A", _ldi(_rHL--, _rA));
    OPCODE(0x33, 8, 1, "INC SP", _incw(_rSP));
    OPCODE(0x34, 12, 1, "INC (HL)", _inci(_rHL));
    OPCODE(0x35, 12, 1, "DEC (HL)", _deci(_rHL));
    OPCODE(0x36, 8, 2, "LD (HL),d8", _ldi(_rHL, _d8()));
    OPCODE(0x37, 4, 1, "SCF", _scf());
    OPCODE(0x38, 8, 2, "JR C,r8", _jr(_flags.C, _r8()));
    OPCODE(0x39, 8, 1, "ADD HL,SP", _addw(_rHL, _rSP));
    OPCODE(0x3A, 8, 1, "LD A, (HL-)", _ld(_rA, _read(_rHL--)));
    OPCODE(0x3B, 8, 1, "DEC SP", _decw(_rSP));
    OPCODE(0x3C, 4, 1, "INC A", _inc(_rA));
    OPCODE(0x3D, 4, 1, "DEC A", _dec(_rA));
    OPCODE(0x3E, 8, 2, "LD A,d8", _ld(_rA, _d8()));
    OPCODE(0x3F, 4, 1, "CCF", _ccf());
    OPCODE(0x40, 4, 1, "LD B,B", _ld(_rB, _rB));
    OPCODE(0x41, 4, 1, "LD B,C", _ld(_rB, _rC));
    OPCODE(0x42, 4, 1, "LD B,D", _ld(_rB, _rD));
    OPCODE(0x43, 4, 1, "LD B,E", _ld(_rB, _rE));
    OPCODE(0x44, 4, 1, "LD B,H", _ld(_rB, _rH));
    OPCODE(0x45, 4, 1, "LD B,L", _ld(_rB, _rL));
    OPCODE(0x46, 4, 1, "LD B,(HL)", _ld(_rB, _read(_rHL)));
    OPCODE(0x47, 4, 1, "LD B,A", _ld(_rB, _rA));
    OPCODE(0x48, 4, 1, "LD C,B", _ld(_rC, _rB));
    OPCODE(0x49, 4, 1, "LD C,C", _ld(_rC, _rC));
    OPCODE(0x4A, 4, 1, "LD C,D", _ld(_rC, _rD));
    OPCODE(0x4B, 4, 1, "LD C,E", _ld(_rC, _rE));
    OPCODE(0x4C, 4, 1, "LD C,H", _ld(_rC, _rH));
    OPCODE(0x4D, 4, 1, "LD C,L", _ld(_rC, _rL));
    OPCODE(0x4E, 4, 1, "LD C,(HL)", _ld(_rC, _read(_rHL)));
    OPCODE(0x4F, 4, 1, "LD C,A", _ld(_rC, _rA));
    OPCODE(0x50, 4, 1, "LD D,B", _ld(_rD, _rB));
    OPCODE(0x51, 4, 1, "LD D,C", _ld(_rD, _rC));
    OPCODE(0x52, 4, 1, "LD D,D", _ld(_rD, _rD));
    OPCODE(0x53, 4, 1, "LD D,E", _ld(_rD, _rE));
    OPCODE(0x54, 4, 1, "LD D,H", _ld(_rD, _rH));
    OPCODE(0x55, 4, 1, "LD D,L", _ld(_rD, _rL));
    OPCODE(0x56, 4, 1, "LD D,(HL)", _ld(_rD, _read(_rHL)));
    OPCODE(0x57, 4, 1, "LD D,A", _ld(_rD, _rA));
    OPCODE(0x58, 4, 1, "LD E,B", _ld(_rE, _rB));
    OPCODE(0x59, 4, 1, "LD E,C", _ld(_rE, _rC));
    OPCODE(0x5A, 4, 1, "LD E,D", _ld(_rE, _rD));
    OPCODE(0x5B, 4, 1, "LD E,E", _ld(_rE, _rE));
    OPCODE(0x5C, 4, 1, "LD E,H", _ld(_rE, _rH));
    OPCODE(0x5D, 4, 1, "LD E,L", _ld(_rE, _rL));
    OPCODE(0x5E, 4, 1, "LD E,(HL)", _ld(_rE, _read(_rHL)));
    OPCODE(0x5F, 4, 1, "LD E,A", _ld(_rE, _rA));
    OPCODE(0x60, 4, 1, "LD H,B", _ld(_rH, _rB));
    OPCODE(0x61, 4, 1, "LD H,C", _ld(_rH, _rC));
    OPCODE(0x62, 4, 1, "LD H,D", _ld(_rH, _rD));
    OPCODE(0x63, 4, 1, "LD H,E", _ld(_rH, _rE));
    OPCODE(0x64, 4, 1, "LD H,H", _ld(_rH, _rH));
    OPCODE(0x65, 4, 1, "LD H,L", _ld(_rH, _rL));
    OPCODE(0x66, 4, 1, "LD H,(HL)", _ld(_rH, _read(_rHL)));
    OPCODE(0x67, 4, 1, "LD H,A", _ld(_rH, _rA));
    OPCODE(0x68, 4, 1, "LD L,B", _ld(_rL, _rB));
    OPCODE(0x69, 4, 1, "LD L,C", _ld(_rL, _rC));
    OPCODE(0x6A, 4, 1, "LD L,D", _ld(_rL, _rD));
    OPCODE(0x6B, 4, 1, "LD L,E", _ld(_rL, _rE));
    OPCODE(0x6C, 4, 1, "LD L,H", _ld(_rL, _rH));
    OPCODE(0x6D, 4, 1, "LD L,L", _ld(_rL, _rL));
    OPCODE(0x6E, 4, 1, "LD L,(HL)", _ld(_rL, _read(_rHL)));
    OPCODE(0x6F, 4, 1, "LD L,A", _ld(_rL, _rA));
    OPCODE(0x70, 4, 1, "LD (HL),B", _ldi(_rHL, _rB));
    OPCODE(0x71, 4, 1, "LD (HL),C", _ldi(_rHL, _rC));
    OPCODE(0x72, 4, 1, "LD (HL),D", _ldi(_rHL, _rD));
    OPCODE(0x73, 4, 1, "LD (HL),E", _ldi(_rHL, _rE));
    OPCODE(0x74, 4, 1, "LD (HL),H", _ldi(_rHL, _rH));
    OPCODE(0x75, 4, 1, "LD (HL),L", _ldi(_rHL, _rL));
    OPCODE(0x76, 4, 1, "HALT", _halt());
    OPCODE(0x77, 4, 1, "LD (HL),A", _ldi(_rHL, _rA));
    OPCODE(0x78, 4, 1, "LD A,B", _ld(_rA, _rB));
    OPCODE(0x79, 4, 1, "LD A,C", _ld(_rA, _rC));
    OPCODE(0x7A, 4, 1, "LD A,D", _ld(_rA, _rD));
    OPCODE(0x7B, 4, 1, "LD A,E", _ld(_rA, _rE));
    OPCODE(0x7C, 4, 1, "LD A,H", _ld(_rA, _rH));
    OPCODE(0x7D, 4, 1, "LD A,L", _ld(_rA, _rL));
    OPCODE(0x7E, 4, 1, "LD A,(HL)", _ld(_rA, _read(_rHL)));
    OPCODE(0x7F, 4, 1, "LD A,A", _ld(_rA, _rA));
    OPCODE(0x80, 4, 1, "ADD A,B", _add(_rA, _rB));
    OPCODE(0x81, 4, 1, "ADD A,C", _add(_rA, _rC));
    OPCODE(0x82, 4, 1, "ADD A,D", _add(_rA, _rD));
    OPCODE(0x83, 4, 1, "ADD A,E", _add(_rA, _rE));
    OPCODE(0x84, 4, 1, "ADD A,H", _add(_rA, _rH));
    OPCODE(0x85, 4, 1, "ADD A,L", _add(_rA, _rL));
    OPCODE(0x86, 4, 1, "ADD A,(HL)", _add(_rA, _read(_rHL)));
    OPCODE(0x87, 4, 1, "ADD A,A", _add(_rA, _rA));
    OPCODE(0x88, 8, 1, "ADC A,B", _adc(_rA, _rB));
    OPCODE(0x89, 8, 1, "ADC A,C", _adc(_rA, _rC));
    OPCODE(0x8A, 8, 1, "ADC A,D", _adc(_rA, _rD));
    OPCODE(0x8B, 8, 1, "ADC A,E", _adc(_rA, _rE));
    OPCODE(0x8C, 8, 1, "ADC A,H", _adc(_rA, _rH));
    OPCODE(0x8D, 8, 1, "ADC A,L", _adc(_rA, _rL));
    OPCODE(0x8E, 8, 1, "ADC A,(HL)", _adc(_rA, _read(_rHL)));
    OPCODE(0x8F, 8, 1, "ADC A,A", _adc(_rA, _rA));
    OPCODE(0x90, 4, 1, "SUB B", _sub(_rA, _rB));
    OPCODE(0x91, 4, 1, "SUB C", _sub(_rA, _rC));
    OPCODE(0x92, 4, 1, "SUB D", _sub(_rA, _rD));
    OPCODE(0x93, 4, 1, "SUB E", _sub(_rA, _rE));
    OPCODE(0x94, 4, 1, "SUB H", _sub(_rA, _rH));
    OPCODE(0x95, 4, 1, "SUB L", _sub(_rA, _rL));
    OPCODE(0x96, 4, 1, "SUB (HL)", _sub(_rA, _read(_rHL)));
    OPCODE(0x97, 4, 1, "SUB A", _sub(_rA, _rA));
    OPCODE(0x98, 8, 1, "SBC A,B", _sbc(_rA, _rB));
    OPCODE(0x99, 8, 1, "SBC A,C", _sbc(_rA, _rC));
    OPCODE(0x9A, 8, 1, "SBC A,D", _sbc(_rA, _rD));
    OPCODE(0x9B, 8, 1, "SBC A,E", _sbc(_rA, _rE));
    OPCODE(0x9C, 8, 1, "SBC A,H", _sbc(_rA, _rH));
    OPCODE(0x9D, 8, 1, "SBC A,L", _sbc(_rA, _rL));
    OPCODE(0x9E, 8, 1, "SBC A,(HL)", _sbc(_rA, _read(_rHL)));
    OPCODE(0x9F, 8, 1, "SBC A,A", _sbc(_rA, _rA));
    OPCODE(0xA0, 4, 1, "AND B", _and(_rA, _rB));
    OPCODE(0xA1, 4, 1, "AND C", _and(_rA, _rC));
    OPCODE(0xA2, 4, 1, "AND D", _and(_rA, _rD));
    OPCODE(0xA3, 4, 1, "AND E", _and(_rA, _rE));
    OPCODE(0xA4, 4, 1, "AND H", _and(_rA, _rH));
    OPCODE(0xA5, 4, 1, "AND L", _and(_rA, _rL));
    OPCODE(0xA6, 4, 1, "AND (HL)", _and(_rA, _read(_rHL)));
    OPCODE(0xA7, 4, 1, "AND A", _and(_rA, _rA));
    OPCODE(0xA8, 4, 1, "XOR B", _xor(_rA, _rB));
    OPCODE(0xA9, 4, 1, "XOR C", _xor(_rA, _rC));
    OPCODE(0xAA, 4, 1, "XOR D", _xor(_rA, _rD));
    OPCODE(0xAB, 4, 1, "XOR E", _xor(_rA, _rE));
    OPCODE(0xAC, 4, 1, "XOR H", _xor(_rA, _rH));
    OPCODE(0xAD, 4, 1, "XOR L", _xor(_rA, _rL));
    OPCODE(0xAE, 7, 2, "XOR (HL)", _xor(_rA, _read(_rHL)));
    OPCODE(0xAF, 4, 1, "XOR A", _xor(_rA, _rA));
    OPCODE(0xB0, 4, 1, "OR B", _or(_rA, _rB));
    OPCODE(0xB1, 4, 1, "OR C", _or(_rA, _rC));
    OPCODE(0xB2, 4, 1, "OR D", _or(_rA, _rD));
    OPCODE(0xB3, 4, 1, "OR E", _or(_rA, _rE));
    OPCODE(0xB4, 4, 1, "OR H", _or(_rA, _rH));
    OPCODE(0xB5, 4, 1, "OR L", _or(_rA, _rL));
    OPCODE(0xB6, 4, 1, "OR (HL)", _or(_rA, _read(_rHL)));
    OPCODE(0xB7, 4, 1, "OR A", _or(_rA, _rA));
    OPCODE(0xB8, 4, 1, "CP B", _cp(_rA, _rB));
    OPCODE(0xB9, 4, 1, "CP C", _cp(_rA, _rC));
    OPCODE(0xBA, 4, 1, "CP D", _cp(_rA, _rD));
    OPCODE(0xBB, 4, 1, "CP E", _cp(_rA, _rE));
    OPCODE(0xBC, 4, 1, "CP H", _cp(_rA, _rH));
    OPCODE(0xBD, 4, 1, "CP L", _cp(_rA, _rL));
    OPCODE(0xBE, 4, 1, "CP (HL)", _cp(_rA, _read(_rHL)));
    OPCODE(0xBF, 4, 1, "CP A", _cp(_rA, _rA));
    OPCODE(0xC0, 8, 1, "RET NZ", _ret(!_flags.Z));
    OPCODE(0xC1, 12, 1, "POP BC", _pop(_rB, _rC));
    OPCODE(0xC2, 12, 0, "JP NZ", _jp(!_flags.Z, _d16()));
    OPCODE(0xC3, 12, 0, "JP", _jp(true, _d16()));
    OPCODE(0xC4, 12, 3, "CALL NZ,a16", _call(!_flags.Z, _d16()));
    OPCODE(0xC5, 16, 1, "PUSH BC", _push(_rB, _rC));
    OPCODE(0xC6, 8, 2, "ADD a,d8", _add(_rA, _d8()));
    OPCODE(0xC7, 16, 1, "RST 00H", _rst(0x00));
    OPCODE(0xC8, 8, 1, "RET Z", _ret(_flags.Z));
    OPCODE(0xC9, 4, 1, "RET", _ret(true));
    OPCODE(0xCA, 12, 3, "JP Z,a16", _jp(_flags.Z, _d16()));
    OPCODE(0xCB, 16, 1, "PREFIX CB", prefix_cb());
    OPCODE(0xCC, 12, 3, "CALL Z,a16", _call(_flags.Z, _d16()));
    OPCODE(0xCD, 24, 3, "CALL a16", _call(true, _d16()));
    OPCODE(0xCE, 8, 2, "ADC a,d8", _adc(_rA, _d8()));
    OPCODE(0xCF, 16, 1, "RST 08H", _rst(0x08));
    OPCODE(0xD0, 8, 1, "RET NC", _ret(!_flags.C));
    OPCODE(0xD1, 12, 1, "POP DE", _pop(_rD, _rE));
    OPCODE(0xD2, 12, 0, "JP NC", _jp(!_flags.C, _d16()));
    OPCODE(0xD4, 12, 3, "CALL NC,a16", _call(!_flags.C, _d16()));
    OPCODE(0xD5, 16, 1, "PUSH DE", _push(_rD, _rE));
    OPCODE(0xD6, 8, 2, "SUB a,d8", _sub(_rA, _d8()));
    OPCODE(0xD7, 16, 1, "RST 10H", _rst(0x10));
    OPCODE(0xD8, 8, 1, "RET C", _ret(_flags.C));
    OPCODE(0xD9, 16, 1, "RETI", _ime = IME::Shadow; _ret(true));
    OPCODE(0xDA, 12, 3, "JP C,a16", _jp(_flags.C, _d16()));
    OPCODE(0xDC, 12, 3, "CALL C, a16", _call(_flags.C, _d16()));
    OPCODE(0xDE, 8, 2, "SBC a,d8", _sbc(_rA, _d8()));
    OPCODE(0xDF, 16, 1, "RST 18H", _rst(0x18));
    OPCODE(0xE0, 12, 2, "LDH (a8), A", _ldi(_a8(), _rA));
    OPCODE(0xE1, 12, 1, "POP HL", _pop(_rH, _rL));
    OPCODE(0xE2, 8, 2, "LD (C), A", _ldi(0xff00 + _rC, _rA));
    OPCODE(0xE5, 16, 1, "PUSH HL", _push(_rH, _rL));
    OPCODE(0xE6, 8, 2, "AND d8", _and(_rA, _d8()));
    OPCODE(0xE7, 16, 1, "RST 20H", _rst(0x20));
    OPCODE(0xE8, 16, 2, "ADD SP, r8", _addsp(_r8()));
    OPCODE(0xE9, 4, 1, "JP (HL)", _jp(true, _rHL));
    OPCODE(0xEA, 16, 3, "LD (a16),A", _ldi(_d16(), _rA));
    OPCODE(0xEE, 16, 1, "XOR d8", _xor(_rA, _d8()));
    OPCODE(0xEF, 16, 1, "RST 28H", _rst(0x28));
    OPCODE(0xF0, 12, 2, "LDH A,(a8)", _ld(_rA, _read(_a8())));
    OPCODE(0xF1, 12, 1, "POP AF", _pop(_rA, _rF); _rF &= 0xf0;);
    OPCODE(0xF2, 8, 2, "LD A, (C)", _ld(_rA, _read(0xff00 + _rC)));
    OPCODE(0xF3, 4, 1, "DI", _ime = IME::Disabled;);
    OPCODE(0xF5, 16, 1, "PUSH AF", _push(_rA, _rF));
    OPCODE(0xF6, 8, 2, "OR d8", _or(_rA, _d8()));
    OPCODE(0xF7, 16, 1, "RST 30H", _rst(0x30));
    OPCODE(0xF8, 12, 2, "LD HL,SP+r8", _ldhlsp(_r8()));
    OPCODE(0xF9, 8, 1, "LD SP,HL", _ldw(_rSP, _rHL));
    OPCODE(0xFa, 16, 3, "LD A,(a16)", _ld(_rA, _read(_d16())));
    OPCODE(0xFB, 4, 1, "EI", _ime = IME::Shadow;);
    OPCODE(0xFE, 8, 2, "CP d8", _cp(_rA, _d8()));
    OPCODE(0xFF, 16, 1, "RST 38H", _rst(0x38));
    default:
      _state = State::Fault;
      throw CpuOpcodeFault(name(), op, pc);
  }

  return _icycles;
}

void LR35902Cpu::save(SaveState &state) {}

void LR35902Cpu::load(LoadState &state) {}

void LR35902Cpu::execute(void) {
  while (true) {
    add_icycles(dispatch());
  }
}

addr_t InterruptVector[] = {
    0x40, /* Interrupt::VBlank */
    0x48, /* Interrupt::LCDStat */
    0x50, /* Interrupt::Timeout */
    0x58, /* Interrupt::Serial */
    0x60, /* Interrupt::Joypad */
};

/**
 * Process interrupts before dispatch
 */
void LR35902Cpu::interrupt(void) {
  if (_ime == IME::Disabled) {
    if (_state == State::Halted)
      for (unsigned i = 0; i < 5; i++)
        if (bit_isset(_IF, i)) _state = State::Running;
    return;
  }
  if (_ime == IME::Shadow) {
    _ime = IME::Enabled;
    return;
  }
  for (unsigned i = 0; i < 5; i++) {
    if (bit_isset(_IF, i)) {
      /* IE */
      if (!bit_isset(_read(0xFFFF), i)) continue;
      _push(_rPCh, _rPCl);
      _rPC = InterruptVector[i];
      bit_set(_IF, i, false);
      _add_icycles(20);
      _ime = IME::Disabled;
      _state = State::Running;
      return;
    }
  }
}
