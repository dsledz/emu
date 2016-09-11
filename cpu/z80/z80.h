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
/**
 * Z80 Cpu
 */
#pragma once

#include "cpu/z80/z80_state.h"
#include "emu/emu.h"

using namespace EMU;

namespace Z80 {

enum class Reg {
  A = 0x07,
  B = 0x00,
  C = 0x01,
  D = 0x02,
  E = 0x03,
  H = 0x04,
  L = 0x05,
  HL = 0x06,
};

class Z80Cpu : public EMU::ClockedDevice {
 public:
  Z80Cpu(Machine *machine, const std::string &name, unsigned hertz,
         AddressBus16 *bus);
  ~Z80Cpu(void);
  Z80Cpu(const Z80Cpu &cpu) = delete;

  virtual void execute(void);
  virtual void line(Line line, LineState state);
  virtual void reset(void);

  /* Register accessors */
  void store(Reg r, byte_t arg);
  byte_t fetch(Reg r);

  /** Hack to place a specifc value on the databus, required
   * for IM 2 */
  void set_data(byte_t arg) { _data = arg; }

  AddressBus8x8 *io(void) { return &_io; };

  /**
   * Load a rom image at a specified offset.
   */
  void load_rom(Rom *rom, addr_t offset);

 private:
  /* Operation specific state */
  struct Z80Op {
    Z80Op(void) : name("NONE") {}

    void reset(void) {
      prefix = NoPrefix;
      d8 = i8 = 0;
      pc = opcode = d16 = i16 = yield = 0;
    }

    enum Prefix {
      NoPrefix = 0x00,
      DDPrefix = 0xDD,
      FDPrefix = 0xFD,
    } prefix;

    addr_t pc;
    byte_t opcode;
    byte_t d8;
    byte_t i8;
    uint16_t d16;
    uint16_t i16;
    int yield;

    std::string name;
  };

  Cycles step(void);
  Cycles dispatch(void);
  void dispatch_cb(void);
  void dispatch_ed(void);
  void interrupt(addr_t addr);

  /* Trace support */
  void op_log(void);
  void op_set(const std::string &name) { m_op.name = name; }

  byte_t _data;
  AddressBus8x8 _io;

  Cycles _icycles; /**< Current number of cycles for instruction */

  Z80Op m_op;

  /* Internal state */
  Z80State m_R; /**< Registers */
  Z80State *state;
  LineState _nmi_line;
  LineState _int0_line;
  LineState _reset_line;
  LineState _wait_line;

  /* Private rom space */
  bvec _rom;

  /* Address Bus */
  AddressBus16 *_bus;

  inline void _add_icycles(unsigned cycles) { _icycles += Cycles(cycles); }

  inline byte_t pc_read(void) { return bus_read(state->PC.d++); }
  inline byte_t bus_read(addr_t addr) {
    if (addr < _rom.size())
      return _rom[addr];
    else
      return _bus->read(addr);
  }
  inline void bus_write(addr_t addr, byte_t arg) {
    if (addr < _rom.size())
      _rom[addr] = arg;
    else
      _bus->write(addr, arg);
  }

  /* decode accessors */
  inline uint16_t _d16(void) {
    m_op.d16 = pc_read() | (pc_read() << 8);
    return m_op.d16;
  }
  inline byte_t _d8(void) {
    m_op.d8 = pc_read();
    return m_op.d8;
  }
  inline byte_t _r8(void) {
    m_op.d8 = pc_read();
    return m_op.d8;
  }
  inline uint16_t _dIX(void) {
    m_op.d8 = pc_read();
    return state->IX.d + m_op.d8;
  }
  inline uint16_t _dIY(void) {
    m_op.d8 = pc_read();
    return state->IY.d + m_op.d8;
  }
  inline addr_t _dAddr(void) {
    switch (m_op.prefix) {
      case Z80Op::NoPrefix:
        m_op.d16 = state->HL.d;
        break;
      case Z80Op::DDPrefix:
        m_op.d16 = _dIX();
        break;
      case Z80Op::FDPrefix:
        m_op.d16 = _dIY();
        break;
    }
    return m_op.d16;
  }
  inline byte_t _iIX(void) { return _i8(_dIX()); }
  inline byte_t _iIY(void) { return _i8(_dIY()); }
  inline byte_t _iHL(void) {
    m_op.i8 = bus_read(state->HL.d);
    return m_op.i8;
  }
  inline byte_t _i8(addr_t addr) {
    m_op.i8 = bus_read(addr);
    return m_op.i8;
  }
  inline byte_t _iAddr(void) { return _i8(_dAddr()); }
  inline uint16_t _i16(void) {
    m_op.d16 = pc_read() | (pc_read() << 8);
    m_op.i16 = bus_read(m_op.d16) | (bus_read(m_op.d16 + 1) << 8);
    return m_op.i16;
  }

 private:
  inline void _set_hflag(uint16_t orig, uint16_t arg, uint16_t result) {
    state->AF.b.f.H = bit_isset(orig ^ arg ^ result, 4);
  }
  inline void _set_cflag(uint16_t orig, uint16_t arg, uint16_t result) {
    state->AF.b.f.C = bit_isset(orig ^ arg ^ result, 8);
  }
  inline void _set_zflag(uint16_t result) {
    state->AF.b.f.Z = (result & 0xff) == 0;
  }
  inline void _set_zsflag(uint16_t result) {
    state->AF.b.f.Z = (result & 0xff) == 0;
    state->AF.b.f.V = bit_isset(result, 8);
    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.X = bit_isset(result, 3);
  };
  inline void _set_nflag(bool neg) { state->AF.b.f.N = neg; }
  inline void _set_parity(byte_t result) {
    state->AF.b.f.V = (0 == (bit_isset(result, 0) ^ bit_isset(result, 1) ^
                             bit_isset(result, 2) ^ bit_isset(result, 3) ^
                             bit_isset(result, 4) ^ bit_isset(result, 5) ^
                             bit_isset(result, 6) ^ bit_isset(result, 7)));
  }
  inline void _set_parity(uint16_t result) { _set_parity((byte_t)result); }

  /* Addition */
  void _add(byte_t &orig, byte_t value);
  void _add16(uint16_t &worig, uint16_t value);
  void _adc(byte_t &orig, byte_t value);
  void _adc16(uint16_t &worig, uint16_t value);
  void _inc(byte_t &orig);
  void _inci(addr_t addr);
  void _inc16(uint16_t &worig);

  /* Subtraction */
  void _sub(byte_t &orig, byte_t value);
  void _sub16(uint16_t &worig, uint16_t value);
  void _sbc(byte_t &orig, byte_t value);
  void _sbc16(uint16_t &worig, uint16_t value);
  void _dec(byte_t &orig);
  void _deci(addr_t addr);
  void _dec16(uint16_t &worig);

  /* Load */
  void _ld(byte_t &orig, byte_t value);
  void _ld16(uint16_t &worig, uint16_t value);
  void _ldmem(addr_t addr, byte_t value);
  void _ld16i(addr_t addr, uint16_t value);

  /* Bitwise ops */
  void _and(byte_t &orig, byte_t value);
  void _xor(byte_t &orig, byte_t value);
  void _or(byte_t &orig, byte_t value);
  void _bit_test(byte_t value, int bit);
  void _bit_reset(byte_t &orig, byte_t value, int bit);
  void _bit_set(byte_t &orig, byte_t value, int bit);
  void _swap(byte_t &orig, byte_t value);
  void _rl(byte_t &orig, byte_t value);
  void _rla(void);
  void _rlc(byte_t &orig, byte_t value);
  void _rlca(void);
  void _rr(byte_t &orig, byte_t value);
  void _rra(void);
  void _rrc(byte_t &orig, byte_t value);
  void _rrca(void);
  void _rrd(void);
  void _rld(void);
  void _sla(byte_t &orig, byte_t value);
  void _sra(byte_t &orig, byte_t value);
  void _sll(byte_t &orig, byte_t value);
  void _srl(byte_t &orig, byte_t value);

  void _cp(byte_t lhs, byte_t rhs);
  void _cpi(void);
  void _cpir(void);
  void _cpd(void);
  void _cpdr(void);
  void _daa(void);
  void _cpl(void);
  void _ccf(void);
  void _scf(void);

  /* Special */
  void _ex(uint16_t &lhs, uint16_t &rhs);
  void _exi(addr_t addr, byte_t &rh, byte_t &rl);
  void _exx(void);
  void _push(byte_t high, byte_t low);
  void _pop(byte_t &high, byte_t &low);
  void _in(byte_t &orig, byte_t port);
  void _out(byte_t port, byte_t value);
  void _neg(byte_t &orig);
  void _otir(void) { /*XXX: nop */
  }
  void _im(int mode) { state->imode = mode; };
  void _ldi(void);
  void _ldir(void);
  void _ldd(void);
  void _lddr(void);
  void _di(void);
  void _ei(void);

  /* Special Control */

  /* Control */
  void _call(bool jump, uint16_t value);
  void _djnz(byte_t value);
  void _halt(void);
  void _jp(bool jump, uint16_t value);
  void _jr(bool jump, byte_t value);
  void _ret(bool jump);
  void _retn(void);
  void _reti(void);
  void _rst(byte_t value);
};

typedef std::unique_ptr<Z80Cpu> Z80Cpu_ptr;
};
