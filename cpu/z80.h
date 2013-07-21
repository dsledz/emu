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

#include "emu.h"
#include <sstream>

namespace Z80 {

using namespace EMU;

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

struct Registers {
public:
    Registers(void);
    ~Registers(void);
    union {
        struct {
            union {
                struct {
                    byte_t C:1;   /**< carry flag */
                    byte_t N:1;   /**< add/substract */
                    byte_t V:1;  /**< parity/overflow */
                    byte_t X:1;
                    byte_t H:1;   /**< half carry */
                    byte_t Y:1;
                    byte_t Z:1;   /**< zero flag */
                    byte_t S:1;   /**< sign flag */
                } f;
                byte_t l;
            };
            byte_t h;
        } b;
        uint16_t d;
    } _AF;
    reg16_t _BC;
    reg16_t _DE;
    reg16_t _HL;
    reg16_t _IX;
    reg16_t _IY;
    reg16_t _SP;
    reg16_t _PC;

    byte_t _I;
    byte_t _R;

    reg16_t _sAF;
    reg16_t _sBC;
    reg16_t _sDE;
    reg16_t _sHL;
};

#define _rAF    _R._AF.d
#define _rA     _R._AF.b.h
#define _rF     _R._AF.b.l
#define _flags  _R._AF.b.f
#define _rBC   _R._BC.d
#define _rB    _R._BC.b.h
#define _rC    _R._BC.b.l
#define _rDE   _R._DE.d
#define _rD    _R._DE.b.h
#define _rE    _R._DE.b.l
#define _rHL   _R._HL.d
#define _rH    _R._HL.b.h
#define _rL    _R._HL.b.l
#define _rSP   _R._SP.d
#define _rSPh  _R._SP.b.h
#define _rSPl  _R._SP.b.l
#define _rPC   _R._PC.d
#define _rPCh  _R._PC.b.h
#define _rPCl  _R._PC.b.l
#define _rIX   _R._IX.d
#define _rIXh  _R._IX.b.h
#define _rIXl  _R._IX.b.l
#define _rIY   _R._IY.d
#define _rIYh  _R._IY.b.h
#define _rIYl  _R._IY.b.l
#define _rsAF  _R._sAF.d
#define _rsBC  _R._sBC.d
#define _rsDE  _R._sDE.d
#define _rsHL  _R._sHL.d
#define _rI    _R._I
#define _rR    _R._R

enum class IME {
    Disabled = 0,
    Shadow = 1,
    Enabled = 2
};

class Z80Cpu: public EMU::CpuDevice {
public:
    Z80Cpu(Machine *machine, const std::string &name, unsigned hertz,
           AddressBus16 *bus);
    ~Z80Cpu(void);
    Z80Cpu(const Z80Cpu &cpu) = delete;

    virtual void execute(Time interval);
    virtual void set_line(InputLine line, LineState state);

    /* Register accessors */
    void store(Reg r, byte_t arg);
    byte_t fetch(Reg r);

    /**
     * Load a rom image at a specified offset.
     */
    void load_rom(Rom *rom, addr_t offset);

private:
    /* Operation specific state */
    struct Z80Op {
        Z80Op(void): name("NONE") { }

        void reset(void) {
            pc = opcode = d8 = i8 = d16 = i16 = 0;
        }

        addr_t pc;
        byte_t opcode;
        byte_t d8;
        byte_t i8;
        uint16_t d16;
        uint16_t i16;

        std::string name;
    };

    void _reset(void);
    Cycles step(void);
    Cycles dispatch(void);
    void dispatch_cb(void);
    void dispatch_dd(void);
    void dispatch_dd_cb(void);
    void dispatch_ed(void);
    void dispatch_fd(void);
    void dispatch_fd_cb(void);
    void interrupt(addr_t addr);

    /* Trace support */
    void op_log(void);
    void op_set(const std::string &name) {
        _op.name = name;
    }

    Cycles _icycles;      /**< Current number of cycles for instruction */

    Z80Op _op;

    /* Internal state */
    Registers _R; /**< Registers */
    bool _iff1;
    bool _iff2;
    bool _iwait;
    int _imode;
    DeviceState _state; /**< CPU state */
    LineState _nmi_line;
    LineState _int0_line;
    LineState _reset_line;
    LineState _wait_line;

    /* Private rom space */
    bvec _rom;

    /* Address Bus */
    AddressBus16 *_bus;

    inline void _add_icycles(unsigned cycles) {
        _icycles += Cycles(cycles);
    }

    inline byte_t pc_read(void) {
        return bus_read(_rPC++);
    }
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
        _op.d16 = pc_read() | (pc_read() << 8);
        return _op.d16;
    }
    inline byte_t _d8(void) {
        _op.d8 = pc_read();
        return _op.d8;
    }
    inline byte_t _r8(void) {
        _op.d8 = pc_read();
        return _op.d8;
    }
    inline uint16_t _dIX(void) {
        _op.d8 = pc_read();
        return _rIX + _op.d8;
    }
    inline uint16_t _dIY(void) {
        _op.d8 = pc_read();
        return _rIY + _op.d8;
    }
    inline byte_t _iIX(void) {
        _op.i8 = bus_read(_dIX());
        return _op.i8;
    }
    inline byte_t _iIY(void) {
        _op.i8 = bus_read(_dIY());
        return _op.i8;
    }
    inline byte_t _i8(addr_t addr) {
        _op.i8 = bus_read(addr);
        return _op.i8;
    }
    inline uint16_t _i16(void) {
        _op.d16 = pc_read() | (pc_read() << 8);
        _op.i16 = bus_read(_op.d16) | (bus_read(_op.d16 + 1) << 8);
        return _op.i16;
    }

private:

    inline void _set_hflag(uint16_t orig, uint16_t arg, uint16_t result) {
        _flags.H = bit_isset(orig ^ arg ^ result, 4);
    }
    inline void _set_cflag(uint16_t orig, uint16_t arg, uint16_t result) {
        _flags.C = bit_isset(orig ^ arg ^ result, 8);
    }
    inline void _set_zflag(uint16_t result) {
        _flags.Z = (result & 0xff) == 0;
    }
    inline void _set_zsflag(uint16_t result) {
        _flags.Z = (result & 0xff) == 0;
        _flags.V = bit_isset(result, 8);
        _flags.S = bit_isset(result, 7);
        _flags.Y = bit_isset(result, 5);
        _flags.X = bit_isset(result, 3);
    };
    inline void _set_nflag(bool neg) {
        _flags.N = neg;
    }
    inline void _set_parity(byte_t result) {
        _flags.V = (0 == (bit_isset(result, 0) ^ bit_isset(result, 1) ^
                          bit_isset(result, 2) ^ bit_isset(result, 3) ^
                          bit_isset(result, 4) ^ bit_isset(result, 5) ^
                          bit_isset(result, 6) ^ bit_isset(result, 7)));
    }

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
    void _in(byte_t &orig, byte_t port) { /* XXX: nop */ }
    void _out(byte_t port, byte_t value) { /* XXX: nop */ }
    void _neg(byte_t &orig);
    void _otir(void) { /*XXX: nop */ }
    void _im(int mode) { _imode = mode; };
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
