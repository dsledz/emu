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

#include "emu.h"

using namespace EMU;

namespace LR35902 {

enum class IME {
    Disabled = 0,
    Shadow = 1,
    Enabled = 2
};

enum class State {
    Running = 0,
    Halted = 1,
    Stopped = 2,
    Fault = 3,
};

enum class Register {
    A = 0x07, B = 0x00, C = 0x01, D = 0x02,
    E = 0x03, H = 0x04, L = 0x05, HL = 0x06,
    F = 0x08,
    AF = 0x09, BC = 0x0A, DE = 0x0B, SP = 0x0C, PC = 0x0D
};

struct Registers {
    Registers(void);

    void set(Register reg, word_t value);
    word_t get(Register reg);

    bool operator ==(const Registers &rhs) const;
    union {
        struct {
            union {
                struct {
                    byte_t reserved:4;
                    byte_t C:1;
                    byte_t H:1;
                    byte_t N:1;
                    byte_t Z:1;
                } f;
                byte_t v;
            } l;
            byte_t h;
        } b;
        word_t w;
    } _AF;
    Word _BC;
    Word _DE;
    Word _HL;
    Word _SP;
    Word _PC;
};

/*
 * XXX: This are defined before the class so we can access
 * them in the inline functions.
 */
#define _rAF    _R._AF.w
#define _rA     _R._AF.b.h
#define _rF     _R._AF.b.l.v
#define _flags  _R._AF.b.l.f
#define _rBC   _R._BC.w
#define _rB    _R._BC.b.h
#define _rC    _R._BC.b.l
#define _rDE   _R._DE.w
#define _rD    _R._DE.b.h
#define _rE    _R._DE.b.l
#define _rHL   _R._HL.w
#define _rH    _R._HL.b.h
#define _rL    _R._HL.b.l
#define _rSP   _R._SP.w
#define _rSPh  _R._SP.b.h
#define _rSPl  _R._SP.b.l
#define _rPC   _R._PC.w
#define _rPCh  _R._PC.b.h
#define _rPCl  _R._PC.b.l

class LR35902Cpu: public CpuDevice {
public:
    LR35902Cpu(Machine *machine, const std::string &name, unsigned hertz,
               AddressBus16 *bus);
    ~LR35902Cpu(void);
    LR35902Cpu(const LR35902Cpu &cpu) = delete;

    virtual void save(SaveState &state);
    virtual void load(LoadState &state);
    // XXX: Do we need to do something here?
    virtual void execute(Time interval);
    virtual void set_line(InputLine, LineState state);

    byte_t fetch(Register reg);
    void store(Register reg, byte_t value);

protected:

    inline void _add_icycles(unsigned cycles) {
        _icycles += Cycles(cycles);
    }

    /* Top level stages */
    Cycles dispatch(void);
    void prefix_cb(void);
    void interrupt(void);
    void _reset(void);

    /* Addition */
    void _add(byte_t &orig, byte_t value);
    void _adc(byte_t &orig, byte_t value);
    void _addw(word_t &worig, word_t value);
    void _inc(byte_t &orig);
    void _inci(addr_t addr);
    void _incw(word_t &worig);

    /* Subtraction */
    void _sub(byte_t &orig, byte_t value);
    void _subw(word_t &worig, word_t value);
    void _sbc(byte_t &orig, byte_t value);
    void _dec(byte_t &orig);
    void _deci(addr_t addr);
    void _decw(word_t &worig);

    /* Load */
    void _ld(byte_t &orig, byte_t value);
    void _ldw(word_t &worig, word_t value);
    void _ldi(addr_t addr, byte_t value);
    void _ldwi(addr_t addr, word_t value);

    /* Bitwise ops */
    void _and(byte_t &orig, byte_t value);
    void _xor(byte_t &orig, byte_t value);
    void _or(byte_t &orig, byte_t value);
    void _bit(byte_t orig, int bit);
    void _reset(byte_t &orig, int bit);
    void _set(byte_t &orig, int bit);
    void _swap(byte_t &orig);
    void _rl(byte_t &orig);
    void _rla(void);
    void _rlc(byte_t &orig);
    void _rlca(void);
    void _rr(byte_t &orig);
    void _rra(void);
    void _rrc(byte_t &orig);
    void _rrca(void);
    void _sla(byte_t &orig);
    void _sra(byte_t &orig);
    void _srl(byte_t &orig);

    void _cp(byte_t lhs, byte_t rhs);
    void _daa(void);
    void _cpl(void);
    void _ccf(void);
    void _scf(void);
    void _ldhlsp(byte_t value);
    void _addsp(byte_t value);

    void _stop(void);
    void _halt(void);
    void _rst(byte_t value);
    void _jr(bool jump, byte_t value);
    void _jp(bool jump, word_t value);
    void _call(bool jump, word_t value);
    void _ret(bool jump);
    void _push(byte_t high, byte_t low);
    void _pop(byte_t &high, byte_t &low);
    void _push(word_t arg);
    void _pop(word_t &arg);

    /* Store/Load */
    inline void _write(addr_t addr, byte_t value) {
        _bus->write(addr, value);
    }
    byte_t _read(addr_t addr) {
        return _bus->read(addr);
    }

    inline void _set_hflag(word_t orig, word_t arg, word_t result) {
        _flags.H = bit_isset(orig ^ arg ^ result, 4);
    }
    inline void _set_cflag(word_t orig, word_t arg, word_t result) {
        _flags.C = bit_isset(orig ^ arg ^ result, 8);
    }
    inline void _set_zflag(word_t result) {
        _flags.Z = (result & 0xff) == 0;
    }
    inline void _set_nflag(bool neg) {
        _flags.N = neg;
    }

    /* decode accessors */
    inline word_t _d16(void) {
        word_t tmp = _read(_rPC) | (_read(_rPC+1) << 8);
        _rPC+=2;
        return tmp;
    }
    inline byte_t _d8(void) {
        byte_t tmp = _read(_rPC);
        _rPC++;
        return tmp;
    }
    inline byte_t _r8(void) {
        byte_t tmp = _read(_rPC);
        _rPC++;
        return tmp;
    }
    inline word_t _a8(void) {
        word_t tmp = _read(_rPC) + 0xff00;
        _rPC++;
        return tmp;
    }

private:
    AddressBus16 *_bus;
    Registers _R;
    IME _ime;
    State _state;
    byte_t _IF;

    LineState _reset_line;

    Cycles _icycles;      /**< Current number of cycles for instruction */
};

};
