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

namespace M6502 {

class M6502Cpu: public CpuDevice
{
public:
    M6502Cpu(Machine *machine, const std::string &name, unsigned clock,
             AddressBus16 *bus);
    ~M6502Cpu(void);

    virtual void save(SaveState &state);
    virtual void load(LoadState &state);
    virtual void execute(Time interval);
    virtual void set_line(InputLine line, LineState state);

private:
    /* Internal state */
    reg16_t  _rPC;
    byte_t _rA;
    byte_t _rX;
    byte_t _rY;
    union {
        byte_t _rSR;
        struct {
            byte_t C:1;
            byte_t Z:1;
            byte_t I:1;
            byte_t D:1;
            byte_t B:1; /* XXX: Break flag */
            byte_t E:1;
            byte_t V:1;
            byte_t N:1;
        } _rF;
    };
    byte_t _rSP;

    LineState _nmi_line;
    LineState _irq_line;

    struct operand {
        bool mem;
        union {
            reg16_t  ad;
            byte_t value;
        };
    } _arg;

    AddressBus16 *_bus;

    /* Dispatch */
    Cycles dispatch(void);
    void _reset(void);

    Cycles _icycles;
    void _add_icycles(unsigned cycles) {
        _icycles += cycles;
    }

    std::string _op_name;
    addr_t _op_pc;
    addr_t _op_ind;
    void op_log(byte_t op);
    void op_set(const std::string &name) {
        _op_name = name;
    }

    byte_t bus_read(addr_t addr) {
        byte_t tmp = _bus->read(addr);
        _add_icycles(1);
        return tmp;
    }

    void bus_write(addr_t addr, byte_t value) {
        _bus->write(addr, value);
        _add_icycles(1);
    }

    byte_t pc_read(void) {
        return bus_read(_rPC.d++);
    };

    byte_t fetch(void) {
        if (_arg.mem) {
            return bus_read(_arg.ad.d);
        } else {
            return _arg.value;
        }
    }

    void store(byte_t value) {
        if (_arg.mem) {
            bus_write(_arg.ad.d, value);
        } else {
            /* XXX: Assume accumulator */
            _rA = value;
        }
    }

    /* Addressing mode */
    inline void Abs(byte_t value = 0) {
        _arg.mem = true;
        _arg.ad.b.l = pc_read();
        _arg.ad.b.h = pc_read();
        /* Handle page overflow */
        _arg.ad.d += value;
        if (_arg.ad.b.l < value)
            _add_icycles(1);
    }
    inline void Ind() {
        _arg.mem = true;
        reg16_t addr;
        addr.b.l = pc_read();
        addr.b.h = pc_read();
        _arg.ad.b.l = bus_read(addr.d);
        addr.b.l++;
        _arg.ad.b.h = bus_read(addr.d);
    }
    inline void IndY(void) {
        _arg.mem = true;
        reg16_t addr;
        addr.b.l = pc_read();
        addr.b.h = 0;
        _arg.ad.b.l = bus_read(addr.d);
        addr.b.l++;
        _arg.ad.b.h = bus_read(addr.d);
        _arg.ad.d += _rY;
        if (_arg.ad.b.l < _rY)
            _add_icycles(1);
    }
    inline void XInd(void) {
        _arg.mem = true;
        addr_t addr = (pc_read() + _rX) & 0xff;
        _arg.ad.b.l = bus_read(addr);
        _arg.ad.b.h = bus_read((addr + 1) & 0xff);
    }
    inline void Imm(void) {
        _arg.mem = false;
        _arg.value = pc_read();
    }
    inline void Acc(void) {
        _arg.mem = false;
        _arg.value = _rA;
    }
    inline void Zpg(byte_t index=0) {
        _arg.mem = true;
        _arg.ad.b.l = pc_read() + index;
        _arg.ad.b.h = 0;
    }
    inline void ZpgX(void) {
        Zpg(_rX);
    }
    inline void ZpgY(void) {
        Zpg(_rY);
    }
    inline void Rel(void) {
        char tmp = pc_read();
        _arg.ad = _rPC;
        _arg.ad.d += tmp;
    }

    inline void set_sz(byte_t result) {
        _rF.N = bit_isset(result, 7);
        _rF.Z = result == 0;
    }

    void push(byte_t value) {
        bus_write(0x0100 + _rSP, value);
        _rSP--;
    }

    byte_t pop(void) {
        _rSP++;
        return bus_read(0x0100 + _rSP);
    }

    /* Operations */
    void op_interrupt(addr_t vector) {
        /* XXX: interrupt */
        push(_rPC.b.h);
        push(_rPC.b.l);
        push(_rSR);
        _rPC.b.l = bus_read(vector);
        _rPC.b.h = bus_read(vector + 1);
    }
    void op_branch(bool jump) {
        if (jump) {
            _rPC = _arg.ad;
        }
    }
    void op_jmp(void) {
        _rPC = _arg.ad;
    }
    void op_ora(void) {
        byte_t arg = fetch();
        byte_t result = _rA | arg;
        set_sz(result);
        _rA = result;
    }
    void op_asl(void) {
        byte_t arg = fetch();
        byte_t result = arg << 1;
        _rF.C = bit_isset(arg, 7);
        set_sz(result);
        store(result);
    }
    void op_bit(void) {
        byte_t arg = fetch();
        _rF.N = bit_isset(arg, 7);
        _rF.V = bit_isset(arg, 6);
        _rF.Z = (_rA & arg) == 0; /* XXX: Is this correct? */
    }
    void op_rol(void) {
        byte_t arg = fetch();
        byte_t result = arg << 1 | _rF.C;
        _rF.C = bit_isset(arg, 7);
        set_sz(result);
        store(result);
    }
    void op_and(void) {
        byte_t arg = fetch();
        byte_t result = _rA & arg;
        set_sz(result);
        _rA = result;
    }
    void op_eor(void) {
        byte_t arg = fetch();
        byte_t result = _rA ^ arg;
        set_sz(result);
        _rA = result;
    }
    void op_adc(void) {
        byte_t arg = fetch();
        if (_rF.D) {
            /* XXX: Decimal */
            throw CpuFault();
        }
        uint32_t result = _rA + arg + _rF.C;
        set_sz(result);
        _rF.C = bit_isset(result, 8);
        _rF.V = bit_isset((_rA^arg^0x80) & (arg^result), 7);
        _rA = result;
    }
    void op_sbc(void) {
        byte_t arg = fetch();
        if (_rF.D) {
            /* XXX: Decimal */
            throw CpuFault();
        }
        uint32_t result = _rA - arg - !_rF.C;
        set_sz(result);
        _rF.C = result < 0x100;
        _rF.V = bit_isset((result^_rA) & (_rA^arg), 7);
        _rA = result;
    }
    void op_ror(void) {
        byte_t arg = fetch();
        byte_t result = arg >> 1 | ( _rF.C << 7);
        _rF.C = bit_isset(arg, 0);
        set_sz(result);
        store(result);
    }
    void op_lsr(void) {
        byte_t arg = fetch();
        byte_t result = arg >> 1;
        _rF.C = bit_isset(arg, 0);
        set_sz(result);
        store(result);
    }
    void op_cmp(byte_t arg1) {
        byte_t arg2 = fetch();
        byte_t result = arg1 - arg2;
        set_sz(result);
        _rF.C = arg2 <= arg1;
    }
    void op_lda(void) {
        byte_t result = fetch();
        set_sz(result);
        _rA = result;
    }
    void op_ldx(void) {
        byte_t result = fetch();
        set_sz(result);
        _rX = result;
    }
    void op_ldy(void) {
        byte_t result = fetch();
        set_sz(result);
        _rY = result;
    }
    void op_dec(void) {
        byte_t arg = fetch();
        byte_t result = arg - 1;
        set_sz(result);
        store(result);
    }
    void op_inc(void) {
        byte_t arg = fetch();
        byte_t result = arg + 1;
        set_sz(result);
        store(result);
    }
    void op_iny(void) {
        byte_t result = _rY + 1;
        set_sz(result);
        _rY = result;
    }
    void op_dey(void) {
        byte_t result = _rY - 1;
        set_sz(result);
        _rY = result;
    }
    void op_inx(void) {
        byte_t result = _rX + 1;
        set_sz(result);
        _rX = result;
    }
    void op_dex(void) {
        byte_t result = _rX - 1;
        set_sz(result);
        _rX = result;
    }
};

};
