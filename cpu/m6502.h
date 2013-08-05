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

#include "emu/emu.h"

using namespace EMU;

namespace M6502 {

class M6502Cpu: public CpuDevice
{
public:
    M6502Cpu(Machine *machine, const std::string &name, unsigned clock,
             AddressBus16 *bus);
    virtual ~M6502Cpu(void);

    virtual void execute(Time interval);
    virtual void line(Line line, LineState state);

protected:
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
    byte_t _zpg;

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
    void _reset(void);

    Cycles _icycles;
    void _add_icycles(unsigned cycles) {
        _icycles += cycles;
    }

    std::string _op_name;
    addr_t _op_pc;
    addr_t _op_ind;
    void log_op(byte_t op);
    void start_op(const std::string &name) {
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
    inline void Ind(byte_t value = 0) {
        _arg.mem = true;
        reg16_t addr;
        addr.b.l = pc_read();
        addr.b.h = pc_read();
        _arg.ad.d += value;
        _arg.ad.b.l = bus_read(addr.d);
        addr.b.l++;
        _arg.ad.b.h = bus_read(addr.d);
    }
    inline void IndY(void) {
        _arg.mem = true;
        reg16_t addr;
        addr.b.l = pc_read();
        addr.b.h = _zpg;
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
    inline void ZpgInd(void) {
        _arg.mem = true;
        reg16_t addr;
        addr.b.l = pc_read();
        addr.b.h = _zpg;
        _arg.ad.b.l = bus_read(addr.d);
        addr.b.l++;
        _arg.ad.b.h = bus_read(addr.d);
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
        _arg.ad.b.h = _zpg;
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
        bus_write((_zpg << 8) + 0x0100 + _rSP, value);
        _rSP--;
    }

    byte_t pop(void) {
        _rSP++;
        return bus_read((_zpg << 8) + 0x0100 + _rSP);
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
            _add_icycles(2);
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
    void op_rti(void) {
        _rSR = pop();
        _rF.E = 1;
        _rPC.b.l = pop();
        _rPC.b.h = pop();
    }
    void op_jsr(void) {
        Abs();
        _rPC.d--;
        push(_rPC.b.h);
        push(_rPC.b.l);
        op_jmp();
    }
    void op_plp(void) {
        _rSR = pop();
        _rF.B = 1;
        _rF.E = 1;
    }
    void op_eor(void) {
        byte_t arg = fetch();
        byte_t result = _rA ^ arg;
        set_sz(result);
        _rA = result;
    }
    void op_adc(void) {
        /* XXX: Decimal */
        if (_rF.D)
            throw CpuFeatureFault(_name, "decimal");
        byte_t arg = fetch();
        uint32_t result = _rA + arg + _rF.C;
        set_sz(result);
        _rF.C = bit_isset(result, 8);
        _rF.V = bit_isset((_rA^arg^0x80) & (arg^result), 7);
        _rA = result;
    }
    void op_sbc(void) {
        /* XXX: Decimal */
        if (_rF.D)
            throw CpuFeatureFault(_name, "decimal");
        byte_t arg = fetch();
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

private:

    Cycles dispatch(void);
};

class n2A03Cpu: public M6502Cpu
{
public:
    n2A03Cpu(Machine *machine, const std::string &name, unsigned clock,
             AddressBus16 *bus);
    ~n2A03Cpu(void);

    virtual void execute(Time interval);
    virtual void line(Line line, LineState state);

private:
    void op_nd_adc(void) {
        byte_t arg = fetch();
        uint32_t result = _rA + arg + _rF.C;
        set_sz(result);
        _rF.C = bit_isset(result, 8);
        _rF.V = bit_isset((_rA^arg^0x80) & (arg^result), 7);
        _rA = result;
    }
    void op_nd_sbc(void) {
        byte_t arg = fetch();
        uint32_t result = _rA - arg - !_rF.C;
        set_sz(result);
        _rF.C = result < 0x100;
        _rF.V = bit_isset((result^_rA) & (_rA^arg), 7);
        _rA = result;
    }

    Cycles dispatch(void);
};

class m65c02Cpu: public M6502Cpu
{
public:
    m65c02Cpu(Machine *machine, const std::string &name, unsigned clock,
              AddressBus16 *bus);
    virtual ~m65c02Cpu(void);

    virtual void execute(Time interval);
    virtual void line(Line line, LineState state);

protected:

    void op_tsb(void) {
        byte_t arg = fetch();
        byte_t result = _rA | arg;
        _rF.Z = (_rA & arg) == 0;
        store(result);
    }

    void op_trb(void) {
        byte_t arg = fetch();
        byte_t result = ~_rA & arg;
        _rF.Z = (_rA & arg) == 0;
        store(result);
    }

    void op_bitimm(void) {
        byte_t arg = fetch();
        _rF.Z = (_rA & arg) == 0;
    }

    void op_ina(void) {
        byte_t result = _rA + 1;
        set_sz(result);
        _rA = result;
    }

    void op_dea(void) {
        byte_t result = _rA - 1;
        set_sz(result);
        _rA = result;
    }

    void op_phx(void) {
        push(_rX);
    }

    void op_plx(void) {
        byte_t result = pop();
        _rX = result;
        set_sz(result);
    }

    void op_phy(void) {
        push(_rY);
    }

    void op_ply(void) {
        byte_t result = pop();
        _rY = result;
        set_sz(result);
    }

    /* Only available on some WDC and Rockwell versions */
    void op_smb(int bit) {
        byte_t arg = fetch();
        byte_t result = arg | (1 << bit);
        store(result);
    }

    void op_rmb(int bit) {
        byte_t arg = fetch();
        byte_t result = arg & ~(1 << bit);
        store(result);
    }

    void op_bbr(int bit) {
        Zpg();
        byte_t arg = fetch();
        Rel();
        op_branch(!bit_isset(arg, bit));
    }

    void op_bbs(int bit) {
        Zpg();
        byte_t arg = fetch();
        Rel();
        op_branch(bit_isset(arg, bit));
    }

private:

    Cycles dispatch(void);
    void reset(void);

};

class hu6280Cpu: public m65c02Cpu
{
public:
    hu6280Cpu(Machine *machine, const std::string &name, unsigned clock,
              AddressBus21 *bus);
    virtual ~hu6280Cpu(void);

    virtual void execute(Time interval);
    virtual void line(Line line, LineState state);

    byte_t irq_read(offset_t offset);
    void irq_write(offset_t offset , byte_t value);

    byte_t timer_read(offset_t offset);

    void timer_write(offset_t offset, byte_t value);

protected:

    void op_sxy(void) {
        _rX ^= _rY;
        _rY ^= _rX;
        _rX ^= _rY;
    }
    void op_sax(void) {
        _rA ^= _rX;
        _rX ^= _rA;
        _rA ^= _rX;
    }
    void op_say(void) {
        _rA ^= _rY;
        _rY ^= _rA;
        _rA ^= _rY;
    }
    void op_set(void) {
        throw CpuFeatureFault(_name, "T flag");
    }
    void op_tstart(reg16_t *src, reg16_t *dest, reg16_t *len) {
        src->b.l = pc_read();
        src->b.h = pc_read();
        dest->b.l = pc_read();
        dest->b.h = pc_read();
        len->b.l = pc_read();
        len->b.h = pc_read();
        push(_rY);
        push(_rA);
        push(_rX);
    }
    void op_tdd(void) {
        reg16_t src, dest, len;
        op_tstart(&src, &dest, &len);

        do {
            byte_t value = bus_read(src.d);
            bus_write(dest.d, value);
            dest.d -= 1;
            src.d -= 1;
            len.d -= 1;
        } while (len.d != 0);

        _rX = pop();
        _rA = pop();
        _rY = pop();
    }
    void op_tii(void) {
        reg16_t src, dest, len;
        op_tstart(&src, &dest, &len);

        do {
            byte_t value = bus_read(src.d);
            bus_write(dest.d, value);
            dest.d += 1;
            src.d += 1;
            len.d -= 1;
        } while (len.d != 0);

        _rX = pop();
        _rA = pop();
        _rY = pop();
    }
    void op_tin(void) {
        reg16_t src, dest, len;
        op_tstart(&src, &dest, &len);

        do {
            byte_t value = bus_read(src.d);
            bus_write(dest.d, value);
            src.d += 1;
            len.d -= 1;
        } while (len.d != 0);

        _rX = pop();
        _rA = pop();
        _rY = pop();
    }
    void op_tai(void) {
        reg16_t src, dest, len;
        op_tstart(&src, &dest, &len);
        int b = 0;

        do {
            byte_t value = bus_read(src.d + b);
            bus_write(dest.d, value);
            dest.d += 1;
            len.d -= 1;
            b ^= 1;
        } while (len.d != 0);

        _rX = pop();
        _rA = pop();
        _rY = pop();
    }
    void op_tia(void) {
        reg16_t src, dest, len;
        op_tstart(&src, &dest, &len);
        int b = 0;

        do {
            byte_t value = bus_read(src.d);
            bus_write(dest.d + b, value);
            src.d += 1;
            len.d -= 1;
            b ^= 1;
        } while (len.d != 0);

        _rX = pop();
        _rA = pop();
        _rY = pop();
    }

    void op_tam(void) {
        byte_t arg = fetch();
        for (int i = 0; i < 8; i++)
            if (bit_isset(arg, i))
                _mmu_map[i] = _rA;
    }
    void op_tma(void) {
        byte_t arg = fetch();
        byte_t result = 0;
        for (int i = 0; i < 8; i++)
            if (bit_isset(arg, i))
                result |= _mmu_map[i];
        _rA = result;
    }
    void op_tst(byte_t value) {
        byte_t arg = fetch();
        _rF.N = bit_isset(arg, 7);
        _rF.V = bit_isset(arg, 6);
        _rF.Z = (value & arg) == 0;
    }
    void op_csl(void) {
        _clock_div = 4;
    }
    void op_csh(void) {
        _clock_div = 1;
    }
    void op_cla(void) {
        _rA = 0;
    }
    void op_clx(void) {
        _rX = 0;
    }
    void op_cly(void) {
        _rY = 0;
    }
    void op_bsr(void) {
        Rel();
        _rPC.d--;
        push(_rPC.b.h);
        push(_rPC.b.l);
        op_jmp();
    }
    void op_st0(void) {
        Imm();
        byte_t arg = fetch();
        _data_bus->write(0x1FE000, arg);
    }
    void op_st1(void) {
        Imm();
        byte_t arg = fetch();
        _data_bus->write(0x1FE002, arg);
    }
    void op_st2(void) {
        Imm();
        byte_t arg = fetch();
        _data_bus->write(0x1FE003, arg);
    }
    void op_irq(addr_t vector, bool brk = false) {
        /* XXX: interrupt */
        _rF.B = brk;
        push(_rPC.b.h);
        push(_rPC.b.l);
        push(_rSR);
        _rF.I = 1;
        _rF.D = 0;
        _rPC.b.l = bus_read(vector);
        _rPC.b.h = bus_read(vector + 1);
    }

private:

    bool interrupt(void);

    byte_t mmu_read(offset_t offset);
    void mmu_write(offset_t offset, byte_t value);

    Cycles dispatch(void);
    void reset(void);

    AddressBus21 *_data_bus;
    AddressBus16 _mmu;
    byte_t _mmu_map[8];
    int _clock_div;

    byte_t _irq_status;
    byte_t _irq_disable;

    bool _timer_status;
    int _timer_load;
    int _timer_value;
};

};
