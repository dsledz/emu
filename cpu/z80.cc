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

#include "z80.h"

using namespace EMU;
using namespace Z80;

Registers::Registers(void):
    _AF({}),
    _BC(0xFFFF),
    _DE(0xFFFF),
    _HL(0xFFFF),
    _IX(0xFFFF),
    _IY(0xFFFF),
    _SP(0xFFFF),
    _PC(0x0000),
    _sAF(0xFFFF),
    _sBC(0xFFFF),
    _sDE(0xFFFF),
    _sHL(0xFFFF)
{
}

Registers::~Registers(void)
{
}

Z80Cpu::Z80Cpu(Machine *machine, const std::string &name, unsigned hertz,
               AddressBus16 *bus):
    CpuDevice(machine, name, hertz),
    _icycles(0),
    _op(),
    _R(),
    _iff1(false),
    _iff2(false),
    _iwait(false),
    _state(DeviceState::Running),
    _nmi_line(LineState::Clear),
    _int0_line(LineState::Clear),
    _reset_line(LineState::Clear),
    _wait_line(LineState::Clear),
    _rom(),
    _bus(bus)
{
}

Z80Cpu::~Z80Cpu(void)
{
}

void
Z80Cpu::execute(Time interval)
{
    _avail += interval.to_cycles(Cycles(_hertz));
    /* XXX: Handle fault states. */

    /* Execute as much as possible. We'll save the left over cycles for the
     * next call. Hopefully this is okay. */
    while (_avail > 0) {
        _avail -= step();
    }
}

Cycles
Z80Cpu::step(void)
{
    _icycles = Cycles(0);

    if (_reset_line == LineState::Pulse) {
        /* XXX: No one seems to clear this line... */
        _reset();
        _reset_line = LineState::Clear;
        return _icycles;
    } else if (_nmi_line == LineState::Pulse) {
        interrupt(0x0066);
        _nmi_line = LineState::Clear;
        return _icycles;
    } else if (_int0_line == LineState::Assert && _iff1 && !_iwait) {
        switch (_imode) {
        case 0:
            throw CpuFault(_name, "Unsupported Interrupt mode 0");
            break;
        case 1:
            interrupt(0x0038);
            return _icycles;
            break;
        case 2:
            throw CpuFault(_name, "Unsupported Interrupt mode 2");
            break;
        }
    }

    if (_iwait)
        _iwait = false;

    dispatch();
    return _icycles;
}

void
Z80Cpu::_reset(void)
{
    DBG("Z80 reset");
    _R = Registers();
    _iff1 = false;
    _iff2 = false;
    _iwait = false;
    _imode = 0;
}

void
Z80Cpu::line(Line line, LineState state)
{
    switch (line) {
    case Line::RESET:
        _reset_line = state;
        break;
    case Line::INT0:
        _int0_line = state;
        break;
    case Line::NMI:
        _nmi_line = state;
        break;
    case Line::WAIT:
        _wait_line = state;
        break;
    default:
        break;
    }
}

/*
 *  ___                       _   _
 * / _ \ _ __   ___ _ __ __ _| |_(_) ___  _ __  ___
 *| | | | '_ \ / _ \ '__/ _` | __| |/ _ \| '_ \/ __|
 *| |_| | |_) |  __/ | | (_| | |_| | (_) | | | \__ \
 * \___/| .__/ \___|_|  \__,_|\__|_|\___/|_| |_|___/
 *      |_|
 */

byte_t
Z80Cpu::fetch(Reg reg)
{
    switch (reg) {
        case Reg::A: return _rA;
        case Reg::B: return _rB;
        case Reg::C: return _rC;
        case Reg::D: return _rD;
        case Reg::E: return _rE;
        case Reg::H: return _rH;
        case Reg::L: return _rL;
        case Reg::HL: return bus_read(_rHL);
        default: throw CpuFault(_name, "Unknown register read");
    }
}

void
Z80Cpu::store(Reg reg, byte_t value)
{
    switch (reg) {
        case Reg::A: _rA = value; break;
        case Reg::B: _rB = value; break;
        case Reg::C: _rC = value; break;
        case Reg::D: _rD = value; break;
        case Reg::E: _rE = value; break;
        case Reg::H: _rH = value; break;
        case Reg::L: _rL = value; break;
        case Reg::HL: bus_write(_rHL, value); break;
        default: throw CpuFault(_name, "Unknown register write");
    }
}

void
Z80Cpu::load_rom(Rom *rom, addr_t offset)
{
    /* Make sure our private rom is large enough */
    if (_rom.size() < rom->size() + offset)
        _rom.resize(rom->size() + offset);
    memcpy(&_rom[offset], rom->direct(0), rom->size());
}

void
Z80Cpu::_add(byte_t &dest, byte_t arg)
{
    uint16_t result = dest + arg;

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result & 0xff) == 0;
    _flags.Y = bit_isset(result, 5);
    _flags.H = bit_isset(dest ^ arg ^ result, 4);
    _flags.X = bit_isset(result, 3);
    _flags.V = bit_isset(dest ^ arg ^ result, 7);
    _flags.N = false;
    _flags.C = bit_isset(result, 8);

    dest = result;
}

void
Z80Cpu::_inc(byte_t &dest)
{
    uint16_t result = dest + 1;

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result & 0xff) == 0;
    _flags.Y = bit_isset(result, 5);
    _flags.H = bit_isset(dest ^ 1 ^ result, 4);
    _flags.X = bit_isset(result, 3);
    _flags.V = (dest == 0x7F);
    _flags.N = false;

    dest = result;
}

void
Z80Cpu::_inci(addr_t addr)
{
    byte_t dest = bus_read(addr);

    _inc(dest);

    bus_write(addr, dest);
}

void
Z80Cpu::_adc(byte_t &dest, byte_t arg)
{
    uint16_t result = dest + arg + _flags.C;

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result & 0xff) == 0;
    _flags.Y = bit_isset(result, 5);
    _flags.H = bit_isset(dest ^ arg ^ result, 4);
    _flags.X = bit_isset(result, 3);
    _flags.V = bit_isset((dest ^ arg ^ 0x80) & (arg ^ result), 7);
    _flags.N = false;
    _flags.C = bit_isset(result, 8);

    dest = result;
}

void
Z80Cpu::_sub(byte_t &dest, byte_t arg)
{
    uint16_t result = dest - arg;

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result & 0xff) == 0;
    _flags.Y = bit_isset(result, 5);
    _flags.H = bit_isset(dest ^ arg ^ result, 4);
    _flags.X = bit_isset(result, 3);
    _flags.V = bit_isset((result^dest) & (dest^arg), 7);
    _flags.N = true;
    _flags.C = bit_isset(dest ^ arg ^ result, 8);

    dest = result;
}

void
Z80Cpu::_dec(byte_t &dest)
{
    uint16_t result = dest - 1;

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result & 0xff) == 0;
    _flags.Y = bit_isset(result, 5);
    _flags.H = bit_isset(dest ^ 1 ^ result, 4);
    _flags.X = bit_isset(result, 3);
    _flags.V = (dest == 0x80);
    _flags.N = true;

    dest = result;
}

void
Z80Cpu::_deci(addr_t addr)
{
    byte_t dest = bus_read(addr);

    _dec(dest);

    bus_write(addr, dest);
}

void
Z80Cpu::_sbc(byte_t &dest, byte_t arg)
{
    uint16_t result = dest - arg - _flags.C;

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result & 0xff) == 0;
    _flags.Y = bit_isset(result, 5);
    _flags.H = bit_isset(dest ^ arg ^ result, 4);
    _flags.X = bit_isset(result, 3);
    _flags.V = bit_isset((result^dest) & (dest^arg), 7);
    _flags.N = true;
    _flags.C = bit_isset(dest ^ arg ^ result, 8);


    dest = result;
}

void
Z80Cpu::_neg(byte_t &dest)
{
    byte_t arg = dest;
    dest = 0;

    _sub(dest, arg);
}

void
Z80Cpu::_cp(byte_t dest, byte_t arg)
{
    uint16_t result = dest - arg;
    _flags.S = bit_isset(result, 7);
    _flags.Z = (result & 0xff) == 0;
    _flags.Y = bit_isset(arg, 5);
    _flags.H = bit_isset(arg ^ dest ^ result, 4);
    _flags.X = bit_isset(arg, 3);
    _flags.V = bit_isset((result^dest) & (dest^arg), 7);
    _flags.N = true;
    _flags.C = bit_isset(dest ^ arg ^ result, 8);
}

void
Z80Cpu::_cpi(void)
{
    byte_t dest = _rA;
    byte_t arg = bus_read(_rHL);
    uint16_t result = dest - arg;
    _rHL++;
    _rBC--;

    _flags.H = bit_isset(arg ^ dest ^ result, 4);
    _flags.S = bit_isset(result, 7);
    _flags.Z = (result & 0xff) == 0;
    _flags.Y = bit_isset(result - _flags.H, 1);
    _flags.X = bit_isset(result - _flags.H, 3);
    _flags.V = (_rBC != 0);
    _flags.N = true;
}

void
Z80Cpu::_cpir(void)
{
    _cpi();
    if (_rBC != 0 && !_flags.Z) {
        _rPC -= 2;
        _add_icycles(5);
    }
}

void
Z80Cpu::_cpd(void)
{
    byte_t dest = _rA;
    byte_t arg = bus_read(_rHL);
    uint16_t result = dest - arg;
    _rHL--;
    _rBC--;

    _flags.H = bit_isset(arg ^ dest ^ result, 4);
    _flags.S = bit_isset(result, 7);
    _flags.Z = (result & 0xff) == 0;
    _flags.Y = bit_isset(result - _flags.H, 1);
    _flags.X = bit_isset(result - _flags.H, 3);
    _flags.V = (_rBC != 0);
    _flags.N = true;
}

void
Z80Cpu::_cpdr(void)
{
    _cpd();
    if (_rBC != 0 && !_flags.Z) {
        _rPC -= 2;
        _add_icycles(5);
    }
}

void
Z80Cpu::_rrd(void)
{
    byte_t value = bus_read(_rHL);

    bus_write(_rHL, (_rA & 0x0F) << 4 | ((value & 0xF0) >> 4));
    _rA = (_rA & 0xF0) | (value & 0x0F);

    _flags.S = bit_isset(_rA, 7);
    _flags.Z = _rA == 0;
    _flags.Y = bit_isset(_rA, 5);
    _flags.H = false;
    _flags.X = bit_isset(_rA, 3);
    _set_parity(_rA);
    _flags.N = false;
}

void
Z80Cpu::_rld(void)
{
    byte_t value = bus_read(_rHL);

    bus_write(_rHL, (_rA & 0x0F) | ((value & 0x0F) << 4));
    _rA = (_rA & 0xF0) | ((value & 0xF0) >> 4);

    _flags.S = bit_isset(_rA, 7);
    _flags.Z = _rA == 0;
    _flags.Y = bit_isset(_rA, 5);
    _flags.H = false;
    _flags.X = bit_isset(_rA, 3);
    _set_parity(_rA);
    _flags.N = false;
}

void
Z80Cpu::_and(byte_t &dest, byte_t arg)
{
    byte_t result = dest & arg;

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result & 0xff) == 0;
    _flags.Y = bit_isset(result, 5);
    _flags.H = true;
    _flags.X = bit_isset(result, 3);
    _set_parity(result);
    _flags.N = false;
    _flags.C = false;

    dest = result;
}

void
Z80Cpu::_xor(byte_t &dest, byte_t arg)
{
    byte_t result = dest ^ arg;

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result & 0xff) == 0;
    _flags.Y = bit_isset(result, 5);
    _flags.H = false;
    _flags.X = bit_isset(result, 3);
    _set_parity(result);
    _flags.N = false;
    _flags.C = false;

    dest = result;
}

void
Z80Cpu::_or(byte_t &dest, byte_t arg)
{
    byte_t result = dest | arg;

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result & 0xff) == 0;
    _flags.Y = bit_isset(result, 5);
    _flags.H = false;
    _flags.X = bit_isset(result, 3);
    _set_parity(result);
    _flags.N = false;
    _flags.C = false;

    dest = result;
}

void
Z80Cpu::_ld(byte_t &dest, byte_t arg)
{
    byte_t result = arg;

    dest = result;
}

void
Z80Cpu::_bit_test(byte_t value, int bit)
{
    _flags.S = bit == 7 && bit_isset(value, bit);
    _flags.Z = !bit_isset(value, bit);
    _flags.Y = bit == 5 && bit_isset(value, bit);
    _flags.H = 1;
    _flags.X = bit == 3 && bit_isset(value, bit);
    _flags.V = !bit_isset(value, bit);
    _flags.N = false;
}

void
Z80Cpu::_bit_reset(byte_t &dest, byte_t value, int bit)
{
    byte_t result = value & ~(1 << bit);

    dest = result;
}

void
Z80Cpu::_bit_set(byte_t &dest, byte_t value, int bit)
{
    byte_t result = value | (1 << bit);

    dest = result;
}

void
Z80Cpu::_rl(byte_t &dest, byte_t value)
{
    byte_t result = (value << 1) | _flags.C;

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result == 0);
    _flags.Y = bit_isset(result, 5);
    _flags.H = false;
    _flags.X = bit_isset(result, 3);
    _set_parity(result);
    _flags.N = false;
    _flags.C = bit_isset(value, 7);

    dest = result;
}

void
Z80Cpu::_rla(void)
{
    bool S = _flags.S;
    bool Z = _flags.Z;
    bool V = _flags.V;
    _rl(_rA, _rA);
    _flags.S = S;
    _flags.Z = Z;
    _flags.V = V;
}

void
Z80Cpu::_rlc(byte_t &dest, byte_t value)
{
    byte_t result = (value << 1) | ((value & 0x80) >> 7);

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result == 0);
    _flags.Y = bit_isset(result, 5);
    _flags.H = false;
    _flags.X = bit_isset(result, 3);
    _set_parity(result);
    _flags.N = false;
    _flags.C = (value & 0x80) != 0;

    dest = result;
}

void
Z80Cpu::_rlca(void)
{
    bool S = _flags.S;
    bool Z = _flags.Z;
    bool V = _flags.V;
    _rlc(_rA, _rA);
    _flags.S = S;
    _flags.Z = Z;
    _flags.V = V;
}

void
Z80Cpu::_rr(byte_t &dest, byte_t value)
{
    byte_t result = (value >> 1) | (_flags.C ? 0x80 : 0x00);

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result == 0);
    _flags.Y = bit_isset(result, 5);
    _flags.H = false;
    _flags.X = bit_isset(result, 3);
    _set_parity(result);
    _flags.N = false;
    _flags.C = (value & 0x01) != 0;

    dest = result;
}

void
Z80Cpu::_rra(void)
{
    bool S = _flags.S;
    bool Z = _flags.Z;
    bool V = _flags.V;
    _rr(_rA, _rA);
    _flags.S = S;
    _flags.Z = Z;
    _flags.V = V;
}

void
Z80Cpu::_rrc(byte_t &dest, byte_t value)
{
    byte_t result = (value >> 1) | (value & 0x01 ? 0x80 : 0x00);

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result == 0);
    _flags.Y = bit_isset(result, 5);
    _flags.H = false;
    _flags.X = bit_isset(result, 3);
    _set_parity(result);
    _flags.N = false;
    _flags.C = (value & 0x01) != 0;

    dest = result;
}

void
Z80Cpu::_rrca(void)
{
    bool S = _flags.S;
    bool Z = _flags.Z;
    bool V = _flags.V;
    _rrc(_rA, _rA);
    _flags.S = S;
    _flags.Z = Z;
    _flags.V = V;
}

void
Z80Cpu::_sla(byte_t &dest, byte_t value)
{
    byte_t result = (value << 1);

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result == 0);
    _flags.Y = bit_isset(result, 5);
    _flags.H = false;
    _flags.X = bit_isset(result, 3);
    _set_parity(result);
    _flags.N = false;
    _flags.C = bit_isset(value, 7);

    dest = result;
}

void
Z80Cpu::_sra(byte_t &dest, byte_t value)
{
    byte_t result = (value >> 1) | (value & 0x80);

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result == 0);
    _flags.Y = bit_isset(result, 5);
    _flags.H = false;
    _flags.X = bit_isset(result, 3);
    _set_parity(result);
    _flags.N = false;
    _flags.C = bit_isset(value, 0);

    dest = result;
}

void
Z80Cpu::_srl(byte_t &dest, byte_t value)
{
    byte_t result = (value >> 1);

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result == 0);
    _flags.Y = bit_isset(result, 5);
    _flags.H = false;
    _flags.X = bit_isset(result, 3);
    _set_parity(result);
    _flags.N = false;
    _flags.C = bit_isset(value, 0);

    dest = result;
}

void
Z80Cpu::_sll(byte_t &dest, byte_t value)
{
    byte_t result = (value << 1) | 0x01;

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result == 0);
    _flags.Y = bit_isset(result, 5);
    _flags.H = false;
    _flags.X = bit_isset(result, 3);
    _set_parity(result);
    _flags.N = false;
    _flags.C = bit_isset(value, 7);

    dest = result;
}

void
Z80Cpu::_rst(byte_t arg)
{
    _push(_rPCh, _rPCl);
    _rPC = arg;
}

void
Z80Cpu::_jr(bool jump, byte_t arg)
{
    if (jump) {
        _rPC += (char)arg;
        _add_icycles(4);
    }
}

void
Z80Cpu::_jp(bool jump, uint16_t arg)
{
    if (jump) {
        _rPC = arg;
        _add_icycles(4);
    }
}

void
Z80Cpu::_call(bool jump, uint16_t addr)
{
    if (jump) {
        _push(_rPCh, _rPCl);

        _rPC = addr;
        _add_icycles(12);
    }
}

void
Z80Cpu::_djnz(byte_t arg)
{
    if (--_rB != 0) {
        _rPC += (char)arg;
        _add_icycles(5);
    }
}

void
Z80Cpu::_ret(bool jump)
{
    if (jump) {
        _pop(_rPCh, _rPCl);
        _add_icycles(16);
    }
}

void
Z80Cpu::_reti(void)
{
    _pop(_rPCh, _rPCl);
    _iff1 = _iff2 = true;
}

void
Z80Cpu::_retn(void)
{
    _pop(_rPCh, _rPCl);
    _iff1 = _iff2;
}

void
Z80Cpu::_di(void)
{
    _iff1 = _iff2 = false;
}

void
Z80Cpu::_ei(void)
{
    _iff1 = _iff2 = true;
    _iwait = true;
}

/*  ____                  _       _    ___
 * / ___| _ __   ___  ___(_) __ _| |  / _ \ _ __  ___
 * \___ \| '_ \ / _ \/ __| |/ _` | | | | | | '_ \/ __|
 *  ___) | |_) |  __/ (__| | (_| | | | |_| | |_) \__ \
 * |____/| .__/ \___|\___|_|\__,_|_|  \___/| .__/|___/
 *       |_|                               |_|
 */
void
Z80Cpu::_ex(uint16_t &lhs, uint16_t &rhs)
{
    lhs ^= rhs;
    rhs ^= lhs;
    lhs ^= rhs;
}

void
Z80Cpu::_exi(addr_t addr, byte_t &rh, byte_t &rl)
{
    byte_t ll = bus_read(addr);
    byte_t lh = bus_read(addr + 1);
    bus_write(addr, rl);
    bus_write(addr + 1, rh);
    rh = lh;
    rl = ll;
}

void
Z80Cpu::_exx(void)
{
    _ex(_rBC, _rsBC);
    _ex(_rDE, _rsDE);
    _ex(_rHL, _rsHL);
}

void
Z80Cpu::_push(byte_t high, byte_t low)
{
    bus_write(--_rSP, high);
    bus_write(--_rSP, low);
}

void
Z80Cpu::_pop(byte_t &high, byte_t &low)
{
    low = bus_read(_rSP++);
    high = bus_read(_rSP++);
}

void
Z80Cpu::_ldi(void)
{
    byte_t value = bus_read(_rHL++);
    bus_write(_rDE++, value);
    value += _rA;
    _rBC--;
    _flags.Y = bit_isset(value, 1);
    _flags.X = bit_isset(value, 3);
    _flags.H = 0;
    _flags.N = 0;
    _flags.V = (_rBC != 0);
}

void
Z80Cpu::_ldir(void)
{
    _ldi();
    if (_rBC != 0) {
        _rPC -= 2;
        _add_icycles(5);
    }
}

void
Z80Cpu::_ldd(void)
{
    byte_t value = bus_read(_rHL--);
    bus_write(_rDE--, value);
    value += _rA;
    _rBC--;
    _flags.Y = bit_isset(value, 1);
    _flags.X = bit_isset(value, 3);
    _flags.H = 0;
    _flags.N = 0;
    _flags.V = (_rBC != 0);
}

void
Z80Cpu::_lddr(void)
{
    _ldd();
    if (_rBC != 0) {
        _rPC -= 2;
        _add_icycles(5);
    }
}


void
Z80Cpu::_halt()
{
    // Cpu is halted until the next interrupt
    _state = DeviceState::Halted;
}

void
Z80Cpu::_add16(uint16_t &wdest, uint16_t arg)
{
    uint32_t result = wdest + arg;

    _flags.Y = bit_isset(result, 13);
    _flags.H = bit_isset(wdest ^ arg ^ result, 12);
    _flags.X = bit_isset(result, 11);
    _flags.C = (result > 0xffff) ? true : false;
    _flags.N = false;

    wdest = result;
}

void
Z80Cpu::_adc16(uint16_t &wdest, uint16_t arg)
{
    uint32_t result = wdest + arg + _flags.C;

    _flags.S = bit_isset(result, 15);
    _flags.Z = (result & 0xffff) == 0;
    _flags.Y = bit_isset(result, 13);
    _flags.H = bit_isset(wdest ^ arg  ^ result, 12);
    _flags.X = bit_isset(result, 11);
    _flags.V = bit_isset((wdest ^ arg ^ 0x8000) & (arg ^ result), 15);
    _flags.N = false;
    _flags.C = bit_isset(result, 16);

    wdest = result;
}

void
Z80Cpu::_sbc16(uint16_t &wdest, uint16_t arg)
{
    uint32_t result = wdest - arg - _flags.C;

    _flags.S = bit_isset(result, 15);
    _flags.Z = (result & 0xffff) == 0;
    _flags.Y = bit_isset(result, 13);
    _flags.H = bit_isset(wdest ^ arg  ^ result, 12);
    _flags.X = bit_isset(result, 11);
    _flags.V = bit_isset((wdest ^ arg) & (wdest ^ result), 15);
    _flags.N = true;
    _flags.C = bit_isset(result, 16);

    wdest = result;
}

void
Z80Cpu::_inc16(uint16_t &wdest)
{
    uint16_t result = wdest + 1;

    wdest = result;
}

void
Z80Cpu::_sub16(uint16_t &wdest, uint16_t arg)
{
    uint16_t result = wdest - arg;

    wdest = result;
}

void
Z80Cpu::_dec16(uint16_t &wdest)
{
    uint16_t result = wdest - 1;

    wdest = result;
}

void
Z80Cpu::_ld16(uint16_t &wdest, uint16_t arg)
{
    wdest = arg;
}

void
Z80Cpu::_ldmem(addr_t addr, byte_t arg)
{
    bus_write(addr, arg);
}

void
Z80Cpu::_ld16i(addr_t addr, uint16_t arg)
{
    bus_write(addr, arg & 0xff);
    bus_write(addr+1, arg >> 8);
}

void
Z80Cpu::_cpl(void)
{
    byte_t &dest = _rA;
    byte_t result = ~dest;

    _flags.Y = bit_isset(result, 5);
    _flags.H = 1;
    _flags.X = bit_isset(result, 3);
    _flags.N = 1;

    dest = result;
}

void
Z80Cpu::_ccf(void)
{
    _flags.Y = bit_isset(_rA, 5);
    _flags.H = _flags.C;
    _flags.X = bit_isset(_rA, 3);
    _flags.N = 0;
    _flags.C = !_flags.C;
}

void
Z80Cpu::_scf(void)
{
    _flags.Y = bit_isset(_rA, 5);
    _flags.H = 0;
    _flags.X = bit_isset(_rA, 3);
    _flags.N = 0;
    _flags.C = 1;

}

void
Z80Cpu::_daa(void)
{
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

    _flags.S = bit_isset(result, 7);
    _flags.Z = (result & 0xff) == 0;
    _flags.Y = bit_isset(result, 5);
    _flags.H = 0;
    _flags.X = bit_isset(result, 3);
    _set_parity(result);
    _flags.C = bit_isset(dest ^ arg ^ result, 8);

    dest = result;
}

/* XXX: rewrite */
void Tokenize(const std::string& str,
              std::vector<std::string>& tokens,
              const std::string& delimiters)
{
    // Skip delimiters at beginning.
    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    // Find first "non-delimiter".
    std::string::size_type pos     = str.find_first_of(delimiters, lastPos);

    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}

void
Z80Cpu::op_log(void)
{
    std::stringstream os;
    os << std::setw(8) << _name << ":"
       << Hex(_op.pc) << ":" << Hex(_op.opcode) << ":"
       << _op.name << " =>";
    const std::string &str = _op.name;
    const std::string &delimiters = " ,";
    auto lastPos = str.find_first_not_of(delimiters, 0);
    auto pos = str.find_first_of(delimiters, lastPos);
    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        std::string it = str.substr(lastPos, pos - lastPos);
        os << " ";
        if (it == "A")
            os << Hex(_rA);
        else if (it == "B")
            os << Hex(_rB);
        else if (it == "C")
            os << Hex(_rC);
        else if (it == "D")
            os << Hex(_rD);
        else if (it == "E")
            os << Hex(_rE);
        else if (it == "H")
            os << Hex(_rH);
        else if (it == "L")
            os << Hex(_rL);
        else if (it == "BC")
            os << Hex(_rBC);
        else if (it == "(BC)")
            os << "(" << Hex(_rBC) << ")";
        else if (it == "DE")
            os << Hex(_rDE);
        else if (it == "(DE)")
            os << "(" << Hex(_rDE) << ")";
        else if (it == "HL")
            os << Hex(_rHL);
        else if (it == "(HL)")
            os << "(" << Hex(_rHL) << ")";
        else if (it == "AF")
            os << Hex(_rAF);
        else if (it == "AF'")
            os << Hex(_rsAF);
        else if (it == "SP")
            os << Hex(_rSP);
        else if (it == "IX")
            os << Hex(_rIX);
        else if (it == "(IX+d)")
            os << "(" << Hex(_rIX) << "+" << Hex(_op.d8) << ")";
        else if (it == "IY")
            os << Hex(_rIY);
        else if (it == "(IY+d)")
            os << "(" << Hex(_rIY) << "+" << Hex(_op.d8) << ")";
        else if (it == "d16" || it == "a16")
            os << Hex(_op.d16);
        else if (it == "(d16)")
            os << "(" << Hex(_op.d16) << ")";
        else if (it == "(d8)")
            os << "(" << Hex(_op.d8) << ")";
        else if (it == "d8" || it == "r8")
            os << Hex(_op.d8);
        else
            os << it;
        lastPos = str.find_first_not_of(delimiters, pos);
        pos = str.find_first_of(delimiters, lastPos);
    }
    TRACE(os.str());
}

#define OPCODE(op, cycles, bytes, name, func) \
    case op: { \
        _op.opcode = op; \
        IF_LOG(Trace) \
            op_set(name); \
        func; \
        _add_icycles(cycles); \
        break; \
    }

Cycles
Z80Cpu::dispatch(void)
{
    _op.pc = _rPC;
    _op.opcode = pc_read();

    if (_state == DeviceState::Halted) {
        _rPC--;
        _op.opcode = 0x00; /* NOP */
    }

    byte_t &vrH = _rH;
    byte_t &vrL = _rL;
    uint16_t &vrHL = _rHL;
    addr_t vAddr = _rHL;

#if 0
    if (_op.opcode == 0xDD) {
        _op.opcode = pc_read();
        vrH = _rIXh;
        vrL = _rIXl;
        vrHL = _rIX;
        vAddr = _rIX;
        _add_icycles(4); /* XXX: Wrong! */
    } else if (_op.opcode == 0xFD) {
        _op.opcode = pc_read();
        vrH = _rIYh;
        vrL = _rIYl;
        vrHL = _rIY;
        vAddr = _rIY; /* XXX: Wrong! */
        _add_icycles(4);
    }
#endif

    _rR = ((_rR + 1) % 128) | (_rR & 0x80);

    /* XXX: Treat the DD and FD prefixes as flags to HL */
    /* XXX: Update the R register */
    switch (_op.opcode) {
        OPCODE(0x00,  4, 1, "NOP", );
        OPCODE(0x01, 10, 3, "LD BC,d16", _ld16(_rBC, _d16()));
        OPCODE(0x02,  7, 1, "LD (BC),A", _ldmem(_rBC, _rA));
        OPCODE(0x03,  6, 1, "INC BC", _inc16(_rBC));
        OPCODE(0x04,  4, 1, "INC B", _inc(_rB));
        OPCODE(0x05,  4, 1, "DEC B", _dec(_rB));
        OPCODE(0x06,  7, 2, "LD B,d8", _ld(_rB, _d8()));
        OPCODE(0x07,  4, 1, "RLCA", _rlca());
        OPCODE(0x08,  4, 1, "EX AF,AF'", _ex(_rAF, _rsAF));
        OPCODE(0x09, 11, 1, "ADD HL,BC", _add16(vrHL, _rBC));
        OPCODE(0x0A,  7, 1, "LD A,(BC)", _ld(_rA, bus_read(_rBC)));
        OPCODE(0x0B,  6, 1, "DEC BC", _dec16(_rBC));
        OPCODE(0x0C,  4, 1, "INC C", _inc(_rC));
        OPCODE(0x0D,  4, 1, "DEC C", _dec(_rC));
        OPCODE(0x0E,  7, 2, "LD C,d8", _ld(_rC, _d8()));
        OPCODE(0x0F,  4, 1, "RRCA", _rrca());
        OPCODE(0x10,  8, 1, "DJNZ r8", _djnz(_d8()));
        OPCODE(0x11, 10, 3, "LD DE,d16", _ld16(_rDE, _d16()));
        OPCODE(0x12,  7, 1, "LD (DE),A", _ldmem(_rDE, _rA));
        OPCODE(0x13,  6, 1, "INC DE", _inc16(_rDE));
        OPCODE(0x14,  4, 1, "INC D", _inc(_rD));
        OPCODE(0x15,  4, 1, "DEC D", _dec(_rD));
        OPCODE(0x16,  7, 2, "LD D,d8", _ld(_rD, _d8()));
        OPCODE(0x17,  4, 1, "RLA", _rla());
        OPCODE(0x18, 12, 2, "JR r8", _jr(true, _r8()));
        OPCODE(0x19, 11, 1, "ADD HL,DE", _add16(vrHL, _rDE));
        OPCODE(0x1A,  7, 1, "LD A,(DE)", _ld(_rA, bus_read(_rDE)));
        OPCODE(0x1B,  6, 1, "DEC DE", _dec16(_rDE));
        OPCODE(0x1C,  4, 1, "INC E", _inc(_rE));
        OPCODE(0x1D,  4, 1, "DEC E", _dec(_rE));
        OPCODE(0x1E,  7, 2, "LD E,d8", _ld(_rE, _d8()));
        OPCODE(0x1F,  4, 1, "RRA", _rra());
        OPCODE(0x20,  7, 2, "JR NZ,r8", _jr(!_flags.Z, _r8()));
        OPCODE(0x21, 10, 3, "LD HL,d16", _ld16(vrHL, _d16()));
        OPCODE(0x22, 16, 3, "LD (d16), HL", _ld16i(_d16(), vrHL));
        OPCODE(0x23,  7, 1, "INC HL", _inc16(vrHL));
        OPCODE(0x24,  4, 1, "INC H", _inc(vrH));
        OPCODE(0x25,  4, 1, "DEC H", _dec(vrH));
        OPCODE(0x26,  7, 2, "LD H,d8", _ld(vrH, _d8()));
        OPCODE(0x27,  4, 1, "DAA", _daa());
        OPCODE(0x28,  7, 2, "JR Z,r8", _jr(_flags.Z, _r8()));
        OPCODE(0x29, 11, 1, "ADD HL,HL", _add16(vrHL, vrHL));
        OPCODE(0x2A, 16, 3, "LD HL, (d16)", _ld16(_rHL, _i16()));
        OPCODE(0x2B,  6, 1, "DEC HL", _dec16(vrHL));
        OPCODE(0x2C,  4, 1, "INC L", _inc(vrL));
        OPCODE(0x2D,  4, 1, "DEC L", _dec(vrL));
        OPCODE(0x2E,  7, 2, "LD L,d8", _ld(vrL, _d8()));
        OPCODE(0x2F,  4, 2, "CPL", _cpl());
        OPCODE(0x30,  7, 2, "JR NC,r8", _jr(!_flags.C, _r8()));
        OPCODE(0x31, 10, 3, "LD SP,d16", _ld16(_rSP, _d16()));
        OPCODE(0x32, 13, 3, "LD (d16), A", _ldmem(_d16(), _rA));
        OPCODE(0x33,  6, 1, "INC SP", _inc16(_rSP));
        OPCODE(0x34, 11, 1, "INC (HL)", _inci(vrHL));
        OPCODE(0x35, 11, 1, "DEC (HL)", _deci(vrHL));
        OPCODE(0x36, 10, 2, "LD (HL),d8", _ldmem(_rHL, _d8()));
        OPCODE(0x37,  4, 1, "SCF", _scf());
        OPCODE(0x38,  7 ,2, "JR C,r8", _jr(_flags.C, _r8()));
        OPCODE(0x39, 11, 1, "ADD HL,SP", _add16(vrHL, _rSP));
        OPCODE(0x3A, 13, 3, "LD A,(d16)", _ld(_rA, _i8(_d16())));
        OPCODE(0x3B,  6, 1, "DEC SP", _dec16(_rSP));
        OPCODE(0x3C,  4, 1, "INC A", _inc(_rA));
        OPCODE(0x3D,  4, 1, "DEC A", _dec(_rA));
        OPCODE(0x3E,  7, 2, "LD A,d8", _ld(_rA, _d8()));
        OPCODE(0x3F,  4, 1, "CCF", _ccf());
        OPCODE(0x40,  4, 1, "LD B,B", _ld(_rB, _rB));
        OPCODE(0x41,  4, 1, "LD B,C", _ld(_rB, _rC));
        OPCODE(0x42,  4, 1, "LD B,D", _ld(_rB, _rD));
        OPCODE(0x43,  4, 1, "LD B,E", _ld(_rB, _rE));
        OPCODE(0x44,  4, 1, "LD B,H", _ld(_rB, vrH));
        OPCODE(0x45,  4, 1, "LD B,L", _ld(_rB, vrL));
        OPCODE(0x46,  7, 1, "LD B,(HL)", _ld(_rB, _i8(vAddr)));
        OPCODE(0x47,  4, 1, "LD B,A", _ld(_rB, _rA));
        OPCODE(0x48,  4, 1, "LD C,B", _ld(_rC, _rB));
        OPCODE(0x49,  4, 1, "LD C,C", _ld(_rC, _rC));
        OPCODE(0x4A,  4, 1, "LD C,D", _ld(_rC, _rD));
        OPCODE(0x4B,  4, 1, "LD C,E", _ld(_rC, _rE));
        OPCODE(0x4C,  4, 1, "LD C,H", _ld(_rC, vrH));
        OPCODE(0x4D,  4, 1, "LD C,L", _ld(_rC, vrL));
        OPCODE(0x4E,  7, 1, "LD C,(HL)", _ld(_rC, _i8(vAddr)));
        OPCODE(0x4F,  4, 1, "LD C,A", _ld(_rC, _rA));
        OPCODE(0x50,  4, 1, "LD D,B", _ld(_rD, _rB));
        OPCODE(0x51,  4, 1, "LD D,C", _ld(_rD, _rC));
        OPCODE(0x52,  4, 1, "LD D,D", _ld(_rD, _rD));
        OPCODE(0x53,  4, 1, "LD D,E", _ld(_rD, _rE));
        OPCODE(0x54,  4, 1, "LD D,H", _ld(_rD, vrH));
        OPCODE(0x55,  4, 1, "LD D,L", _ld(_rD, vrL));
        OPCODE(0x56,  7, 1, "LD D,(HL)", _ld(_rD, _i8(vAddr)));
        OPCODE(0x57,  4, 1, "LD D,A", _ld(_rD, _rA));
        OPCODE(0x58,  4, 1, "LD E,B", _ld(_rE, _rB));
        OPCODE(0x59,  4, 1, "LD E,C", _ld(_rE, _rC));
        OPCODE(0x5A,  4, 1, "LD E,D", _ld(_rE, _rD));
        OPCODE(0x5B,  4, 1, "LD E,E", _ld(_rE, _rE));
        OPCODE(0x5C,  4, 1, "LD E,H", _ld(_rE, vrH));
        OPCODE(0x5D,  4, 1, "LD E,L", _ld(_rE, vrL));
        OPCODE(0x5E,  7, 1, "LD E,(HL)", _ld(_rE, _i8(vAddr)));
        OPCODE(0x5F,  4, 1, "LD E,A", _ld(_rE, _rA));
        OPCODE(0x60,  4, 1, "LD H,B", _ld(vrH, _rB));
        OPCODE(0x61,  4, 1, "LD H,C", _ld(vrH, _rC));
        OPCODE(0x62,  4, 1, "LD H,D", _ld(vrH, _rD));
        OPCODE(0x63,  4, 1, "LD H,E", _ld(vrH, _rE));
        OPCODE(0x64,  4, 1, "LD H,H", _ld(vrH, vrH));
        OPCODE(0x65,  4, 1, "LD H,L", _ld(vrH, vrL));
        OPCODE(0x66,  7, 1, "LD H,(HL)", _ld(vrH, _i8(vAddr)));
        OPCODE(0x67,  4, 1, "LD H,A", _ld(vrH, _rA));
        OPCODE(0x68,  4, 1, "LD L,B", _ld(vrL, _rB));
        OPCODE(0x69,  4, 1, "LD L,C", _ld(vrL, _rC));
        OPCODE(0x6A,  4, 1, "LD L,D", _ld(vrL, _rD));
        OPCODE(0x6B,  4, 1, "LD L,E", _ld(vrL, _rE));
        OPCODE(0x6C,  4, 1, "LD L,H", _ld(vrL, vrH));
        OPCODE(0x6D,  4, 1, "LD L,L", _ld(vrL, vrL));
        OPCODE(0x6E,  7, 1, "LD L,(HL)", _ld(vrL, _i8(vAddr)));
        OPCODE(0x6F,  4, 1, "LD L,A", _ld(vrL, _rA));
        OPCODE(0x70,  7, 1, "LD (HL),B", _ldmem(vrHL, _rB));
        OPCODE(0x71,  7, 1, "LD (HL),C", _ldmem(vrHL, _rC));
        OPCODE(0x72,  7, 1, "LD (HL),D", _ldmem(vrHL, _rD));
        OPCODE(0x73,  7, 1, "LD (HL),E", _ldmem(vrHL, _rE));
        OPCODE(0x74,  7, 1, "LD (HL),H", _ldmem(vrHL, vrH));
        OPCODE(0x75,  7, 1, "LD (HL),L", _ldmem(vrHL, vrL));
        OPCODE(0x76,  4, 1, "HALT", _halt());
        OPCODE(0x77,  7, 1, "LD (HL),A", _ldmem(vrHL, _rA));
        OPCODE(0x78,  4, 1, "LD A,B", _ld(_rA, _rB));
        OPCODE(0x79,  4, 1, "LD A,C", _ld(_rA, _rC));
        OPCODE(0x7A,  4, 1, "LD A,D", _ld(_rA, _rD));
        OPCODE(0x7B,  4, 1, "LD A,E", _ld(_rA, _rE));
        OPCODE(0x7C,  4, 1, "LD A,H", _ld(_rA, vrH));
        OPCODE(0x7D,  4, 1, "LD A,L", _ld(_rA, vrL));
        OPCODE(0x7E,  7, 1, "LD A,(HL)", _ld(_rA, _i8(vAddr)));
        OPCODE(0x7F,  4, 1, "LD A,A", _ld(_rA, _rA));
        OPCODE(0x80,  4, 1, "ADD A,B", _add(_rA, _rB));
        OPCODE(0x81,  4, 1, "ADD A,C", _add(_rA, _rC));
        OPCODE(0x82,  4, 1, "ADD A,D", _add(_rA, _rD));
        OPCODE(0x83,  4, 1, "ADD A,E", _add(_rA, _rE));
        OPCODE(0x84,  4, 1, "ADD A,H", _add(_rA, vrH));
        OPCODE(0x85,  4, 1, "ADD A,L", _add(_rA, vrL));
        OPCODE(0x86,  7, 1, "ADD A,(HL)", _add(_rA, _i8(vAddr)));
        OPCODE(0x87,  4, 1, "ADD A,A", _add(_rA, _rA));
        OPCODE(0x88,  4, 1, "ADC A,B", _adc(_rA, _rB));
        OPCODE(0x89,  4, 1, "ADC A,C", _adc(_rA, _rC));
        OPCODE(0x8A,  4, 1, "ADC A,D", _adc(_rA, _rD));
        OPCODE(0x8B,  4, 1, "ADC A,E", _adc(_rA, _rE));
        OPCODE(0x8C,  4, 1, "ADC A,H", _adc(_rA, vrH));
        OPCODE(0x8D,  4, 1, "ADC A,L", _adc(_rA, vrL));
        OPCODE(0x8E,  7, 1, "ADC A,(HL)", _adc(_rA, _i8(vAddr)));
        OPCODE(0x8F,  4, 1, "ADC A,A", _adc(_rA, _rA));
        OPCODE(0x90,  4, 1, "SUB B", _sub(_rA, _rB));
        OPCODE(0x91,  4, 1, "SUB C", _sub(_rA, _rC));
        OPCODE(0x92,  4, 1, "SUB D", _sub(_rA, _rD));
        OPCODE(0x93,  4, 1, "SUB E", _sub(_rA, _rE));
        OPCODE(0x94,  4, 1, "SUB H", _sub(_rA, vrH));
        OPCODE(0x95,  4, 1, "SUB L", _sub(_rA, vrL));
        OPCODE(0x96,  7, 1, "SUB (HL)", _sub(_rA, _i8(vAddr)));
        OPCODE(0x97,  4, 1, "SUB A", _sub(_rA, _rA));
        OPCODE(0x98,  4, 1, "SBC A,B", _sbc(_rA, _rB));
        OPCODE(0x99,  4, 1, "SBC A,C", _sbc(_rA, _rC));
        OPCODE(0x9A,  4, 1, "SBC A,D", _sbc(_rA, _rD));
        OPCODE(0x9B,  4, 1, "SBC A,E", _sbc(_rA, _rE));
        OPCODE(0x9C,  4, 1, "SBC A,H", _sbc(_rA, vrH));
        OPCODE(0x9D,  4, 1, "SBC A,L", _sbc(_rA, vrL));
        OPCODE(0x9E,  7, 1, "SBC A,(HL)", _sbc(_rA, _i8(vAddr)));
        OPCODE(0x9F,  4, 1, "SBC A,A", _sbc(_rA, _rA));
        OPCODE(0xA0,  4, 1, "AND B", _and(_rA, _rB));
        OPCODE(0xA1,  4, 1, "AND C", _and(_rA, _rC));
        OPCODE(0xA2,  4, 1, "AND D", _and(_rA, _rD));
        OPCODE(0xA3,  4, 1, "AND E", _and(_rA, _rE));
        OPCODE(0xA4,  4, 1, "AND H", _and(_rA, vrH));
        OPCODE(0xA5,  4, 1, "AND L", _and(_rA, vrL));
        OPCODE(0xA6,  4, 1, "AND (HL)", _and(_rA, _i8(vAddr)));
        OPCODE(0xA7,  4, 1, "AND A", _and(_rA, _rA));
        OPCODE(0xA8,  4, 1, "XOR B", _xor(_rA, _rB));
        OPCODE(0xA9,  4, 1, "XOR C", _xor(_rA, _rC));
        OPCODE(0xAA,  4, 1, "XOR D", _xor(_rA, _rD));
        OPCODE(0xAB,  4, 1, "XOR E", _xor(_rA, _rE));
        OPCODE(0xAC,  4, 1, "XOR H", _xor(_rA, vrH));
        OPCODE(0xAD,  4, 1, "XOR L", _xor(_rA, vrL));
        OPCODE(0xAE,  7, 2, "XOR (HL)", _xor(_rA, _i8(vAddr)));
        OPCODE(0xAF,  4, 1, "XOR A", _xor(_rA, _rA));
        OPCODE(0xB0,  4, 1, "OR B", _or(_rA, _rB));
        OPCODE(0xB1,  4, 1, "OR C", _or(_rA, _rC));
        OPCODE(0xB2,  4, 1, "OR D", _or(_rA, _rD));
        OPCODE(0xB3,  4, 1, "OR E", _or(_rA, _rE));
        OPCODE(0xB4,  4, 1, "OR H", _or(_rA, vrH));
        OPCODE(0xB5,  4, 1, "OR L", _or(_rA, vrL));
        OPCODE(0xB6,  7, 1, "OR (HL)", _or(_rA, _i8(vAddr)));
        OPCODE(0xB7,  4, 1, "OR A", _or(_rA, _rA));
        OPCODE(0xB8,  4, 1, "CP B", _cp(_rA, _rB));
        OPCODE(0xB9,  4, 1, "CP C", _cp(_rA, _rC));
        OPCODE(0xBA,  4, 1, "CP D", _cp(_rA, _rD));
        OPCODE(0xBB,  4, 1, "CP E", _cp(_rA, _rE));
        OPCODE(0xBC,  4, 1, "CP H", _cp(_rA, vrH));
        OPCODE(0xBD,  4, 1, "CP L", _cp(_rA, vrL));
        OPCODE(0xBE,  7, 1, "CP (HL)", _cp(_rA, _i8(vAddr)));
        OPCODE(0xBF,  4, 1, "CP A", _cp(_rA, _rA));
        OPCODE(0xC0,  5, 1, "RET NZ", _ret(!_flags.Z));
        OPCODE(0xC1, 10, 1, "POP BC", _pop(_rB, _rC));
        OPCODE(0xC2, 10, 0, "JP NZ", _jp(!_flags.Z, _d16()));
        OPCODE(0xC3, 10, 0, "JP a16", _jp(true, _d16()));
        OPCODE(0xC4, 10, 3, "CALL NZ,a16", _call(!_flags.Z, _d16()));
        OPCODE(0xC5, 11, 1, "PUSH BC", _push(_rB, _rC));
        OPCODE(0xC6,  7, 2, "ADD A,d8", _add(_rA, _d8()));
        OPCODE(0xC7, 11, 1, "RST 00H", _rst(0x00));
        OPCODE(0xC8,  5, 1, "RET Z", _ret(_flags.Z));
        OPCODE(0xC9,  6, 1, "RET", _ret(true));
        OPCODE(0xCA, 10, 3, "JP Z,a16", _jp(_flags.Z, _d16()));
        OPCODE(0xCB,  0, 1, "PREFIX CB", dispatch_cb());
        OPCODE(0xCC, 10, 3, "CALL Z,a16", _call(_flags.Z, _d16()));
        OPCODE(0xCD, 10, 3, "CALL a16", _call(true, _d16()));
        OPCODE(0xCE,  7, 2, "ADC A,d8", _adc(_rA, _d8()));
        OPCODE(0xCF, 11, 1, "RST 08H", _rst(0x08));
        OPCODE(0xD0,  5, 1, "RET NC", _ret(!_flags.C));
        OPCODE(0xD1, 10, 1, "POP DE", _pop(_rD, _rE));
        OPCODE(0xD2, 10, 0, "JP NC", _jp(!_flags.C, _d16()));
        OPCODE(0xD3, 11, 2, "OUT d8, A", _out(_d8(), _rA));
        OPCODE(0xD4, 10, 3, "CALL NC,a16", _call(!_flags.C, _d16()));
        OPCODE(0xD5, 11, 1, "PUSH DE", _push(_rD, _rE));
        OPCODE(0xD6,  7, 2, "SUB A,d8", _sub(_rA, _d8()));
        OPCODE(0xD7, 11, 1, "RST 10H", _rst(0x10));
        OPCODE(0xD8,  5, 1, "RET C", _ret(_flags.C));
        OPCODE(0xD9,  4, 1, "EXX BC DE HL", _exx());
        OPCODE(0xDA, 10, 3, "JP C,a16", _jp(_flags.C, _d16()));
        OPCODE(0xDB, 11, 2, "IN A, d8", _in(_rA, _d8()));
        OPCODE(0xDC, 10, 3, "CALL C, a16", _call(_flags.C, _d16()));
        OPCODE(0xDD,  0, 1, "PREFIX IX", dispatch_dd());
        OPCODE(0xDE,  7, 2, "SBC A,d8", _sbc(_rA, _d8()));
        OPCODE(0xDF, 11, 1, "RST 18H", _rst(0x18));
        OPCODE(0xE0,  5, 1, "RET PO", _ret(!_flags.V));
        OPCODE(0xE1, 10, 1, "POP HL", _pop(vrH, vrL));
        OPCODE(0xE2, 10, 3, "JP PO,a16", _jp(!_flags.V, _d16()));
        OPCODE(0xE3, 19, 1, "EX (SP),HL", _exi(_rSP, vrH, vrL));
        OPCODE(0xE4, 10, 3, "CALL PO,a16", _call(!_flags.V, _d16()));
        OPCODE(0xE5, 16, 1, "PUSH HL", _push(vrH, vrL));
        OPCODE(0xE6,  7, 2, "AND d8", _and(_rA, _d8()));
        OPCODE(0xE7, 16, 1, "RST 20H", _rst(0x20));
        OPCODE(0xE8,  5, 1, "RET PE", _ret(_flags.V));
        OPCODE(0xE9,  4, 1, "JP HL", _jp(true, vrHL));
        OPCODE(0xEA,  4, 3, "JP PE,a16", _jp(_flags.V, _d16()));
        OPCODE(0xEB,  4, 1, "EX DE, HL", _ex(_rDE, _rHL));
        OPCODE(0xEC, 10, 3, "CALL PE,a16", _call(_flags.V, _d16()));
        OPCODE(0xED,  0, 0, "EXTD", dispatch_ed());
        OPCODE(0xEE,  7, 1, "XOR d8", _xor(_rA, _d8()));
        OPCODE(0xEF, 11, 1, "RST 28H", _rst(0x28));
        OPCODE(0xF0,  5, 1, "RET P", _ret(!_flags.S));
        OPCODE(0xF1, 10, 1, "POP AF", _pop(_rA, _rF));
        OPCODE(0xF2, 10, 3, "JP P,a16", _jp(!_flags.S, _d16()));
        OPCODE(0xF3,  4, 1, "DI", _di());
        OPCODE(0xF4, 10, 3, "CALL P,a16", _call(!_flags.S, _d16()));
        OPCODE(0xF5, 11, 1, "PUSH AF", _push(_rA, _rF));
        OPCODE(0xF6,  7, 2, "OR d8", _or(_rA, _d8()));
        OPCODE(0xF7, 11, 1, "RST 30H", _rst(0x30));
        OPCODE(0xF8,  5, 1, "RET M", _ret(_flags.S));
        OPCODE(0xF9,  6, 1, "LD SP,HL", _ld16(_rSP, vrHL));
        OPCODE(0xFA,  4, 3, "JP M,a16", _jp(_flags.S, _d16()));
        OPCODE(0xFB,  4, 1, "EI", _ei());
        OPCODE(0xFC, 10, 3, "CALL M,a16", _call(_flags.S, _d16()));
        OPCODE(0xFD,  0, 0, "PREFIX IY", dispatch_fd());
        OPCODE(0xFE,  7, 2, "CP d8", _cp(_rA, _d8()));
        OPCODE(0xFF, 11, 1, "RST 38H", _rst(0x38));
    default:
        _state = DeviceState::Fault;
        throw CpuOpcodeFault("z80", _op.opcode, _op.pc);
    }

    IF_LOG(Trace)
        op_log();
    _op.reset();

    return _icycles;
}

void
Z80Cpu::dispatch_cb(void)
{
    byte_t op = pc_read();
    int bit = (op & 0x38) >> 3;
    Reg reg = Reg(op & 0x07);
    byte_t value = fetch(reg);
    byte_t dest = 0;
    if ((op & 0xC0) == 0x00) {
        if ((op & 0xF8) == 0x00) {
            _rlc(dest, value);
        } else if ((op & 0xF8) == 0x08) {
            _rrc(dest, value);
        } else if ((op & 0xF8) == 0x10) {
            _rl(dest, value);
        } else if ((op & 0xF8) == 0x18) {
            _rr(dest, value);
        } else if ((op & 0xF8) == 0x20) {
            _sla(dest, value);
        } else if ((op & 0xF8) == 0x28) {
            _sra(dest, value);
        } else if ((op & 0xF8) == 0x30) {
            _sll(dest, value);
        } else if ((op & 0xF8) == 0x38) {
            _srl(dest, value);
        }
        store(reg, dest);
        (reg == Reg::HL) ? _add_icycles(15) : _add_icycles(8);
    } else if ((op & 0xC0) == 0x40) {
        _bit_test(value, bit);
        (reg == Reg::HL) ? _add_icycles(12) : _add_icycles(8);
    } else if ((op & 0xC0) == 0x80) {
        _bit_reset(dest, value, bit);
        store(reg, dest);
        (reg == Reg::HL) ? _add_icycles(15) : _add_icycles(8);
    } else if ((op & 0xC0) == 0xC0) {
        _bit_set(dest, value, bit);
        store(reg, dest);
        (reg == Reg::HL) ? _add_icycles(15) : _add_icycles(8);
    }
}

void
Z80Cpu::dispatch_dd(void)
{
    byte_t op = pc_read();

    switch (op) {
        OPCODE(0x09, 15, 2, "ADD IX, BC", _add16(_rIX, _rBC));
        OPCODE(0x19, 15, 2, "ADD IX, DE", _add16(_rIX, _rDE));
        OPCODE(0x21, 14, 4, "LD IX, d16", _ld16(_rIX, _d16()));
        OPCODE(0x22, 20, 4, "LD (d16), IX", _ld16i(_d16(), _rIX));
        OPCODE(0x23, 10, 3, "INC IX", _inc16(_rIX));
        OPCODE(0x24,  8, 2, "INC IXh", _inc(_rIXh));
        OPCODE(0x25,  8, 2, "DEC IXh", _dec(_rIXh));
        OPCODE(0x26, 11, 3, "LD IXh, d8", _ld(_rIXh, _d8()));
        OPCODE(0x29, 15, 2, "ADD IX, IX", _add16(_rIX, _rIX));
        OPCODE(0x2A, 20, 4, "LD IX, (d16)", _ld16(_rIX, _i16()));
        OPCODE(0x2B, 10, 3, "DEC IX", _dec16(_rIX));
        OPCODE(0x2C,  8, 2, "INC IXl", _inc(_rIXl));
        OPCODE(0x2D,  8, 2, "DEC IXl", _dec(_rIXl));
        OPCODE(0x2E, 11, 3, "LD IXl, d8", _ld(_rIXl, _d8()));
        OPCODE(0x34, 23, 2, "INC (IX+d)", _inci(_dIX()));
        OPCODE(0x35, 23, 2, "DEC (IX+d)", _deci(_dIX()));
        OPCODE(0x36, 19, 4, "LD (IX+d), n", _ldmem(_dIX(), _d8()));
        OPCODE(0x39, 15, 2, "ADD IX, SP", _add16(_rIX, _rSP));
        OPCODE(0x44,  8, 2, "LD B, IXh", _ld(_rB, _rIXh));
        OPCODE(0x45,  8, 2, "LD B, IXl", _ld(_rB, _rIXl));
        OPCODE(0x46, 19, 3, "LD B, (IX+d)", _ld(_rB, _iIX()));
        OPCODE(0x4C,  8, 2, "LD C, IXh", _ld(_rC, _rIXh));
        OPCODE(0x4D,  8, 2, "LD C, IXl", _ld(_rC, _rIXl));
        OPCODE(0x4E, 19, 3, "LD C, (IX+d)", _ld(_rC, _iIX()));
        OPCODE(0x54,  8, 2, "LD D, IXh", _ld(_rD, _rIXh));
        OPCODE(0x55,  8, 2, "LD D, IXl", _ld(_rD, _rIXl));
        OPCODE(0x56, 19, 3, "LD D, (IX+d)", _ld(_rD, _iIX()));
        OPCODE(0x5C,  8, 2, "LD E, IXh", _ld(_rE, _rIXh));
        OPCODE(0x5D,  8, 2, "LD E, IXl", _ld(_rE, _rIXl));
        OPCODE(0x5E, 19, 3, "LD E, (IX+d)", _ld(_rE, _iIX()));
        OPCODE(0x60,  8, 2, "LD IXh, B", _ld(_rIXh, _rB));
        OPCODE(0x61,  8, 2, "LD IXh, C", _ld(_rIXh, _rC));
        OPCODE(0x62,  8, 2, "LD IXh, D", _ld(_rIXh, _rD));
        OPCODE(0x63,  8, 2, "LD IXh, E", _ld(_rIXh, _rE));
        OPCODE(0x64,  8, 2, "LD IXh, IXh", _ld(_rIXh, _rIXh));
        OPCODE(0x65,  8, 2, "LD IXh, IXl", _ld(_rIXh, _rIXl));
        OPCODE(0x66, 19, 3, "LD H, (IX+d)", _ld(_rH, _iIX()));
        OPCODE(0x67,  8, 2, "LD IXh, A", _ld(_rIXh, _rA));
        OPCODE(0x68,  8, 2, "LD IXl, B", _ld(_rIXl, _rB));
        OPCODE(0x69,  8, 2, "LD IXl, C", _ld(_rIXl, _rC));
        OPCODE(0x6A,  8, 2, "LD IXl, D", _ld(_rIXl, _rD));
        OPCODE(0x6B,  8, 2, "LD IXl, E", _ld(_rIXl, _rE));
        OPCODE(0x6C,  8, 2, "LD IXl, IXh", _ld(_rIXl, _rIXh));
        OPCODE(0x6D,  8, 2, "LD IXl, IXl", _ld(_rIXl, _rIXl));
        OPCODE(0x6E, 19, 3, "LD L, (IX+d)", _ld(_rL, _iIX()));
        OPCODE(0x6F,  8, 2, "LD IXl, A", _ld(_rIXl, _rA));
        OPCODE(0x70, 19, 3, "LD (IX+d), B", _ldmem(_dIX(), _rB));
        OPCODE(0x71, 19, 3, "LD (IX+d), C", _ldmem(_dIX(), _rC));
        OPCODE(0x72, 19, 3, "LD (IX+d), D", _ldmem(_dIX(), _rD));
        OPCODE(0x73, 19, 3, "LD (IX+d), E", _ldmem(_dIX(), _rE));
        OPCODE(0x74, 19, 3, "LD (IX+d), H", _ldmem(_dIX(), _rH));
        OPCODE(0x75, 19, 3, "LD (IX+d), L", _ldmem(_dIX(), _rL));
        OPCODE(0x77, 19, 3, "LD (IX+d), A", _ldmem(_dIX(), _rA));
        OPCODE(0x7C,  8, 2, "LD A, IXh", _ld(_rA, _rIXh));
        OPCODE(0x7D,  8, 2, "LD A, IXl", _ld(_rA, _rIXl));
        OPCODE(0x7E, 19, 3, "LD A, (IX+d)", _ld(_rA, _iIX()));
        OPCODE(0x84,  8, 2, "ADD A, IXh", _add(_rA, _rIXh));
        OPCODE(0x85,  8, 2, "ADD A, IXl", _add(_rA, _rIXl));
        OPCODE(0x86,  8, 3, "ADD A, (IX+d)", _add(_rA, _iIX()));
        OPCODE(0x8C,  8, 2, "ADC A, IXh", _adc(_rA, _rIXh));
        OPCODE(0x8D,  8, 2, "ADC A, IXl", _adc(_rA, _rIXl));
        OPCODE(0x8E,  8, 3, "ADC A, (IX+d)", _adc(_rA, _iIX()));
        OPCODE(0x94,  8, 2, "SUB A, IXh", _sub(_rA, _rIXh));
        OPCODE(0x95,  8, 2, "SUB A, IXl", _sub(_rA, _rIXl));
        OPCODE(0x96,  8, 3, "SUB A, (IX+d)", _sub(_rA, _iIX()));
        OPCODE(0x9C,  8, 2, "SBC A, IXh", _sbc(_rA, _rIXh));
        OPCODE(0x9D,  8, 2, "SBC A, IXl", _sbc(_rA, _rIXl));
        OPCODE(0x9E,  8, 3, "SBC A, (IX+d)", _sbc(_rA, _iIX()));
        OPCODE(0xA4,  8, 2, "AND A, IXh", _and(_rA, _rIXh));
        OPCODE(0xA5,  8, 2, "AND A, IXl", _and(_rA, _rIXl));
        OPCODE(0xA6,  8, 3, "AND A, (IX+d)", _and(_rA, _iIX()));
        OPCODE(0xAC,  8, 2, "XOR A, IXh", _xor(_rA, _rIXh));
        OPCODE(0xAD,  8, 2, "XOR A, IXl", _xor(_rA, _rIXl));
        OPCODE(0xAE,  8, 3, "XOR A, (IX+d)", _xor(_rA, _iIX()));
        OPCODE(0xB4,  8, 2, "OR A, IXh", _or(_rA, _rIXh));
        OPCODE(0xB5,  8, 2, "OR A, IXl", _or(_rA, _rIXl));
        OPCODE(0xB6,  8, 3, "OR A, (IX+d)", _or(_rA, _iIX()));
        OPCODE(0xBC,  8, 2, "CP A, IXh", _cp(_rA, _rIXh));
        OPCODE(0xBD,  8, 2, "CP A, IXl", _cp(_rA, _rIXl));
        OPCODE(0xBE,  8, 3, "CP A, (IX+d)", _cp(_rA, _iIX()));
        OPCODE(0xCB,  0, 2, "IX BITS", dispatch_dd_cb());
        OPCODE(0xE1, 14, 2, "POP IX", _pop(_rIXh, _rIXl));
        OPCODE(0xE3, 23, 2, "EX (SP), IX", _exi(_rSP, _rIXh, _rIXl));
        OPCODE(0xE5, 15, 2, "PUSH IX", _push(_rIXh, _rIXl));
        OPCODE(0xE9,  8, 2, "JP IX", _ld16(_rPC, _rIX));
        OPCODE(0xF9, 10, 2, "LD SP, IX", _ld16(_rSP, _rIX));
    default:
        break;
#if 0
        {
            std::stringstream ss;
            ss << "Unknown opcode:" << Hex(op);
            ERROR(ss.str());
        }
        _state = DeviceState::Fault;
        throw CpuFault();
#endif
    }
}

void
Z80Cpu::dispatch_dd_cb(void)
{
    addr_t addr = _dIX();
    byte_t op = pc_read();
    int bit = (op & 0x38) >> 3;
    byte_t value = _i8(addr);
    byte_t dest = 0;
    if ((op & 0xC0) == 0x00) {
        if ((op & 0xF8) == 0x00) {
            _rlc(dest, value);
        } else if ((op & 0xF8) == 0x08) {
            _rrc(dest, value);
        } else if ((op & 0xF8) == 0x10) {
            _rl(dest, value);
        } else if ((op & 0xF8) == 0x18) {
            _rr(dest, value);
        } else if ((op & 0xF8) == 0x20) {
            _sla(dest, value);
        } else if ((op & 0xF8) == 0x28) {
            _sra(dest, value);
        } else if ((op & 0xF8) == 0x30) {
            _sll(dest, value);
        } else if ((op & 0xF8) == 0x38) {
            _srl(dest, value);
        }
        if (Reg::HL == Reg(op & 0x7))
            bus_write(addr, dest);
        else
            store(Reg(op & 0x7), dest);
        _add_icycles(23);
    } else if ((op & 0xC0) == 0x40) {
        _bit_test(value, bit);
        _flags.Y = bit_isset(addr, 13);
        _flags.X = bit_isset(addr, 11);
        _add_icycles(20);
    } else if ((op & 0xC0) == 0x80) {
        _bit_reset(dest, value, bit);
        _ldmem(addr, dest);
        _add_icycles(23);
    } else if ((op & 0xC0) == 0xC0) {
        _bit_set(dest, value, bit);
        _ldmem(addr, dest);
        _add_icycles(23);
    }
}

void
Z80Cpu::dispatch_ed(void)
{
    const addr_t pc = _rPC;

    byte_t op = pc_read();
    switch (op) {
        OPCODE(0x00,  5, 2, "NOP", );
        OPCODE(0x40, 12, 2, "IN B, (C)", _in(_rB, _rC));
        OPCODE(0x41, 12, 2, "OUT (C), B", _out(_rC, _rB));
        OPCODE(0x42, 15, 2, "SBC HL, BC", _sbc16(_rHL, _rBC));
        OPCODE(0x43, 20, 4, "LD (d16), BC", _ld16i(_d16(), _rBC));
        OPCODE(0x44,  8, 2, "NEG", _neg(_rA));
        OPCODE(0x45, 14, 2, "RET N", _retn());
        OPCODE(0x48, 12, 2, "IN C, (C)", _in(_rC, _rC));
        OPCODE(0x49, 12, 2, "OUT (C), C", _out(_rC, _rC));
        OPCODE(0x4A, 15, 2, "ADC HL, BC", _adc16(_rHL, _rBC));
        OPCODE(0x4B, 20, 2, "LD BC, (d16)", _ld16(_rBC, _i16()));
        OPCODE(0x4C,  8, 2, "NEG", _neg(_rA));
        OPCODE(0x4D, 14, 2, "RET I", _reti());
        OPCODE(0x4F,  9, 2, "LD R, A", _ld(_rR, _rA));
        OPCODE(0x50, 12, 2, "IN D, (C)", _in(_rD, _rC));
        OPCODE(0x51, 12, 2, "OUT (C), D", _out(_rC, _rD));
        OPCODE(0x52, 15, 2, "SBC HL, DE", _sbc16(_rHL, _rDE));
        OPCODE(0x53, 20, 4, "LD (d16), DE", _ld16i(_d16(), _rDE));
        OPCODE(0x54,  8, 2, "NEG", _neg(_rA));
        OPCODE(0x55, 14, 2, "RET N", _retn());
        OPCODE(0x56,  8, 2, "IM 1", _im(1));
        OPCODE(0x58, 12, 2, "IN E, (C)", _in(_rE, _rC));
        OPCODE(0x59, 12, 2, "OUT (C), E", _out(_rC, _rE));
        OPCODE(0x5A, 15, 2, "ADC HL, DE", _adc16(_rHL, _rDE));
        OPCODE(0x5B, 20, 2, "LD DE, (d16)", _ld16(_rDE, _i16()));
        OPCODE(0x5C,  8, 2, "NEG", _neg(_rA));
        OPCODE(0x5D, 14, 2, "RET N", _retn());
        OPCODE(0x5F,  9, 2, "LD A, R", _ld(_rA, _rR));
        OPCODE(0x60, 12, 2, "IN H, (C)", _in(_rH, _rC));
        OPCODE(0x61, 12, 2, "OUT (C), H", _out(_rC, _rH));
        OPCODE(0x62, 15, 2, "SBC HL, HL", _sbc16(_rHL, _rHL));
        OPCODE(0x63, 20, 4, "LD (d16), HL", _ld16i(_d16(), _rHL));
        OPCODE(0x64,  8, 2, "NEG", _neg(_rA));
        OPCODE(0x65, 14, 2, "RET N", _retn());
        OPCODE(0x67, 18, 2, "RRD" , _rrd());
        OPCODE(0x68, 12, 2, "IN L, (C)", _in(_rL, _rC));
        OPCODE(0x69, 12, 2, "OUT (C), L", _out(_rC, _rL));
        OPCODE(0x6A, 15, 2, "ADC HL, HL", _adc16(_rHL, _rHL));
        OPCODE(0x6B, 20, 2, "LD HL, (d16)", _ld16(_rHL, _i16()));
        OPCODE(0x6C,  8, 2, "NEG", _neg(_rA));
        OPCODE(0x6D, 14, 2, "RET N", _retn());
        OPCODE(0x6F, 18, 2, "RLD" , _rld());
        OPCODE(0x70, 12, 2, "IN (C)", _in(_rH, _rC));
        OPCODE(0x71, 12, 2, "OUT (C), 0", _out(_rC, 0));
        OPCODE(0x72, 15, 2, "SBC HL, SP", _sbc16(_rHL, _rSP));
        OPCODE(0x73, 20, 4, "LD (d16), SP", _ld16i(_d16(), _rSP));
        OPCODE(0x74,  8, 2, "NEG", _neg(_rA));
        OPCODE(0x75, 14, 2, "RET N", _retn());
        OPCODE(0x78, 12, 2, "IN A, (C)", _in(_rA, _rC));
        OPCODE(0x79, 12, 2, "OUT (C), A", _out(_rC, _rA));
        OPCODE(0x7A, 15, 2, "ADC HL, SP", _adc16(_rHL, _rSP));
        OPCODE(0x7B, 20, 2, "LD SP, (d16)", _ld16(_rSP, _i16()));
        OPCODE(0x7C,  8, 2, "NEG", _neg(_rA));
        OPCODE(0x7D, 14, 2, "RET N", _retn());
        OPCODE(0xA0, 16, 2, "LDI (HL) (DE) BC", _ldi());
        OPCODE(0xA1, 16, 2, "CPI", _cpi());
        OPCODE(0xA8, 16, 2, "LDD (HL) (DE) BC", _ldd());
        OPCODE(0xA9, 16, 2, "CPD", _cpd());
        OPCODE(0xB0, 16, 2, "LDIR", _ldir());
        OPCODE(0xB1, 16, 2, "CPIR", _cpir());
        OPCODE(0xB3, 16, 2, "OTIR", _otir());
        OPCODE(0xB8, 16, 2, "LDIR", _lddr());
        OPCODE(0xB9, 16, 2, "CPDR", _cpdr());
    default:
        _state = DeviceState::Fault;
        throw CpuOpcodeFault("z80", op, pc);
    }
}

void
Z80Cpu::dispatch_fd(void)
{
    byte_t op = pc_read();

    switch (op) {
        OPCODE(0x09, 15, 2, "ADD IY, BC", _add16(_rIY, _rBC));
        OPCODE(0x19, 15, 2, "ADD IY, DE", _add16(_rIY, _rDE));
        OPCODE(0x21, 14, 4, "LD IY, d16", _ld16(_rIY, _d16()));
        OPCODE(0x22, 20, 4, "LD (d16), IY", _ld16i(_d16(), _rIY));
        OPCODE(0x23, 10, 3, "INC IY", _inc16(_rIY));
        OPCODE(0x24,  8, 2, "INC IYh", _inc(_rIYh));
        OPCODE(0x25,  8, 2, "DEC IYh", _dec(_rIYh));
        OPCODE(0x26, 11, 3, "LD IYh, d8", _ld(_rIYh, _d8()));
        OPCODE(0x29, 15, 2, "ADD IY, IY", _add16(_rIY, _rIY));
        OPCODE(0x2A, 20, 4, "LD IY, (d16)", _ld16(_rIY, _i16()));
        OPCODE(0x2B, 10, 3, "DEC IY", _dec16(_rIY));
        OPCODE(0x2C,  8, 2, "INC IYl", _inc(_rIYl));
        OPCODE(0x2D,  8, 2, "DEC IYl", _dec(_rIYl));
        OPCODE(0x2E, 11, 3, "LD IYl, d8", _ld(_rIYl, _d8()));
        OPCODE(0x34, 23, 2, "INC (IY+d)", _inci(_dIY()));
        OPCODE(0x35, 23, 2, "DEC (IY+d)", _deci(_dIY()));
        OPCODE(0x36, 19, 4, "LD (IY+d), n", _ldmem(_dIY(), _d8()));
        OPCODE(0x39, 15, 2, "ADD IY, SP", _add16(_rIY, _rSP));
        OPCODE(0x44,  8, 2, "LD B, IYh", _ld(_rB, _rIYh));
        OPCODE(0x45,  8, 2, "LD B, IYl", _ld(_rB, _rIYl));
        OPCODE(0x46, 19, 3, "LD B, (IY+d)", _ld(_rB, _iIY()));
        OPCODE(0x4C,  8, 2, "LD C, IYh", _ld(_rC, _rIYh));
        OPCODE(0x4D,  8, 2, "LD C, IYl", _ld(_rC, _rIYl));
        OPCODE(0x4E, 19, 3, "LD C, (IY+d)", _ld(_rC, _iIY()));
        OPCODE(0x54,  8, 2, "LD D, IYh", _ld(_rD, _rIYh));
        OPCODE(0x55,  8, 2, "LD D, IYl", _ld(_rD, _rIYl));
        OPCODE(0x56, 19, 3, "LD D, (IY+d)", _ld(_rD, _iIY()));
        OPCODE(0x5C,  8, 2, "LD E, IYh", _ld(_rE, _rIYh));
        OPCODE(0x5D,  8, 2, "LD E, IYl", _ld(_rE, _rIYl));
        OPCODE(0x5E, 19, 3, "LD E, (IY+d)", _ld(_rE, _iIY()));
        OPCODE(0x60,  8, 2, "LD IYh, B", _ld(_rIYh, _rB));
        OPCODE(0x61,  8, 2, "LD IYh, C", _ld(_rIYh, _rC));
        OPCODE(0x62,  8, 2, "LD IYh, D", _ld(_rIYh, _rD));
        OPCODE(0x63,  8, 2, "LD IYh, E", _ld(_rIYh, _rE));
        OPCODE(0x64,  8, 2, "LD IYh, IYh", _ld(_rIYh, _rIYh));
        OPCODE(0x65,  8, 2, "LD IYh, IYl", _ld(_rIYh, _rIYl));
        OPCODE(0x66, 19, 3, "LD H, (IY+d)", _ld(_rH, _iIY()));
        OPCODE(0x67,  8, 2, "LD IYh, A", _ld(_rIYh, _rA));
        OPCODE(0x68,  8, 2, "LD IYl, B", _ld(_rIYl, _rB));
        OPCODE(0x69,  8, 2, "LD IYl, C", _ld(_rIYl, _rC));
        OPCODE(0x6A,  8, 2, "LD IYl, D", _ld(_rIYl, _rD));
        OPCODE(0x6B,  8, 2, "LD IYl, E", _ld(_rIYl, _rE));
        OPCODE(0x6C,  8, 2, "LD IYl, IYh", _ld(_rIYl, _rIYh));
        OPCODE(0x6D,  8, 2, "LD IYl, IYl", _ld(_rIYl, _rIYl));
        OPCODE(0x6E, 19, 3, "LD L, (IY+d)", _ld(_rL, _iIY()));
        OPCODE(0x6F,  8, 2, "LD IYl, A", _ld(_rIYl, _rA));
        OPCODE(0x70, 19, 3, "LD (IY+d), B", _ldmem(_dIY(), _rB));
        OPCODE(0x71, 19, 3, "LD (IY+d), C", _ldmem(_dIY(), _rC));
        OPCODE(0x72, 19, 3, "LD (IY+d), D", _ldmem(_dIY(), _rD));
        OPCODE(0x73, 19, 3, "LD (IY+d), E", _ldmem(_dIY(), _rE));
        OPCODE(0x74, 19, 3, "LD (IY+d), H", _ldmem(_dIY(), _rH));
        OPCODE(0x75, 19, 3, "LD (IY+d), L", _ldmem(_dIY(), _rL));
        OPCODE(0x77, 19, 3, "LD (IY+d), A", _ldmem(_dIY(), _rA));
        OPCODE(0x7C,  8, 2, "LD A, IYh", _ld(_rA, _rIYh));
        OPCODE(0x7D,  8, 2, "LD A, IYl", _ld(_rA, _rIYl));
        OPCODE(0x7E, 19, 3, "LD A, (IY+d)", _ld(_rA, _iIY()));
        OPCODE(0x84,  8, 2, "ADD A, IYh", _add(_rA, _rIYh));
        OPCODE(0x85,  8, 2, "ADD A, IYl", _add(_rA, _rIYl));
        OPCODE(0x86,  8, 3, "ADD A, (IY+d)", _add(_rA, _iIY()));
        OPCODE(0x8C,  8, 2, "ADC A, IYh", _adc(_rA, _rIYh));
        OPCODE(0x8D,  8, 2, "ADC A, IYl", _adc(_rA, _rIYl));
        OPCODE(0x8E,  8, 3, "ADC A, (IY+d)", _adc(_rA, _iIY()));
        OPCODE(0x94,  8, 2, "SUB A, IYh", _sub(_rA, _rIYh));
        OPCODE(0x95,  8, 2, "SUB A, IYl", _sub(_rA, _rIYl));
        OPCODE(0x96,  8, 3, "SUB A, (IY+d)", _sub(_rA, _iIY()));
        OPCODE(0x9C,  8, 2, "SBC A, IYh", _sbc(_rA, _rIYh));
        OPCODE(0x9D,  8, 2, "SBC A, IYl", _sbc(_rA, _rIYl));
        OPCODE(0x9E,  8, 3, "SBC A, (IY+d)", _sbc(_rA, _iIY()));
        OPCODE(0xA4,  8, 2, "AND A, IYh", _and(_rA, _rIYh));
        OPCODE(0xA5,  8, 2, "AND A, IYl", _and(_rA, _rIYl));
        OPCODE(0xA6,  8, 3, "AND A, (IY+d)", _and(_rA, _iIY()));
        OPCODE(0xAC,  8, 2, "XOR A, IYh", _xor(_rA, _rIYh));
        OPCODE(0xAD,  8, 2, "XOR A, IYl", _xor(_rA, _rIYl));
        OPCODE(0xAE,  8, 3, "XOR A, (IY+d)", _xor(_rA, _iIY()));
        OPCODE(0xB4,  8, 2, "OR A, IYh", _or(_rA, _rIYh));
        OPCODE(0xB5,  8, 2, "OR A, IYl", _or(_rA, _rIYl));
        OPCODE(0xB6,  8, 3, "OR A, (IY+d)", _or(_rA, _iIY()));
        OPCODE(0xBC,  8, 2, "CP A, IYh", _cp(_rA, _rIYh));
        OPCODE(0xBD,  8, 2, "CP A, IYl", _cp(_rA, _rIYl));
        OPCODE(0xBE,  8, 3, "CP A, (IY+d)", _cp(_rA, _iIY()));
        OPCODE(0xCB,  0, 2, "IY BITS", dispatch_fd_cb());
        OPCODE(0xE1, 14, 2, "POP IY", _pop(_rIYh, _rIYl));
        OPCODE(0xE3, 23, 2, "EX (SP), IY", _exi(_rSP, _rIYh, _rIYl));
        OPCODE(0xE5, 15, 2, "PUSH IY", _push(_rIYh, _rIYl));
        OPCODE(0xE9,  8, 2, "JP IY", _ld16(_rPC, _rIY));
        OPCODE(0xF9, 10, 2, "LD SP, IY", _ld16(_rSP, _rIY));
    default:
        break;
#if 0
        {
            std::stringstream ss;
            ss << "Unknown opcode:" << Hex(op);
            ERROR(ss.str());
        }
        _state = DeviceState::Fault;
        throw CpuFault();
#endif
    }
}

void
Z80Cpu::dispatch_fd_cb(void)
{
    addr_t addr = _dIY();
    byte_t op = pc_read();
    int bit = (op & 0x38) >> 3;
    byte_t value = _i8(addr);
    byte_t dest = 0;
    if ((op & 0xC0) == 0x00) {
        if ((op & 0xF8) == 0x00) {
            _rlc(dest, value);
        } else if ((op & 0xF8) == 0x08) {
            _rrc(dest, value);
        } else if ((op & 0xF8) == 0x10) {
            _rl(dest, value);
        } else if ((op & 0xF8) == 0x18) {
            _rr(dest, value);
        } else if ((op & 0xF8) == 0x20) {
            _sla(dest, value);
        } else if ((op & 0xF8) == 0x28) {
            _sra(dest, value);
        } else if ((op & 0xF8) == 0x30) {
            _sll(dest, value);
        } else if ((op & 0xF8) == 0x38) {
            _srl(dest, value);
        }
        if (Reg::HL == Reg(op & 0x7))
            bus_write(addr, dest);
        else
            store(Reg(op & 0x7), dest);
        _add_icycles(23);
    } else if ((op & 0xC0) == 0x40) {
        _bit_test(value, bit);
        _flags.Y = bit_isset(addr, 13);
        _flags.X = bit_isset(addr, 11);
        _add_icycles(20);
    } else if ((op & 0xC0) == 0x80) {
        _bit_reset(dest, value, bit);
        _ldmem(addr, dest);
        _add_icycles(23);
    } else if ((op & 0xC0) == 0xC0) {
        _bit_set(dest, value, bit);
        _ldmem(addr, dest);
        _add_icycles(23);
    }
}

void
Z80Cpu::interrupt(addr_t addr)
{
    _push(_rPCh, _rPCl);
    _rPC = addr;
    _iff2 = _iff1;
    _iff1 = false;
    _add_icycles(20);
}

