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

#include "cpu/lib/cpu.h"
#include "cpu/z80/z80.h"
#include <sstream>

using namespace EMU;
using namespace CPU;
using namespace Z80;

Z80Cpu::Z80Cpu(Machine *machine, const std::string &name, unsigned hertz,
               AddressBus16 *bus):
    ClockedDevice(machine, name, hertz),
    _icycles(0),
    m_op(),
    m_R(),
    state(&m_R),
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
Z80Cpu::execute(void)
{
    /* XXX: Handle fault states. */

    while (true) {
        add_icycles(step());
    }
}

Cycles
Z80Cpu::step(void)
{
    _icycles = Cycles(0);

    if (_reset_line == LineState::Pulse) {
        reset();
        _reset_line = LineState::Clear;
        return _icycles;
    } else if (_nmi_line == LineState::Pulse) {
        interrupt(0x0066);
        _nmi_line = LineState::Clear;
        return _icycles;
    } else if (_int0_line == LineState::Assert && state->iff1 && !state->iwait) {
        switch (state->imode) {
        case 0:
            throw CpuFault(name(), "Unsupported Interrupt mode 0");
            break;
        case 1:
            interrupt(0x0038);
            return _icycles;
            break;
        case 2: {
            // XXX: We should do this when we read _data
            _int0_line = LineState::Clear;
            reg16_t irq;
            irq.b.l = bus_read((state->I << 8) | _data);
            irq.b.h = bus_read(((state->I << 8) | _data) + 1);
            interrupt(irq.d);
            return _icycles;
            break;
        }
        }
    }

    if (state->iwait)
        state->iwait = false;

    dispatch();
    return _icycles;
}

void
Z80Cpu::reset(void)
{
    DEVICE_DEBUG("reset");
    m_R = Z80State();
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
    /* XXX: force a context switch */
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
        case Reg::A: return state->AF.b.h;
        case Reg::B: return state->BC.b.h;
        case Reg::C: return state->BC.b.l;
        case Reg::D: return state->DE.b.h;
        case Reg::E: return state->DE.b.l;
        case Reg::H: return state->HL.b.h;
        case Reg::L: return state->HL.b.l;
        default: throw CpuFault(name(), "Unknown register read");
    }
}

void
Z80Cpu::store(Reg reg, byte_t value)
{
    switch (reg) {
        case Reg::A: state->AF.b.h = value; break;
        case Reg::B: state->BC.b.h = value; break;
        case Reg::C: state->BC.b.l = value; break;
        case Reg::D: state->DE.b.h = value; break;
        case Reg::E: state->DE.b.l = value; break;
        case Reg::H: state->HL.b.h = value; break;
        case Reg::L: state->HL.b.l = value; break;
        default: throw CpuFault(name(), "Unknown register write");
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

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result & 0xff) == 0;
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = bit_isset(dest ^ arg ^ result, 4);
    state->AF.b.f.X = bit_isset(result, 3);
    state->AF.b.f.V = bit_isset(dest ^ arg ^ result, 7);
    state->AF.b.f.N = false;
    state->AF.b.f.C = bit_isset(result, 8);

    dest = result;
}

void
Z80Cpu::_inc(byte_t &dest)
{
    uint16_t result = dest + 1;

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result & 0xff) == 0;
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = bit_isset(dest ^ 1 ^ result, 4);
    state->AF.b.f.X = bit_isset(result, 3);
    state->AF.b.f.V = (dest == 0x7F);
    state->AF.b.f.N = false;

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
    uint16_t result = dest + arg + state->AF.b.f.C;

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result & 0xff) == 0;
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = bit_isset(dest ^ arg ^ result, 4);
    state->AF.b.f.X = bit_isset(result, 3);
    state->AF.b.f.V = bit_isset((dest ^ arg ^ 0x80) & (arg ^ result), 7);
    state->AF.b.f.N = false;
    state->AF.b.f.C = bit_isset(result, 8);

    dest = (byte_t)result;
}

void
Z80Cpu::_sub(byte_t &dest, byte_t arg)
{
    uint16_t result = dest - arg;

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result & 0xff) == 0;
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = bit_isset(dest ^ arg ^ result, 4);
    state->AF.b.f.X = bit_isset(result, 3);
    state->AF.b.f.V = bit_isset((result^dest) & (dest^arg), 7);
    state->AF.b.f.N = true;
    state->AF.b.f.C = bit_isset(dest ^ arg ^ result, 8);

    dest = (byte_t)result;
}

void
Z80Cpu::_dec(byte_t &dest)
{
    uint16_t result = dest - 1;

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result & 0xff) == 0;
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = bit_isset(dest ^ 1 ^ result, 4);
    state->AF.b.f.X = bit_isset(result, 3);
    state->AF.b.f.V = (dest == 0x80);
    state->AF.b.f.N = true;

    dest = (byte_t)result;
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
    uint16_t result = dest - arg - state->AF.b.f.C;

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result & 0xff) == 0;
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = bit_isset(dest ^ arg ^ result, 4);
    state->AF.b.f.X = bit_isset(result, 3);
    state->AF.b.f.V = bit_isset((result^dest) & (dest^arg), 7);
    state->AF.b.f.N = true;
    state->AF.b.f.C = bit_isset(dest ^ arg ^ result, 8);


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
    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result & 0xff) == 0;
    state->AF.b.f.Y = bit_isset(arg, 5);
    state->AF.b.f.H = bit_isset(arg ^ dest ^ result, 4);
    state->AF.b.f.X = bit_isset(arg, 3);
    state->AF.b.f.V = bit_isset((result^dest) & (dest^arg), 7);
    state->AF.b.f.N = true;
    state->AF.b.f.C = bit_isset(dest ^ arg ^ result, 8);
}

void
Z80Cpu::_cpi(void)
{
    byte_t dest = state->AF.b.h;
    byte_t arg = bus_read(state->HL.d);
    uint16_t result = dest - arg;
    state->HL.d++;
    state->BC.d--;

    state->AF.b.f.H = bit_isset(arg ^ dest ^ result, 4);
    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result & 0xff) == 0;
    state->AF.b.f.Y = bit_isset(result - state->AF.b.f.H, 1);
    state->AF.b.f.X = bit_isset(result - state->AF.b.f.H, 3);
    state->AF.b.f.V = (state->BC.d != 0);
    state->AF.b.f.N = true;
}

void
Z80Cpu::_cpir(void)
{
    _cpi();
    if (state->BC.d != 0 && !state->AF.b.f.Z) {
        state->PC.d -= 2;
        _add_icycles(5);
    }
}

void
Z80Cpu::_cpd(void)
{
    byte_t dest = state->AF.b.h;
    byte_t arg = bus_read(state->HL.d);
    uint16_t result = dest - arg;
    state->HL.d--;
    state->BC.d--;

    state->AF.b.f.H = bit_isset(arg ^ dest ^ result, 4);
    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result & 0xff) == 0;
    state->AF.b.f.Y = bit_isset(result - state->AF.b.f.H, 1);
    state->AF.b.f.X = bit_isset(result - state->AF.b.f.H, 3);
    state->AF.b.f.V = (state->BC.d != 0);
    state->AF.b.f.N = true;
}

void
Z80Cpu::_cpdr(void)
{
    _cpd();
    if (state->BC.d != 0 && !state->AF.b.f.Z) {
        state->PC.d -= 2;
        _add_icycles(5);
    }
}

void
Z80Cpu::_rrd(void)
{
    byte_t value = bus_read(state->HL.d);

    bus_write(state->HL.d, (state->AF.b.h & 0x0F) << 4 | ((value & 0xF0) >> 4));
    state->AF.b.h = (state->AF.b.h & 0xF0) | (value & 0x0F);

    state->AF.b.f.S = bit_isset(state->AF.b.h, 7);
    state->AF.b.f.Z = state->AF.b.h == 0;
    state->AF.b.f.Y = bit_isset(state->AF.b.h, 5);
    state->AF.b.f.H = false;
    state->AF.b.f.X = bit_isset(state->AF.b.h, 3);
    _set_parity(state->AF.b.h);
    state->AF.b.f.N = false;
}

void
Z80Cpu::_rld(void)
{
    byte_t value = bus_read(state->HL.d);

    bus_write(state->HL.d, (state->AF.b.h & 0x0F) | ((value & 0x0F) << 4));
    state->AF.b.h = (state->AF.b.h & 0xF0) | ((value & 0xF0) >> 4);

    state->AF.b.f.S = bit_isset(state->AF.b.h, 7);
    state->AF.b.f.Z = state->AF.b.h == 0;
    state->AF.b.f.Y = bit_isset(state->AF.b.h, 5);
    state->AF.b.f.H = false;
    state->AF.b.f.X = bit_isset(state->AF.b.h, 3);
    _set_parity(state->AF.b.h);
    state->AF.b.f.N = false;
}

void
Z80Cpu::_and(byte_t &dest, byte_t arg)
{
    byte_t result = dest & arg;

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result & 0xff) == 0;
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = true;
    state->AF.b.f.X = bit_isset(result, 3);
    _set_parity(result);
    state->AF.b.f.N = false;
    state->AF.b.f.C = false;

    dest = result;
}

void
Z80Cpu::_xor(byte_t &dest, byte_t arg)
{
    byte_t result = dest ^ arg;

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result & 0xff) == 0;
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = false;
    state->AF.b.f.X = bit_isset(result, 3);
    _set_parity(result);
    state->AF.b.f.N = false;
    state->AF.b.f.C = false;

    dest = result;
}

void
Z80Cpu::_or(byte_t &dest, byte_t arg)
{
    byte_t result = dest | arg;

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result & 0xff) == 0;
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = false;
    state->AF.b.f.X = bit_isset(result, 3);
    _set_parity(result);
    state->AF.b.f.N = false;
    state->AF.b.f.C = false;

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
    state->AF.b.f.S = bit == 7 && bit_isset(value, bit);
    state->AF.b.f.Z = !bit_isset(value, bit);
    state->AF.b.f.Y = bit == 5 && bit_isset(value, bit);
    state->AF.b.f.H = 1;
    state->AF.b.f.X = bit == 3 && bit_isset(value, bit);
    state->AF.b.f.V = !bit_isset(value, bit);
    state->AF.b.f.N = false;
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
    byte_t result = (value << 1) | state->AF.b.f.C;

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result == 0);
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = false;
    state->AF.b.f.X = bit_isset(result, 3);
    _set_parity(result);
    state->AF.b.f.N = false;
    state->AF.b.f.C = bit_isset(value, 7);

    dest = result;
}

void
Z80Cpu::_rla(void)
{
    bool S = state->AF.b.f.S;
    bool Z = state->AF.b.f.Z;
    bool V = state->AF.b.f.V;
    _rl(state->AF.b.h, state->AF.b.h);
    state->AF.b.f.S = S;
    state->AF.b.f.Z = Z;
    state->AF.b.f.V = V;
}

void
Z80Cpu::_rlc(byte_t &dest, byte_t value)
{
    byte_t result = (value << 1) | ((value & 0x80) >> 7);

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result == 0);
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = false;
    state->AF.b.f.X = bit_isset(result, 3);
    _set_parity(result);
    state->AF.b.f.N = false;
    state->AF.b.f.C = (value & 0x80) != 0;

    dest = result;
}

void
Z80Cpu::_rlca(void)
{
    bool S = state->AF.b.f.S;
    bool Z = state->AF.b.f.Z;
    bool V = state->AF.b.f.V;
    _rlc(state->AF.b.h, state->AF.b.h);
    state->AF.b.f.S = S;
    state->AF.b.f.Z = Z;
    state->AF.b.f.V = V;
}

void
Z80Cpu::_rr(byte_t &dest, byte_t value)
{
    byte_t result = (value >> 1) | (state->AF.b.f.C ? 0x80 : 0x00);

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result == 0);
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = false;
    state->AF.b.f.X = bit_isset(result, 3);
    _set_parity(result);
    state->AF.b.f.N = false;
    state->AF.b.f.C = (value & 0x01) != 0;

    dest = result;
}

void
Z80Cpu::_rra(void)
{
    bool S = state->AF.b.f.S;
    bool Z = state->AF.b.f.Z;
    bool V = state->AF.b.f.V;
    _rr(state->AF.b.h, state->AF.b.h);
    state->AF.b.f.S = S;
    state->AF.b.f.Z = Z;
    state->AF.b.f.V = V;
}

void
Z80Cpu::_rrc(byte_t &dest, byte_t value)
{
    byte_t result = (value >> 1) | (value & 0x01 ? 0x80 : 0x00);

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result == 0);
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = false;
    state->AF.b.f.X = bit_isset(result, 3);
    _set_parity(result);
    state->AF.b.f.N = false;
    state->AF.b.f.C = (value & 0x01) != 0;

    dest = result;
}

void
Z80Cpu::_rrca(void)
{
    bool S = state->AF.b.f.S;
    bool Z = state->AF.b.f.Z;
    bool V = state->AF.b.f.V;
    _rrc(state->AF.b.h, state->AF.b.h);
    state->AF.b.f.S = S;
    state->AF.b.f.Z = Z;
    state->AF.b.f.V = V;
}

void
Z80Cpu::_sla(byte_t &dest, byte_t value)
{
    byte_t result = (value << 1);

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result == 0);
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = false;
    state->AF.b.f.X = bit_isset(result, 3);
    _set_parity(result);
    state->AF.b.f.N = false;
    state->AF.b.f.C = bit_isset(value, 7);

    dest = result;
}

void
Z80Cpu::_sra(byte_t &dest, byte_t value)
{
    byte_t result = (value >> 1) | (value & 0x80);

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result == 0);
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = false;
    state->AF.b.f.X = bit_isset(result, 3);
    _set_parity(result);
    state->AF.b.f.N = false;
    state->AF.b.f.C = bit_isset(value, 0);

    dest = result;
}

void
Z80Cpu::_srl(byte_t &dest, byte_t value)
{
    byte_t result = (value >> 1);

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result == 0);
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = false;
    state->AF.b.f.X = bit_isset(result, 3);
    _set_parity(result);
    state->AF.b.f.N = false;
    state->AF.b.f.C = bit_isset(value, 0);

    dest = result;
}

void
Z80Cpu::_sll(byte_t &dest, byte_t value)
{
    byte_t result = (value << 1) | 0x01;

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result == 0);
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = false;
    state->AF.b.f.X = bit_isset(result, 3);
    _set_parity(result);
    state->AF.b.f.N = false;
    state->AF.b.f.C = bit_isset(value, 7);

    dest = result;
}

void
Z80Cpu::_rst(byte_t arg)
{
    _push(state->PC.b.h, state->PC.b.l);
    state->PC.d = arg;
}

void
Z80Cpu::_jr(bool jump, byte_t arg)
{
    if (jump) {
        state->PC.d += (char)arg;
        _add_icycles(4);
    }
}

void
Z80Cpu::_jp(bool jump, uint16_t arg)
{
    if (jump) {
        state->PC.d = arg;
        _add_icycles(4);
    }
}

void
Z80Cpu::_call(bool jump, uint16_t addr)
{
    if (jump) {
        _push(state->PC.b.h, state->PC.b.l);

        state->PC.d = addr;
        _add_icycles(12);
    }
}

void
Z80Cpu::_djnz(byte_t arg)
{
    if (--state->BC.b.h != 0) {
        state->PC.d += (char)arg;
        _add_icycles(5);
    }
}

void
Z80Cpu::_ret(bool jump)
{
    if (jump) {
        _pop(state->PC.b.h, state->PC.b.l);
        _add_icycles(16);
    }
}

void
Z80Cpu::_reti(void)
{
    _pop(state->PC.b.h, state->PC.b.l);
    state->iff1 = state->iff2 = true;
}

void
Z80Cpu::_retn(void)
{
    _pop(state->PC.b.h, state->PC.b.l);
    state->iff1 = state->iff2;
}

void
Z80Cpu::_di(void)
{
    state->iff1 = state->iff2 = false;
}

void
Z80Cpu::_ei(void)
{
    state->iff1 = state->iff2 = true;
    state->iwait = true;
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
    _ex(state->BC.d, state->BC2.d);
    _ex(state->DE.d, state->DE2.d);
    _ex(state->HL.d, state->HL2.d);
}

void
Z80Cpu::_push(byte_t high, byte_t low)
{
    bus_write(--state->SP.d, high);
    bus_write(--state->SP.d, low);
}

void
Z80Cpu::_pop(byte_t &high, byte_t &low)
{
    low = bus_read(state->SP.d++);
    high = bus_read(state->SP.d++);
}

void
Z80Cpu::_in(byte_t &orig, byte_t port)
{
    orig = _io.read(port);
    m_op.yield = 1;
}

void
Z80Cpu::_out(byte_t port, byte_t value)
{
    _io.write(port, value);
    m_op.yield = 1;
}

void
Z80Cpu::_ldi(void)
{
    byte_t value = bus_read(state->HL.d++);
    bus_write(state->DE.d++, value);
    value += state->AF.b.h;
    state->BC.d--;
    state->AF.b.f.Y = bit_isset(value, 1);
    state->AF.b.f.X = bit_isset(value, 3);
    state->AF.b.f.H = 0;
    state->AF.b.f.N = 0;
    state->AF.b.f.V = (state->BC.d != 0);
}

void
Z80Cpu::_ldir(void)
{
    _ldi();
    if (state->BC.d != 0) {
        state->PC.d -= 2;
        _add_icycles(5);
    }
}

void
Z80Cpu::_ldd(void)
{
    byte_t value = bus_read(state->HL.d--);
    bus_write(state->DE.d--, value);
    value += state->AF.b.h;
    state->BC.d--;
    state->AF.b.f.Y = bit_isset(value, 1);
    state->AF.b.f.X = bit_isset(value, 3);
    state->AF.b.f.H = 0;
    state->AF.b.f.N = 0;
    state->AF.b.f.V = (state->BC.d != 0);
}

void
Z80Cpu::_lddr(void)
{
    _ldd();
    if (state->BC.d != 0) {
        state->PC.d -= 2;
        _add_icycles(5);
    }
}


void
Z80Cpu::_halt()
{
    /* XXX: Is there a race here? */
    // Cpu is halted until the next interrupt
    set_status(DeviceStatus::Halted);
    wait_status(DeviceStatus::Running);
}

void
Z80Cpu::_add16(uint16_t &wdest, uint16_t arg)
{
    uint32_t result = wdest + arg;

    state->AF.b.f.Y = bit_isset(result, 13);
    state->AF.b.f.H = bit_isset(wdest ^ arg ^ result, 12);
    state->AF.b.f.X = bit_isset(result, 11);
    state->AF.b.f.C = (result > 0xffff) ? true : false;
    state->AF.b.f.N = false;

    wdest = result;
}

void
Z80Cpu::_adc16(uint16_t &wdest, uint16_t arg)
{
    uint32_t result = wdest + arg + state->AF.b.f.C;

    state->AF.b.f.S = bit_isset(result, 15);
    state->AF.b.f.Z = (result & 0xffff) == 0;
    state->AF.b.f.Y = bit_isset(result, 13);
    state->AF.b.f.H = bit_isset(wdest ^ arg  ^ result, 12);
    state->AF.b.f.X = bit_isset(result, 11);
    state->AF.b.f.V = bit_isset((wdest ^ arg ^ 0x8000) & (arg ^ result), 15);
    state->AF.b.f.N = false;
    state->AF.b.f.C = bit_isset(result, 16);

    wdest = result;
}

void
Z80Cpu::_sbc16(uint16_t &wdest, uint16_t arg)
{
    uint32_t result = wdest - arg - state->AF.b.f.C;

    state->AF.b.f.S = bit_isset(result, 15);
    state->AF.b.f.Z = (result & 0xffff) == 0;
    state->AF.b.f.Y = bit_isset(result, 13);
    state->AF.b.f.H = bit_isset(wdest ^ arg  ^ result, 12);
    state->AF.b.f.X = bit_isset(result, 11);
    state->AF.b.f.V = bit_isset((wdest ^ arg) & (wdest ^ result), 15);
    state->AF.b.f.N = true;
    state->AF.b.f.C = bit_isset(result, 16);

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
    byte_t &dest = state->AF.b.h;
    byte_t result = ~dest;

    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = 1;
    state->AF.b.f.X = bit_isset(result, 3);
    state->AF.b.f.N = 1;

    dest = result;
}

void
Z80Cpu::_ccf(void)
{
    state->AF.b.f.Y = bit_isset(state->AF.b.h, 5);
    state->AF.b.f.H = state->AF.b.f.C;
    state->AF.b.f.X = bit_isset(state->AF.b.h, 3);
    state->AF.b.f.N = 0;
    state->AF.b.f.C = !state->AF.b.f.C;
}

void
Z80Cpu::_scf(void)
{
    state->AF.b.f.Y = bit_isset(state->AF.b.h, 5);
    state->AF.b.f.H = 0;
    state->AF.b.f.X = bit_isset(state->AF.b.h, 3);
    state->AF.b.f.N = 0;
    state->AF.b.f.C = 1;

}

void
Z80Cpu::_daa(void)
{
    byte_t &dest = state->AF.b.h;
    uint16_t arg = 0;
    uint16_t result = dest;

    if (!state->AF.b.f.N) {
        if (state->AF.b.f.H || (dest & 0x0f) > 9) arg += 0x06;
        if (state->AF.b.f.C || dest > 0x99) arg += 0x60;
        result += arg;
    } else {
        if (state->AF.b.f.H) arg += 0x6;
        if (state->AF.b.f.C) arg += 0x60;
        result -= arg;
    }

    state->AF.b.f.S = bit_isset(result, 7);
    state->AF.b.f.Z = (result & 0xff) == 0;
    state->AF.b.f.Y = bit_isset(result, 5);
    state->AF.b.f.H = 0;
    state->AF.b.f.X = bit_isset(result, 3);
    _set_parity(result);
    state->AF.b.f.C = bit_isset(dest ^ arg ^ result, 8);

    dest = (byte_t)result;
}

void
Z80Cpu::op_log(void)
{
    std::stringstream os;
    os << Hex(m_op.pc) << ":";
    const std::string &str = m_op.name;
    const std::string &delimiters = " ,";
    auto lastPos = str.find_first_not_of(delimiters, 0);
    auto pos = str.find_first_of(delimiters, lastPos);
    std::stringstream op;
    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        std::string it = str.substr(lastPos, pos - lastPos);
        op << " ";
        if (it == "(IX+d)")
            op << "(IX+" << Hex(m_op.d8) << ")";
        else if (it == "(IY+d)")
            op << "(IY" << Hex(m_op.d8) << ")";
        else if (it == "d16" || it == "a16")
            op << Hex(m_op.d16);
        else if (it == "HL") {
            switch (m_op.prefix) {
            case Z80Prefix::DDPrefix: op << "IX"; break;
            case Z80Prefix::FDPrefix: op << "IY"; break;
            default: op << "HL"; break;
            }
        } else if (it == "L") {
            switch (m_op.prefix) {
            case Z80Prefix::DDPrefix: op << "IXl"; break;
            case Z80Prefix::FDPrefix: op << "IYl"; break;
            default: op << "L"; break;
            }
        } else if (it == "H") {
            switch (m_op.prefix) {
            case Z80Prefix::DDPrefix: op << "IXh"; break;
            case Z80Prefix::FDPrefix: op << "IYh"; break;
            default: op << "H"; break;
            }
        } else if (it == "(d16)")
            op << "(" << Hex(m_op.d16) << ")";
        else if (it == "(d8)")
            op << "(" << Hex(m_op.d8) << ")";
        else if (it == "d8" || it == "r8")
            op << Hex(m_op.d8);
        else
            op << it;

        lastPos = str.find_first_not_of(delimiters, pos);
        pos = str.find_first_of(delimiters, lastPos);
    }
    os << std::setfill(' ') << std::left << std::setw(20) << op.str();
    os << "  CPU:";
    os << " PC:" << Hex(state->PC.d);
    os << ",AF:" << Hex(state->AF.d);
    os << ",BC:" << Hex(state->BC.d);
    os << ",DE:" << Hex(state->DE.d);
    os << ",HL:" << Hex(state->HL.d);
    os << ",IX:" << Hex(state->IX.d);
    os << ",IY:" << Hex(state->IY.d);
    os << ",SP:" << Hex(state->SP.d);

    LOG_TRACE(os.str());
}

#define OPCODE(op, cycles, bytes, name, func) \
    case op: { \
        m_op.opcode = op; \
        IF_LOG(Trace) \
            op_set(name); \
        func; \
        _add_icycles(cycles); \
        break; \
    }

Cycles
Z80Cpu::dispatch(void)
{
    m_op.pc = state->PC.d;
    m_op.opcode = pc_read();

    byte_t *vrH = &state->HL.b.h;
    byte_t *vrL = &state->HL.b.l;
    uint16_t *vrHL = &state->HL.d;

    switch (m_op.opcode) {
    case Z80Op::DDPrefix:
        m_op.prefix = Z80Op::DDPrefix;
        m_op.opcode = pc_read();
        vrH = &state->IX.b.h;
        vrL = &state->IX.b.l;
        vrHL = &state->IX.d;
        _add_icycles(4); /* XXX: Wrong! */
        break;
    case Z80Op::FDPrefix:
        m_op.prefix = Z80Op::FDPrefix;
        m_op.opcode = pc_read();
        vrH = &state->IY.b.h;
        vrL = &state->IY.b.l;
        vrHL = &state->IY.d;
        _add_icycles(4);
        break;
    default:
        m_op.prefix = Z80Op::NoPrefix;
        break;
    }

    state->R = ((state->R + 1) % 128) | (state->R & 0x80);

    /* XXX: Treat the DD and FD prefixes as flags to HL */
    /* XXX: Update the R register */
    switch (m_op.opcode) {
        OPCODE(0x00,  4, 1, "NOP", );
        OPCODE(0x01, 10, 3, "LD BC,d16", _ld16(state->BC.d, _d16()));
        OPCODE(0x02,  7, 1, "LD (BC),A", _ldmem(state->BC.d, state->AF.b.h));
        OPCODE(0x03,  6, 1, "INC BC", _inc16(state->BC.d));
        OPCODE(0x04,  4, 1, "INC B", _inc(state->BC.b.h));
        OPCODE(0x05,  4, 1, "DEC B", _dec(state->BC.b.h));
        OPCODE(0x06,  7, 2, "LD B,d8", _ld(state->BC.b.h, _d8()));
        OPCODE(0x07,  4, 1, "RLCA", _rlca());
        OPCODE(0x08,  4, 1, "EX AF,AF'", _ex(state->AF.d, state->AF2.d));
        OPCODE(0x09, 11, 1, "ADD HL,BC", _add16(*vrHL, state->BC.d));
        OPCODE(0x0A,  7, 1, "LD A,(BC)", _ld(state->AF.b.h, bus_read(state->BC.d)));
        OPCODE(0x0B,  6, 1, "DEC BC", _dec16(state->BC.d));
        OPCODE(0x0C,  4, 1, "INC C", _inc(state->BC.b.l));
        OPCODE(0x0D,  4, 1, "DEC C", _dec(state->BC.b.l));
        OPCODE(0x0E,  7, 2, "LD C,d8", _ld(state->BC.b.l, _d8()));
        OPCODE(0x0F,  4, 1, "RRCA", _rrca());
        OPCODE(0x10,  8, 1, "DJNZ r8", _djnz(_d8()));
        OPCODE(0x11, 10, 3, "LD DE,d16", _ld16(state->DE.d, _d16()));
        OPCODE(0x12,  7, 1, "LD (DE),A", _ldmem(state->DE.d, state->AF.b.h));
        OPCODE(0x13,  6, 1, "INC DE", _inc16(state->DE.d));
        OPCODE(0x14,  4, 1, "INC D", _inc(state->DE.b.h));
        OPCODE(0x15,  4, 1, "DEC D", _dec(state->DE.b.h));
        OPCODE(0x16,  7, 2, "LD D,d8", _ld(state->DE.b.h, _d8()));
        OPCODE(0x17,  4, 1, "RLA", _rla());
        OPCODE(0x18, 12, 2, "JR r8", _jr(true, _r8()));
        OPCODE(0x19, 11, 1, "ADD HL,DE", _add16(*vrHL, state->DE.d));
        OPCODE(0x1A,  7, 1, "LD A,(DE)", _ld(state->AF.b.h, bus_read(state->DE.d)));
        OPCODE(0x1B,  6, 1, "DEC DE", _dec16(state->DE.d));
        OPCODE(0x1C,  4, 1, "INC E", _inc(state->DE.b.l));
        OPCODE(0x1D,  4, 1, "DEC E", _dec(state->DE.b.l));
        OPCODE(0x1E,  7, 2, "LD E,d8", _ld(state->DE.b.l, _d8()));
        OPCODE(0x1F,  4, 1, "RRA", _rra());
        OPCODE(0x20,  7, 2, "JR NZ,r8", _jr(!state->AF.b.f.Z, _r8()));
        OPCODE(0x21, 10, 3, "LD HL,d16", _ld16(*vrHL, _d16()));
        OPCODE(0x22, 16, 3, "LD (d16), HL", _ld16i(_d16(), *vrHL));
        OPCODE(0x23,  7, 1, "INC HL", _inc16(*vrHL));
        OPCODE(0x24,  4, 1, "INC H", _inc(*vrH));
        OPCODE(0x25,  4, 1, "DEC H", _dec(*vrH));
        OPCODE(0x26,  7, 2, "LD H,d8", _ld(*vrH, _d8()));
        OPCODE(0x27,  4, 1, "DAA", _daa());
        OPCODE(0x28,  7, 2, "JR Z,r8", _jr(state->AF.b.f.Z, _r8()));
        OPCODE(0x29, 11, 1, "ADD HL,HL", _add16(*vrHL, *vrHL));
        OPCODE(0x2A, 16, 3, "LD HL, (d16)", _ld16(*vrHL, _i16()));
        OPCODE(0x2B,  6, 1, "DEC HL", _dec16(*vrHL));
        OPCODE(0x2C,  4, 1, "INC L", _inc(*vrL));
        OPCODE(0x2D,  4, 1, "DEC L", _dec(*vrL));
        OPCODE(0x2E,  7, 2, "LD L,d8", _ld(*vrL, _d8()));
        OPCODE(0x2F,  4, 2, "CPL", _cpl());
        OPCODE(0x30,  7, 2, "JR NC,r8", _jr(!state->AF.b.f.C, _r8()));
        OPCODE(0x31, 10, 3, "LD SP,d16", _ld16(state->SP.d, _d16()));
        OPCODE(0x32, 13, 3, "LD (d16), A", _ldmem(_d16(), state->AF.b.h));
        OPCODE(0x33,  6, 1, "INC SP", _inc16(state->SP.d));
        OPCODE(0x34, 11, 1, "INC (HL)", _inci(*vrHL));
        OPCODE(0x35, 11, 1, "DEC (HL)", _deci(*vrHL));
        OPCODE(0x36, 10, 2, "LD (HL),d8", _ldmem(_dAddr(), _d8()));
        OPCODE(0x37,  4, 1, "SCF", _scf());
        OPCODE(0x38,  7 ,2, "JR C,r8", _jr(state->AF.b.f.C, _r8()));
        OPCODE(0x39, 11, 1, "ADD HL,SP", _add16(*vrHL, state->SP.d));
        OPCODE(0x3A, 13, 3, "LD A,(d16)", _ld(state->AF.b.h, _i8(_d16())));
        OPCODE(0x3B,  6, 1, "DEC SP", _dec16(state->SP.d));
        OPCODE(0x3C,  4, 1, "INC A", _inc(state->AF.b.h));
        OPCODE(0x3D,  4, 1, "DEC A", _dec(state->AF.b.h));
        OPCODE(0x3E,  7, 2, "LD A,d8", _ld(state->AF.b.h, _d8()));
        OPCODE(0x3F,  4, 1, "CCF", _ccf());
        OPCODE(0x40,  4, 1, "LD B,B", _ld(state->BC.b.h, state->BC.b.h));
        OPCODE(0x41,  4, 1, "LD B,C", _ld(state->BC.b.h, state->BC.b.l));
        OPCODE(0x42,  4, 1, "LD B,D", _ld(state->BC.b.h, state->DE.b.h));
        OPCODE(0x43,  4, 1, "LD B,E", _ld(state->BC.b.h, state->DE.b.l));
        OPCODE(0x44,  4, 1, "LD B,H", _ld(state->BC.b.h, *vrH));
        OPCODE(0x45,  4, 1, "LD B,L", _ld(state->BC.b.h, *vrL));
        OPCODE(0x46,  7, 1, "LD B,(HL)", _ld(state->BC.b.h, _iAddr()));
        OPCODE(0x47,  4, 1, "LD B,A", _ld(state->BC.b.h, state->AF.b.h));
        OPCODE(0x48,  4, 1, "LD C,B", _ld(state->BC.b.l, state->BC.b.h));
        OPCODE(0x49,  4, 1, "LD C,C", _ld(state->BC.b.l, state->BC.b.l));
        OPCODE(0x4A,  4, 1, "LD C,D", _ld(state->BC.b.l, state->DE.b.h));
        OPCODE(0x4B,  4, 1, "LD C,E", _ld(state->BC.b.l, state->DE.b.l));
        OPCODE(0x4C,  4, 1, "LD C,H", _ld(state->BC.b.l, *vrH));
        OPCODE(0x4D,  4, 1, "LD C,L", _ld(state->BC.b.l, *vrL));
        OPCODE(0x4E,  7, 1, "LD C,(HL)", _ld(state->BC.b.l, _iAddr()));
        OPCODE(0x4F,  4, 1, "LD C,A", _ld(state->BC.b.l, state->AF.b.h));
        OPCODE(0x50,  4, 1, "LD D,B", _ld(state->DE.b.h, state->BC.b.h));
        OPCODE(0x51,  4, 1, "LD D,C", _ld(state->DE.b.h, state->BC.b.l));
        OPCODE(0x52,  4, 1, "LD D,D", _ld(state->DE.b.h, state->DE.b.h));
        OPCODE(0x53,  4, 1, "LD D,E", _ld(state->DE.b.h, state->DE.b.l));
        OPCODE(0x54,  4, 1, "LD D,H", _ld(state->DE.b.h, *vrH));
        OPCODE(0x55,  4, 1, "LD D,L", _ld(state->DE.b.h, *vrL));
        OPCODE(0x56,  7, 1, "LD D,(HL)", _ld(state->DE.b.h, _iAddr()));
        OPCODE(0x57,  4, 1, "LD D,A", _ld(state->DE.b.h, state->AF.b.h));
        OPCODE(0x58,  4, 1, "LD E,B", _ld(state->DE.b.l, state->BC.b.h));
        OPCODE(0x59,  4, 1, "LD E,C", _ld(state->DE.b.l, state->BC.b.l));
        OPCODE(0x5A,  4, 1, "LD E,D", _ld(state->DE.b.l, state->DE.b.h));
        OPCODE(0x5B,  4, 1, "LD E,E", _ld(state->DE.b.l, state->DE.b.l));
        OPCODE(0x5C,  4, 1, "LD E,H", _ld(state->DE.b.l, *vrH));
        OPCODE(0x5D,  4, 1, "LD E,L", _ld(state->DE.b.l, *vrL));
        OPCODE(0x5E,  7, 1, "LD E,(HL)", _ld(state->DE.b.l, _iAddr()));
        OPCODE(0x5F,  4, 1, "LD E,A", _ld(state->DE.b.l, state->AF.b.h));
        OPCODE(0x60,  4, 1, "LD H,B", _ld(*vrH, state->BC.b.h));
        OPCODE(0x61,  4, 1, "LD H,C", _ld(*vrH, state->BC.b.l));
        OPCODE(0x62,  4, 1, "LD H,D", _ld(*vrH, state->DE.b.h));
        OPCODE(0x63,  4, 1, "LD H,E", _ld(*vrH, state->DE.b.l));
        OPCODE(0x64,  4, 1, "LD H,H", _ld(*vrH, *vrH));
        OPCODE(0x65,  4, 1, "LD H,L", _ld(*vrH, *vrL));
        OPCODE(0x66,  7, 1, "LD H,(HL)", _ld(state->HL.b.h, _iAddr()));
        OPCODE(0x67,  4, 1, "LD H,A", _ld(*vrH, state->AF.b.h));
        OPCODE(0x68,  4, 1, "LD L,B", _ld(*vrL, state->BC.b.h));
        OPCODE(0x69,  4, 1, "LD L,C", _ld(*vrL, state->BC.b.l));
        OPCODE(0x6A,  4, 1, "LD L,D", _ld(*vrL, state->DE.b.h));
        OPCODE(0x6B,  4, 1, "LD L,E", _ld(*vrL, state->DE.b.l));
        OPCODE(0x6C,  4, 1, "LD L,H", _ld(*vrL, *vrH));
        OPCODE(0x6D,  4, 1, "LD L,L", _ld(*vrL, *vrL));
        OPCODE(0x6E,  7, 1, "LD L,(HL)", _ld(state->HL.b.l, _iAddr()));
        OPCODE(0x6F,  4, 1, "LD L,A", _ld(*vrL, state->AF.b.h));
        OPCODE(0x70,  7, 1, "LD (HL),B", _ldmem(_dAddr(), state->BC.b.h));
        OPCODE(0x71,  7, 1, "LD (HL),C", _ldmem(_dAddr(), state->BC.b.l));
        OPCODE(0x72,  7, 1, "LD (HL),D", _ldmem(_dAddr(), state->DE.b.h));
        OPCODE(0x73,  7, 1, "LD (HL),E", _ldmem(_dAddr(), state->DE.b.l));
        OPCODE(0x74,  7, 1, "LD (HL),H", _ldmem(_dAddr(), state->HL.b.h));
        OPCODE(0x75,  7, 1, "LD (HL),L", _ldmem(_dAddr(), state->HL.b.l));
        OPCODE(0x76,  4, 1, "HALT", _halt());
        OPCODE(0x77,  7, 1, "LD (HL),A", _ldmem(_dAddr(), state->AF.b.h));
        OPCODE(0x78,  4, 1, "LD A,B", _ld(state->AF.b.h, state->BC.b.h));
        OPCODE(0x79,  4, 1, "LD A,C", _ld(state->AF.b.h, state->BC.b.l));
        OPCODE(0x7A,  4, 1, "LD A,D", _ld(state->AF.b.h, state->DE.b.h));
        OPCODE(0x7B,  4, 1, "LD A,E", _ld(state->AF.b.h, state->DE.b.l));
        OPCODE(0x7C,  4, 1, "LD A,H", _ld(state->AF.b.h, *vrH));
        OPCODE(0x7D,  4, 1, "LD A,L", _ld(state->AF.b.h, *vrL));
        OPCODE(0x7E,  7, 1, "LD A,(HL)", _ld(state->AF.b.h, _iAddr()));
        OPCODE(0x7F,  4, 1, "LD A,A", _ld(state->AF.b.h, state->AF.b.h));
        OPCODE(0x80,  4, 1, "ADD A,B", _add(state->AF.b.h, state->BC.b.h));
        OPCODE(0x81,  4, 1, "ADD A,C", _add(state->AF.b.h, state->BC.b.l));
        OPCODE(0x82,  4, 1, "ADD A,D", _add(state->AF.b.h, state->DE.b.h));
        OPCODE(0x83,  4, 1, "ADD A,E", _add(state->AF.b.h, state->DE.b.l));
        OPCODE(0x84,  4, 1, "ADD A,H", _add(state->AF.b.h, *vrH));
        OPCODE(0x85,  4, 1, "ADD A,L", _add(state->AF.b.h, *vrL));
        OPCODE(0x86,  7, 1, "ADD A,(HL)", _add(state->AF.b.h, _iAddr()));
        OPCODE(0x87,  4, 1, "ADD A,A", _add(state->AF.b.h, state->AF.b.h));
        OPCODE(0x88,  4, 1, "ADC A,B", _adc(state->AF.b.h, state->BC.b.h));
        OPCODE(0x89,  4, 1, "ADC A,C", _adc(state->AF.b.h, state->BC.b.l));
        OPCODE(0x8A,  4, 1, "ADC A,D", _adc(state->AF.b.h, state->DE.b.h));
        OPCODE(0x8B,  4, 1, "ADC A,E", _adc(state->AF.b.h, state->DE.b.l));
        OPCODE(0x8C,  4, 1, "ADC A,H", _adc(state->AF.b.h, *vrH));
        OPCODE(0x8D,  4, 1, "ADC A,L", _adc(state->AF.b.h, *vrL));
        OPCODE(0x8E,  7, 1, "ADC A,(HL)", _adc(state->AF.b.h, _iAddr()));
        OPCODE(0x8F,  4, 1, "ADC A,A", _adc(state->AF.b.h, state->AF.b.h));
        OPCODE(0x90,  4, 1, "SUB B", _sub(state->AF.b.h, state->BC.b.h));
        OPCODE(0x91,  4, 1, "SUB C", _sub(state->AF.b.h, state->BC.b.l));
        OPCODE(0x92,  4, 1, "SUB D", _sub(state->AF.b.h, state->DE.b.h));
        OPCODE(0x93,  4, 1, "SUB E", _sub(state->AF.b.h, state->DE.b.l));
        OPCODE(0x94,  4, 1, "SUB H", _sub(state->AF.b.h, *vrH));
        OPCODE(0x95,  4, 1, "SUB L", _sub(state->AF.b.h, *vrL));
        OPCODE(0x96,  7, 1, "SUB (HL)", _sub(state->AF.b.h, _iAddr()));
        OPCODE(0x97,  4, 1, "SUB A", _sub(state->AF.b.h, state->AF.b.h));
        OPCODE(0x98,  4, 1, "SBC A,B", _sbc(state->AF.b.h, state->BC.b.h));
        OPCODE(0x99,  4, 1, "SBC A,C", _sbc(state->AF.b.h, state->BC.b.l));
        OPCODE(0x9A,  4, 1, "SBC A,D", _sbc(state->AF.b.h, state->DE.b.h));
        OPCODE(0x9B,  4, 1, "SBC A,E", _sbc(state->AF.b.h, state->DE.b.l));
        OPCODE(0x9C,  4, 1, "SBC A,H", _sbc(state->AF.b.h, *vrH));
        OPCODE(0x9D,  4, 1, "SBC A,L", _sbc(state->AF.b.h, *vrL));
        OPCODE(0x9E,  7, 1, "SBC A,(HL)", _sbc(state->AF.b.h, _iAddr()));
        OPCODE(0x9F,  4, 1, "SBC A,A", _sbc(state->AF.b.h, state->AF.b.h));
        OPCODE(0xA0,  4, 1, "AND B", _and(state->AF.b.h, state->BC.b.h));
        OPCODE(0xA1,  4, 1, "AND C", _and(state->AF.b.h, state->BC.b.l));
        OPCODE(0xA2,  4, 1, "AND D", _and(state->AF.b.h, state->DE.b.h));
        OPCODE(0xA3,  4, 1, "AND E", _and(state->AF.b.h, state->DE.b.l));
        OPCODE(0xA4,  4, 1, "AND H", _and(state->AF.b.h, *vrH));
        OPCODE(0xA5,  4, 1, "AND L", _and(state->AF.b.h, *vrL));
        OPCODE(0xA6,  4, 1, "AND (HL)", _and(state->AF.b.h, _iAddr()));
        OPCODE(0xA7,  4, 1, "AND A", _and(state->AF.b.h, state->AF.b.h));
        OPCODE(0xA8,  4, 1, "XOR B", _xor(state->AF.b.h, state->BC.b.h));
        OPCODE(0xA9,  4, 1, "XOR C", _xor(state->AF.b.h, state->BC.b.l));
        OPCODE(0xAA,  4, 1, "XOR D", _xor(state->AF.b.h, state->DE.b.h));
        OPCODE(0xAB,  4, 1, "XOR E", _xor(state->AF.b.h, state->DE.b.l));
        OPCODE(0xAC,  4, 1, "XOR H", _xor(state->AF.b.h, *vrH));
        OPCODE(0xAD,  4, 1, "XOR L", _xor(state->AF.b.h, *vrL));
        OPCODE(0xAE,  7, 2, "XOR (HL)", _xor(state->AF.b.h, _iAddr()));
        OPCODE(0xAF,  4, 1, "XOR A", _xor(state->AF.b.h, state->AF.b.h));
        OPCODE(0xB0,  4, 1, "OR B", _or(state->AF.b.h, state->BC.b.h));
        OPCODE(0xB1,  4, 1, "OR C", _or(state->AF.b.h, state->BC.b.l));
        OPCODE(0xB2,  4, 1, "OR D", _or(state->AF.b.h, state->DE.b.h));
        OPCODE(0xB3,  4, 1, "OR E", _or(state->AF.b.h, state->DE.b.l));
        OPCODE(0xB4,  4, 1, "OR H", _or(state->AF.b.h, *vrH));
        OPCODE(0xB5,  4, 1, "OR L", _or(state->AF.b.h, *vrL));
        OPCODE(0xB6,  7, 1, "OR (HL)", _or(state->AF.b.h, _iAddr()));
        OPCODE(0xB7,  4, 1, "OR A", _or(state->AF.b.h, state->AF.b.h));
        OPCODE(0xB8,  4, 1, "CP B", _cp(state->AF.b.h, state->BC.b.h));
        OPCODE(0xB9,  4, 1, "CP C", _cp(state->AF.b.h, state->BC.b.l));
        OPCODE(0xBA,  4, 1, "CP D", _cp(state->AF.b.h, state->DE.b.h));
        OPCODE(0xBB,  4, 1, "CP E", _cp(state->AF.b.h, state->DE.b.l));
        OPCODE(0xBC,  4, 1, "CP H", _cp(state->AF.b.h, *vrH));
        OPCODE(0xBD,  4, 1, "CP L", _cp(state->AF.b.h, *vrL));
        OPCODE(0xBE,  7, 1, "CP (HL)", _cp(state->AF.b.h, _iAddr()));
        OPCODE(0xBF,  4, 1, "CP A", _cp(state->AF.b.h, state->AF.b.h));
        OPCODE(0xC0,  5, 1, "RET NZ", _ret(!state->AF.b.f.Z));
        OPCODE(0xC1, 10, 1, "POP BC", _pop(state->BC.b.h, state->BC.b.l));
        OPCODE(0xC2, 10, 0, "JP NZ a16", _jp(!state->AF.b.f.Z, _d16()));
        OPCODE(0xC3, 10, 0, "JP a16", _jp(true, _d16()));
        OPCODE(0xC4, 10, 3, "CALL NZ,a16", _call(!state->AF.b.f.Z, _d16()));
        OPCODE(0xC5, 11, 1, "PUSH BC", _push(state->BC.b.h, state->BC.b.l));
        OPCODE(0xC6,  7, 2, "ADD A,d8", _add(state->AF.b.h, _d8()));
        OPCODE(0xC7, 11, 1, "RST 00H", _rst(0x00));
        OPCODE(0xC8,  5, 1, "RET Z", _ret(state->AF.b.f.Z));
        OPCODE(0xC9,  6, 1, "RET", _ret(true));
        OPCODE(0xCA, 10, 3, "JP Z,a16", _jp(state->AF.b.f.Z, _d16()));
        OPCODE(0xCB,  0, 1, "PREFIX CB", dispatch_cb());
        OPCODE(0xCC, 10, 3, "CALL Z,a16", _call(state->AF.b.f.Z, _d16()));
        OPCODE(0xCD, 10, 3, "CALL a16", _call(true, _d16()));
        OPCODE(0xCE,  7, 2, "ADC A,d8", _adc(state->AF.b.h, _d8()));
        OPCODE(0xCF, 11, 1, "RST 08H", _rst(0x08));
        OPCODE(0xD0,  5, 1, "RET NC", _ret(!state->AF.b.f.C));
        OPCODE(0xD1, 10, 1, "POP DE", _pop(state->DE.b.h, state->DE.b.l));
        OPCODE(0xD2, 10, 0, "JP NC", _jp(!state->AF.b.f.C, _d16()));
        OPCODE(0xD3, 11, 2, "OUT d8, A", _out(_d8(), state->AF.b.h));
        OPCODE(0xD4, 10, 3, "CALL NC,a16", _call(!state->AF.b.f.C, _d16()));
        OPCODE(0xD5, 11, 1, "PUSH DE", _push(state->DE.b.h, state->DE.b.l));
        OPCODE(0xD6,  7, 2, "SUB A,d8", _sub(state->AF.b.h, _d8()));
        OPCODE(0xD7, 11, 1, "RST 10H", _rst(0x10));
        OPCODE(0xD8,  5, 1, "RET C", _ret(state->AF.b.f.C));
        OPCODE(0xD9,  4, 1, "EXX BC DE HL", _exx());
        OPCODE(0xDA, 10, 3, "JP C,a16", _jp(state->AF.b.f.C, _d16()));
        OPCODE(0xDB, 11, 2, "IN A, d8", _in(state->AF.b.h, _d8()));
        OPCODE(0xDC, 10, 3, "CALL C, a16", _call(state->AF.b.f.C, _d16()));
        OPCODE(0xDE,  7, 2, "SBC A,d8", _sbc(state->AF.b.h, _d8()));
        OPCODE(0xDF, 11, 1, "RST 18H", _rst(0x18));
        OPCODE(0xE0,  5, 1, "RET PO", _ret(!state->AF.b.f.V));
        OPCODE(0xE1, 10, 1, "POP HL", _pop(*vrH, *vrL));
        OPCODE(0xE2, 10, 3, "JP PO,a16", _jp(!state->AF.b.f.V, _d16()));
        OPCODE(0xE3, 19, 1, "EX (SP),HL", _exi(state->SP.d, *vrH, *vrL));
        OPCODE(0xE4, 10, 3, "CALL PO,a16", _call(!state->AF.b.f.V, _d16()));
        OPCODE(0xE5, 16, 1, "PUSH HL", _push(*vrH, *vrL));
        OPCODE(0xE6,  7, 2, "AND d8", _and(state->AF.b.h, _d8()));
        OPCODE(0xE7, 16, 1, "RST 20H", _rst(0x20));
        OPCODE(0xE8,  5, 1, "RET PE", _ret(state->AF.b.f.V));
        OPCODE(0xE9,  4, 1, "JP HL", _jp(true, *vrHL));
        OPCODE(0xEA,  4, 3, "JP PE,a16", _jp(state->AF.b.f.V, _d16()));
        OPCODE(0xEB,  4, 1, "EX DE, HL", _ex(state->DE.d, state->HL.d));
        OPCODE(0xEC, 10, 3, "CALL PE,a16", _call(state->AF.b.f.V, _d16()));
        OPCODE(0xED,  0, 0, "EXTD", dispatch_ed());
        OPCODE(0xEE,  7, 1, "XOR d8", _xor(state->AF.b.h, _d8()));
        OPCODE(0xEF, 11, 1, "RST 28H", _rst(0x28));
        OPCODE(0xF0,  5, 1, "RET P", _ret(!state->AF.b.f.S));
        OPCODE(0xF1, 10, 1, "POP AF", _pop(state->AF.b.h, state->AF.b.l));
        OPCODE(0xF2, 10, 3, "JP P,a16", _jp(!state->AF.b.f.S, _d16()));
        OPCODE(0xF3,  4, 1, "DI", _di());
        OPCODE(0xF4, 10, 3, "CALL P,a16", _call(!state->AF.b.f.S, _d16()));
        OPCODE(0xF5, 11, 1, "PUSH AF", _push(state->AF.b.h, state->AF.b.l));
        OPCODE(0xF6,  7, 2, "OR d8", _or(state->AF.b.h, _d8()));
        OPCODE(0xF7, 11, 1, "RST 30H", _rst(0x30));
        OPCODE(0xF8,  5, 1, "RET M", _ret(state->AF.b.f.S));
        OPCODE(0xF9,  6, 1, "LD SP,HL", _ld16(state->SP.d, *vrHL));
        OPCODE(0xFA,  4, 3, "JP M,a16", _jp(state->AF.b.f.S, _d16()));
        OPCODE(0xFB,  4, 1, "EI", _ei());
        OPCODE(0xFC, 10, 3, "CALL M,a16", _call(state->AF.b.f.S, _d16()));
        OPCODE(0xFE,  7, 2, "CP d8", _cp(state->AF.b.h, _d8()));
        OPCODE(0xFF, 11, 1, "RST 38H", _rst(0x38));
    default:
        throw CpuOpcodeFault("z80", m_op.opcode, m_op.pc);
    }

    IF_LOG(Trace)
        op_log();
    if (m_op.yield)
        Task::yield();
    m_op.reset();

    return _icycles;
}

void
Z80Cpu::dispatch_cb(void)
{
    byte_t op;
    byte_t value;
    addr_t addr;
    switch (m_op.prefix) {
    case Z80Op::DDPrefix:
        addr = _dIX();
        op = pc_read();
        value = _i8(addr);
        _add_icycles(8);
        break;
    case Z80Op::FDPrefix:
        addr = _dIY();
        op = pc_read();
        value = _i8(addr);
        _add_icycles(8);
        break;
    default:
        addr = state->HL.d;
        op = pc_read();
        if (Reg(op & 0x07) == Reg::HL)
            value = _i8(addr);
        else
            value = fetch(Reg(op & 0x07));
        break;
    }
    int bit = (op & 0x38) >> 3;
    Reg reg = Reg(op & 0x07);
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
        if (Reg::HL == reg) {
            _add_icycles(15);
            bus_write(addr, dest);
        } else {
            _add_icycles(8);
            store(reg, dest);
        }
    } else if ((op & 0xC0) == 0x40) {
        _bit_test(value, bit);
        (reg == Reg::HL) ? _add_icycles(12) : _add_icycles(8);
    } else if ((op & 0xC0) == 0x80) {
        _bit_reset(dest, value, bit);
        if (Reg::HL == reg) {
            _add_icycles(15);
            bus_write(addr, dest);
        } else {
            _add_icycles(8);
            store(reg, dest);
        }
    } else if ((op & 0xC0) == 0xC0) {
        _bit_set(dest, value, bit);
        if (Reg::HL == reg) {
            _add_icycles(15);
            bus_write(addr, dest);
        } else {
            _add_icycles(8);
            store(reg, dest);
        }
    }
}

void
Z80Cpu::dispatch_ed(void)
{
    const addr_t pc = state->PC.d;

    byte_t op = pc_read();
    switch (op) {
        OPCODE(0x00,  5, 2, "NOP", );
        OPCODE(0x40, 12, 2, "IN B, (C)", _in(state->BC.b.h, state->BC.b.l));
        OPCODE(0x41, 12, 2, "OUT (C), B", _out(state->BC.b.l, state->BC.b.h));
        OPCODE(0x42, 15, 2, "SBC HL, BC", _sbc16(state->HL.d, state->BC.d));
        OPCODE(0x43, 20, 4, "LD (d16), BC", _ld16i(_d16(), state->BC.d));
        OPCODE(0x44,  8, 2, "NEG", _neg(state->AF.b.h));
        OPCODE(0x45, 14, 2, "RET N", _retn());
        OPCODE(0x47,  9, 2, "LD I, A", _ld(state->I, state->AF.b.h));
        OPCODE(0x48, 12, 2, "IN C, (C)", _in(state->BC.b.l, state->BC.b.l));
        OPCODE(0x49, 12, 2, "OUT (C), C", _out(state->BC.b.l, state->BC.b.l));
        OPCODE(0x4A, 15, 2, "ADC HL, BC", _adc16(state->HL.d, state->BC.d));
        OPCODE(0x4B, 20, 2, "LD BC, (d16)", _ld16(state->BC.d, _i16()));
        OPCODE(0x4C,  8, 2, "NEG", _neg(state->AF.b.h));
        OPCODE(0x4D, 14, 2, "RET I", _reti());
        OPCODE(0x4F,  9, 2, "LD R, A", _ld(state->R, state->AF.b.h));
        OPCODE(0x50, 12, 2, "IN D, (C)", _in(state->DE.b.h, state->BC.b.l));
        OPCODE(0x51, 12, 2, "OUT (C), D", _out(state->BC.b.l, state->DE.b.h));
        OPCODE(0x52, 15, 2, "SBC HL, DE", _sbc16(state->HL.d, state->DE.d));
        OPCODE(0x53, 20, 4, "LD (d16), DE", _ld16i(_d16(), state->DE.d));
        OPCODE(0x54,  8, 2, "NEG", _neg(state->AF.b.h));
        OPCODE(0x55, 14, 2, "RET N", _retn());
        OPCODE(0x56,  8, 2, "IM 1", _im(1));
        OPCODE(0x58, 12, 2, "IN E, (C)", _in(state->DE.b.l, state->BC.b.l));
        OPCODE(0x59, 12, 2, "OUT (C), E", _out(state->BC.b.l, state->DE.b.l));
        OPCODE(0x5A, 15, 2, "ADC HL, DE", _adc16(state->HL.d, state->DE.d));
        OPCODE(0x5B, 20, 2, "LD DE, (d16)", _ld16(state->DE.d, _i16()));
        OPCODE(0x5C,  8, 2, "NEG", _neg(state->AF.b.h));
        OPCODE(0x5D, 14, 2, "RET N", _retn());
        OPCODE(0x5E,  8, 2, "IM 2", _im(2));
        OPCODE(0x5F,  9, 2, "LD A, R", _ld(state->AF.b.h, state->R));
        OPCODE(0x60, 12, 2, "IN H, (C)", _in(state->HL.b.h, state->BC.b.l));
        OPCODE(0x61, 12, 2, "OUT (C), H", _out(state->BC.b.l, state->HL.b.h));
        OPCODE(0x62, 15, 2, "SBC HL, HL", _sbc16(state->HL.d, state->HL.d));
        OPCODE(0x63, 20, 4, "LD (d16), HL", _ld16i(_d16(), state->HL.d));
        OPCODE(0x64,  8, 2, "NEG", _neg(state->AF.b.h));
        OPCODE(0x65, 14, 2, "RET N", _retn());
        OPCODE(0x67, 18, 2, "RRD" , _rrd());
        OPCODE(0x68, 12, 2, "IN L, (C)", _in(state->HL.b.l, state->BC.b.l));
        OPCODE(0x69, 12, 2, "OUT (C), L", _out(state->BC.b.l, state->HL.b.l));
        OPCODE(0x6A, 15, 2, "ADC HL, HL", _adc16(state->HL.d, state->HL.d));
        OPCODE(0x6B, 20, 2, "LD HL, (d16)", _ld16(state->HL.d, _i16()));
        OPCODE(0x6C,  8, 2, "NEG", _neg(state->AF.b.h));
        OPCODE(0x6D, 14, 2, "RET N", _retn());
        OPCODE(0x6F, 18, 2, "RLD" , _rld());
        OPCODE(0x70, 12, 2, "IN (C)", _in(state->HL.b.h, state->BC.b.l));
        OPCODE(0x71, 12, 2, "OUT (C), 0", _out(state->BC.b.l, 0));
        OPCODE(0x72, 15, 2, "SBC HL, SP", _sbc16(state->HL.d, state->SP.d));
        OPCODE(0x73, 20, 4, "LD (d16), SP", _ld16i(_d16(), state->SP.d));
        OPCODE(0x74,  8, 2, "NEG", _neg(state->AF.b.h));
        OPCODE(0x75, 14, 2, "RET N", _retn());
        OPCODE(0x78, 12, 2, "IN A, (C)", _in(state->AF.b.h, state->BC.b.l));
        OPCODE(0x79, 12, 2, "OUT (C), A", _out(state->BC.b.l, state->AF.b.h));
        OPCODE(0x7A, 15, 2, "ADC HL, SP", _adc16(state->HL.d, state->SP.d));
        OPCODE(0x7B, 20, 2, "LD SP, (d16)", _ld16(state->SP.d, _i16()));
        OPCODE(0x7C,  8, 2, "NEG", _neg(state->AF.b.h));
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
        throw CpuOpcodeFault("z80", op, pc);
    }
}

void
Z80Cpu::interrupt(addr_t addr)
{
    _push(state->PC.b.h, state->PC.b.l);
    state->PC.d = addr;
    state->iff2 = state->iff1;
    state->iff1 = false;
    _add_icycles(20);
}

