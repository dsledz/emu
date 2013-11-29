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

#include "cpu/m6502/m6502.h"
#include "cpu/m6502/m6502_ops.h"
#include "cpu/m6502/m6502_jit_ops.h"

using namespace EMU;
using namespace M6502v2;
using namespace std::placeholders;

#define JIT 1

extern "C" uint8_t
m6502_bus_read(void *ctx, uint16_t addr)
{
    M6502Cpu *cpu = reinterpret_cast<M6502Cpu *>(ctx);

    return cpu->bus_read(addr);
}
extern "C" void
m6502_bus_write(void *ctx, uint16_t addr, uint8_t value)
{
    M6502Cpu *cpu = reinterpret_cast<M6502Cpu *>(ctx);

    cpu->bus_write(addr, value);
}

extern "C" uint8_t
m6502_get_flags(void *ctx, uint16_t flags)
{
    M6502Cpu *cpu = reinterpret_cast<M6502Cpu *>(ctx);

    M6502State *state = cpu->get_state();

    state->F.C = bit_isset(flags, Flags::CF);
    state->F.Z = bit_isset(flags, Flags::ZF);
    state->F.V = bit_isset(flags, Flags::OF);
    state->F.N = bit_isset(flags, Flags::SF);

    return state->SR;
}

extern "C" uint16_t
m6502_set_flags(void *ctx, uint16_t flags)
{
    M6502Cpu *cpu = reinterpret_cast<M6502Cpu *>(ctx);

    M6502State *state = cpu->get_state();

    bit_set(flags, Flags::CF, state->F.C);
    bit_set(flags, Flags::ZF, state->F.Z);
    bit_set(flags, Flags::OF, state->F.V);
    bit_set(flags, Flags::SF, state->F.N);

    return flags;
}

#define OPCODE(code, bytes, cycles, name, addr, op) { \
    code, \
    name, \
    bytes, \
    cycles, \
    std::bind(&addr, _1, _2), \
    std::bind(&op, _1, _2), \
    std::bind(&addr ## _jit, _1, _2, _3), \
    std::bind(&op ## _jit, _1, _2), \
}

M6502Cpu::M6502Cpu(Machine *machine, const std::string &name,
                            unsigned hertz, AddressBus16 *bus):
    Cpu(machine, name, hertz, bus),
    _nmi_line(LineState::Clear),
    _irq_line(LineState::Clear),
    _reset_line(LineState::Clear),
    _state(),
    _jit(),
    _jit_state(this, m6502_bus_read, m6502_bus_write, m6502_get_flags)
{
    M6502Opcode opcodes[] = {
        OPCODE(0x00, 1, 7, "BRK", Inherent, BRK),
        OPCODE(0x01, 2, 6, "ORA X,ind", XIndirect, ORA),
        OPCODE(0x05, 2, 3, "ORA zpg", ZeroPage, ORA),
        OPCODE(0x06, 2, 5, "ASL zpg", ZeroPage, ASL),
        OPCODE(0x08, 1, 3, "PHP", Inherent, PHP),
        OPCODE(0x09, 2, 2, "ORA #", Immediate, ORA),
        OPCODE(0x0A, 1, 2, "ASL A", Inherent, ASLA),
        OPCODE(0x0D, 3, 4, "ORA abs", Absolute, ORA),
        OPCODE(0x0E, 3, 5, "ASL abs", Absolute, ASL),
        OPCODE(0x10, 2, 2, "BPL", Relative, BPL),
        OPCODE(0x11, 2, 5, "ORA ind,Y", IndirectY, ORA),
        OPCODE(0x15, 2, 4, "ORA zpg,X", ZeroPageX, ORA),
        OPCODE(0x16, 2, 6, "ASL zpg,X", ZeroPageX, ASL),
        OPCODE(0x18, 1, 2, "CLC", Inherent, CLC),
        OPCODE(0x19, 3, 4, "ORA abs,Y", AbsoluteY, ORA),
        OPCODE(0x1D, 3, 4, "ORA abs,X", AbsoluteX, ORA),
        OPCODE(0x1E, 3, 7, "ASL abs,X", AbsoluteX, ASL),
        OPCODE(0x20, 3, 6, "JSR", Absolute, JSR),
        OPCODE(0x21, 2, 6, "AND X,ind", XIndirect, AND),
        OPCODE(0x24, 2, 3, "BIT zpg", ZeroPage, BIT),
        OPCODE(0x25, 2, 3, "AND zpg", ZeroPage, AND),
        OPCODE(0x26, 2, 5, "ROL zpg", ZeroPage, ROL),
        OPCODE(0x28, 1, 4, "PLP", Inherent, PLP),
        OPCODE(0x29, 2, 2, "AND #", Immediate, AND),
        OPCODE(0x2A, 1, 2, "ROL A", Inherent, ROLA),
        OPCODE(0x2C, 3, 4, "BIT abs", Absolute, BIT),
        OPCODE(0x2D, 3, 4, "AND abs", Absolute, AND),
        OPCODE(0x2E, 3, 6, "ROL abs", Absolute, ROL),
        OPCODE(0x30, 2, 2, "BMI", Relative, BMI),
        OPCODE(0x31, 2, 5, "AND ind,Y", IndirectY, AND),
        OPCODE(0x35, 2, 4, "AND zpg,X", ZeroPageX, AND),
        OPCODE(0x36, 2, 6, "ROL zpg,X", ZeroPageX, ROL),
        OPCODE(0x38, 1, 2, "SEC", Inherent, SEC),
        OPCODE(0x39, 3, 4, "AND abs,Y", AbsoluteY, AND),
        OPCODE(0x3D, 3, 4, "AND abs,X", AbsoluteX, AND),
        OPCODE(0x3E, 3, 7, "ROL abs,X", AbsoluteX, ROL),
        OPCODE(0x40, 1, 6, "RTI", Inherent, RTI),
        OPCODE(0x41, 2, 6, "EOR X,ind", XIndirect, EOR),
        OPCODE(0x45, 2, 3, "EOR zpg", ZeroPage, EOR),
        OPCODE(0x46, 2, 5, "LSR zpg", ZeroPage, LSR),
        OPCODE(0x48, 1, 3, "PHA", Inherent, PHA),
        OPCODE(0x49, 2, 2, "EOR #", Immediate, EOR),
        OPCODE(0x4A, 1, 2, "LSR A", Inherent, LSRA),
        OPCODE(0x4C, 3, 3, "JMP abs", Absolute, JMP),
        OPCODE(0x4D, 3, 4, "EOR abs", Absolute, EOR),
        OPCODE(0x4E, 3, 6, "LSR abs", Absolute, LSR),
        OPCODE(0x50, 2, 2, "BVC", Relative, BVC),
        OPCODE(0x51, 2, 5, "EOR ind,Y", IndirectY, EOR),
        OPCODE(0x55, 2, 4, "EOR zpg,X", ZeroPageX, EOR),
        OPCODE(0x56, 2, 6, "LSR zpg,X", ZeroPageX, LSR),
        OPCODE(0x58, 1, 2, "CLI", Inherent, CLI),
        OPCODE(0x59, 3, 4, "EOR abs,Y", AbsoluteY, EOR),
        OPCODE(0x5D, 3, 4, "EOR abs,X", AbsoluteX, EOR),
        OPCODE(0x5E, 3, 7, "LSR abs,X", AbsoluteX, LSR),
        OPCODE(0x60, 1, 6, "RTS", Inherent, RTS),
        OPCODE(0x61, 2, 6, "ADC X,ind", XIndirect, ADC),
        OPCODE(0x65, 2, 3, "ADC zpg", ZeroPage, ADC),
        OPCODE(0x66, 2, 5, "ROR zpg", ZeroPage, ROR),
        OPCODE(0x68, 1, 4, "PLA", Inherent, PLA),
        OPCODE(0x69, 2, 2, "ADC #", Immediate, ADC),
        OPCODE(0x6A, 1, 2, "ROR A", Inherent, RORA),
        OPCODE(0x6C, 3, 5, "JMP ind", Indirect, JMP),
        OPCODE(0x6D, 3, 4, "ADC abs", Absolute, ADC),
        OPCODE(0x6E, 3, 6, "ROR abs", Absolute, ROR),
        OPCODE(0x70, 2, 2, "BVS", Relative, BVS),
        OPCODE(0x71, 2, 5, "ADC ind,Y", IndirectY, ADC),
        OPCODE(0x75, 2, 4, "ADC zpg,X", ZeroPageX, ADC),
        OPCODE(0x76, 2, 6, "ROR zpg,X", ZeroPageX, ROR),
        OPCODE(0x78, 1, 2, "SEI", Inherent, SEI),
        OPCODE(0x79, 3, 4, "ADC abs,Y", AbsoluteY, ADC),
        OPCODE(0x7D, 3, 4, "ADC abs,X", AbsoluteX, ADC),
        OPCODE(0x7E, 3, 7, "ROR abs,X", AbsoluteX, ROR),
        OPCODE(0x81, 2, 6, "STA X,ind", XIndirect, STA),
        OPCODE(0x84, 2, 3, "STY zpg", ZeroPage, STY),
        OPCODE(0x85, 2, 3, "STA zpg", ZeroPage, STA),
        OPCODE(0x86, 2, 3, "STX zpg", ZeroPage, STX),
        OPCODE(0x88, 1, 2, "DEY", Inherent, DEY),
        OPCODE(0x8A, 1, 2, "TXA", Inherent, TXA),
        OPCODE(0x8C, 3, 4, "STY abs", Absolute, STY),
        OPCODE(0x8D, 3, 4, "STA abs", Absolute, STA),
        OPCODE(0x8E, 3, 4, "STX abs", Absolute, STX),
        OPCODE(0x90, 2, 2, "BCC", Relative, BCC),
        OPCODE(0x91, 2, 6, "STA ind,Y", IndirectY, STA),
        OPCODE(0x94, 2, 4, "STY zpg,X", ZeroPageX, STY),
        OPCODE(0x95, 2, 4, "STA zpg,X", ZeroPageX, STA),
        OPCODE(0x96, 2, 4, "STX zpg,Y", ZeroPageY, STX),
        OPCODE(0x98, 1, 2, "TYA", Inherent, TYA),
        OPCODE(0x99, 3, 5, "STA abs,Y", AbsoluteY, STA),
        OPCODE(0x9A, 1, 2, "TXS", Inherent, TXS),
        OPCODE(0x9D, 3, 5, "STA abs,X", AbsoluteX, STA),
        OPCODE(0xA0, 2, 2, "LDY #", Immediate, LDY),
        OPCODE(0xA1, 2, 6, "LDA X,ind", XIndirect, LDA),
        OPCODE(0xA2, 2, 2, "LDX #", Immediate, LDX),
        OPCODE(0xA4, 2, 3, "LDY zpg", ZeroPage, LDY),
        OPCODE(0xA5, 2, 3, "LDA zpg", ZeroPage, LDA),
        OPCODE(0xA6, 2, 3, "LDX zpg", ZeroPage, LDX),
        OPCODE(0xA8, 1, 2, "TAY", Inherent, TAY),
        OPCODE(0xA9, 2, 2, "LDA #", Immediate, LDA),
        OPCODE(0xAA, 1, 2, "TAX", Inherent, TAX),
        OPCODE(0xAC, 3, 4, "LDY abs", Absolute, LDY),
        OPCODE(0xAD, 3, 4, "LDA abs", Absolute, LDA),
        OPCODE(0xAE, 3, 4, "LDX abs", Absolute, LDX),
        OPCODE(0xB0, 2, 2, "BCS", Relative, BCS),
        OPCODE(0xB1, 2, 5, "LDA ind,Y", IndirectY, LDA),
        OPCODE(0xB4, 2, 4, "LDY zpg,X", ZeroPageX, LDY),
        OPCODE(0xB5, 2, 4, "LDA zpg,X", ZeroPageX, LDA),
        OPCODE(0xB6, 2, 4, "LDX zpg,Y", ZeroPageY, LDX),
        OPCODE(0xB8, 1, 2, "CLV", Inherent, CLV),
        OPCODE(0xB9, 3, 4, "LDA abs,Y", AbsoluteY, LDA),
        OPCODE(0xBA, 1, 2, "TSX", Inherent, TSX),
        OPCODE(0xBC, 3, 4, "LDY abs,X", AbsoluteX, LDY),
        OPCODE(0xBD, 3, 4, "LDA abs,X", AbsoluteX, LDA),
        OPCODE(0xBE, 3, 4, "LDX abs,Y", AbsoluteY, LDX),
        OPCODE(0xC0, 2, 2, "CPY #", Immediate, CPY),
        OPCODE(0xC1, 2, 6, "CMP X,ind", XIndirect, CMP),
        OPCODE(0xC4, 2, 3, "CPY zpg", ZeroPage, CPY),
        OPCODE(0xC5, 2, 3, "CMP zpg", ZeroPage, CMP),
        OPCODE(0xC6, 2, 5, "DEC zpg", ZeroPage, DEC),
        OPCODE(0xC8, 1, 2, "INY", Inherent, INY),
        OPCODE(0xC9, 2, 2, "CMP #", Immediate, CMP),
        OPCODE(0xCA, 1, 2, "DEX", Inherent, DEX),
        OPCODE(0xCC, 3, 4, "CPY abs", Absolute, CPY),
        OPCODE(0xCD, 3, 4, "CMP abs", Absolute, CMP),
        OPCODE(0xCE, 3, 3, "DEC abs", Absolute, DEC),
        OPCODE(0xD0, 2, 2, "BNE", Relative, BNE),
        OPCODE(0xD1, 2, 5, "CMP ind,Y", IndirectY, CMP),
        OPCODE(0xD5, 2, 4, "CMP zpg,X", ZeroPageX, CMP),
        OPCODE(0xD6, 2, 6, "DEC zpg,X", ZeroPageX, DEC),
        OPCODE(0xD8, 1, 2, "CLD", Inherent, CLD),
        OPCODE(0xD9, 3, 4, "CMP abs,Y", AbsoluteY, CMP),
        OPCODE(0xDD, 3, 4, "CMP abs,X", AbsoluteX, CMP),
        OPCODE(0xDE, 3, 7, "DEC abs,X", AbsoluteX, DEC),
        OPCODE(0xE0, 2, 2, "CPX #", Immediate, CPX),
        OPCODE(0xE1, 2, 6, "SBC X,ind", XIndirect, SBC),
        OPCODE(0xE4, 2, 3, "CPX zpg", ZeroPage, CPX),
        OPCODE(0xE5, 2, 3, "SBC zpg", ZeroPage, SBC),
        OPCODE(0xE6, 2, 5, "INC zpg", ZeroPage, INC),
        OPCODE(0xE8, 1, 2, "INX", Inherent, INX),
        OPCODE(0xE9, 2, 2, "SBC imm", Immediate, SBC),
        OPCODE(0xEA, 1, 2, "NOP", Inherent, NOP),
        OPCODE(0xEB, 2, 2, "SBC", Immediate, NOP), /* XXX: undocumented */
        OPCODE(0xEC, 3, 4, "CPX abs", Absolute, CPX),
        OPCODE(0xED, 3, 4, "SBC abs", Absolute, SBC),
        OPCODE(0xEE, 3, 6, "INC abs", Absolute, INC),
        OPCODE(0xF0, 2, 2, "BEQ", Relative, BEQ),
        OPCODE(0xF1, 2, 5, "SBC ind,Y", IndirectY, SBC),
        OPCODE(0xF5, 2, 4, "SBC zpg,X", ZeroPageX, SBC),
        OPCODE(0xF6, 2, 6, "INC zpg,X", ZeroPageX, INC),
        OPCODE(0xF8, 1, 2, "SED", Inherent, SED),
        OPCODE(0xF9, 3, 4, "SBC abs,Y", AbsoluteY, SBC),
        OPCODE(0xFD, 3, 4, "SBC abs,X", AbsoluteX, SBC),
        OPCODE(0xFE, 3, 7, "INC abs,X", AbsoluteX, INC),
    };

    for (int i = 0; i < sizeof(opcodes)/sizeof(opcodes[0]); i++) {
        _opcodes[opcodes[i].code] = opcodes[i];
    }
}

M6502Cpu::~M6502Cpu(void)
{
}

void
M6502Cpu::line(Line line, LineState state)
{
    switch (line) {
    case Line::RESET:
        /* XXX: We should treat this as a line */
        _reset_line = state;
        break;
    case Line::INT0:
        _irq_line = state;
        break;
    case Line::NMI:
        _nmi_line = state;
        break;
    default:
        break;
    }
}

void
M6502Cpu::reset(void)
{
    _state.PC.d = 0;
    _state.A = 0;
    _state.X = 0;
    _state.Y = 0;
    _state.SR = 0;
    _state.SP = 0xff;
    _state.PC.b.l = bus_read(0xFFFC);
    _state.PC.b.h = bus_read(0xFFFD);
}

void
M6502Cpu::test_step(void)
{
#if JIT
    step();
#else
    uint8_t val = 0;
    do {
        val = bus_read(_state.PC.d);
        if (val != 0x00)
            step();
        else
            _state.PC.d++;
    } while ((val & 0x0F) != 0 || ((val == 0xA0) || (val == 0xC0) || (val == 0xE0)));

#endif
}

void
M6502Cpu::step(void)
{
    /* Interrupts */
    if (_reset_line == LineState::Pulse) {
        DEVICE_DEBUG("Reset triggered");
        reset();
        _reset_line = LineState::Clear;
        return;
    } else if (_nmi_line == LineState::Pulse) {
        DEVICE_DEBUG("NMI triggered");
        _state.F.B = 0;
        push(_state.PC.b.h);
        push(_state.PC.b.l);
        push(_state.SR);
        _state.PC.b.l = bus_read(0xFFFA);
        _state.PC.b.h = bus_read(0xFFFB);
        _nmi_line = LineState::Clear;
        return;
    } else if (_state.F.I == 0 && _irq_line == LineState::Assert) {
        DEVICE_DEBUG("IRQ triggered");
        _state.F.B = 0;
        _state.F.I = 1;
        push(_state.PC.b.h);
        push(_state.PC.b.l);
        push(_state.SR);
        _state.PC.b.l = bus_read(0xFFFE);
        _state.PC.b.h = bus_read(0xFFFF);
        return;
    }

    uint16_t pc = _state.PC.d;
#if JIT
    jit_dispatch(pc);
#else
    dispatch(pc);
#endif
}

jit_block_ptr
M6502Cpu::jit_compile(uint16_t start_pc)
{
    _jit.reset();
    bool done = false;
    uint32_t len = 0;
    uint32_t cycles = 0;
    auto br = std::bind(&M6502Cpu::bus_read, this, _1);
    bvec source;

    while (!done) {
        uint16_t pc = start_pc + len;
        uint8_t code = bus_read(pc);

        auto it = _opcodes.find(code);
        if (it == _opcodes.end()) {
            std::cout << "Unknown opcode: " << Hex(code) << std::endl;
            throw CpuOpcodeFault(name(), code, pc);
        }
        M6502Opcode *op = &it->second;

        for (unsigned i = 0; i < op->bytes; i++) {
            source.push_back(bus_read(pc + i));
        }

        len += op->bytes;
        cycles += op->cycles;
        op->jit_address(&_jit, br, pc);
        done = !op->jit_op(&_jit, pc);
    }

    _jit.xMOV16(RegEA, start_pc + len);
    _jit.xSETPC(RegEA);
    _jit.xRETQ();
    return jit_block_ptr(new JITBlock(_jit.code(), start_pc, source, len, cycles));
}

void
M6502Cpu::log_state(void)
{
    std::stringstream os;
    os << "CPU:";
    os << " PC: " << Hex(_state.PC);
    os << " A: " << Hex(_state.A);
    os << " X: " << Hex(_state.X);
    os << " Y: " << Hex(_state.Y);
    os << " F: " << Hex(_state.SR);
    os << " S: " << Hex(_state.SP);
    DEVICE_TRACE(os.str());
}

void
M6502Cpu::log_op(const M6502Opcode *op, uint16_t pc, const uint8_t *instr)
{
    std::stringstream os;
    os << std::setw(8) << name() << ":"
       << Hex(pc) << ":" << Hex(op->code) << ":"
       << op->name << " ";
    if (op->bytes == 2)
        os << Hex(instr[1]);
    else if (op->bytes == 3)
        os << Hex(instr[1] | (instr[2] << 8));
    DEVICE_TRACE(os.str());
    return;
    os << " = >";
    const std::string &str = op->name;
    const std::string &delimiters = " ";
    auto lastPos = str.find_first_not_of(delimiters, 0);
    auto pos = str.find_first_of(delimiters, lastPos);
    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        std::string it = str.substr(lastPos, pos - lastPos);
        os << " ";
        if (it == "abs" || it == "abs,X" || it == "abs,Y")
            os << Hex(_state.EA);
        else if (it == "ind")
            os << " " << Hex(_state.EA);
        else if (it == "LDX")
            os << Hex(_state.X);
        else if (it == "LDY")
            os << Hex(_state.Y);
        else if (it == "LDA")
            os << Hex(_state.A);
        else if (it == "STX")
            os << Hex(_state.X);
        else if (it == "STY")
            os << Hex(_state.Y);
        else if (it == "STA")
            os << Hex(_state.A);
        else if (it == "#")
            os << Hex(_state.ARG);
        else if (it == "BEQ")
            os << it << " " << Hex(_state.EA);
        else if (it == "zpg" || it == "zpg,X" || it == "zpg,Y" ||
                 it == "ind,Y" || it == "X,ind")
            os << Hex(_state.EA);
        else if (it == "(zpg)")
            os << Hex(_state.EA);
        else
            os << it;
        lastPos = str.find_first_not_of(delimiters, pos);
        pos = str.find_first_of(delimiters, lastPos);
    }
    DEVICE_TRACE(os.str());
}

std::string
M6502Cpu::dasm(addr_type addr)
{
    return "";
}

void
M6502Cpu::dispatch(uint16_t pc)
{
    uint8_t buf[8] = {};
    buf[0] = pc_read();

    auto it = _opcodes.find(buf[0]);
    if (it == _opcodes.end()) {
        std::cout << "Unknown opcode: " << Hex(buf[0]) << std::endl;
        throw CpuOpcodeFault(name(), buf[0], pc);
    }

    M6502Opcode *op = &it->second;

    op->addr_mode(this, &_state);
    op->operation(this, &_state);

    IF_LOG(Trace) {
        log_op(op, pc, buf);
        log_state();
    }

    return;
}

void
M6502Cpu::jit_dispatch(uint16_t pc)
{
    JITBlock *block = NULL;

    auto it = _jit_cache.find(pc);
    if (it == _jit_cache.end()) {
        jit_block_ptr b = jit_compile(pc);
        block = b.get();
        _jit_cache.insert(std::make_pair(pc, std::move(b)));
    } else if (!it->second->valid(std::bind(&M6502Cpu::bus_read, this, _1))) {
        _jit_cache.erase(it);
        jit_block_ptr b = jit_compile(pc);
        block = b.get();
        _jit_cache.insert(std::make_pair(pc, std::move(b)));
    } else {
        block = it->second.get();
    }

    IF_LOG(Trace) {
        log_state();
        assert(block->pc != 0);
        std::cout << "Executing block at " << Hex(block->pc)
                  << " (" << block->len << " bytes, "
                  << block->cycles.v << " cycles)" << std::endl;
        int off = 0;
        const bvec & source = block->source();
        while (off < source.size()) {
            uint8_t code = source.at(off);
            auto it = _opcodes.find(code);
            if (it == _opcodes.end()) {
                std::cout << "Unknown opcode: " << Hex(code) << std::endl;
                throw CpuOpcodeFault(name(), code, pc + off);
            }
            M6502Opcode *op = &it->second;
            log_op(op, pc + off, source.data() + off);
            off += op->bytes;
        }
    }

    uintptr_t func = reinterpret_cast<uintptr_t>(block->code());

    bit_set(_state.NativeFlags.d, Flags::CF, _state.F.C);
    bit_set(_state.NativeFlags.d, Flags::ZF, _state.F.Z);
    bit_set(_state.NativeFlags.d, Flags::OF, _state.F.V);
    bit_set(_state.NativeFlags.d, Flags::SF, _state.F.N);

    asm volatile(
        "mov %0, %%r15;\n"
        "mov %1, %%r14;\n"
        "movw 0(%%r15), %%bx;\n"
        "movw 2(%%r15), %%cx;\n"
        "pushw 8(%%r15);\n" /* flags */
        "popfw;\n"
        "callq *%2;\n"
        "pushfw;\n"
        "popw 8(%%r15);\n"
        "movw %%bx, 0(%%r15);\n"
        "movw %%cx, 2(%%r15);\n"
        :
        : "r"(&_state), "r"(&_jit_state), "m"(func)
        : "rax", "rbx", "rcx", "rdx", "rdi", "rsi", "r15", "r14", "rsp", "rbp"
    );

    // Update our flags
    _state.F.C = bit_isset(_state.NativeFlags.d, Flags::CF);
    _state.F.Z = bit_isset(_state.NativeFlags.d, Flags::ZF);
    _state.F.V = bit_isset(_state.NativeFlags.d, Flags::OF);
    _state.F.N = bit_isset(_state.NativeFlags.d, Flags::SF);

    // Account for the extra cycles if we branched
    add_icycles(block->cycles);
    if (_state.PC.d != (block->pc + block->len))
        add_icycles(2);
}

