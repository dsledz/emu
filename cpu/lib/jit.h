/*
 * Copyright (c) 2013, Dan Sledz * All rights reserved.
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

struct JITError: public EmuException {
    JITError(const std::string &op): EmuException("JIT Error: ")
    {
        msg += op;
    }
};

namespace JITx64 {

enum RegIdx8 {
    RegAL = 0,
    RegCL = 1,
    RegDL = 2,
    RegBL = 3,
    RegAH = 4,
    RegCH = 5,
    RegDH = 6,
    RegBH = 7
};

enum RegIdx16 {
    RegAX = 0,
    RegCX = 1,
    RegDX = 2,
    RegBX = 3,
    RegSP = 4,
    RegBP = 5,
    RegSI = 6,
    RegDI = 7,
};

enum RegIdx32 {
    RegEAX = 0,
    RegECX = 1,
    RegEDX = 2,
    RegEBX = 3,
    RegESP = 4,
    RegEBP = 5,
    RegESI = 6,
    RegEDI = 7
};

enum RegIdx64 {
    RegRAX = 0,
    RegRCX = 1,
    RegRDX = 2,
    RegRBX = 3,
    RegRSP = 4,
    RegRBP = 5,
    RegRSI = 6,
    RegRDI = 7
};

enum RegIdx8L {
    RegR8b  = 0,
    RegR9b  = 1,
    RegR10b = 2,
    RegR11b = 3,
    RegR12b = 4,
    RegR13b = 5,
    RegR14b = 6,
    RegR15b = 7
};

enum RegIdx16L {
    RegR8w  = 0,
    RegR9w  = 1,
    RegR10w = 2,
    RegR11w = 3,
    RegR12w = 4,
    RegR13w = 5,
    RegR14w = 6,
    RegR15w = 7
};

enum RegIdx32L {
    RegR8d  = 0,
    RegR9d  = 1,
    RegR10d = 2,
    RegR11d = 3,
    RegR12d = 4,
    RegR13d = 5,
    RegR14d = 6,
    RegR15d = 7
};

enum RegIdx64L {
    RegR8   = 0,
    RegR9   = 1,
    RegR10  = 2,
    RegR11  = 3,
    RegR12  = 4,
    RegR13  = 5,
    RegR14  = 6,
    RegR15  = 7
};

enum Condition {
    OFSet    = 0,
    OFClear  = 1,
    CFSet    = 2,
    CFClear  = 3,
    ZFSet    = 4,
    ZFClear  = 5,
    SFSet    = 8,
    SFClear  = 9,
    PFSet    = 10,
    PFClear  = 11,
};

enum Flags {
    CF = 0,
    PF = 2,
    AF = 4,
    ZF = 6,
    SF = 7,
    TF = 8,
    IF = 9,
    DF = 10,
    OF = 11,
};

/**
 * Each JITTed CPU keeps track of it's private data here.
 */
struct JITState
{
public:
    typedef uint8_t (*jit_bus_read_t)(void *, uint16_t addr);
    typedef void    (*jit_bus_write_t)(void *, uint16_t addr, uint8_t value);

    JITState(uintptr_t ctx, jit_bus_read_t read, jit_bus_write_t write):
        m_ctx(reinterpret_cast<uintptr_t>(ctx)),
        m_bus_read(reinterpret_cast<uintptr_t>(read)),
        m_bus_write(reinterpret_cast<uintptr_t>(write))
    {
    }

private:
    uintptr_t m_ctx;
    uintptr_t m_bus_read;
    uintptr_t m_bus_write;

} __attribute__((packed));

/**
 * A JITTed code block
 */
class JITBlock
{
public:
    JITBlock(exec_buf_t code, uint32_t pc, bvec source, int len, int cycles):
        pc(pc), len(len), cycles(cycles), m_code(code), m_source(source)
    {
    }

    const uint8_t *code(void) const
    {
        return m_code.data();
    }

    const bvec &source(void) const
    {
        return m_source;
    }

    bool valid(std::function<uint8_t (uint16_t)> read_cb) const
    {
        for (unsigned i = 0; i < m_source.size(); i++) {
            uint8_t v = read_cb(pc + i);
            if (v != m_source[i])
                return false;
        }
        return true;
    }

    void execute(uintptr_t cpu_state, uintptr_t jit_state)
    {
        uintptr_t func = reinterpret_cast<uintptr_t>(m_code.data());

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
            : "r"(cpu_state), "r"(jit_state), "m"(func)
            : "rax", "rbx", "rcx", "rdx", "rdi", "rsi", "r15", "r14", "rsp", "rbp"
            );

    }

    uint32_t pc;
    uint32_t len;
    Cycles cycles;

private:
    exec_buf_t m_code;
    bvec m_source;
};

typedef std::unique_ptr<JITBlock> jit_block_ptr;

/**
 * A simple x64 Assembler that emits JITTed code.
 */
class JITEmitter
{
public:
    JITEmitter(void);
    ~JITEmitter(void);

    void reset(void);

    const exec_buf_t &code(void) const;
    const bvec &source(void) const;

    void xMOV8(RegIdx8 dst, RegIdx8 src);
    void xMOV8(RegIdx8L dst, RegIdx8 src);
    void xMOV8(RegIdx8 dst, RegIdx8L src);
    void xMOV8(RegIdx8L dst, RegIdx8L src);
    void xMOV8(RegIdx8 dst, uint8_t imm);
    void xMOV16(RegIdx16 dst, RegIdx16 src);
    void xMOV16(RegIdx16 dst, uint16_t imm);
    void xMOV16(RegIdx16 dst, RegIdx64L src, uint8_t offset);
    void xMOV16(RegIdx64L dst, uint8_t offset, RegIdx16 src);
    void xMOV64(RegIdx64 dst, RegIdx64L src, uint8_t offset);

    void xINC(RegIdx8 dst, RegIdx16 addr);
    void xINC(RegIdx8 dst);

    void xDEC(RegIdx8 dst, RegIdx16 addr);
    void xDEC(RegIdx8 dst);

    void xAbsolute(RegIdx16 dst, RegIdx8 src, int16_t base);

    void xPUSH(RegIdx16 dst);
    void xPUSH(RegIdx16L dst);
    void xPUSH(RegIdx32 dst);
    void xPUSHF(void);
    void xPUSHF16(void);

    void xCALL(RegIdx64L base, uint8_t offset);

    void xPOP(RegIdx16 dst);
    void xPOP(RegIdx16L dst);
    void xPOP(RegIdx32 dst);
    void xPOPF(void);
    void xPOPF16(void);

    void xLAHF(void);
    void xSAHF(void);

    void xFLAGS(RegIdx8 dst);
    void xLOAD(RegIdx8 dst, RegIdx16 addr);
    void xSTORE(RegIdx16 addr, RegIdx8 src);

    void xAND(RegIdx8 dst, RegIdx8 src, RegIdx16 addr);
    void xAND(RegIdx8 dst, RegIdx8 src);
    void xAND(RegIdx16 dst, RegIdx16 src);
    void xAND(RegIdx16 dst, uint16_t imm);
    void xAND(RegIdx8 dst, uint8_t imm);

    void xCMP(RegIdx8 dst, RegIdx8 src, RegIdx16 addr);
    void xCMP(RegIdx8 dst, RegIdx8 src);
    void xCMP(RegIdx8 dst, uint8_t imm);

    void xXOR(RegIdx8 dst, RegIdx8 src, RegIdx16 addr);
    void xXOR(RegIdx8 dst, RegIdx8 src);

    void xBT(RegIdx16 dst, uint8_t bit);
    void xBTR(RegIdx16 dst, uint8_t bit);
    void xBTS(RegIdx16 dst, uint8_t bit);

    void xJMP(int8_t imm);

    void xOR(RegIdx8 dst, RegIdx8 src, RegIdx16 addr);
    void xOR(RegIdx8 dst, RegIdx8 src);
    void xOR(RegIdx16 dst, RegIdx16 src);

    void xRCL(RegIdx8 dst, RegIdx16 addr);
    void xRCL(RegIdx8 dst);

    void xRCR(RegIdx8 dst, RegIdx16 addr);
    void xRCR(RegIdx8 dst);

    void xROL(RegIdx8 dst, RegIdx16 addr);
    void xROL(RegIdx8 dst);
    void xROL(RegIdx16 dst, uint8_t bit);

    void xROR(RegIdx8 dst, RegIdx16 addr);
    void xROR(RegIdx8 dst);
    void xROR(RegIdx16 dst, uint8_t bit);

    void xADD(RegIdx16 dst, RegIdx16 src);
    void xADD(RegIdx16 dst, uint8_t imm);
    void xADD(RegIdx8 dst, uint8_t imm);

    void xADC(RegIdx8 dst, RegIdx8 src, RegIdx16 addr);
    void xADC(RegIdx8 dst, RegIdx8 src);

    void xSBC(RegIdx8 dst, RegIdx8 src, RegIdx16 addr);
    void xSBC(RegIdx8 dst, RegIdx8 src);

    void xCLC(void);
    void xCMC(void);
    void xSTC(void);

    void xSETCC(RegIdx8 dst, enum Condition cc);
    void xSETCC(RegIdx16 dst, enum Condition cc);

    void xCMOV(RegIdx16 dst, enum Condition cc);

    void xBR(enum Condition cc, RegIdx16 pc);

    void xSETPC(RegIdx16 dst);
    void xSETPC(uint16_t pc);

    void xRETQ(void);

private:

    void w8(uint8_t op)
    {
        _code.push_back(op);
    }

    void w16(uint16_t op)
    {
        for (int i = 0; i < sizeof(op); i++) {
            _code.push_back(op & 0xFF);
            op >>= 8;
        }
    }

    void w32(uint32_t op)
    {
        for (int i = 0; i < sizeof(op); i++) {
            _code.push_back(op & 0xFF);
            op >>= 8;
        }
    }

    void wModRM(RegIdx8 dst, RegIdx8 src)
    {
        w8(0xC0 + (src << 3) + dst);
    }

    void wModRM(RegIdx16 dst, RegIdx16 src)
    {
        w8(0xC0 + (src << 3) + dst);
    }

    void wModRM(RegIdx8 dst)
    {
        w8(0xC0 + dst);
    }

    void wModRM(int mod, int from, int to)
    {
        w8(((mod & 0x3) << 6) + ((from & 0x7) << 3) + (to & 0x7));
    }

    void wOverride(void)
    {
        w8(0x66);
    }

public:
    exec_buf_t _code;
    bvec _source;
};

};
