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
/**
 * x64 Jitter
 */

#include "emu/jit.h"

using namespace JITx64;

JITEmitter::JITEmitter(void):
    _code(0)
{
}

JITEmitter::~JITEmitter(void)
{
}

void
JITEmitter::reset(void)
{
    _code.resize(0);
}

const jit_buf_t &
JITEmitter::code(void) const
{
    return _code;
}

/*   ___                      _
 *  / _ \ _ __   ___ ___   __| | ___  ___
 * | | | | '_ \ / __/ _ \ / _` |/ _ \/ __|
 * | |_| | |_) | (_| (_) | (_| |  __/\__ \
 *  \___/| .__/ \___\___/ \__,_|\___||___/
 *       |_|
 */

void
JITEmitter::xMOV8(RegIdx8 dst, RegIdx8 src)
{
    w8(0x88);
    wModRM(dst, src);
}

void
JITEmitter::xMOV8(RegIdx8 dst, uint8_t imm)
{
    w8(0xB0 + dst);
    w8(imm);
}

void
JITEmitter::xMOV16(RegIdx16 dst, RegIdx16 src)
{
    wOverride();
    w8(0x89);
    wModRM(3, src, dst);
}

void
JITEmitter::xMOV16(RegIdx16 dst, uint16_t imm)
{
    /* mov $base, %si */
    wOverride();
    w8(0xB8 + dst);
    w16(imm);
}

void
JITEmitter::xMOV64(RegIdx64 dst, RegIdx64L src, uint8_t offset)
{
    w8(0x49); /* XXX: REX */
    w8(0x8B);
    wModRM(1, dst, src); /* XXX: 0x7E = 0(r14), rdi  which seems weird */
    w8(offset);
}

void
JITEmitter::xINC(RegIdx16 addr)
{
    xLOAD(RegIdx8::RegDL, addr);
    xINC(RegIdx8::RegDL);
    xSTORE(addr, RegIdx8::RegDL);
}

void
JITEmitter::xINC(RegIdx8 dst)
{
    w8(0xFE);
    wModRM(dst);
}

void
JITEmitter::xDEC(RegIdx16 addr)
{
    xLOAD(RegIdx8::RegDL, addr);
    xDEC(RegIdx8::RegDL);
    xSTORE(addr, RegIdx8::RegDL);
}

void
JITEmitter::xDEC(RegIdx8 dst)
{
    w8(0xFE);
    wModRM(3, 1, dst);
}

void
JITEmitter::xAbsolute(RegIdx16 dst, RegIdx8 src, int16_t base)
{
    /* pushf */
    _code.push_back(0x9C);

    /* mov %bl, %dl */
    _code.push_back(0x88);
    _code.push_back(0xC0 + (src << 3) + RegIdx8::RegDL);

    /* mov $00, %dh */
    _code.push_back(0xC6);
    _code.push_back(0xC0 + RegIdx8::RegDH);
    _code.push_back(0x00);

    /* add %dx, imm8 */
    _code.push_back(0x66);
    _code.push_back(0x83);
    _code.push_back(0xC0 + (0 << 3) + RegIdx8::RegDL);
    _code.push_back(base);

    /* mov %dx, %dst */
    xMOV16(dst, RegIdx16::RegDX);

    /* popf */
    _code.push_back(0x9D);
}

void
JITEmitter::xPUSH(RegIdx16 dst)
{
    w8(0x50 + dst);
}

void
JITEmitter::xPUSH(RegIdx16L dst)
{
    w8(0x41);
    w8(0x50 + dst);
}

void
JITEmitter::xPUSHF(void)
{
    w8(0x9C);
}

void
JITEmitter::xPOP(RegIdx16 dst)
{
    w8(0x58 + dst);
}

void
JITEmitter::xPOP(RegIdx16L dst)
{
    w8(0x41);
    w8(0x58 + dst);
}

void
JITEmitter::xPOPF(void)
{
    w8(0x9D);
}

void
JITEmitter::xCALL(RegIdx64L base, uint8_t offset)
{
    w8(0x41);
    w8(0xff);
    wModRM(1, 2, base);
    w8(offset);
}

void
JITEmitter::xLOAD(RegIdx8 dst, RegIdx16 addr)
{
    /* pushf */
    xPUSHF();

    xPUSH(RegIdx16::RegAX);
    xPUSH(RegIdx16::RegBX);
    xPUSH(RegIdx16::RegCX);

    /* Arg 2 */
    xPUSH(RegIdx16::RegSI);
    xMOV16(RegIdx16::RegSI, addr);

    /* Arg 1 */
    xPUSH(RegIdx16::RegDI);
    xMOV64(RegIdx64::RegRDI, RegIdx64L::RegR14, 0);

    xCALL(RegIdx64L::RegR14, 8);
    xPOP(RegIdx16::RegDI);
    xPOP(RegIdx16::RegSI);

    /* Copy the result */
    xMOV8(RegIdx8::RegDH, RegIdx8::RegAL);

    xPOP(RegIdx16::RegCX);
    xPOP(RegIdx16::RegBX);
    xPOP(RegIdx16::RegAX);

    xMOV8(dst, RegIdx8::RegDH);

    /* popf */
    xPOPF();
}

void
JITEmitter::xSTORE(RegIdx16 addr, RegIdx8 src)
{
    xPUSHF();

    xPUSH(RegIdx16::RegAX);
    xPUSH(RegIdx16::RegBX);
    xPUSH(RegIdx16::RegCX);

    /* arg 3 */
    xPUSH(RegIdx16::RegDX);
    xMOV8(RegIdx8::RegDL, src);

    /* arg 2 */
    xPUSH(RegIdx16::RegSI);
    xMOV16(RegIdx16::RegSI, addr);

    /* arg 1 */
    xPUSH(RegIdx16::RegDI);
    xMOV64(RegIdx64::RegRDI, RegIdx64L::RegR14, 0);

    xCALL(RegIdx64L::RegR14, 16);
    xPOP(RegIdx16::RegDI);
    xPOP(RegIdx16::RegSI);
    xPOP(RegIdx16::RegDX);

    xPOP(RegIdx16::RegCX);
    xPOP(RegIdx16::RegBX);
    xPOP(RegIdx16::RegAX);

    xPOPF();
}

void
JITEmitter::xAND(RegIdx8 dst, RegIdx16 src)
{
    xLOAD(RegIdx8::RegDL, src);
    xAND(dst, RegIdx8::RegDL);
}

void
JITEmitter::xAND(RegIdx8 dst, RegIdx8 src)
{
    w8(0x20); /* AND r/m8,r8 */
    wModRM(dst, src);
}

void
JITEmitter::xCMP(RegIdx8 dst, RegIdx16 src)
{
    xLOAD(RegIdx8::RegDL, src);
    xCMP(dst, RegIdx8::RegDL);
}

void
JITEmitter::xCMP(RegIdx8 dst, RegIdx8 src)
{
    w8(0x38); /* CMP r/m8,r8 */
    wModRM(dst, src);
}

void
JITEmitter::xCMP(RegIdx8 dst, uint8_t imm)
{
    w8(0x80); /* CMP r/m8,imm8 */
    wModRM(3, 7, dst);
    w8(imm);
}

void
JITEmitter::xXOR(RegIdx8 dst, RegIdx16 src)
{
    xLOAD(RegIdx8::RegDL, src);
    xXOR(dst, RegIdx8::RegDL);
}

void
JITEmitter::xXOR(RegIdx8 dst, RegIdx8 src)
{
    w8(0x30); /* XOR r/m8,r8 */
    wModRM(dst, src);
}

void
JITEmitter::xOR(RegIdx8 dst, RegIdx16 src)
{
    xLOAD(RegIdx8::RegDL, src);
    xOR(dst, RegIdx8::RegDL);
}

void
JITEmitter::xRCL(RegIdx16 addr)
{
    xLOAD(RegIdx8::RegDL, addr);
    xRCL(RegIdx8::RegDL);
    xSTORE(addr, RegIdx8::RegDL);
}

void
JITEmitter::xRCL(RegIdx8 dst)
{
    w8(0xD0); /* r/m8,1 */
    wModRM(3, 2, dst);
}

void
JITEmitter::xRCR(RegIdx16 addr)
{
    xLOAD(RegIdx8::RegDL, addr);
    xRCR(RegIdx8::RegDL);
    xSTORE(addr, RegIdx8::RegDL);
}

void
JITEmitter::xRCR(RegIdx8 dst)
{
    w8(0xD0); /* r/m8,1 */
    wModRM(3, 3, dst);
}

void
JITEmitter::xSHL(RegIdx16 addr)
{
    xLOAD(RegIdx8::RegDL, addr);
    xSHL(RegIdx8::RegDL);
    xSTORE(addr, RegIdx8::RegDL);
}

void
JITEmitter::xSHL(RegIdx8 dst)
{
    w8(0xD0); /* r/m8,1 */
    wModRM(3, 4, dst);
}

void
JITEmitter::xSHR(RegIdx16 addr)
{
    xLOAD(RegIdx8::RegDL, addr);
    xSHR(RegIdx8::RegDL);
    xSTORE(addr, RegIdx8::RegDL);
}

void
JITEmitter::xSHR(RegIdx8 dst)
{
    w8(0xD0); /* r/m8,1 */
    wModRM(3, 5, dst);
}

void
JITEmitter::xOR(RegIdx8 dst, RegIdx8 src)
{
    w8(0x08); /* OR r/m8,r8 */
    wModRM(dst, src);
}

void
JITEmitter::xADC(RegIdx8 dst, RegIdx16 src)
{
    xLOAD(RegIdx8::RegDL, src);
    xADC(dst, RegIdx8::RegDL);
}

void
JITEmitter::xADC(RegIdx8 dst, RegIdx8 src)
{
    w8(0x10); /* ADC r/m8,r8 */
    wModRM(dst, src);
}

void
JITEmitter::xSBC(RegIdx8 dst, RegIdx16 src)
{
    xLOAD(RegIdx8::RegDL, src);
    xSBC(dst, RegIdx8::RegDL);
}

void
JITEmitter::xSBC(RegIdx8 dst, RegIdx8 src)
{
    w8(0x18); /* SBB r/m8,r8 */
    wModRM(dst, src);
}

void
JITEmitter::xCLC(void)
{
    w8(0xF8); /* CLC */
}

void
JITEmitter::xSTC(void)
{
    w8(0xF9); /* STC */
}

void
JITEmitter::xSETCC(RegIdx8 dst, enum Condition cc)
{
    w8(0x0F);
    w8(0x90 + cc);
    wModRM(3, 0, dst);
}

void
JITEmitter::xCMOV(RegIdx16 dst, enum Condition cc)
{
    /* jcc 5 */
    w8(0x0F);
    w8(0x80 + cc);
    w32(0x05);

    /* XXX: Assumes xSETPC is 5 bytes */
    xSETPC(dst);
}

void
JITEmitter::xBR(enum Condition cc, RegIdx16 pc)
{
    xSETCC(RegIdx8::RegDL, cc);

    xCMP(RegIdx8::RegDL, 0x01);

    /* jcc 5 */
    w8(0x0F);
    w8(0x80 + Condition::ZFClear);
    w32(0x06);

    xSETPC(pc);
    xRETQ();
}

void
JITEmitter::xSETPC(RegIdx16 dst)
{
    /* movw %dst, 6(%r15) */
    wOverride();
    w8(0x41);
    w8(0x89);
    wModRM(1, dst, 7);
    w8(0x06);
}

void
JITEmitter::xRETQ(void)
{
    w8(0xC3);
}



