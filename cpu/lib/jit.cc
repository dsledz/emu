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

#include "cpu/lib/jit.h"

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
    _source.resize(0);
}

const exec_buf_t &
JITEmitter::code(void) const
{
    return _code;
}

const bvec &
JITEmitter::source(void) const
{
    return _source;
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
JITEmitter::xMOV16(RegIdx16 dst, RegIdx64L src, uint8_t offset)
{
    wOverride();
    w8(0x49); /* XXX: REX */
    w8(0x8B);
    wModRM(1, dst, src);
    w8(offset);
}

void
JITEmitter::xMOV16(RegIdx64L dst, uint8_t offset, RegIdx16 src)
{
    wOverride();
    w8(0x49); /* XXX: REX */
    w8(0x89);
    wModRM(1, src, dst);
    w8(offset);
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
JITEmitter::xINC(RegIdx8 dst, RegIdx16 addr)
{
    xLOAD(dst, addr);
    xINC(dst);
    xSTORE(addr, dst);
}

void
JITEmitter::xINC(RegIdx8 dst)
{
    xPUSHF();
    w8(0xFE);
    wModRM(dst);
    xPOPF();
}

void
JITEmitter::xDEC(RegIdx8 dst, RegIdx16 addr)
{
    xLOAD(dst, addr);
    xDEC(dst);
    xSTORE(addr, dst);
}

void
JITEmitter::xDEC(RegIdx8 dst)
{
    //xPUSHF();
    w8(0xFE);
    wModRM(3, 1, dst);
    //xPOPF();
}

void
JITEmitter::xAbsolute(RegIdx16 dst, RegIdx8 src, int16_t base)
{
    assert(src != RegIdx8::RegAL && src != RegIdx8::RegDH);
    assert(dst != RegIdx16::RegAX);

    xPUSHF();

    xMOV8(RegIdx8::RegAL, src);
    xMOV8(RegIdx8::RegAH, 0);

    xMOV16(dst, base);
    xADD(dst, RegIdx16::RegAX);

    xPOPF();
}

void
JITEmitter::xPUSH(RegIdx16 dst)
{
    wOverride();
    w8(0x50 + dst);
}

void
JITEmitter::xPUSH(RegIdx16L dst)
{
    w8(0x41);
    w8(0x50 + dst);
}

void
JITEmitter::xPUSH(RegIdx32 dst)
{
    w8(0x50 + dst);
}

void
JITEmitter::xPUSHF(void)
{
    w8(0x9C);
}

void
JITEmitter::xPUSHF16(void)
{
    wOverride();
    w8(0x9C);
}

void
JITEmitter::xPOP(RegIdx16 dst)
{
    wOverride();
    w8(0x58 + dst);
}

void
JITEmitter::xPOP(RegIdx16L dst)
{
    w8(0x41);
    w8(0x58 + dst);
}

void
JITEmitter::xPOP(RegIdx32 dst)
{
    wOverride();
    w8(0x58 + dst);
}

void
JITEmitter::xPOPF(void)
{
    w8(0x9D);
}

void
JITEmitter::xPOPF16(void)
{
    wOverride();
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

    xPUSH(RegIdx16::RegBX);
    xPUSH(RegIdx16::RegBX);
    xPUSH(RegIdx16::RegBX);
    xPUSH(RegIdx16::RegCX);
    xPUSH(RegIdx16::RegDX);
    xPUSH(RegIdx16::RegSI);
    xPUSH(RegIdx16::RegDI);

    /* Arg 2 */
    xMOV16(RegIdx16::RegSI, addr);

    /* Arg 1 */
    xMOV64(RegIdx64::RegRDI, RegIdx64L::RegR14, 0);

    /* Save AX */
    xPUSH(RegIdx16::RegAX);

    xCALL(RegIdx64L::RegR14, 8);

    /* Restore AX */
    xPOP(RegIdx16::RegBX);
    if (dst == RegIdx8::RegAL) {
        xMOV8(RegIdx8::RegAL, RegIdx8::RegAL);
        xMOV8(RegIdx8::RegAH, RegIdx8::RegBH);
    } else if (dst == RegIdx8::RegAH) {
        xMOV8(RegIdx8::RegAH, RegIdx8::RegAL);
        xMOV8(RegIdx8::RegAL, RegIdx8::RegBL);
        dst = RegIdx8::RegAL;
    }

    xPOP(RegIdx16::RegDI);
    xPOP(RegIdx16::RegSI);
    xPOP(RegIdx16::RegDX);
    xPOP(RegIdx16::RegCX);
    xPOP(RegIdx16::RegBX);
    xPOP(RegIdx16::RegBX);
    xPOP(RegIdx16::RegBX);

    /* Copy the result */
    xMOV8(dst, RegIdx8::RegAL);

    /* popf */
    xPOPF();
}

void
JITEmitter::xSTORE(RegIdx16 addr, RegIdx8 src)
{
    xPUSHF();

    xPUSH(RegIdx16::RegAX);
    xPUSH(RegIdx16::RegAX);
    xPUSH(RegIdx16::RegAX);
    xPUSH(RegIdx16::RegBX);
    xPUSH(RegIdx16::RegCX);
    xPUSH(RegIdx16::RegDX);
    xPUSH(RegIdx16::RegSI);
    xPUSH(RegIdx16::RegDI);

    /* XXX: We have to do arg 2 first since we override DX :( */
    /* arg 2 */
    xMOV16(RegIdx16::RegSI, addr);

    /* arg 3 */
    xMOV8(RegIdx8::RegDL, src);

    /* arg 1 */
    xMOV64(RegIdx64::RegRDI, RegIdx64L::RegR14, 0);

    xCALL(RegIdx64L::RegR14, 16);
    xPOP(RegIdx16::RegDI);
    xPOP(RegIdx16::RegSI);
    xPOP(RegIdx16::RegDX);
    xPOP(RegIdx16::RegCX);
    xPOP(RegIdx16::RegBX);
    xPOP(RegIdx16::RegAX);
    xPOP(RegIdx16::RegAX);
    xPOP(RegIdx16::RegAX);

    xPOPF();
}

void
JITEmitter::xAND(RegIdx8 dst, RegIdx8 src, RegIdx16 addr)
{
    assert(dst != src);

    xLOAD(src, addr);
    xAND(dst, src);
}

void
JITEmitter::xAND(RegIdx8 dst, RegIdx8 src)
{
    xPUSHF();
    w8(0x20); /* AND r/m8,r8 */
    wModRM(dst, src);
    xPOPF();
}

void
JITEmitter::xAND(RegIdx16 dst, uint16_t immm)
{
    wOverride();
    w8(0x81);
    wModRM(4, 0, dst);
    w16(immm);
}

void
JITEmitter::xAND(RegIdx8 dst, uint8_t immm)
{
    wOverride();
    w8(0x80);
    wModRM(4, 0, dst);
    w8(immm);
}

void
JITEmitter::xBT(RegIdx16 dst, uint8_t bit)
{
    wOverride();
    w8(0x0F);
    w8(0xBA);
    wModRM(3, 4, dst);
    w8(bit);
}

void
JITEmitter::xBTR(RegIdx16 dst, uint8_t bit)
{
    wOverride();
    w8(0x0F);
    w8(0xBA);
    wModRM(3, 6, dst);
    w8(bit);
}

void
JITEmitter::xBTS(RegIdx16 dst, uint8_t bit)
{
    wOverride();
    w8(0x0F);
    w8(0xBA);
    wModRM(3, 5, dst);
    w8(bit);
}

void
JITEmitter::xCMP(RegIdx8 dst, RegIdx8 src, RegIdx16 addr)
{
    assert(dst != src);
    xPUSHF();
    xLOAD(src, addr);
    xCMP(dst, src);
    xLAHF();
    xPOPF();
    xSAHF();
}

void
JITEmitter::xCMP(RegIdx8 dst, RegIdx8 src)
{
    w8(0x38); /* CMP r/m8,r8 */
    wModRM(dst, src);
    xCMC();
}

void
JITEmitter::xCMP(RegIdx8 dst, uint8_t imm)
{
    w8(0x80); /* CMP r/m8,imm8 */
    wModRM(3, 7, dst);
    w8(imm);
}

void
JITEmitter::xXOR(RegIdx8 dst, RegIdx8 src, RegIdx16 addr)
{
    assert(dst != src);
    xLOAD(src, addr);
    xXOR(dst, src);
}

void
JITEmitter::xXOR(RegIdx8 dst, RegIdx8 src)
{
    xPUSHF();
    w8(0x30); /* XOR r/m8,r8 */
    wModRM(dst, src);
    xPOPF();
}

void
JITEmitter::xOR(RegIdx8 dst, RegIdx8 src)
{
    xPUSHF();
    w8(0x08); /* OR r/m8,r8 */
    wModRM(dst, src);
    xPOPF();
}

void
JITEmitter::xOR(RegIdx8 dst, RegIdx8 src, RegIdx16 addr)
{
    assert(dst != src);
    xLOAD(src, addr);
    xOR(dst, src);
}

void
JITEmitter::xOR(RegIdx16 dst, RegIdx16 src)
{
    wOverride();
    w8(0x09);
    wModRM(dst, src);
}

void
JITEmitter::xRCL(RegIdx8 dst, RegIdx16 addr)
{
    assert(addr != RegIdx16::RegAX);
    xLOAD(dst, addr);
    xRCL(dst);
    xSTORE(addr, dst);
}

void
JITEmitter::xLAHF(void)
{
    w8(0x9F);
}

void
JITEmitter::xSAHF(void)
{
    w8(0x9E);
}

void
JITEmitter::xRCL(RegIdx8 dst)
{
    xPUSHF();
    w8(0xC0); /* r/m8,1 */
    wModRM(3, 2, dst);
    w8(1);
    xLAHF();
    xPOPF();
    xSAHF();
}

void
JITEmitter::xRCR(RegIdx8 dst, RegIdx16 addr)
{
    assert(addr != RegIdx16::RegAX);
    xLOAD(dst, addr);
    xRCR(dst);
    xSTORE(addr, dst);
}

void
JITEmitter::xRCR(RegIdx8 dst)
{
    w8(0xD0); /* r/m8,1 */
    wModRM(3, 3, dst);
}

void
JITEmitter::xROL(RegIdx8 dst, RegIdx16 addr)
{
    assert(addr != RegIdx16::RegAX);
    xLOAD(dst, addr);
    xROL(dst);
    xSTORE(addr, dst);
}

void
JITEmitter::xROL(RegIdx8 dst)
{
    xPUSHF();
    w8(0xD0); /* r/m8,1 */
    wModRM(3, 4, dst);
    xLAHF();
    xPOPF();
    xSAHF();
}

void
JITEmitter::xROL(RegIdx16 dst, uint8_t bit)
{
    wOverride();
    w8(0xC1);
    wModRM(3, 0, dst);
    w8(bit);
}

void
JITEmitter::xROR(RegIdx8 dst, RegIdx16 addr)
{
    assert(addr != RegIdx16::RegAX);
    xLOAD(dst, addr);
    xROR(dst);
    xSTORE(addr, dst);
}

void
JITEmitter::xROR(RegIdx8 dst)
{
    xPUSHF();
    w8(0xD0); /* r/m8,1 */
    wModRM(3, 5, dst);
    xLAHF();
    xPOPF();
    xSAHF();
}

void
JITEmitter::xROR(RegIdx16 dst, uint8_t bit)
{
    wOverride();
    w8(0xC1);
    wModRM(3, 1, dst);
    w8(bit);
}

void
JITEmitter::xADD(RegIdx16 dst, RegIdx16 src)
{
    wOverride();
    w8(0x01);  /* ADD r/m16,r16 */
    wModRM(dst, src);
}

void
JITEmitter::xADD(RegIdx16 dst, uint8_t imm)
{
    wOverride();
    w8(0x83);
    wModRM(3, 0, dst);
    w8(imm);
}

void
JITEmitter::xADD(RegIdx8 dst, uint8_t imm)
{
    w8(0x80);
    wModRM(3, 0, dst);
    w8(imm);
}

void
JITEmitter::xADC(RegIdx8 dst, RegIdx8 src, RegIdx16 addr)
{
    assert(dst != src);
    xLOAD(src, addr);
    xADC(dst, src);
}

void
JITEmitter::xADC(RegIdx8 dst, RegIdx8 src)
{
    w8(0x10); /* ADC r/m8,r8 */
    wModRM(dst, src);
}

void
JITEmitter::xSBC(RegIdx8 dst, RegIdx8 src, RegIdx16 addr)
{
    assert(dst != src);
    xLOAD(src, addr);
    xSBC(dst, src);
}

void
JITEmitter::xSBC(RegIdx8 dst, RegIdx8 src)
{
    /* Implemented as A + NOT(B) + C*/
    w8(0xF6);
    wModRM(3, 2, src);
    w8(0x10); /* ADC r/m8,r8 */
    wModRM(dst, src);
}

void
JITEmitter::xCLC(void)
{
    w8(0xF8); /* CLC */
}

void
JITEmitter::xCMC(void)
{
    w8(0xF5);
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
JITEmitter::xJMP(int8_t imm)
{
    w8(0xEB);
    w8(imm);
}

void
JITEmitter::xBR(enum Condition cc, RegIdx16 pc)
{
    /* jcc 6 */
    w8(0x0F);
    w8(0x80 + (cc ^ 1));
    w32(0x06);

    xSETPC(pc);
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
    xRETQ();
}

void
JITEmitter::xRETQ(void)
{
    w8(0xC3);
}



