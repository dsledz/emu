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

namespace Z80v2 {

enum Z80Arg {
    ArgRegB  = 0,
    ArgRegC  = 1,
    ArgRegD  = 2,
    ArgRegE  = 3,
    ArgRegH  = 4,
    ArgRegL  = 5,
    ArgRegHL = 6,
    ArgRegA  = 7
};

struct Z80State {
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
                reg8_t l;
            };
            reg8_t h;
        } b;
        uint16_t d;
    } AF;
    reg16_t BC;
    reg16_t DE;
    reg16_t HL;
    reg16_t IX;
    reg16_t IY;
    reg16_t SP;
    reg16_t PC;

    reg8_t I;
    reg8_t R;

    reg16_t AF2;
    reg16_t BC2;
    reg16_t DE2;
    reg16_t HL2;

    reg16_t EA;
} __attribute__((packed));

class Z80Cpu;

struct Z80Opcode
{
    uint8_t code;
    const char *name;
    int bytes;
    int cycles;
    std::function<void (Z80Cpu *, Z80State *)> operation;
};

class Z80Cpu: public Cpu<AddressBus16>
{
public:
    Z80Cpu(Machine *machine, const std::string &name, unsigned hertz,
           bus_type *bus);
    ~Z80Cpu(void);
    Z80Cpu(const Z80Cpu &cpu) = delete;

    virtual void line(Line line, LineState state);
    virtual void test_step(void);
    virtual Cycles step(void);
    virtual std::string dasm(addr_type addr);

    Z80State *get_state(void) {
        return &_state;
    }

    reg8_t fetch8(Z80Reg r);

    reg16_t fetch16(Z80Reg r);

private:
    Z80State _state;

    std::unordered_map<uint8_t, Z80Opcode> _opcodes;
};
