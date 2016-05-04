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
#include "cpu/m6502/m6502.h"
#include "cpu/m6502/m65c02.h"

namespace M6502v2
{

class HuC6280Cpu: public M65c02Cpu
{
public:
    HuC6280Cpu(Machine *machine, const std::string &name, unsigned clock,
              AddressBus21 *bus);
    virtual ~HuC6280Cpu(void);

    virtual void reset(void);
    virtual bool Interrupt(void);
    virtual void line(Line line, LineState state);
    virtual void execute(void);

    byte_t irq_read(offset_t offset);
    void irq_write(offset_t offset, byte_t value);

    byte_t timer_read(offset_t offset);
    void timer_write(offset_t offset, byte_t value);

private:

    void step(void);
    uint8_t mmu_read(offset_t offset);
    void mmu_write(offset_t offset, uint8_t value);

    AddressBus21 *m_data_bus;
    AddressBus16 m_mmu;
    uint8_t m_irq_status;
    uint8_t m_irq_disable;
    bool m_timer_status;
    int m_timer_load;
    int m_timer_value;
};

};
