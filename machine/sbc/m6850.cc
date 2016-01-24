/*
 * Copyright (c) 2013, Dan Sledz
 */

#include "emu/emu.h"
#include "machine/sbc/m6850.h"

using namespace EMU;
using namespace Device;

M6850::M6850(Machine *machine, const std::string &name, unsigned hertz):
    ClockedDevice(machine, name, hertz),
    m_reg(),
    m_send(0),
    m_recv(0),
    m_in(),
    m_out()
{
}

M6850::~M6850(void)
{
}

void
M6850::reset(void)
{
    memset(&m_reg, 0, sizeof(m_reg));
}

void
M6850::execute(void)
{
    while (true) {
        bool interrupt = false;

        /* Update our clock */
        switch (m_reg.CR & 0x03) {
        case 0x00: add_icycles(1); break;
        case 0x01: add_icycles(16); break;
        case 0x02: add_icycles(64); break;
        case 0x03: /* XXX: How should we handle reset? */ break;
        }

        if (m_send > 0) {
            /* Consume one of our bits */
            m_send--;
            if (m_send == 0) {
                std::cout << (char)m_reg.TDR;
                std::cout.flush();
                m_reg.SR |= 0x02; /* Transmit Data Register Empty */
                if ((m_reg.CR & 0x60) == 0x20)
                    interrupt = true;
            }
        } else if (m_recv > 0) {
            m_recv--;
            if (m_recv == 0) {
                m_reg.SR |= 0x01; /* Receive Data Register Full */
                if (m_reg.CR & 0x80) {
                    m_reg.SR |= 0x80;
                    interrupt = true;
                }
            }
        }
        if (interrupt) {
            /* XXX: Can yield */
            machine()->set_line("cpu", Line::INT0, LineState::Pulse);
            /* XXX: Return from the context switch */
        }
    }
}

void
M6850::write(bool rs, byte_t value)
{
    if (rs) {
        /* Write */
        m_out.push_back(value);
        m_reg.TDR = value;
        m_reg.SR &= ~0x02;
        /* XXX: Honor CR2-CR4 for number of bits */
        m_send = 10;
    } else {
        m_reg.CR = value;
        /* XXX: Honor CR2-CR4 */
    }
}

byte_t
M6850::read(bool rs)
{
    byte_t value = '<';
    if (rs) {
        /* Read */
        value = m_reg.RDR = m_in.front();
        m_in.pop_front();
        m_reg.SR &= ~0x01;
        /* Restart send */
        if (!m_in.empty())
            /* XXX: Honor CR2-CR4 for number of bits */
            m_recv = 10;
    } else {
        /* Status */
        value = m_reg.SR;
    }
    return value;
}
