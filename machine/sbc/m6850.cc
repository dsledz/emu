/*
 * Copyright (c) 2013, Dan Sledz
 */

#include "machine/sbc/m6850.h"
#include "emu/emu.h"

using namespace EMU;
using namespace Device;

M6850::M6850(Machine *machine, const std::string &name, unsigned hertz)
    : ClockedDevice(machine, name, hertz),
      m_reg(),
      m_send(0),
      m_recv(0),
      m_in(),
      m_out() {
  m_reg.SR = 0x02;
}

M6850::~M6850(void) {}

void M6850::reset(void) {
  memset(&m_reg, 0, sizeof(m_reg));
  m_reg.SR = 0x02;
}

void M6850::execute(void) {
  while (true) {
    bool interrupt = false;

    /* Update our clock */
    switch (m_reg.CR & 0x03) {
      case 0x00:
        add_icycles(1);
        break;
      case 0x01:
        add_icycles(16);
        break;
      case 0x02:
        add_icycles(64);
        break;
      case 0x03: /* XXX: How should we handle reset? */
        break;
    }

    if (m_send > 0) {
      /* Consume one of our bits */
      m_send--;
      if (m_send == 0) {
        LOG_DEBUG(m_reg.TDR);
        m_reg.SR |= 0x02; /* Transmit Data Register Empty */
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
      machine()->set_line("cpu", Line::INT0, LineState::Assert);
      /* XXX: Return from the context switch */
    }
    Task::yield();
  }
}

void M6850::write(bool rs, byte_t value) {
  if (rs) {
    /* Write */
    m_out.push_back(value);
    m_reg.TDR = value;
    m_reg.SR &= ~0x82;
    /* XXX: Honor CR2-CR4 for number of bits */
    m_send = 10;
    machine()->set_line("cpu", Line::INT0, LineState::Clear);
  } else {
    m_reg.CR = value;
    /* XXX: Honor CR2-CR4 */
    if (m_reg.CR & 0x80 && !m_in.empty() && m_recv == 0) m_recv = 10;
  }
}

byte_t M6850::read(bool rs) {
  byte_t value = '<';
  if (rs) {
    /* Read */
    value = m_reg.RDR = m_in.front();
    m_in.pop_front();
    m_reg.SR &= ~0x81;
    /* Restart send */
    if (!m_in.empty()) /* XXX: Honor CR2-CR4 for number of bits */
      m_recv = 10;
    machine()->set_line("cpu", Line::INT0, LineState::Clear);
  } else {
    /* Status */
    value = m_reg.SR;
  }
  return value;
}
