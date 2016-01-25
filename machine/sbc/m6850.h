/*
 * Copyright (c) 2013, Dan Sledz
 */
#pragma once

#include <deque>

#include "emu/emu.h"

using namespace EMU;

namespace Device {

class M6850: public ClockedDevice
{
public:
    M6850(Machine *machine, const std::string &name, unsigned hertz);
    virtual ~M6850(void);

    virtual void execute(void);
    virtual void reset(void);

    byte_t read(bool rs);
    void write(bool rs, byte_t value);

private:

    struct RegisterFile {
        byte_t TDR;
        byte_t RDR;
        byte_t CR;
        byte_t SR;
    } m_reg;

    int m_send;
    int m_recv;
    std::deque<byte_t> m_in;
    std::deque<byte_t> m_out;

public:

    bool RTS(void) {
        return (m_reg.CR & 0x60) == 0x40;
    }

    std::pair<bool, byte_t> debug_read(void) {
        if (m_out.begin() != m_out.end()) {
            byte_t out = m_out.front();
            m_out.pop_front();
            return std::make_pair(true, out);
        } else {
            return std::make_pair(false, 0);
        }
    }

    void debug_write(byte_t value) {
        if (m_recv == 0 && m_in.empty())
            m_recv = 10;
        m_in.push_back(value);
    }

    void debug_write(const std::string &cmd) {
        if (m_recv == 0 && m_in.empty())
            m_recv = 10;
        for (auto it = cmd.begin(); it != cmd.end(); it++)
            m_in.push_back(*it);
    }
};

typedef std::unique_ptr<M6850> M6850_ptr;
}

