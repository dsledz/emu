/*
 * Copyright (c) 2013, Dan Sledz
 * All rights reserved.
 */

#pragma once

#include "core/bits.h"

template<typename obj_type, int maximum>
class RingBuffer {
    public:
        RingBuffer(void): m_front(0), m_back(0), m_used(0), m_objects() {}
        ~RingBuffer(void) {}

        void push_back(const obj_type & obj) {
            m_objects[m_back] = obj;
            m_back = (m_back + 1) % maximum;
            m_used++;
            assert(m_back != m_front);
        }

        void pop_front(void) {
            assert(m_front != m_back);
            m_front = (m_front + 1) % maximum;
            m_used--;
        }

        const obj_type & front(void) {
            assert(m_front != m_back);
            return m_objects[m_front];
        }

        int size(void) {
            return m_used;
        }

        bool empty(void) {
            return m_used == 0;
        }

    private:
        int m_front;
        int m_back;
        int m_used;
        obj_type m_objects[maximum];
};

