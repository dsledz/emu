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

#include <cassert>
#include "core/bits.h"
#include "core/enum.h"

namespace EMU {

enum class PageFlags  {
    None = 0x00,
    Write = 0x01,
    Read = 0x02,
};

struct PageException: public CoreException {
    PageException(off_t off ):
        CoreException("Page Fault: "), offset(off)
    {
        std::stringstream ss;
        ss << Hex(off);
        msg += ss.str();
    }
    off_t offset;
};

template<typename _addr_type, typename _data_type>
void NoopWrite(_addr_type offset, _data_type data)
{
}

template<typename _addr_type, typename _data_type>
_data_type ReadWrite(_addr_type offset)
{
    return _data_type();
}

template<typename _addr_type, typename _data_type, unsigned _page_width>
class Page {
public:
    Page(void):m_page(),m_flags({PageFlags::Write, PageFlags::Read})
    {
    }
    Page(std::initializer_list<PageFlags> flags):m_page(), m_flags(flags)
    {
    }

    ~Page(void)
    {
    }

    void write(_addr_type offset, _data_type data)
    {
        if (unlikely(m_flags.is_clear(PageFlags::Write)))
            throw PageException(offset);
        m_page[offset] = data;
    }

    _data_type read(_addr_type offset)
    {
        return m_page[offset];
    }

    _data_type m_page[_page_width];
    BitField<PageFlags> m_flags;
};

};
