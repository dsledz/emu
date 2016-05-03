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
/*
 * Utilities and helpers for Just in time compilation of instructions.
 */
#pragma once

#include <vector>

template<typename T>
class ExecPolicy {
public:
    //    typedefs
    typedef T value_type;
    typedef value_type* pointer;
    typedef const value_type* const_pointer;
    typedef value_type& reference;
    typedef const value_type& const_reference;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;

public:
    template<typename U>
    struct rebind {
        typedef ExecPolicy<U> other;
    };

public:

    //    memory allocation
    inline pointer allocate(size_type cnt,
          typename std::allocator<void>::const_pointer = 0) {
#ifdef WIN32
		return reinterpret_cast<T*>(VirtualAllocEx(
			GetCurrentProcess(),
			NULL,
			sizeof(T) * cnt,
			MEM_COMMIT,
			PAGE_EXECUTE_READWRITE)
			);
#else
        return reinterpret_cast<T*>(mmap(0, sizeof(T) * cnt,
            PROT_READ|PROT_WRITE|PROT_EXEC, MAP_ANON|MAP_PRIVATE, -1, 0));
#endif
    }
    inline void deallocate(pointer p, size_type cnt) {
#ifdef WIN32
		VirtualFreeEx(GetCurrentProcess(),
			p,
			sizeof(T) * cnt,
			MEM_DECOMMIT);
#else
        munmap(p, sizeof(T) * cnt);
#endif
    }

    //    size
    inline size_type max_size() const {
        return (std::numeric_limits<size_type>::max)() / sizeof(T);
    }

private:
};    //    end of class ExecPolicy

//    determines if memory from another allocator
//    can be deallocated from this one
template<typename T>
inline bool operator==(ExecPolicy<T> const&, ExecPolicy<T> const&) {
    return true;
}
template<typename T, typename T2>
inline bool operator==(ExecPolicy<T> const&, ExecPolicy<T2> const&) {
    return false;
}
template<typename T, typename OtherAllocator>
inline bool operator==(ExecPolicy<T> const&, OtherAllocator const&) {
    return false;
}

#ifdef WIN32
typedef std::vector<uint8_t> exec_buf_t;
#else
typedef std::vector<uint8_t, ExecPolicy<uint8_t> > exec_buf_t;
#endif


