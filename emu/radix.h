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
/**
 * Simple (incomplete) radix tree implementation.
 * Used for address map.
 */
#pragma once

#include "bits.h"
#include "exception.h"

#define MASK(width) ((1 << width) - 1)

template<class Val, int width>
class RadixTree {
private:
    struct Leaf {
        Leaf(void): key(MASK(width)), mask(0x0000), value(Val()) { }
        Leaf(addr_t key, addr_t mask, const Val &value):
            key(key), mask(mask), value(value) { }

        bool match(addr_t value) {
            return ((mask & value) == key);
        }

        addr_t  key;
        addr_t  mask;
        Val     value;
    };
    struct Inner {
        Inner(Leaf *leaf):
            depth(0), leaf(leaf), zero(NULL), one(NULL) { }
        Inner(int depth, addr_t key):
            depth(depth), key(key), leaf(NULL), zero(NULL), one(NULL) { }
        ~Inner(void) {
            if (leaf != NULL)
                delete leaf;
            if (zero != NULL)
                delete zero;
            if (one != NULL)
                delete one;
        }

        bool match(addr_t value) {
            for (int d = width - 1; d > depth; d--)
                if (bit_isset(value, d) != bit_isset(key, d))
                    return false;
            return true;
        }

        int      depth;
        addr_t   key;
        Leaf    *leaf;
        Inner   *zero;
        Inner   *one;
    };

public:
    RadixTree(void): _head(NULL) {
        _head = new Inner(width - 1, 0);
    }
    ~RadixTree(void) {
        if (_head != NULL)
            delete _head;
        _head = NULL;
    }

    Val &find(addr_t key) {
        Inner *ele = _head;
        while (ele != NULL && ele->leaf == NULL) {
            if (bit_isset(key, ele->depth))
                ele = ele->one;
            else
                ele = ele->zero;
        }
        if (ele != NULL && ele->leaf != NULL && ele->leaf->match(key))
            return ele->leaf->value;
        else
            throw EMU::BusError(key);
    }

    void add(addr_t key, addr_t keymask, const Val &val) {
        Inner **ele = &_head;
        int depth = (*ele)->depth;
        addr_t mask = 0;
        // Locate the Correct inner object for the element
        while (bit_isset(keymask, depth)) {
            mask |= bit_isset(key, depth) << depth;
            if (*ele == NULL)
                *ele = new Inner(depth, mask);
            else if ((*ele)->leaf != NULL)
                throw EMU::BusError(key);
            if (bit_isset(key, depth))
                ele = &(*ele)->one;
            else
                ele = &(*ele)->zero;
            depth--;
        }
        *ele = new Inner(new Leaf(key, keymask, val));
    }

    void add(addr_t key, const Val &val) {
        add(key, MASK(width), val);
    }

private:
    Inner *_head;
};


