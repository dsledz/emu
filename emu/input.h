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
#include <unordered_map>

#include "core/bits.h"
#include "core/exception.h"

#include "emu/io.h"

using namespace Core;

namespace EMU {
/**
 * Input signal
 */
enum class InputKey {
  Service = 0,
  Coin1 = 1,
  Coin2 = 2,
  Start1 = 3,
  Start2 = 4,
  Select1 = 5,
  Select2 = 6,
  Joy1Up = 10,
  Joy1Down = 11,
  Joy1Left = 12,
  Joy1Right = 13,
  Joy1Btn1 = 14,
  Joy1Btn2 = 15,
  Joy1Btn3 = 16,
  Joy1Btn4 = 17,
  Joy1Btn5 = 18,
  Joy1Btn6 = 19,
  Joy2Up = 20,
  Joy2Down = 21,
  Joy2Left = 22,
  Joy2Right = 23,
  Joy2Btn1 = 24,
  Joy2Btn2 = 25,
  Joy2Btn3 = 26,
  Joy2Btn4 = 27,
  Joy2Btn5 = 28,
  Joy2Btn6 = 29,
};

struct InputError : public CoreException {
  InputError(InputKey key) : CoreException("Input error"), key(key) {}
  InputKey key;
};

struct DuplicateInput : public InputError {
  DuplicateInput(InputKey key) : InputError(key) {}
};

struct UnmappedInput : public InputError {
  UnmappedInput(InputKey key) : InputError(key) {}
};

typedef std::function<void(LineState state)> input_fn;

struct InputKeyHash {
  size_t operator()(const InputKey &key) const {
    auto n = static_cast<typename std::underlying_type<InputKey>::type>(key);
    return std::hash<int>()(n);
  }
};

struct InputSignal {
  InputSignal(InputKey key, IOPort *port, int bit, bool active_high)
      : key(key), port(port), bit(bit), active_high(active_high) {}

  InputKey key;
  IOPort *port;
  int bit;
  bool active_high;
};

/**
 * Class to manage input (mostly controller) mapping.
 */
class InputMap {
 public:
  InputMap(void);
  ~InputMap(void);

  void add(const InputSignal &signal);

  /**
   * Simulate triggering an input.
   */
  void depress(InputKey in);

  /**
   * Release the input @a in.
   */
  void release(InputKey in);

  /**
   * Pulse the input @a in.
   */
  void pulse(InputKey in);

 private:
  input_fn _find(InputKey in) {
    auto it = _input_map.find(in);
    if (it == _input_map.end()) throw UnmappedInput(in);
    return it->second;
  }

  void add_input(InputKey in, input_fn fn);

  std::unordered_map<InputKey, input_fn, InputKeyHash> _input_map;
};

};
