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

#include "core/bits.h"
#include "core/exception.h"
#include "emu/io.h"

using namespace Core;

namespace EMU {

class Machine;

struct DipswitchValueException : public CoreException {
  DipswitchValueException(const std::string &value = "")
      : CoreException("Unknown dipswitch value"), value(value) {
    if (value != "") msg += ": " + value;
  }
  std::string value;
};

/**
 * Describe a single setting.
 */
class Dipswitch {
 public:
  Dipswitch(const std::string &name, const std::string &port, byte_t mask,
            byte_t def);
  ~Dipswitch(void);

  void add_option(const std::string &name, byte_t value);

  void select(Machine *machine, const std::string &name);

  void set_default(Machine *machine);

 private:
  std::string _name;
  std::string _port;
  byte_t _mask;
  byte_t _def;
  std::map<std::string, byte_t> _options;
};

typedef std::shared_ptr<Dipswitch> dipswitch_ptr;
};
