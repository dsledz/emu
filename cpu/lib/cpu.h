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
/**
 * Cpu Abstraction.
 */
#pragma once

#include "cpu/lib/jit.h"
#include "emu/device.h"

using namespace EMU;
using namespace JITx64;

namespace CPU {

/**
 * CPU Fault
 */
struct CpuFault : public DeviceFault {
  CpuFault(const std::string &cpu, const std::string &msg)
      : DeviceFault(cpu, msg) {}
};

/**
 * CPU Opcode error
 */
struct CpuOpcodeFault : public CpuFault {
  CpuOpcodeFault(const std::string &cpu, unsigned opcode, unsigned addr)
      : CpuFault(cpu, "invalid opcode"), op(opcode), pc(addr) {
    std::stringstream ss;
    ss << " " << Hex(opcode);
    if (addr != 0) ss << " at address " << Hex(addr);
    msg += ss.str();
  }
  unsigned op;
  unsigned pc;
};

struct CpuRegisterFault : public CpuFault {
  CpuRegisterFault(const std::string &cpu, unsigned index)
      : CpuFault(cpu, "invalid regsiter") {
    std::stringstream ss;
    ss << " " << Hex(index);
    msg += ss.str();
  }
};

struct CpuFeatureFault : public CpuFault {
  CpuFeatureFault(const std::string &cpu, const std::string &feature = "")
      : CpuFault(cpu, "unsupported feature") {
    std::stringstream ss;
    if (feature != "") {
      ss << " " << feature;
      msg += ss.str();
    }
  }
};

template <typename _addr_width, typename _data_width>
struct CpuTraits {
  typedef _addr_width pc_type;
  typedef _addr_width addr_type;
  typedef _data_width data_type;
};

template <class _bus_type, class _cpu_traits, class _state_type,
          typename _opcode_type>
class Cpu : public ClockedDevice {
 public:
  typedef _bus_type bus_type;
  typedef _state_type state_type;
  typedef _opcode_type opcode_type;
  typedef _cpu_traits cpu_traits;
  typedef typename cpu_traits::addr_type pc_type;
  typedef typename cpu_traits::addr_type addr_type;
  typedef typename cpu_traits::data_type data_type;

  struct Opcode {
    opcode_type code;
    const char *name;
    int bytes;
    int cycles;
    std::function<void(state_type *)> addr_mode;
    std::function<void(state_type *)> operation;
    std::function<void(JITEmitter *, std::function<addr_type(addr_type)>,
                       addr_type)>
        jit_address;
    std::function<bool(JITEmitter *, pc_type)> jit_op;
  };

  Cpu(Machine *machine, const std::string &name, unsigned hertz, bus_type *bus)
      : ClockedDevice(machine, name, hertz), m_state() {
    m_state.bus = bus;
  }
  virtual ~Cpu(void) {}
  Cpu(const Cpu &cpu) = delete;

  virtual void execute(void) = 0;

  virtual void line(Line line, LineState state) { Device::line(line, state); }

  state_type *state(void) { return &m_state; }

  data_type bus_read(addr_type addr) {
    data_type tmp = m_state.bus_read(addr);
    return tmp;
  }

  void bus_write(addr_type addr, data_type value) {
    m_state.bus_write(addr, value);
  }

  virtual std::string dasm(addr_type addr) { return "NOP"; }

  virtual void log_op(state_type *state, const Opcode *op, uint16_t pc,
                      const uint8_t *instr) {
    std::stringstream os;
    os << Hex(pc) << ":" << Hex(op->code) << ":" << op->name;
    DEVICE_TRACE(os.str());
  }

 protected:
  unsigned dispatch(pc_type pc) {
    unsigned cycles;
    opcode_type opcode = m_state.bus_read(m_state.PC.d++);
    auto it = m_opcodes.find(opcode);
    if (it == m_opcodes.end()) {
      DEVICE_ERROR("Unknown opcode: ", Hex(opcode), " at ", Hex(pc));
      throw CpuOpcodeFault(name(), opcode, pc);
    }

    Opcode *op = &it->second;

    m_state.icycles = 0;
    uint8_t instr[3] = {};

    op->addr_mode(&m_state);
    op->operation(&m_state);
    cycles = (op->cycles + m_state.icycles) * m_state.clock_divider;
    add_icycles(cycles);

    IF_LOG(Trace) { log_op(&m_state, op, pc, instr); }
    return cycles;
  }

  static data_type jit_bus_read(void *ctx, addr_type addr) {
    auto *cpu =
        static_cast<Cpu<bus_type, cpu_traits, state_type, opcode_type> *>(ctx);
    return cpu->m_state.bus_read(addr);
  }

  static void jit_bus_write(void *ctx, addr_type addr, data_type value) {
    auto *cpu =
        static_cast<Cpu<bus_type, cpu_traits, state_type, opcode_type> *>(ctx);
    cpu->m_state.bus_write(addr, value);
  }

  void jit_dispatch(pc_type pc) {
    using namespace std::placeholders;
    JITBlock *block = NULL;

    try {
      auto it = m_jit_cache.find(pc);
      if (it == m_jit_cache.end()) {
        jit_block_ptr b = jit_compile(pc);
        block = b.get();
        m_jit_cache.insert(std::make_pair(pc, std::move(b)));
      } else if (!it->second->valid(std::bind(jit_bus_read, this, _1))) {
        m_jit_cache.erase(it);
        jit_block_ptr b = jit_compile(pc);
        block = b.get();
        m_jit_cache.insert(std::make_pair(pc, std::move(b)));
      } else {
        block = it->second.get();
      }
    } catch (JITError &e) {
      block = NULL;
    }

    if (block == NULL) {
      dispatch(pc);
      return;
    }

    bit_set(m_state.NativeFlags.d, Flags::CF, m_state.F.C);
    bit_set(m_state.NativeFlags.d, Flags::ZF, m_state.F.Z);
    bit_set(m_state.NativeFlags.d, Flags::OF, m_state.F.V);
    bit_set(m_state.NativeFlags.d, Flags::SF, m_state.F.N);

    JITState jit_state(reinterpret_cast<uintptr_t>(this), jit_bus_read,
                       jit_bus_write);

    block->execute(reinterpret_cast<uintptr_t>(&m_state),
                   reinterpret_cast<uintptr_t>(&jit_state));

    // Update our flags
    m_state.F.C = bit_isset(m_state.NativeFlags.d, Flags::CF);
    m_state.F.Z = bit_isset(m_state.NativeFlags.d, Flags::ZF);
    m_state.F.V = bit_isset(m_state.NativeFlags.d, Flags::OF);
    m_state.F.N = bit_isset(m_state.NativeFlags.d, Flags::SF);

    // Account for the extra cycles if we branched
    add_icycles(block->cycles * m_state.clock_divider);
    if (m_state.PC.d != (block->pc + block->len))
      add_icycles(2 * m_state.clock_divider);
  }

  jit_block_ptr jit_compile(pc_type start_pc) {
    using namespace std::placeholders;
    JITEmitter jit;
    bool done = false;
    uint32_t len = 0;
    uint32_t cycles = 0;
    auto br = std::bind(&jit_bus_read, this, _1);
    bvec source;

    while (!done) {
      pc_type pc = start_pc + len;
      opcode_type code = bus_read(pc);

      auto it = m_opcodes.find(code);
      if (it == m_opcodes.end()) {
        std::cout << "Unknown opcode: " << Hex(code) << std::endl;
        throw CpuOpcodeFault(name(), code, pc);
      }
      Opcode *op = &it->second;

      for (unsigned i = 0; i < op->bytes; i++) {
        source.push_back(bus_read(pc + i));
      }

      len += op->bytes;
      cycles += op->cycles;
      op->jit_address(&jit, br, pc);
      done = !op->jit_op(&jit, pc);
    }

    jit.xMOV16(RegIdx16::RegDX, start_pc + len);
    jit.xSETPC(RegIdx16::RegDX);
    return jit_block_ptr(
        new JITBlock(jit.code(), start_pc, source, len, cycles));
  }

  state_type m_state;
  std::unordered_map<opcode_type, Opcode> m_opcodes;
  std::unordered_map<pc_type, jit_block_ptr> m_jit_cache;
};
};
