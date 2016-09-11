/*
 * Copyright (c) 2016, Dan Sledz
 */
#pragma once

#include "core/bits.h"
#include "emu/debug.h"

namespace EMU {

typedef std::pair<std::string, std::string> debug_var_t;

typedef std::list<debug_var_t> debug_vars_t;

class Debuggable {
 public:
  Debuggable(const std::string &name) : m_name(name) {}
  ~Debuggable() {}

  const std::string &name() const { return m_name; }

  const std::list<std::string> &list_registers(void) { return m_register_list; }

  std::string read_register(const std::string &reg) {
    auto it = m_registers.find(reg);
    if (it == m_registers.end()) return "Unknown";
    return it->second.read();
  }

  void write_register(const std::string &reg, const std::string &value) {
    auto it = m_registers.find(reg);
    if (it == m_registers.end()) return;
    it->second.write(value);
  }

  debug_vars_t read_registers(void) {
    debug_vars_t result;
    for (auto it = m_registers.begin(); it != m_registers.end(); it++) {
      result.push_back(std::make_pair(it->first, it->second.read()));
    }
    result.sort([&](const debug_var_t &v1, const debug_var_t &v2) -> bool {
      return v1.first < v2.first;
    });
    return result;
  }

 protected:
  void add_register(const DebugVariable &reg) {
    m_register_list.push_back(reg.name());
    m_registers.insert(std::make_pair(reg.name(), reg));
  }

  void add_debug_var(const std::string &name, uint8_t &reg) {
    DebugVariable debug_reg(
        name, [&](void) -> std::string { return std::to_string(reg); },
        [&](const std::string &value) {
          reg = static_cast<uint8_t>(stoi(value));
        });
    add_register(debug_reg);
  }

  void add_debug_var(const std::string &name, reg16_t &reg) {
    DebugVariable debug_reg(
        name, [&](void) -> std::string { return std::to_string(reg.d); },
        [&](const std::string &value) {
          reg.d = static_cast<uint16_t>(stoi(value));
        });
    add_register(debug_reg);
  }

 private:
  std::string m_name;
  std::list<std::string> m_register_list;
  std::unordered_map<std::string, DebugVariable> m_registers;
};

class Debugger {
 public:
  Debugger() : m_devices() {}
  ~Debugger() {}

  void add_debuggable(Debuggable *debuggable) {
    m_devices.insert(make_pair(debuggable->name(), debuggable));
  }

  void remove_debuggable(Debuggable *debuggable) {
    m_devices.erase(debuggable->name());
  }

  Debuggable *get_debuggable(const std::string &dev) {
    return m_devices.at(dev);
  }

 private:
  std::unordered_map<std::string, Debuggable *> m_devices;
};
};
