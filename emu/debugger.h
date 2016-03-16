/*
 * Copyright (c) 2016, Dan Sledz
 */
#pragma once

namespace EMU {

typedef std::list<std::pair<std::string, std::string> > debug_vars_t;

class Debuggable
{
public:
    Debuggable(const std::string &name);
    ~Debuggable();

    virtual std::list<std::string> list_variables(void) = 0;
    virtual std::string read_variable(const std::string &variable) = 0;
    virtual void write_variable(const std::string &variable,
                                const std::string &value) = 0;

    virtual debug_vars_t read_variables(void) = 0;

private:

    std::string m_name;
};

class Debugger
{
public:
    Debugger();
    ~Debugger();

private:

};

};
