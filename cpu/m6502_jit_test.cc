
#include "emu/test.h"

#include "emu/jit.h"

using namespace EMU;
using namespace EMUTest;

using namespace JITx64;

#define RegA RegIdx8::RegAL
#define RegX RegIdx8::RegBL
#define RegY RegIdx8::RegCL
#define RegTmp RegIdx8::RegDL
#define RegEA RegIdx16::RegSI

extern "C" uint8_t
jit_bus_read(JITCpu *cpu, uint16_t addr);
extern "C" void
jit_bus_write(JITCpu *cpu, uint16_t addr, uint8_t value);

class JITTestCpu: public Cpu<AddressBus16> {
public:

    struct CpuState {
        uint8_t A;
        uint8_t X;
        uint8_t Y;
        uint8_t _reserved;
        reg16_t PC;
        reg16_t Flags;
    } __attribute__((packed));

    JITTestCpu(Machine *machine, const std::string &name, unsigned hertz,
           bus_type *bus):
        Cpu(machine, name, hertz, bus),
        _state(),
        _jit_state(this, jit_bus_read, jit_bus_write),
        _jit()
    {
    }
    virtual ~JITTestCpu(void)
    {
    }

    CpuState *state()
    {
        return &_state;
    }

protected:
    virtual Cycles step(void)
    {
        uint32_t pc = _state.PC.d;
        auto it = _cache.find(pc);
        if (it == _cache.end()) {
            JITBlock block = compile(pc);
            _cache.insert(std::make_pair(block.pc, block));
            it = _cache.find(pc);
        }
        jit_execute(&it->second);
        //_state.PC.d += it->second.len;
        return it->second.cycles;
    }
    virtual std::string dasm(addr_type addr)
    {
        return "";
    }

    void jit_execute(JITBlock *block)
    {
        uintptr_t func = reinterpret_cast<uintptr_t>(block->code());

        asm volatile(
            "mov %0, %%r15;\n"
            "mov %1, %%r14;\n"
            "movb 0(%%r15), %%al;\n"
            "movb 1(%%r15), %%bl;\n"
            "movb 2(%%r15), %%cl;\n"
            "push %%r15;\n"
            "pushw 6(%%r15);\n" /* flags */
            "popfw;\n"
            "mov 16(%%r14), %%rdi;\n"
            "callq *%2;\n"
            "pop %%r15;\n"
            "pushfw;\n"
            "popw 6(%%r15);\n"
            "movb %%al, 0(%%r15);\n"
            "movb %%bl, 1(%%r15);\n"
            "movb %%cl, 2(%%r15);\n"
            :
            : "r"(&_state), "r"(&_jit_state), "m"(func)
            : "rax", "rbx", "rcx"
        );
    }

    JITBlock compile(uint32_t orig_pc)
    {
        _jit.reset();
        bool done = false;
        int len = 0;
        while (!done) {
            uint16_t pc = orig_pc + len;
            uint8_t op = bus_read(pc);
            switch (op) {
            case 0x2e: {
                uint16_t tmp = bus_read(pc + 1) | (bus_read(pc + 2) << 8);
                _jit.xMOV16(RegEA, tmp);
                _jit.xRCL(RegEA);
                len += 3;
                break;
            }
            case 0x58: {
                _jit.xCLC();
                len += 1;
                break;
            }
            case 0x65: {
                uint8_t tmp = bus_read(pc + 1);
                _jit.xMOV16(RegEA, tmp);
                _jit.xADC(RegA, RegEA);
                len += 2;
                break;
            }
            case 0x66: {
                uint8_t tmp = bus_read(pc + 1);
                _jit.xMOV16(RegEA, tmp);
                _jit.xRCR(RegEA);
                len += 2;
                break;
            };
            case 0x8D: {
                uint16_t tmp = bus_read(pc + 1) | (bus_read(pc + 2) << 8);
                _jit.xMOV16(RegEA, tmp);
                _jit.xSTORE(RegEA, RegA);
                len += 3;
                break;
            }
            case 0x95: {
                uint16_t tmp = bus_read(pc + 1);
                _jit.xAbsolute(RegEA, RegX, tmp);
                _jit.xSTORE(RegEA, RegA);
                len += 2;
                break;
            };
            case 0xA0: {
                uint8_t tmp = bus_read(pc + 1);
                _jit.xMOV8(RegY, tmp);
                len += 2;
                break;
            }
            case 0xA2: {
                uint8_t tmp = bus_read(pc + 1);
                _jit.xMOV8(RegX, tmp);
                len += 2;
                break;
            }
            case 0xA5: {
                uint8_t tmp = bus_read(pc + 1);
                _jit.xMOV16(RegEA, tmp);
                _jit.xLOAD(RegA, RegEA);
                len += 2;
                break;
            }
            case 0xA8: {
                _jit.xMOV8(RegY, RegA);
                len += 1;
                break;
            }
            case 0xA9: {
                uint8_t tmp = bus_read(pc + 1);
                _jit.xMOV8(RegA, tmp);
                len += 2;
                break;
            }
            case 0xB5: {
                uint16_t tmp = bus_read(pc + 1);
                _jit.xAbsolute(RegEA, RegX, tmp);
                _jit.xLOAD(RegA, RegEA);
                len += 2;
                break;
            }
            case 0xBD: {
                uint16_t tmp = bus_read(pc + 1) | (bus_read(pc + 2) << 8);
                _jit.xAbsolute(RegEA, RegX, tmp);
                _jit.xLOAD(RegA, RegEA);
                len += 2;
                break;
            }
            case 0xCD: {
                uint16_t tmp = bus_read(pc + 1) | (bus_read(pc + 2) << 8);
                _jit.xMOV16(RegEA, tmp);
                _jit.xCMP(RegA, RegEA);
                len += 3;
                break;
            }
            case 0xEA: {
                len += 1;
                break;
            }
            case 0xF0: {
                _jit.xMOV16(RegEA, pc + 2);
                _jit.xSETPC(RegEA);
                int8_t tmp = bus_read(pc + 1);
                uint16_t addr = pc + 2 + tmp;
                _jit.xMOV16(RegEA, addr);
                _jit.xCMOV(RegEA, Condition::ZFClear);
                len += 2;
                done = true;
                break;
            }
            default:
                /* End the block */
                _jit.xMOV16(RegEA, pc);
                _jit.xSETPC(RegEA);
                done = true;
                break;
            }
        }
        _jit.xRETQ();
        return JITBlock(_jit.code(), orig_pc);
    }

private:

    CpuState _state;
    JITState _jit_state;
    JITEmitter _jit;
    std::unordered_map<uint32_t, JITBlock> _cache;
};

uint8_t
jit_bus_read(JITCpu *cpu, uint16_t addr)
{
    return cpu->bus_read(addr);
}

void
jit_bus_write(JITCpu *cpu, uint16_t addr, uint8_t value)
{
    cpu->bus_write(addr, value);
}

typedef TestMachine<JITTestCpu, 0x0000> JITMachine;

TEST(JITTest, test)
{
    JITMachine machine;
}

TEST(JITTest, opcode_ea)
{
    JITMachine machine;

    LOAD1(0xEA);

    machine.cpu.test_step();
}

TEST(JITTest, opcode_a0)
{
    JITMachine machine;

    LOAD2(0xA0, 0x10);

    EXPECT_EQ(0x00, machine.cpu.state()->Y);

    machine.cpu.test_step();

    EXPECT_EQ(0x10, machine.cpu.state()->Y);
}

TEST(JITTest, opcode_a2)
{
    JITMachine machine;

    LOAD2(0xA2, 0x10);

    EXPECT_EQ(0x00, machine.cpu.state()->X);

    machine.cpu.test_step();

    EXPECT_EQ(0x10, machine.cpu.state()->X);
}

TEST(JITTest, opcode_a8)
{
    JITMachine machine;

    LOAD2(0xA9, 0x10);
    LOAD1(0xA8);

    machine.cpu.test_step();
    EXPECT_EQ(0x10, machine.cpu.state()->A);
    EXPECT_EQ(0x10, machine.cpu.state()->Y);
}

TEST(JITTest, opcode_a9)
{
    JITMachine machine;

    LOAD2(0xA9, 0x10);

    EXPECT_EQ(0x00, machine.cpu.state()->A);

    machine.cpu.test_step();

    EXPECT_EQ(0x10, machine.cpu.state()->A);
}

TEST(JITTest, opcode_a5)
{
    JITMachine machine;

    LOAD2(0xA5, 0x00); /* LDA zpg */

    machine.cpu.test_step();

    EXPECT_EQ(0xA5, machine.cpu.state()->A);
}

TEST(JITTest, opcode_8D)
{
    JITMachine machine;

    LOAD2(0xA5, 0x00); /* LDA zpg */
    LOAD3(0x8D, 0x10, 0x00); /* STA abs */

    machine.cpu.test_step();

    EXPECT_EQ(0xA5, machine.ram.read8(0x10));
}

TEST(JITTest, opcode_65)
{
    JITMachine machine;

    LOAD2(0xA5, 0x00); /* LDA zpg */
    LOAD1(0x58);       /* CLC */
    LOAD2(0x65, 0x04); /* ADC zpg 0xA5 + 0x04 = 0xA9 */
    LOAD2(0x65, 0x03); /* ADC zpg 0xA9 + 0x65 = 0x10e */
    LOAD2(0x65, 0x04); /* ADC zpg 0x0e + 0x04 + 0x01 = 0x13 */

    machine.cpu.test_step();

    EXPECT_EQ(0x13, machine.cpu.state()->A);
}

TEST(JITTest, opcode_2e)
{
    JITMachine machine;

    LOAD2(0xA5, 0x00);       /* LDA zpg */
    LOAD3(0x8D, 0x10, 0x00); /* STA abs */
    LOAD1(0x58);             /* CLC */
    LOAD3(0x2e, 0x10, 0x00); /* ROL abs */

    machine.cpu.test_step();

    EXPECT_EQ(0x4A, machine.ram.read8(0x10));
}

TEST(JITTest, opcode_66)
{
    JITMachine machine;

    LOAD2(0xA5, 0x00);       /* LDA zpg */
    LOAD3(0x8D, 0x10, 0x00); /* STA abs */
    LOAD1(0x58);             /* CLC */
    LOAD2(0x66, 0x10);

    machine.cpu.test_step();

    EXPECT_EQ(0x52, machine.ram.read8(0x10));
}

TEST(JITTest, opcode_95)
{
    JITMachine machine;
    LOAD2(0xA5, 0x00);   /* LDA zpg */
    LOAD2(0xA2, 0x10);   /* LDX #$10 */
    LOAD2(0x95, 0x00);   /* STA $10 */

    machine.cpu.test_step();

    EXPECT_EQ(0xA5, machine.ram.read8(0x10));
}

TEST(JITTest, opcode_b5)
{
    JITMachine machine;
    LOAD2(0xA2, 0x2);    /* LDX #$2 */
    LOAD2(0xB5, 0x00);   /* LDA [X + $0] */

    machine.cpu.test_step();

    EXPECT_EQ(0x02, machine.cpu.state()->X);
    EXPECT_EQ(0xB5, machine.cpu.state()->A);
}

TEST(JITTest, opcode_bd)
{
    JITMachine machine;
    LOAD2(0xA2, 0x1);   /* LDX #$10 */
    LOAD3(0xBD, 0x01, 0x00);  /* LDA [X + $1] (abs,X) */

    machine.cpu.test_step();

    EXPECT_EQ(0xBD, machine.cpu.state()->A);
}

TEST(JITTest, opcode_f0)
{
    JITMachine machine;
    LOAD2(0xA9, 0x01);        /* LDA #$01 */
    LOAD3(0xCD, 0x01, 0x00);  /* CMP #$01 */
    LOAD2(0xF0, 0xF9);        /* BEQ #-7 */

    machine.cpu.test_step();

    EXPECT_EQ(0x01, machine.cpu.state()->A);
}
