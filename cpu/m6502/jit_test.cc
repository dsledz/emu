
#include "cpu/lib/jit.h"
#include "emu/test.h"

#include "cpu/m6502/m6502.h"

using namespace EMU;
using namespace EMUTest;

using namespace JITx64;
using namespace M6502;

typedef TestMachine<M6502Cpu, 0x0000> JITMachine;

TEST(JITTest, test) { JITMachine machine; }

TEST(JITTest, opcode_ea) {
  Core::log.set_level(LogLevel::Trace);
  JITMachine machine;

  LOAD1(0xEA);

  machine.cpu_step();
}

TEST(JITTest, opcode_a0) {
  JITMachine machine;

  LOAD2(0xA0, 0x10); /* LDY #$10 */

  EXPECT_EQ(0x00, machine.cpu.get_state()->Y);

  machine.cpu_step();

  EXPECT_EQ(0x10, machine.cpu.get_state()->Y);
}

TEST(JITTest, opcode_a2) {
  JITMachine machine;

  LOAD2(0xA2, 0x10);

  EXPECT_EQ(0x00, machine.cpu.get_state()->X);

  machine.cpu_step();

  EXPECT_EQ(0x10, machine.cpu.get_state()->X);
}

TEST(JITTest, opcode_a8) {
  JITMachine machine;

  LOAD2(0xA9, 0x10);
  LOAD1(0xA8);

  machine.cpu_step();
  EXPECT_EQ(0x10, machine.cpu.get_state()->A);
  EXPECT_EQ(0x10, machine.cpu.get_state()->Y);
}

TEST(JITTest, opcode_a9) {
  JITMachine machine;

  LOAD2(0xA9, 0x10);

  EXPECT_EQ(0x00, machine.cpu.get_state()->A);

  machine.cpu_step();

  EXPECT_EQ(0x10, machine.cpu.get_state()->A);
}

TEST(JITTest, opcode_a9b) {
  JITMachine machine;

  LOAD2(0xA9, 0x00); /* LDA #$00 */
  LOAD1(0x00);
  LOAD2(0xA9, 0x80); /* LDA #$00 */

  EXPECT_EQ(0x00, machine.cpu.get_state()->A);

  machine.cpu_step();
  EXPECT_EQ(0x00, machine.cpu.get_state()->A);
  EXPECT_EQ(0x00, machine.cpu.get_state()->F.N);
  EXPECT_EQ(0x01, machine.cpu.get_state()->F.Z);

  machine.cpu_step();
  EXPECT_EQ(0x80, machine.cpu.get_state()->A);
  EXPECT_EQ(0x01, machine.cpu.get_state()->F.N);
  EXPECT_EQ(0x00, machine.cpu.get_state()->F.Z);
}

TEST(JITTest, opcode_a5) {
  JITMachine machine;

  LOAD2(0xA5, 0x00); /* LDA zpg */

  machine.cpu_step();

  EXPECT_EQ(0xA5, machine.cpu.get_state()->A);
}

TEST(JITTest, opcode_8d) {
  JITMachine machine;

  LOAD2(0xA5, 0x00);       /* LDA zpg */
  LOAD3(0x8D, 0x10, 0x00); /* STA abs */

  machine.cpu_step();

  EXPECT_EQ(0xA5, machine.ram.read8(0x10));
}

TEST(JITTest, opcode_65) {
  JITMachine machine;

  LOAD2(0xA5, 0x00); /* LDA zpg */
  LOAD1(0x18);       /* CLC */
  LOAD2(0x65, 0x04); /* ADC zpg 0xA5 + 0x04 = 0xA9 */

  machine.cpu_step();

  EXPECT_EQ(0xA9, machine.cpu.get_state()->A);
}

TEST(JITTest, opcode_65b) {
  JITMachine machine;

  LOAD2(0xA5, 0x00); /* LDA zpg */
  LOAD1(0x18);       /* CLC */
  LOAD2(0x65, 0x04); /* ADC zpg 0xA5 + 0x04 = 0xA9 */
  LOAD2(0x65, 0x03); /* ADC zpg 0xA9 + 0x65 = 0x10e */
  LOAD2(0x65, 0x04); /* ADC zpg 0x0e + 0x04 + 0x01 = 0x13 */

  machine.cpu_step();

  EXPECT_EQ(0x13, machine.cpu.get_state()->A);
}

TEST(JITTest, opcode_2a) {
  JITMachine machine;

  LOAD2(0xA9, 0x80); /* LDA #$80 */
  LOAD1(0x2A);       /* ROLA */

  machine.cpu_step();
  EXPECT_EQ(0x01, machine.cpu.get_state()->F.C);
  EXPECT_EQ(0x01, machine.cpu.get_state()->F.Z);
  EXPECT_EQ(0x00, machine.cpu.get_state()->F.V);
}

TEST(JITTest, opcode_2ab) {
  JITMachine machine;

  LOAD1(0x38);       /* SEC */
  LOAD2(0xA9, 0x80); /* LDA #$80 */
  LOAD1(0x2A);       /* ROLA */

  machine.cpu_step();
  EXPECT_EQ(0x01, machine.cpu.get_state()->A);
  EXPECT_EQ(0x01, machine.cpu.get_state()->F.C);
  EXPECT_EQ(0x00, machine.cpu.get_state()->F.Z);
}

TEST(JITTest, opcode_e9) {
  JITMachine machine;

  LOAD1(0x38);       /* SEC */
  LOAD2(0xA9, 0x81); /* LDA #$81 */
  LOAD1(0x00);
  LOAD2(0xE9, 0x7F); /* SBC #$79 */

  machine.cpu_step();
  EXPECT_EQ(0x01, machine.cpu.get_state()->F.C);

  machine.cpu_step();
  EXPECT_EQ(0x02, machine.cpu.get_state()->A);
  EXPECT_EQ(0x01, machine.cpu.get_state()->F.V);
}

TEST(JITTest, opcode_e9b) {
  JITMachine machine;

  LOAD2(0xA9, 0x40); /* LDA #$40 */
  LOAD1(0x38);       /* SEC */
  LOAD2(0xE9, 0x40); /* SBC #$40 */

  machine.cpu_step();
  EXPECT_EQ(0x00, machine.cpu.get_state()->A);
  EXPECT_EQ(0x00, machine.cpu.get_state()->F.N);
}

TEST(JITTest, opcode_2e) {
  JITMachine machine;

  LOAD2(0xA5, 0x00);       /* LDA zpg */
  LOAD3(0x8D, 0x10, 0x00); /* STA abs */
  LOAD1(0x18);             /* CLC */
  LOAD3(0x2e, 0x10, 0x00); /* ROL abs */

  machine.cpu_step();

  EXPECT_EQ(0x4A, machine.ram.read8(0x10));
}

TEST(JITTest, opcode_66) {
  JITMachine machine;

  LOAD2(0xA5, 0x00);       /* LDA zpg */
  LOAD3(0x8D, 0x10, 0x00); /* STA abs */
  LOAD1(0x18);             /* CLC */
  LOAD2(0x66, 0x10);

  machine.cpu_step();

  EXPECT_EQ(0x52, machine.ram.read8(0x10));
}

TEST(JITTest, opcode_95) {
  JITMachine machine;
  LOAD2(0xA5, 0x00); /* LDA zpg */
  LOAD2(0xA2, 0x10); /* LDX #$10 */
  LOAD2(0x95, 0x00); /* STA $10 */

  machine.cpu_step();

  EXPECT_EQ(0xA5, machine.ram.read8(0x10));
}

TEST(JITTest, opcode_b5) {
  JITMachine machine;
  LOAD2(0xA2, 0x2);  /* LDX #$2 */
  LOAD2(0xB5, 0x00); /* LDA [X + $0] */

  machine.cpu_step();

  EXPECT_EQ(0x02, machine.cpu.get_state()->X);
  EXPECT_EQ(0xB5, machine.cpu.get_state()->A);
}

TEST(JITTest, opcode_bd) {
  JITMachine machine;
  LOAD2(0xA2, 0x1);        /* LDX #$10 */
  LOAD3(0xBD, 0x01, 0x00); /* LDA [X + $1] (abs,X) */

  machine.cpu_step();

  EXPECT_EQ(0xBD, machine.cpu.get_state()->A);
}

TEST(JITTest, opcode_f0) {
  JITMachine machine;
  LOAD2(0xA9, 0x01);       /* LDA #$01 */
  LOAD3(0xCD, 0x01, 0x00); /* CMP $01 */
  LOAD2(0xF0, 0xF9);       /* BEQ #-7 */

  machine.cpu_step();

  EXPECT_EQ(0x01, machine.cpu.get_state()->A);
  EXPECT_EQ(0x00, machine.cpu.get_state()->PC.d);
}

TEST(JITTest, opcode_f0b) {
  JITMachine machine;
  LOAD2(0xA9, 0x01);       /* LDA #$01 */
  LOAD3(0xCD, 0x00, 0x00); /* CMP $01 */
  LOAD2(0xF0, 0xF9);       /* BEQ #-7 */

  machine.cpu_step();

  EXPECT_EQ(0x01, machine.cpu.get_state()->A);
  EXPECT_EQ(0x07, machine.cpu.get_state()->PC.d);
}

TEST(JITTest, opcode_48) {
  JITMachine machine;

  LOAD2(0xA9, 0x88); /* LDA #$88 */
  LOAD1(0x48);       /* PHA */
  LOAD2(0xA9, 0xBB); /* LDA #$BB */
  LOAD1(0x68);       /* PLA */

  machine.cpu_step();

  EXPECT_EQ(0x88, machine.cpu.get_state()->A);
}

TEST(JITTest, opcode_08) {
  JITMachine machine;

  LOAD2(0xA9, 0x88); /* LDA #$88 */
  LOAD2(0xC9, 0x88); /* CMP #$88 */
  LOAD1(0x08);       /* PHP */
  LOAD1(0x68);       /* PHA */

  machine.cpu_step();

  EXPECT_EQ(0x03, machine.cpu.get_state()->A);
}

TEST(JITTest, opcode_78) {
  JITMachine machine;

  LOAD1(0x78); /* SEI */

  machine.cpu_step();

  EXPECT_EQ(1, machine.cpu.get_state()->F.I);
}

TEST(JITTest, opcode_60) {
  JITMachine machine;

  LOAD2(0xA9, 0x88); /* LDA #$88 */
  LOAD1(0x48);       /* PHA */
  LOAD1(0x48);       /* PHA */
  LOAD1(0x60);       /* RTS */

  machine.cpu_step();

  EXPECT_EQ(0x8889, machine.cpu.get_state()->PC.d);
}

TEST(JITTest, opcode_20) {
  JITMachine machine;

  LOAD2(0xA9, 0x88);       /* LDA #$88 */
  LOAD3(0x20, 0x10, 0x00); /* JSR #$0010 */
  LOAD2(0xA9, 0xBB);       /* LDA #$88 */
  LOAD3(0x00, 0x00, 0x00);
  LOAD3(0x00, 0x00, 0x00);
  LOAD3(0x00, 0x00, 0x00);
  LOAD1(0x60); /* RTS */
  LOAD1(0x00);

  machine.cpu_step();
  EXPECT_EQ(0x88, machine.cpu.get_state()->A);
  EXPECT_EQ(0x10, machine.cpu.get_state()->PC.d);
  EXPECT_EQ(0xFD, machine.cpu.get_state()->SP);
  EXPECT_EQ(0x00, machine.ram.read8(0x01FF));
  EXPECT_EQ(0x04, machine.ram.read8(0x01FE));

  machine.cpu_step();
  EXPECT_EQ(0x88, machine.cpu.get_state()->A);
  EXPECT_EQ(0x05, machine.cpu.get_state()->PC.d);

  machine.cpu_step();
  EXPECT_EQ(0x08, machine.cpu.get_state()->PC.d);
  EXPECT_EQ(0xBB, machine.cpu.get_state()->A);
}

TEST(JITTest, opcode_d0) {
  JITMachine machine;
  LOAD2(0xA9, 0x00); /* LDA #$01 */
  LOAD2(0xD0, 0x20); /* BNE #-7 */
  LOAD2(0xF0, 0x40); /* BEQ #-7 */

  machine.cpu_step();
  EXPECT_EQ(0x04, machine.cpu.get_state()->PC.d);
  EXPECT_EQ(0x01, machine.cpu.get_state()->F.Z);

  machine.cpu_step();
  EXPECT_EQ(0x46, machine.cpu.get_state()->PC.d);
}

TEST(JITTest, opcode_4a) {
  JITMachine machine;
  LOAD2(0xA9, 0x03); /* LDA #$01 */
  LOAD1(0x4A);       /* LSR A */
  LOAD1(0x4A);       /* LSR A */

  machine.cpu_step();
  EXPECT_EQ(0x00, machine.cpu.get_state()->A);
  EXPECT_EQ(0x01, machine.cpu.get_state()->F.C);
}

TEST(JITTest, opcode_b1) {
  JITMachine machine;
  LOAD2(0xA0, 0x02); /* LDY #$02 */
  LOAD2(0xB1, 0x05); /* LDA ind,Y */
  LOAD1(0x00);
  LOAD2(0xAA, 0xBB);

  machine.ram.write8(0xBBAA + 0x02, 0x55);

  machine.cpu_step();
  EXPECT_EQ(0x55, machine.cpu.get_state()->A);
}
