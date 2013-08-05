
#include "gtest/gtest.h"

#include "emu/emu.h"

using namespace EMU;

static void
add_switches(Machine *machine)
{
    dipswitch_ptr sw;

    machine->add_ioport("DSWA");
    machine->add_ioport("DSWB");

    sw = machine->add_switch("Difficulty", "DSWA", 0x03, 0x0);
    sw->add_option("Easy", 0x03);
    sw->add_option("Medium", 0x00);
    sw->add_option("Hard", 0x01);
    sw->add_option("Hardest", 0x02);

    sw = machine->add_switch("Demo Sounds", "DSWA", 0x08, 0x08);
    sw->add_option("Off", 0x00);
    sw->add_option("On", 0x08);

    sw = machine->add_switch("Pause", "DSWA", 0x10, 0x10);
    sw->add_option("Off", 0x10);
    sw->add_option("On", 0x00);

    sw = machine->add_switch("Rack Test", "DSWA", 0x20, 0x20);
    sw->add_option("Off", 0x20);
    sw->add_option("On", 0x00);

    sw = machine->add_switch("Cabinet", "DSWA", 0x80, 0x80);
    sw->add_option("Upright", 0x80);
    sw->add_option("Cocktail", 0x00);

    sw = machine->add_switch("Coinage", "DSWB", 0x07, 0x07);
    sw->add_option("4 Coins / 1 Credit", 0x04);
    sw->add_option("3 Coins / 1 Credit", 0x02);
    sw->add_option("2 Coins / 1 Credit", 0x06);
    sw->add_option("1 Coin  / 1 Credit", 0x07);
    sw->add_option("2 Coins / 3 Credit", 0x01);
    sw->add_option("1 Coin  / 2 Credit", 0x03);
    sw->add_option("1 Coin  / 3 Credit", 0x05);
    sw->add_option("Free Play", 0x00);

    sw = machine->add_switch("Bonus Life", "DSWB", 0x38, 0x10);
    sw->add_option("None", 0x00);
    sw->add_option("30K, 100K, Every 100K", 0x08);
    sw->add_option("20, 70K, Every 70K", 0x10);
    sw->add_option("20K and 60K Only", 0x18);
    sw->add_option("20K, 60K, Every 60K", 0x20);
    sw->add_option("30K, 120K, Every 120K", 0x28);
    sw->add_option("20K, 80K, Every 80K", 0x30);
    sw->add_option("30K and 80K Only", 0x38);

    sw = machine->add_switch("Lives", "DSWB", 0xC0, 0x80);
    sw->add_option("2", 0x00);
    sw->add_option("3", 0x80);
    sw->add_option("4", 0x40);
    sw->add_option("5", 0xC0);
}

TEST(DipswitchTest, declare)
{
    Machine machine;

    add_switches(&machine);

    machine.reset_switches();
}

TEST(DipswitchTest, set)
{
    Machine machine;

    add_switches(&machine);

    EXPECT_THROW(machine.set_switch("Random", "Value"), EmuException);

    machine.set_switch("Lives", "5");

    byte_t actual = machine.read_ioport("DSWB") & 0xC0;
    EXPECT_EQ(0xC0, actual);
}

