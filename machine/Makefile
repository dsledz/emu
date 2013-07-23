# Machine Library

SRCS=
SRCS+= galaga.cc namco51.cc namco06.cc
OBJS+= galaga.o namco51.o namco06.o
SRCS+= gb.cc gbgfx.cc gbmbc.cc
OBJS+= gb.o gbgfx.o gbmbc.o
SRCS+= nes.cc nes_mapper.cc nes_ppu.cc
OBJS+= nes.o nes_mapper.o nes_ppu.o
SRCS+= emulator.cc
OBJS+= emulator.o
LIBRARY= machine
CPPFLAGS+= -I../emu -I../cpu
LIBS+= -L${OBJS_DIR} -lemu -lcpu
CHECK_OBJS= galaga_test.cc gb_test.cc

include ../Makefile.inc
