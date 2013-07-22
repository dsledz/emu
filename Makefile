# Sample Makefile for OS X
export

CC= clang++
DEBUG_FLAGS= -g
CPPFLAGS= -Wall -Werror \
	  -std=c++11 -stdlib=libc++ \
	  -DGTEST_USE_OWN_TR1_TUPLE=1 -I${GTEST_INCLUDE} \
	  ${CFLAGS} ${FLAGS} ${DEBUG_FLAGS} -I${CURDIR}/emu
LIBS= -L${OBJS_DIR}

OBJS_DIR=${CURDIR}/objs

SUBDIRS= emu cpu machine sdl

all: ${SUBDIRS} check

ALL_LIBS=emu.a cpu.a machine.a sdl.a wrapper.a

${OBJS_DIR}:
	mkdir -p ${OBJS_DIR}

# Clean
SUBDIRS_CLEAN = $(patsubst %,%.clean,$(SUBDIRS))
.PHONY:: clean $(SUBDIRS_CLEAN)
clean: ${SUBDIRS_CLEAN}
	rm -rf *.o *.a .depend ${OBJS_DIR}
$(SUBDIRS_CLEAN):
	@make -C $(@:.clean=) clean

# Check
GTEST_LIB= ${CURDIR}/libgtest.a
GTEST_DIR= ${CURDIR}/../gtest-1.6.0
GTEST_INCLUDE= ${GTEST_DIR}/include
${GTEST_LIB}:
	${CC} ${CPPFLAGS} -I${GTEST_INCLUDE} -I${GTEST_DIR} -c ${GTEST_DIR}/src/gtest-all.cc
	ar -rv ${GTEST_LIB} gtest-all.o
SUBDIRS_CHECK = $(patsubst %,%.check,$(SUBDIRS))
.PHONY:: check $(SUBDIRS_CHECK)
check: ${SUBDIRS_CHECK}
$(SUBDIRS_CHECK): ${GTEST_LIB}
	@make -C $(@:.check=) check

.PHONY:: ${SUBDIRS}
${SUBDIRS}: ${OBJS_DIR}
	${MAKE} -C $@

