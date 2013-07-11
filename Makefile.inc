# Basic Makefile

DIR?=${CURDIR}
OBJS_DIR?=${CURDIR}

.cc.o:
	cd ${CURDIR} && ${CC} ${CPPFLAGS} -c ${DIR}/$<

TARGET_LIB=${OBJS_DIR}/lib${LIBRARY}.a

${TARGET_LIB}: ${OBJS}
	ar -rv ${TARGET_LIB} ${OBJS}

all: ${TARGET_LIB} ${PROGRAM}

check: ${GTEST_LIB} ${CHECK_OBJS} ${TARGET_LIB}
	${CC} ${CPPFLAGS} -c ${GTEST_DIR}/src/gtest_main.cc
	${CC} ${CPPFLAGS} ${GTEST_LIB} ${CHECK_OBJS} \
            -L${OBJS_DIR} ${LIBS} ${TARGET_LIB} gtest_main.o -o check

clean:
	rm -rf *.o *.a .depend

depend: .depend

.depend: $(SRCS)
	rm -f ${DIR}/.depend
	$(CC) $(CPPFLAGS) -MM $^ > ${DIR}/.depend;

-include .depend

