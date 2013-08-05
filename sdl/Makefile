# SDL library
export
SDL_DIR= ../../SDL
CPPFLAGS+= -I${SDL_DIR}/include
LIBS+= -L${SDL_DIR}/lib -lSDLmain -lSDL -framework OpenGL -framework Cocoa

SRCS= sdl_main.cc sdl_input.cc sdl_gfx.cc
OBJS= sdl_main.o sdl_input.o sdl_gfx.o
LIBRARY= sdl_driver

PROGRAM=driver

LIBS+= -L${OBJS_DIR} -lemu -lcpu -lmachine -lsdl_driver

driver: ${OBJS_DIR}/libsdl_driver.a \
	${OBJS_DIR}/libcpu.a \
	${OBJS_DIR}/libemu.a \
	${OBJS_DIR}/libmachine.a
	${CC} ${CPPFLAGS} ${LIBS} -o $@

include ../Makefile.inc
