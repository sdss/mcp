.c.o :
	$(CC) -c $(CFLAGS) $*.c

CPU = MC68040

VW = $(VX_DSC_DIR)/
VWINC		= $(VW)h/
CONFIG		= $(VW)/config/mv162/
VWSYSINC	= $(VWINC)drv/
LIBFILE		= usrtime.a

PROGINCLUDES	= usrtime.h

VWINCLUDES	= $(VWINC)types.h $(VWINC)vxWorks.h $(VWINC)sysLib.h \
		  $(VWINC)string.h $(VWINC)limits.h

VWSYSINCLUDES	= $(CONFIG)mv162.h $(VWINC)drv/multi/mcchip.h

INCLUDES	= $(PROGINCLUDES) $(VWINCLUDES) $(VWSYSINCLUDES)

OBJECTS		= iptime.o usrtime.o rtc.o microtime.o
EXTRA_OBJS      = dscTrace.o loadtime.o timer.o tasks.o

## tools
CC		= cc68k
LD		= ld68k
RM		= rm -f
AR		= ar68k
RANLIB		= ranlib68k
LINK = ld68k

## flags

INCLUDEFLAGS	= -I. -I$(VWINC) -I$(CONFIG) -I$(VWSYSINC) \
			-I$(VX_TOOLS_DIR)/include \
			-I$(ASTRODA_DIR)/include \
                        -I$(MURMUR_CLIENT_VX_DIR)/include
DEBUGFLAG	=
CFLAGS		= -O -ansi -pipe -Wall $(DEBUGFLAG) $(INCLUDEFLAGS) \
		  -DCPU=MC68040 -m68040 $(FEATURES)
LFLAGS = -r


## actions

all default : $(LIBFILE) $(EXTRA_OBJS)

$(LIBFILE): $(OBJECTS)
	$(RM) $(LIBFILE)
	$(AR) cru $(LIBFILE) $(OBJECTS)
	$(RANLIB) $(LIBFILE)
	$(LINK) $(LFLAGS) -o utiltim.out $(OBJECTS)

install : all
	cp Makefile *.[cho] *.out $(MCP_DIR)/util

clean	:
	  $(RM) $.o *~


## dependencies

usrtime.o : $(INCLUDES) usrtime.c
rtc.o   : $(INCLUDES) rtc.c
microtime.o : $(INCLUDES) microtime.c
iptime.o : $(INCLUDES) iptime.c
timer.o : $(INCLUDES) timer.c
