#
#	Makefile for Allen-Bradley DH+ class.
#
CPU = MC68040

HOST = irix

DESTDIR = ./
MEIDSP = ../mei-new/
IP = ../ip
SYSTRAN = ../ip/systran/
ACROMAG = ../ip/acromag/
AB = ../ab
VW = $(VX_DSC_DIR)/
VWINC		= $(VW)h/
CONFIG		= $(VW)/config/mv162/
VWSYSINC	= $(VWINC)drv/
MAKEFILE	= Makefile
LIBFILE		= usrtime.a

PROGINCLUDES	= usrtime.h

VWINCLUDES	= $(VWINC)types.h $(VWINC)vxWorks.h $(VWINC)sysLib.h \
		  $(VWINC)string.h $(VWINC)limits.h

VWSYSINCLUDES	= $(CONFIG)mv162.h $(VWINC)drv/multi/mcchip.h

INCLUDES	= $(PROGINCLUDES) $(VWINCLUDES) $(VWSYSINCLUDES)

OBJECTS		= iptime.o usrtime.o rtc.o microtime.o timer.o
EXTRA_OBJS      = dscTrace.o

## tools
CC		= cc68k
LD		= ld68k
RM		= rm -f
AR		= ar68k
RANLIB		= ranlib68k
LINK = ld68k

## flags

INCLUDEFLAGS	= -I. -I$(VWINC) -I$(CONFIG) -I$(VWSYSINC) \
			-I$(VX_TOOLS_DIR)/include
DEBUGFLAG	=
CFLAGS		= -c -O -ansi -pipe -Wall $(DEBUGFLAG) $(INCLUDEFLAGS) \
		  -DCPU=MC68040 -m68040 $(FEATURES)
LIBS		=
LFLAGS = -r


## actions

all default : $(LIBFILE) $(EXTRA_OBJS)

$(LIBFILE): $(OBJECTS)
	    $(RM) $(LIBFILE)
	    $(AR) cru $(LIBFILE) $(OBJECTS)
	    $(RANLIB) $(LIBFILE)
	$(LINK) $(LFLAGS) -o  utiltim.out \
	iptime.o usrtime.o rtc.o microtime.o

install : all
	cp Makefile *.[cho] *.out $(MCP_DIR)/util

clean	:
	  $(RM) $.o *~


## dependencies

usrtime.o : $(INCLUDES) usrtime.c
	    $(CC) $(CFLAGS) usrtime.c

rtc.o   : $(INCLUDES) rtc.c
	  $(CC) $(CFLAGS) rtc.c

microtime.o : $(INCLUDES) microtime.c
	      $(CC) $(CFLAGS) microtime.c

iptime.o : $(INCLUDES) iptime.c
	      $(CC) $(CFLAGS) iptime.c

timer.o : $(INCLUDES) timer.c
	      $(CC) $(CFLAGS) timer.c

