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
VW = $(VXWORKS_DIR)/
#VW = /p/vx_dsc/v5_1_1f3/
#VW = /usr/vw_52/
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

SOURCES		= usrtime.c rtc.c microtime.c iptime.c timer.c

OBJECTS		= iptime.o usrtime.o rtc.o microtime.o timer.o

ALLFILES	= $(MAKEFILE) $(INCLUDES) $(SOURCES)


## tools
CC		= cc68k
AS		= as68k
LD		= ld68k
RM		= rm -f
PRINT		= pr
AR		= ar68k
RANLIB		= ranlib68k
LINK = ld68k


## flags

INCLUDEFLAGS	= -I. -I$(VWINC) -I$(CONFIG) -I$(VWSYSINC)
DEBUGFLAG	=
CFLAGS		= -c -O -ansi -pipe -Wall $(DEBUGFLAG) $(INCLUDEFLAGS) \
		  -DCPU=MC68040 -m68040 $(FEATURES)
CASFLAGS	= -E $(INCLUDEFLAGS) -DCPU=MC68040
LDFLAGS		= -d -r
LIBS		=
LFLAGS = -r -M


## actions

default : $(LIBFILE)

$(LIBFILE): $(OBJECTS)
	    $(RM) $(LIBFILE)
	    $(AR) cru $(LIBFILE) $(OBJECTS)
	    $(RANLIB) $(LIBFILE)
	$(LINK) $(LFLAGS) -o  utiltim.out \
	iptime.o usrtime.o rtc.o microtime.o \
	>$(DESTDIR)utiltim.map

print	: 
	  $(PRINT) $(MAKEFILE) $(PROGINCLUDES) $(SOURCES) | lp -opr

clean	:
	  $(RM) *.o


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

