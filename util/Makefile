#
#	Makefile for Allen-Bradley DH+ class.
#
CPU = MC68040

#FEATURES = -DMICROTIME
FEATURES =

## files and directories

VW		= /usr/vw_52/
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

SOURCES		= usrtime.c rtc.c microtime.c iptime.c auxtime.c getvec.c

OBJECTS		= iptime.o usrtime.o rtc.o microtime.o auxtime.o getvec.o

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

auxtime.o : $(INCLUDES) auxtime.c
	      $(CC) $(CFLAGS) auxtime.c

getvec.o : $(INCLUDES) getvec.c
	      $(CC) $(CFLAGS) getvec.c

utiltime.o: $(OBJECTS)
	$(LINK) $(LFLAGS) -o  utiltime.out \
	$(OBJECTS) \
	>$(DESTDIR)utiltime.map

utiltim.out: $(OBJECTS)
	$(LINK) $(LFLAGS) -o  utiltim.out \
	iptime.o usrtime.o rtc.o microtime.o \
	>$(DESTDIR)utiltim.map

