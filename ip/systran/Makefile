CPU = MC68040

HOST = irix

VW = $(VX_DSC_DIR)/
VWH = $(VW)h/
H1 = $(VWH)
H2 = $(VWH)net/
H3 = $(VWH)rpc/
H4 = $(VWH)68k/
H5 = $(VWH)private/

INCLUDES = -I$(H1) -I$(H2) -I$(H3) -I$(H4) -I$(H5)
CC = cc68k
CFLAGS = -g -ansi -O2 -pipe -fvolatile -m68040 -DCPU=$(CPU) -DVXWORKS -DMEI_VW \
-DMEI_MC680X0_CPU -D__USE_GETC__
#CFLAGS = -g -ansi -mc68040
#CFLAGS = -S -O -traditional -mc68040

CPP = cpp68k
#CPP = gcc-cpp  
CPPFLAGS = -P
#CPPFLAGS = -P -Caa

ASM = as68k
AFLAGS = -mc68040

LINK = ld68k                       
LFLAGS = -r -M
AR = ar68k
ARFLAGS = crsv

SRCS = dio316dc.c  dio316ie.c  dio316ri.c  dio316rr.c  dio316wr.c \
dio316ic.c  dio316oc.c  dio316rp.c  dio316wp.c \
da128vri.c  da128vsv.c da128vrr.c  da128vwr.c da128vrs.c \
ad12f1ri.c  ad12f1rr.c  ad12f1rv.c  ad12f1uc.c  ad12f1wr.c

ASRCS =

OBJS =dio316dc.o  dio316ie.o  dio316ri.o  dio316rr.o  dio316wr.o \
dio316ic.o  dio316oc.o  dio316rp.o  dio316wp.o \
da128vri.o  da128vsv.o da128vrr.o  da128vwr.o da128vrs.o \
ad12f1ri.o  ad12f1rr.o  ad12f1rv.o  ad12f1uc.o  ad12f1wr.o

.SUFFIXES: .o .c .asm .s

.c.o:
	$(CC) -DVXWORKS -D CPU=$(CPU) $(CFLAGS) $(INCLUDES) -c $*.c

.asm.s:
	$(CPP) $(CPPFLAGS) $*.asm $*.s

.s.o:
	$(ASM) $(AFLAGS) -o $*.o $*.s

all dg: $(ASRCS) $(OBJS)
	$(LINK) $(LFLAGS) -o dio316.out \
		dio316dc.o  dio316ie.o  dio316ri.o  dio316rr.o  dio316wr.o \
		dio316ic.o  dio316oc.o  dio316rp.o  dio316wp.o \
	>dio316.map

	$(LINK) $(LFLAGS) -o dac128v.out \
		da128vri.o  da128vsv.o da128vrr.o  da128vwr.o da128vrs.o \
	>dac128v.map

	$(LINK) $(LFLAGS) -o adc128f1.out \
		ad12f1ri.o  ad12f1rr.o  ad12f1rv.o  ad12f1uc.o  ad12f1wr.o \
	>adc128f1.map

install : all
	cp Makefile *.[cho] *.out $(MCP_DIR)/ip/systran

#
# Don't delete did48.out, we cannot (yet) recreate it
#
clean :
	/bin/rm -f *.o *~ *.map

