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

LINK = ld68k                       
LFLAGS = -r -M

OBJS =dio316dc.o  dio316ie.o  dio316ri.o  dio316rr.o  dio316wr.o \
dio316ic.o  dio316oc.o  dio316rp.o  dio316wp.o \
da128vri.o  da128vsv.o da128vrr.o  da128vwr.o da128vrs.o \
ad12f1ri.o  ad12f1rr.o  ad12f1rv.o  ad12f1uc.o  ad12f1wr.o

.SUFFIXES: .o .c

.c.o:
	$(CC) -DVXWORKS -D CPU=$(CPU) $(CFLAGS) $(INCLUDES) -c $*.c

all : dio316.out dac128v.out adc128f1.out

dio316.out : $(OBJS)
	$(LINK) $(LFLAGS) -o dio316.out \
		dio316dc.o  dio316ie.o  dio316ri.o  dio316rr.o  dio316wr.o \
		dio316ic.o  dio316oc.o  dio316rp.o  dio316wp.o \
	>dio316.map

dac128v.out : $(OBJS)
	$(LINK) $(LFLAGS) -o dac128v.out \
		da128vri.o  da128vsv.o da128vrr.o  da128vwr.o da128vrs.o \
	>dac128v.map

adc128f1.out : $(OBJS)
	$(LINK) $(LFLAGS) -o adc128f1.out \
		ad12f1ri.o  ad12f1rr.o  ad12f1rv.o  ad12f1uc.o  ad12f1wr.o \
	>adc128f1.map

install : all
	cp Makefile *.[cho] *.out $(MCP_DIR)/ip/systran

#
# Don't delete did48.out, we cannot (yet) recreate it
#
clean :
	/bin/rm -f *.o *~ *.map dio316.out dac128v.out adc128f1.out

