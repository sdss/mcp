CPU = MC68020

HOST = hpux

DESTDIR = /users/briegel/vw/acnet/systran/
GENDIR = /users/briegel/vw/acnet/systran/
LIBDIR = /users/briegel/vw/acnet/systran
LIBSSM = /users/briegel/vw/acnet/ssm/
LIBUTILS = /users/briegel/vw/acnet/utils/
#VW = /usr/vw/
VW = /usr/vw_51/
VWH = $(VW)h/
H1 = $(VWH)
H2 = $(VWH)net/   
H3 = $(VWH)rpc/
H4 = $(VWH)68k/

INCLUDES = -I$(H1) -I$(H2) -I$(H3) -I$(H4) -I/usr/include -I$(GENDIR)
CC = cc68k
CFLAGS = -g -ansi -mc68020 -m68881
#CFLAGS = -g -traditional -mc68020 -m68881
#CFLAGS = -S -O -traditional -mc68020 -m68881

CPP = gcc-cpp
CPPFLAGS = -P
#CPPFLAGS = -P -Caa

ASM = as68k
AFLAGS = -mc68020

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

.c.o: $*.c
	$(CC) -DVXWORKS -D CPU=$(CPU) $(CFLAGS) $(INCLUDES) -c $*.c

.asm.s: $*.asm
	$(CPP) $(CPPFLAGS) $*.asm $*.s

.s.o: $*.s
	$(ASM) $(AFLAGS) -o $*.o $*.s

dg: $(ASRCS) $(OBJS)
	$(LINK) $(LFLAGS) -o dio316.out \
		dio316dc.o  dio316ie.o  dio316ri.o  dio316rr.o  dio316wr.o \
		dio316ic.o  dio316oc.o  dio316rp.o  dio316wp.o \
	>$(DESDIR)dio316.map
	$(LINK) $(LFLAGS) -o dac128v.out \
		da128vri.o  da128vsv.o da128vrr.o  da128vwr.o da128vrs.o \
	>$(DESDIR)dac128v.map
	$(LINK) $(LFLAGS) -o adc128f1.out \
		ad12f1ri.o  ad12f1rr.o  ad12f1rv.o  ad12f1uc.o  ad12f1wr.o \
	>$(DESDIR)adc128f1.map
