#
#	Makefile for AB
#
CPU = MC68040

HOST = irix

VW = $(VX_DSC_DIR)/
#VW = /p/vx_dsc/v5_1_1f3/
VWH = $(VW)h/
H1 = $(VWH)
H2 = $(VWH)net/
H3 = $(VWH)rpc/
H4 = $(VWH)68k/
H5 = $(VWH)private/

INCLUDES = -I$(H1) -I$(H2) -I$(H3) -I$(H4) -I$(H5)
CC = cc68k

CFLAGS = -g -ansi -O2 -pipe -fvolatile -m68040 -DCPU=$(CPU) -DVXWORKS $(INCLUDES)

CPP = cpp68k
CPPFLAGS = -P

#VXCCFLAGS=-Wall -g -ansi -O2 -pipe -fvolatile -m68040 -DCPU=$(CPU) \
#	-DVXWORKS

#VXCC=cc68k $(VXCCFLAGS) $(INCLUDES)

LINK = ld68k
LFLAGS = -r -M
AR = ar68k
ARFLAGS = crsv

DHP_SRC = std_dhp.c dhpd.c diag.c sbslc.c rdslc.c vmeinst.c
DHP_OBJS = std_dhp.o dhpd.o diag.o sbslc.o rdslc.o vmeinst.o

.SUFFIXES: .o .c .asm .s                                

.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $*.c

.asm.s:
	$(CPP) $(CPPFLAGS) $*.asm $*.s

.s.o:
	$(ASM) $(AFLAGS) -o $*.o $*.s

dg: $(ASRCS) $(DHP_OBJS)
	$(LINK) $(LFLAGS) -o  dhp.out \
	$(DHP_OBJS) \
	>$(DESTDIR)dhp.map

clean:
	rm -rf *.o
