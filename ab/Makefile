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

CFLAGS = -ansi -O -pipe -fvolatile -m68040 -DCPU=$(CPU) -DVXWORKS $(INCLUDES)

LINK = ld68k
LFLAGS = -r -M

DHP_OBJS = std_dhp.o dhpd.o diag.o sbslc.o rdslc.o vmeinst.o

.SUFFIXES: .o .c

.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $*.c

all : dhp.out

dhp.out : $(DHP_OBJS)
	$(LINK) $(LFLAGS) -o  dhp.out $(DHP_OBJS) > $(DESTDIR)dhp.map

install :
	cp Makefile *.[cho] *.out *.bin $(MCP_DIR)/ab

#
# Don't delete sddhp.bin sdhdhp.bin sdudhp.bin; we don't have source for them
#
clean:
	rm -rf *.o *.out *.map
