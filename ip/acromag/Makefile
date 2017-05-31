# This is a makefile for compiling library Routines for the
# ip480 and the example routines to demonstrate them.

.c.o:
	$(CC) -c $(CFLAGS) $*.c

CC = cc68k
CFLAGS = -ansi -O2 -m68040 -DVXWORKS -DCPU=$(CPU) -I$(VX_DSC_DIR)/h
CPU = MC68040

OBJECTS = rmid480.o drvr480.o carrier.o cnfg480.o vxworks.o

all : ip480.out

ip480.out : $(OBJECTS)
	ld68k -r -M -o ip480.out $(OBJECTS) > ip480.map

install : all
	cp Makefile *.[cho] *.out $(MCP_DIR)/ip/acromag

clean:
	/bin/rm -f *.out *.o *.map *~
