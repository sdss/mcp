#
#	Makefile for AB
#
CPU = MC68040

HOST = irix

DESTDIR = ./
VW = /p/vx_dsc/v5_1_1f3/
VWH = $(VW)h/
H1 = $(VWH)
H2 = $(VWH)net/
H3 = $(VWH)rpc/
H4 = $(VWH)68k/
H5 = $(VWH)private/

INCLUDES = -I$(H1) -I$(H2) -I$(H3) -I$(H4) -I$(H5) -I$(DESTDIR) 
CC = cc68k

CFLAGS = -g -ansi -O2 -pipe -fvolatile -m68040 -DCPU=$(CPU) -DVXWORKS -DMEI_VW +
-DMEI_MC680X0_CPU -D__USE_GETC__

CPP = cpp68k
CPPFLAGS = -P

#VXCCFLAGS=-Wall -g -ansi -O2 -pipe -fvolatile -m68040 -DCPU=$(CPU) \
#	-DVXWORKS

#VXCC=cc68k $(VXCCFLAGS) $(INCLUDES)

LINK = ld68k
LFLAGS = -r -M
AR = ar68k
ARFLAGS = crsv

DHP_OBJS = std_dhp.o dhpd.o diag.o sbslc.o rdslc.o vmeinst.o

vmeinst.o : vmeinst.cc
	$(VXCC) -c vmeinst.cc

std_dhp.o : std_dhp.c 
	$(VXCC) -c std_dhp.c

dhpd.o : dhpd.c 
	$(VXCC) -c dhpd.c

diag.o : diag.c 
	$(VXCC) -c diag.c

sbslc.o : sbslc.c 
	$(VXCC) -c sbslc.c

rdslc.o : rdslc.c 
	$(VXCC) -c rdslc.c

dhp.out: $(ASRCS) $(OBJS) $(LIBRARIES)
	$(LINK) $(LFLAGS) -o  dhp.out \
	$(DHP_OBJS) \
	$(LIBRARIES) \
	>$(DESTDIR)dhp.map
#
#	This dependency cleans up the directories.
#
clean :
	rm -f *.dvi *.map *.ps *.o *.a *.aux *.log *.dat
