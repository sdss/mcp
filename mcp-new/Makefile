CPU = MC68040

HOST = irix

DESTDIR = ./
MEIDSP = ../mei-new/
IP = ../ip
SYSTRAN = ../ip/systran/
ACROMAG = ../ip/acromag/
UTIL = ../util
AB = ../ab
VW = $(VX_DSC_DIR)/
VWH = $(VW)h/
H1 = $(VWH)
H2 = $(VWH)net/
H3 = $(VWH)rpc/
H4 = $(VWH)68k/
H5 = $(VWH)private/

INCLUDES = -I$(H1) -I$(H2) -I$(H3) -I$(H4) -I$(H5) -I$(MEIDSP) -I$(DESTDIR) \
	-I$(IP) -I$(SYSTRAN) -I$(AB) -I$(ACROMAG) -I$(UTIL) \
	-I$(VX_TOOLS_DIR)/include -I$(ASTRODA_DIR)/include \
	-I$(MURMUR_CLIENT_VX_DIR)/include

CFLAGS = $(SDSS_CFLAGS) -DMEI_VW -DMEI_MC680X0_CPU -D__USE_GETC__

OBJS =cmd.o axis_cmds.o pvt.o counter_weight.o serial.o util.o \
instrument_lift.o telescope_motion.o ipcast.o ipsym.o data_collection.o \
display.o umbilical.o telnetCmds.o timers.o fiducials.o spectro.o tbars.o \
tagname.o brakes.o

LIBRARIES = $(MEIDSP)/medvxw40.a

.SUFFIXES: .o .c .asm .s                                

#$(CC) $(SDSS_CFLAGS) -DMEI_VW -DMEI_MC680X0_CPU -D__USE_GETC__ $(INCLUDES) -c $*.c
.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $*.c

all : mcpnew.out
#
# N.b. we rebuild tagname.o to get the current date/time in version strings
#
mcpnew.out : $(OBJS) $(LIBRARIES)
	$(CC) $(CFLAGS) $(INCLUDES) -c tagname.c
	$(LD) -o mcpnew.out $(OBJS) $(MEIDSP)llfirm.o \
	$(LIBRARIES)

install : all
	cp Makefile *.[cho] *.out $(MCP_DIR)/mcp-new

clean:
	rm -rf *.o mcp.out *~

make :
	$(SDSSTOOLS_DIR)/bin/make_make \
		-cc '$(CC) $(CFLAGS)' -nostd -nofix -file Makefile *.c
	../bin/fix-make_make Makefile
#
# include file dependencies.
#
# All line below START_DEPEND are machine generated; Do Not Edit
#
#START_DEPEND
cmd.o : copyright.h
data_collection.o : copyright.h
display.o : copyright.h
display.o : display.h
serial.o : copyright.h
