CPU = MC68040

HOST = irix

DESTDIR = ./
MEIDSP = ../mei-new/
IP = ../ip
SYSTRAN = ../ip/systran/
ACROMAG = ../ip/acromag/
AB = ../ab
VW = $(VX_DSC_DIR)/
#VW = /p/vx_dsc/v5_1_1f3/
#VW = /usr/vw_52/
VWH = $(VW)h/
H1 = $(VWH)
H2 = $(VWH)net/
H3 = $(VWH)rpc/
H4 = $(VWH)68k/
H5 = $(VWH)private/

INCLUDES = -I$(H1) -I$(H2) -I$(H3) -I$(H4) -I$(H5) -I$(MEIDSP) -I$(DESTDIR) \
	-I$(IP) -I$(SYSTRAN) -I$(AB) -I$(ACROMAG)
CC = cc68k
CFLAGS = -g -ansi -O2 -pipe -fvolatile -m68040 -DCPU=$(CPU) -DVXWORKS -DMEI_VW -Wall \
-DMEI_MC680X0_CPU -D__USE_GETC__
#CFLAGS = -g -ansi -mc68020 -m68881
#CFLAGS = -S -O -traditional -mc68020 -m68881

CPP = cpp68k
#CPP = gcc-cpp  
CPPFLAGS = -P
#CPPFLAGS = -P -Caa

LINK = ld68k                       
LFLAGS = -r -M

OBJS =cmd.o axis_cmds.o counter_weight.o serial.o util.o \
instrument_lift.o telescope_motion.o ipcast.o ipsym.o data_collection.o \
display.o umbilical.o telnetCmds.o

LIBRARIES = $(MEIDSP)/medvxw40.a

.SUFFIXES: .o .c .asm .s                                

#$(CC) $(SDSS_CFLAGS) -DMEI_VW -DMEI_MC680X0_CPU -D__USE_GETC__ $(INCLUDES) -c $*.c
.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $*.c

all dg : $(OBJS) $(LIBRARIES)
	$(LINK) $(LFLAGS) -o mcpnew.out $(OBJS) $(MEIDSP)llfirm.o \
	$(LIBRARIES) >$(DESTDIR)mcpnew.map

clean:
	rm -rf *.o mcp.map mcp.out *~

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
axis_cmds.o : copyright.h
axis_cmds.o : frame.h
axis_cmds.o : ms.h
axis_cmds.o : data_collection.h
axis_cmds.o : io.h
axis_cmds.o : tm.h
axis_cmds.o : axis.h
axis_cmds.o : cw.h
cmd.o : copyright.h
cmd.o : frame.h
cmd.o : ms.h
cmd.o : axis.h
cmd.o : cmd.h
counter_weight.o : copyright.h
counter_weight.o : cw.h
counter_weight.o : data_collection.h
counter_weight.o : io.h
data_collection.o : copyright.h
data_collection.o : data_collection.h
data_collection.o : tm.h
data_collection.o : io.h
data_collection.o : abdh.h
data_collection.o : ipcast.h
display.o : copyright.h
display.o : display.h
display.o : frame.h
display.o : ms.h
display.o : axis.h
display.o : tm.h
display.o : cw.h
display.o : io.h
display.o : data_collection.h
instrument_lift.o : copyright.h
instrument_lift.o : cw.h
instrument_lift.o : tm.h
instrument_lift.o : data_collection.h
instrument_lift.o : abdh.h
ipcast.o : copyright.h
ipsym.o : copyright.h
ipsym.o : timerint.h
serial.o : copyright.h
telescope_motion.o : copyright.h
telescope_motion.o : ms.h
telescope_motion.o : abdh.h
telescope_motion.o : axis.h
telescope_motion.o : frame.h
telescope_motion.o : data_collection.h
telescope_motion.o : tm.h
telnetCmds.o : axis.h
telnetCmds.o : copyright.h
telnetCmds.o : cmd.h
umbilical.o : copyright.h
util.o : mv162.h
util.o : vmechip2.h
