CPU = MC68040

HOST = irix

DESTDIR = ./
MEIDSP = ../mei-new/
IP = ../ip
SYSTRAN = ../ip/systran/
ACROMAG = ../ip/acromag/
AB = ../ab
VW = $(VX_DSC_DIR)/
VWH = $(VW)h/
H1 = $(VWH)
H2 = $(VWH)net/
H3 = $(VWH)rpc/
H4 = $(VWH)68k/
H5 = $(VWH)private/

INCLUDES = -I$(H1) -I$(H2) -I$(H3) -I$(H4) -I$(H5) -I$(MEIDSP) -I$(DESTDIR) \
	-I$(IP) -I$(SYSTRAN) -I$(AB) -I$(ACROMAG) \
	-I$(VX_TOOLS_DIR)/include -I$(ASTRODA_DIR)/include \
	-I$(MURMUR_CLIENT_VX_DIR)/include

CC = cc68k
CFLAGS = -g -ansi -fvolatile -m68040 -DCPU=$(CPU) -DVXWORKS -DMEI_VW -Wall \
	-DMEI_MC680X0_CPU -D__USE_GETC__

LINK = ld68k                       
LFLAGS = -r

OBJS =cmd.o axis_cmds.o counter_weight.o serial.o util.o \
instrument_lift.o telescope_motion.o ipcast.o ipsym.o data_collection.o \
display.o umbilical.o telnetCmds.o timers.o fiducials.o spectro.o tbars.o \
tagname.o

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
	$(LINK) $(LFLAGS) -o mcpnew.out $(OBJS) $(MEIDSP)llfirm.o \
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
axis_cmds.o : copyright.h
axis_cmds.o : frame.h
axis_cmds.o : ms.h
axis_cmds.o : data_collection.h
axis_cmds.o : io.h
axis_cmds.o : tm.h
axis_cmds.o : axis.h
axis_cmds.o : cw.h
axis_cmds.o : serial.h
axis_cmds.o : mcpUtils.h
axis_cmds.o : mcpMsgQ.h
axis_cmds.o : abdh.h
axis_cmds.o : mcpTimers.h
axis_cmds.o : mcpFiducials.h
axis_cmds.o : cmd.h
cmd.o : copyright.h
cmd.o : frame.h
cmd.o : ms.h
cmd.o : axis.h
cmd.o : cmd.h
cmd.o : mcpFiducials.h
cmd.o : mcpUtils.h
counter_weight.o : cw.h
counter_weight.o : copyright.h
counter_weight.o : data_collection.h
counter_weight.o : instruments.h
counter_weight.o : io.h
counter_weight.o : mcpUtils.h
counter_weight.o : mcpMsgQ.h
counter_weight.o : mcpTimers.h
counter_weight.o : axis.h
counter_weight.o : cmd.h
data_collection.o : copyright.h
data_collection.o : data_collection.h
data_collection.o : tm.h
data_collection.o : io.h
data_collection.o : abdh.h
data_collection.o : ipcast.h
data_collection.o : axis.h
data_collection.o : cw.h
data_collection.o : instruments.h
data_collection.o : mcpFiducials.h
display.o : copyright.h
display.o : display.h
display.o : frame.h
display.o : ms.h
display.o : axis.h
display.o : tm.h
display.o : cw.h
display.o : io.h
display.o : data_collection.h
display.o : instruments.h
display.o : mcpUtils.h
display.o : mcpFiducials.h
fiducials.o : tm.h
fiducials.o : mcpMsgQ.h
fiducials.o : mcpFiducials.h
fiducials.o : mcpTimers.h
fiducials.o : mcpUtils.h
fiducials.o : axis.h
fiducials.o : data_collection.h
fiducials.o : serial.h
fiducials.o : cmd.h
instrument_lift.o : cw.h
instrument_lift.o : copyright.h
instrument_lift.o : tm.h
instrument_lift.o : data_collection.h
instrument_lift.o : abdh.h
instrument_lift.o : instruments.h
instrument_lift.o : axis.h
instrument_lift.o : display.h
ipcast.o : copyright.h
ipsym.o : copyright.h
ipsym.o : timerint.h
ipsym.o : mcpUtils.h
serial.o : copyright.h
serial.o : serial.h
serial.o : cmd.h
spectro.o : abdh.h
spectro.o : frame.h
spectro.o : copyright.h
spectro.o : data_collection.h
spectro.o : tm.h
spectro.o : axis.h
spectro.o : cmd.h
spectro.o : mcpMsgQ.h
tagname.o : axis.h
telescope_motion.o : ms.h
telescope_motion.o : copyright.h
telescope_motion.o : abdh.h
telescope_motion.o : axis.h
telescope_motion.o : frame.h
telescope_motion.o : data_collection.h
telescope_motion.o : tm.h
telescope_motion.o : cw.h
telescope_motion.o : instruments.h
telescope_motion.o : mcpFiducials.h
telnetCmds.o : axis.h
telnetCmds.o : cmd.h
timers.o : mcpTimers.h
timers.o : mcpUtils.h
timers.o : data_collection.h
timers.o : axis.h
timers.o : mcpMsgQ.h
umbilical.o : copyright.h
umbilical.o : instruments.h
util.o : mv162.h
util.o : vmechip2.h
util.o : mcpUtils.h
