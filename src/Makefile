CPU = MC68040

HOST = irix

DESTDIR = ./
MEIDSP = ../mei-src/
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
	-I$(VX_TOOLS_DIR)/include -I$(MURMUR_CLIENT_VX_DIR)/include

CFLAGS = $(SDSS_CFLAGS) -DMEI_VW -DMEI_MC680X0_CPU -D__USE_GETC__

OBJS =cmd.o axis_cmds.o pvt.o counter_weight.o serial.o util.o \
instrument_lift.o telescope_motion.o ipcast.o ipsym.o data_collection.o \
instruments.o telnetCmds.o timers.o fiducials.o spectro.o tbars.o \
tagname.o brakes.o as2.o dictionary.o

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
	cp Makefile *.[cho] *.out $(MCP_DIR)/src

clean:
	rm -rf *.o mcp.out *~

make :
	../bin/make_make -f Makefile *.c
	../bin/fix-make_make Makefile
#
# include file dependencies.
#
# All line below START_DEPEND are machine generated; Do Not Edit
#
#START_DEPEND
as2.o : as2.h
as2.o : cmd.h
as2.o : mcpMsgQ.h
axis_cmds.o : abdh.h
axis_cmds.o : as2.h
axis_cmds.o : axis.h
axis_cmds.o : cmd.h
axis_cmds.o : cw.h
axis_cmds.o : data_collection.h
axis_cmds.o : frame.h
axis_cmds.o : instruments.h
axis_cmds.o : io.h
axis_cmds.o : mcpFiducials.h
axis_cmds.o : mcpMsgQ.h
axis_cmds.o : mcpTimers.h
axis_cmds.o : mcpUtils.h
axis_cmds.o : ms.h
axis_cmds.o : serial.h
axis_cmds.o : tm.h
brakes.o : abdh.h
brakes.o : axis.h
brakes.o : cmd.h
brakes.o : data_collection.h
brakes.o : mcpMsgQ.h
brakes.o : tm.h
cmd.o : as2.h
cmd.o : axis.h
cmd.o : cmd.h
cmd.o : copyright.h
cmd.o : frame.h
cmd.o : mcpFiducials.h
cmd.o : mcpMsgQ.h
cmd.o : mcpTimers.h
cmd.o : mcpUtils.h
cmd.o : ms.h
counter_weight.o : axis.h
counter_weight.o : cmd.h
counter_weight.o : cw.h
counter_weight.o : data_collection.h
counter_weight.o : instruments.h
counter_weight.o : io.h
counter_weight.o : mcpMsgQ.h
counter_weight.o : mcpTimers.h
counter_weight.o : mcpUtils.h
data_collection.o : abdh.h
data_collection.o : axis.h
data_collection.o : cmd.h
data_collection.o : copyright.h
data_collection.o : cw.h
data_collection.o : data_collection.h
data_collection.o : frame.h
data_collection.o : instruments.h
data_collection.o : io.h
data_collection.o : ipcast.h
data_collection.o : mcpFiducials.h
data_collection.o : mcpSpectro.h
data_collection.o : mcpUtils.h
data_collection.o : serial.h
data_collection.o : tm.h
dictionary.o : as2.h
fiducials.o : as2.h
fiducials.o : axis.h
fiducials.o : cmd.h
fiducials.o : data_collection.h
fiducials.o : mcpFiducials.h
fiducials.o : mcpMsgQ.h
fiducials.o : mcpTimers.h
fiducials.o : mcpUtils.h
fiducials.o : serial.h
fiducials.o : tm.h
instrument_lift.o : abdh.h
instrument_lift.o : axis.h
instrument_lift.o : cw.h
instrument_lift.o : data_collection.h
instrument_lift.o : instruments.h
instrument_lift.o : tm.h
instruments.o : abdh.h
instruments.o : as2.h
instruments.o : axis.h
instruments.o : cmd.h
instruments.o : cw.h
instruments.o : data_collection.h
instruments.o : instruments.h
instruments.o : tm.h
ipsym.o : mcpUtils.h
ipsym.o : timerint.h
pvt.o : abdh.h
pvt.o : axis.h
pvt.o : cmd.h
pvt.o : cw.h
pvt.o : data_collection.h
pvt.o : frame.h
pvt.o : io.h
pvt.o : mcpFiducials.h
pvt.o : mcpMsgQ.h
pvt.o : mcpTimers.h
pvt.o : mcpUtils.h
pvt.o : ms.h
pvt.o : serial.h
pvt.o : tm.h
serial.o : axis.h
serial.o : cmd.h
serial.o : copyright.h
serial.o : mcpUtils.h
serial.o : serial.h
spectro.o : abdh.h
spectro.o : as2.h
spectro.o : axis.h
spectro.o : cmd.h
spectro.o : data_collection.h
spectro.o : frame.h
spectro.o : mcpMsgQ.h
spectro.o : mcpSpectro.h
spectro.o : tm.h
tbars.o : abdh.h
tbars.o : axis.h
tbars.o : cmd.h
tbars.o : data_collection.h
tbars.o : mcpMsgQ.h
telescope_motion.o : abdh.h
telescope_motion.o : axis.h
telescope_motion.o : cmd.h
telescope_motion.o : cw.h
telescope_motion.o : data_collection.h
telescope_motion.o : frame.h
telescope_motion.o : instruments.h
telescope_motion.o : mcpFiducials.h
telescope_motion.o : ms.h
telescope_motion.o : tm.h
telnetCmds.o : as2.h
telnetCmds.o : axis.h
telnetCmds.o : cmd.h
telnetCmds.o : mcpUtils.h
timers.o : axis.h
timers.o : cmd.h
timers.o : data_collection.h
timers.o : mcpMsgQ.h
timers.o : mcpTimers.h
util.o : mcpUtils.h
util.o : mv162.h
util.o : vmechip2.h
