#	vw.mak
#
#	MEI DSP motion controller C function library:
#	rebuild instructions for VxWorks/68k
#

CFLAGS		=	-c -O -DCPU=MC68040 -I$(VX_DSC_DIR)/h -DMEI_VW -Wall -DMEI_MC680X0_CPU
CC			=	cc68k
AR			=	ar68k
ARFLAGS		=	crsv
RAN			=	ranlib68k

LIB			=	medvxw40.a

OBJS =	idsp.o imemory.o iframe.o ifixed.o \
	\
	llframe.o llstatus.o llpid.o llpos.o llef.o llio.o llsetvel.o \
	llmove.o llconfig.o llink.o llbm.o llfirm.o \
	\
	bconfig.o blimits.o mldsp.o mluom.o mlpos.o mlef.o mlsetvel.o \
	mlmove.o mlsmove.o mlconfig.o mlio.o mlstatus.o mllink.o mlerror.o \
	mlframe.o mlpmove.o mldspf.o mlrmove.o mlrsmove.o mlspline.o \
	\
	hldsp.o envutil.o backward.o extras.o mboard.o \
	\
	system.o mlserc.o llserc.o sercrset.o llsrset.o sercmisc.o \
	sercdiag.o sydsp.o \
	\
	mllsr.o mltan.o sincomm.o

.c.o:
	$(CC) $(CFLAGS) $*.c

all : $(LIB)

$(LIB):	$(OBJS)
	$(AR) $(ARFLAGS) $(LIB) $?
	$(RAN) $(LIB)

install : all
	cp Makefile *.[acho] $(MCP_DIR)/mei-src

	if [ ! -d $(MCP_DIR)/doc ]; then \
		mkdir $(MCP_DIR)/doc; \
	fi
	cp doc/*.txt $(MCP_DIR)/doc

clean :
	/bin/rm -f $(OBJS) *.a core *~ doc/*~
