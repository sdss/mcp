#	vw.mak
#
#	MEI DSP motion controller C function library:
#	rebuild instructions for VxWorks/68k
#

CFLAGS		=	-c -O -DCPU=MC68040 -I$(VX_VW_BASE)/h -DMEI_VW -Wall -DMEI_MC680X0_CPU
CC			=	cc68k
AR			=	ar68k
ARFLAGS		=	crsv
RAN			=	ranlib68k

LIB			=	medvxw40.a
OBJ			=	o

OBJS		=	idsp.$(OBJ)			\
				imemory.$(OBJ)		\
				iframe.$(OBJ)		\
				ifixed.$(OBJ)		\
							\
				llframe.$(OBJ)		\
				llstatus.$(OBJ)		\
				llpid.$(OBJ)		\
				llpos.$(OBJ)		\
				llef.$(OBJ)		\
				llio.$(OBJ)		\
				llsetvel.$(OBJ)		\
				llmove.$(OBJ)		\
				llconfig.$(OBJ)		\
				llink.$(OBJ)		\
				llbm.$(OBJ)		\
				llfirm.$(OBJ)		\
							\
				bconfig.$(OBJ)		\
				blimits.$(OBJ)		\
							\
				mldsp.$(OBJ)		\
				mluom.$(OBJ)		\
				mlpos.$(OBJ)		\
				mlef.$(OBJ)		\
				mlsetvel.$(OBJ)		\
				mlmove.$(OBJ)		\
				mlsmove.$(OBJ)		\
				mlconfig.$(OBJ)		\
				mlio.$(OBJ)		\
				mlstatus.$(OBJ)		\
				mllink.$(OBJ)		\
				mlerror.$(OBJ)		\
				mlframe.$(OBJ)		\
				mlpmove.$(OBJ)		\
				mldspf.$(OBJ)		\
				mlrmove.$(OBJ)		\
		                mlrsmove.$(OBJ)		\
		                mlspline.$(OBJ)		\
							\
				hldsp.$(OBJ)		\
				envutil.$(OBJ)		\
				backward.$(OBJ)		\
				extras.$(OBJ)		\
				mboard.$(OBJ)		\
							\
				system.$(OBJ)		\
				mlserc.$(OBJ)		\
				llserc.$(OBJ)		\
				sercrset.$(OBJ)		\
				llsrset.$(OBJ)		\
				sercmisc.$(OBJ)		\
				sercdiag.$(OBJ)		\
				sydsp.$(OBJ)		\
							\
		                mllsr.$(OBJ)		\
		                mltan.$(OBJ)		\
				sincomm.$(OBJ)	

.c.$(OBJ):
	$(CC) $(CFLAGS) $*.c
	$(AR) $(ARFLAGS) $(LIB) $*.$(OBJ)


$(LIB):		$(OBJS)
	$(RAN) $(LIB)
