CPU = MC68040

HOST = irix

VW = $(VX_VW_BASE)/
VWH = $(VW)h/
H1 = $(VWH)
H2 = $(VWH)net/
H3 = $(VWH)rpc/
H4 = $(VWH)68k/
H5 = $(VWH)private/

INCLUDES = -I$(H1) -I$(H2) -I$(H3) -I$(H4) -I$(H5)
CC = cc68k
CFLAGS = -g -ansi -O2 -pipe -fvolatile -m68040 -DCPU=$(CPU) -DVXWORKS -DMEI_VW \
-DMEI_MC680X0_CPU -D__USE_GETC__
#CFLAGS = -g -ansi -mc68040
#CFLAGS = -S -O -traditional -mc68040

CPP = cpp68k
#CPP = gcc-cpp  
CPPFLAGS = -P
#CPPFLAGS = -P -Caa

ASM = as68k
AFLAGS = -mc68040

LINK = ld68k                       
LFLAGS = -r -M
AR = ar68k
ARFLAGS = crsv

ASRCS =

DIRS = acromag systran

SRCS =mv162IndPackInit.c ipOctalSerial.c

OBJS =mv162IndPackInit.o ipOctalSerial.o


.SUFFIXES: .o .c .asm .s                                

.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $*.c

.asm.s:
	$(CPP) $(CPPFLAGS) $*.asm $*.s

.s.o:
	$(ASM) $(AFLAGS) -o $*.o $*.s

all dg: $(ASRCS) $(OBJS) 
	@for d in $(DIRS); do \
		echo $$d; \
		(cd $$d; $(MAKE) $(MFLAGS) all ); \
	done

clean :
	/bin/rm -f *.o *~ *.map *.out
	@for d in $(DIRS); do \
		echo $$d; \
		(cd $$d; $(MAKE) $(MFLAGS) clean ); \
	done
