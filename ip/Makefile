CPU = MC68040

HOST = irix

IP = /home/briegel/mcp/ip/
SYSTRAN = /home/briegel/mcp/ip/systran/
ACROMAG = /home/briegel/mcp/ip/acromag/
AB = /home/briegel/mcp/ab/
VW = /p/vx_dsc/v5_1_1f3/
VWH = $(VW)h/
H1 = $(VWH)
H2 = $(VWH)net/
H3 = $(VWH)rpc/
H4 = $(VWH)68k/
H5 = $(VWH)private/

INCLUDES = -I$(H1) -I$(H2) -I$(H3) -I$(H4) -I$(H5)  \
	-I$(IP) -I$(SYSTRAN) -I$(AB) -I$(ACROMAG) /
CC = cc68k
CFLAGS = -g -ansi -O2 -pipe -fvolatile -m68040 -DCPU=$(CPU) -DVXWORKS -DMEI_VW \
-DMEI_MC680X0_CPU -D__USE_GETC__
#CFLAGS = -g -ansi -mc68020 -m68881
#CFLAGS = -S -O -traditional -mc68020 -m68881

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

SRCS =mv162IndPackInit.c ipOctalSerial.c

OBJS =mv162IndPackInit.o ipOctalSerial.o


.SUFFIXES: .o .c .asm .s                                

.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $*.c

.asm.s:
	$(CPP) $(CPPFLAGS) $*.asm $*.s

.s.o:
	$(ASM) $(AFLAGS) -o $*.o $*.s

dg: $(ASRCS) $(OBJS) 
