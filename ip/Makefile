CPU = MC68040

HOST = irix

VW = $(VX_VW_BASE)/
VWH = $(VW)h/
H1 = $(VWH)
H2 = $(VWH)net/
H3 = $(VWH)rpc/
H4 = $(VWH)68k/
H5 = $(VWH)private/

INCLUDES = -I$(H1) -I$(H2) -I$(H3) -I$(H4) -I$(H5) \
	-I$(VX_TOOLS_DIR)/include -I$(MCP_DIR)/util \
	-I$(MURMUR_CLIENT_VX_DIR)/include

CC = cc68k
CFLAGS = -g -ansi -O2 -pipe -fvolatile -m68040 -DCPU=$(CPU) -DVXWORKS -DMEI_VW \
-DMEI_MC680X0_CPU -D__USE_GETC__

DIRS = acromag systran

OBJS =mv162IndPackInit.o ipOctalSerial.o


.SUFFIXES: .o .c .asm .s                                

.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $*.c

all dg: $(OBJS) 
	@for d in $(DIRS); do \
		echo $$d; \
		(cd $$d; $(MAKE) $(MFLAGS) all ); \
	done

install : all
	cp Makefile *.[cho] $(MCP_DIR)/ip
	@for d in $(DIRS); do \
		echo $$d; \
		if [ ! -d $(MCP_DIR)/$$d ]; then \
			mkdir $(MCP_DIR)/ip/$$d; \
		fi; \
		(cd $$d; $(MAKE) $(MFLAGS) install ); \
	done

clean :
	/bin/rm -f *.o *~ *.map *.out
	@for d in $(DIRS); do \
		echo $$d; \
		(cd $$d; $(MAKE) $(MFLAGS) clean ); \
	done
