/* -------------------------------------------------------------------------
Name:		vmeinst.c

Function:	loads Z180 encoded or unencoded executable code into S-S 5163-SD-VME
	adapter.  Originally created by Lorne Diebel.  This program runs on
	a Xycom XVME IBM-PC/AT compatible VMEBus computer.

Current revision: 5.2 (see manifest VERSION, below)

Copyright (C) 1988-1992, Sutherland-Schultz Limited
Copyright (C) 1992,1993, S-S Technologies, Inc.  - Company Confidential Material

Revision History

revision	when		who		what
-----------------------------------------------------------------
5.0	    93/01/29	wjm	created from WR2PRMMC.C (5136-SD-MCA version)
5.1         93/02/26    wjm     fixed error message report that dropped first
				character of card-generated error message
5.2         93/04/23    wjm     added delay to ensure that Z180 gets reset
				before loading
				changed exit codes to match the Micro Channel
				loader
5.3         93/04/27    wjm     added -norun option, to accomodate Siemens,
				who use vmeinst to load, then read module from
				the card for later installation by plc
NOTE: compiled using Borland Turbo C++ 1.0
5.4	    97/01/31	cib	FNAL - ported to VxWorks
----------------------------------------------------------------------------- */

/* changes and additions

93/01/08 - translated from MCINST.C for the VME adapter - wjm
*/
#include "vxWorks.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "taskLib.h"

#define	VERSION		"5.4 (97/01/31)"

typedef unsigned short uint;
typedef unsigned char uchr;

#define Z180_BUSY		0xA5
#define Z180_OK			0x00
#define Z180_ERR		0x01

/* Adapter control-status register equates - SD2 */

#define Z180_RST		0x01
#define	HOST_INT_Z80	0x02
#define	CLR_INT			0x04		/* write only */
#define RLED			0x08
#define	SYSFAIL			0x10
#define	DPR_ENA			0x20
#define	WR_PROTECT		0x40	/* write-protect lower 32K blockm */
#define	Z80_INT_HOST	0x80		/* read only */

#define	IDLE_NORAM		RLED | Z180_RST
#define	IDLE			DPR_ENA | RLED | Z180_RST
#define	RUN				DPR_ENA | WR_PROTECT

/* miscellaneous other manifests */
#define IF_NOT_SILENT	if(!sil)

/* program return codes */

typedef enum {
	INST_SUCC=0, INV_PARAM=11, BAD_PORT, MEMORY_ERR, CPU_FAIL,
	CARD_ERR, MODULE_NF, OUT_OF_RANGE, CARD_NF, BUS_ERR
	} rtnCodeType;

	typedef enum {
		UM_OK=2, UM_IP, UM_BP, UM_ME, UM_CF, UM_CE, UM_MN, UM_BE
		}umType;

static char identification[] =
		"\nVMEINST Version %s\n"
		"5136-SD-VME Module installer for MVME-162 Processor Module\n"
		"Copyright (C) 1988-1993, S-S Technologies, Inc.\n"
		"  Charlie Briegel Modification for 68K (C) 1997, FNAL\n\n",
	ext[] = ".bin",
	unkOpt[] = "VMEINST: unknown option \"%s\"\n";

uchr *z80Mem;	/* pointer to base of adapter RAM */
uchr *ctrlReg;	/* pointer to adapter control register */
uchr *statIdReg;	/* pointer to adapter interrupt status/ID register */
uchr *memBaseReg;	/* pointer to adapter memory base address register */

static uint cpAddr = 0,
	 messAddr = 0;

static int sil,	/* indicates verbose/non-verbose installation */
	 touch,		/* TRUE when register pointers have been set correctly */
	 runModule;	/* TRUE when module is to be run after loading */

int VMEINST_verbose=FALSE;
/* -----------------------------------------------------------------------------
HELP

Prints the program usage message, and exits with code INV_PARAM.
----------------------------------------------------------------------------- */

void vmeinst_help( void )
{
	int i, l;
	static char	*useMsg[] = {
		"Usage:   vmeinst (SIO_address, RAM_address, \"filename\")\n",
		"\n",
		"Options: -q          set 'silent' mode\n",
		"         -s<address> set base address for short access (on a 1KB boundary,\n",
		"                       as set by switch positions 1-6 on the adapter)\n",
		"         -t<address> set base address for standard access (choose an\n",
		"                       address, on a 64KB boundary, at which the 64KB of\n",
		"                       adapter dual-ported RAM will be placed)\n",
		"         -v<intID>   set the 8-bit value to be loaded into the interrupt\n",
		"                       status/ID register for interrupt processing\n",
		"         -m	      scans the 64K VME short access space for non-FF values,\n",
		"                       and exits (no other arguments necessary)\n",
		"         -n          loads the specified module, but does not release the\n",
		"                       local processor to run it; module remains unaltered\n",
		"                       at offset 0000 of dual-ported RAM\n",
		"         filename    code image to load onto adapter (.BIN extension)\n",
		"\n",
		"Default: non-silent install, -s0000, -t000000 -v00\n",
		"Example: VMEINST -s0400 -t180000 -v3F MCDHP\n",
		"Example: VMEINST (0x0400, 0x0e0000, 0x3F, \"MCDHP\")\n",
		"\n",
		""
	},
	*errExpl[] = {
		"      Errors are defined by exit code when in silent mode\n",
		"      ---------------------------------------------------\n",
		"      %-2u - Installation Successful\n",
		"      %-2u - Invalid Parameter(s)\n",
		"      %-2u - Incorrect Short Access Space Address\n",
		"      %-2u - Card Memory Error\n",
		"      %-2u - Card Processor Failure\n",
		"      %-2u - Card Error (message in card memory, offset 8001)\n",
		"      %-2u - Card Software Module not Found\n",
		"      %-2u - VMEBus access error\n",
		"\n",
		""
	};

	fprintf( stderr, identification, VERSION );		/* prints 5 lines of identification information */
	for( i = 0, l = 5; useMsg[ i ][ 0 ] != '\0'; ++i )
	{
		fputs( useMsg[ i ], stderr );				/* print a line of the usage message */
	}
	for( i = 0; errExpl[ i ][ 0 ] != '\0'; ++i )
	{
/* print a line of the error message, with exit code if appropriate */

		if( i == UM_OK )
			fprintf( stderr, errExpl[ i ], INST_SUCC );
		else if( i == UM_IP )
			fprintf( stderr, errExpl[ i ], INV_PARAM );
		else if( i == UM_BP )
			fprintf( stderr, errExpl[ i ], BAD_PORT );
		else if( i == UM_ME )
			fprintf( stderr, errExpl[ i ], MEMORY_ERR );
		else if( i == UM_CF )
			fprintf( stderr, errExpl[ i ], CPU_FAIL );
		else if( i == UM_CE )
			fprintf( stderr, errExpl[ i ], CARD_ERR );
		else if( i == UM_MN )
			fprintf( stderr, errExpl[ i ], MODULE_NF );
		else if( i == UM_BE )
			fprintf( stderr, errExpl[ i ], BUS_ERR );
		else
			fputs( errExpl[ i ], stderr );
	}

	exit( INV_PARAM );
}

/* -----------------------------------------------------------------------------
GETBYTE

Returns the next byte read from the file, or EOF.  The offset of the module
identification is at file offset 8, and if verbose installation is enabled,
the module identification is printed.
----------------------------------------------------------------------------- */

int getByte( FILE *fp )
{
	int temp;

	if( (temp = getc( fp )) == EOF )
		return EOF;

	IF_NOT_SILENT
	{
		if( cpAddr == 8 )
			messAddr += temp;
		else if( cpAddr == 9 )
			messAddr += temp<<8;
		else if( messAddr && cpAddr >= messAddr )
			if( temp )
				putchar( temp );
			else
			{
				messAddr = 0;
				printf( "\n5136-SD-VME Adapter running diagnostics\n" );
			}
		++cpAddr;
	}
	return temp;
}


/* -----------------------------------------------------------------------------
ABEND

Prints the passed error message, disables the adapter RAM and processor, if the
pointer have been set, and exits with the passed exit code.
----------------------------------------------------------------------------- */

void abend( char *str, rtnCodeType errCode )
{
	IF_NOT_SILENT fputs( str, stderr );
	if( touch )							/* disable adapter, if pointers set */
	{
		*ctrlReg = IDLE_NORAM;			/* red LED on, processor held, DPRAM disabled */
	}
/*	outportb( IBM_NMI_ENA, ENA_NMI );*/	/* re-enable machine NMI */
	exit( errCode );
}

/* -----------------------------------------------------------------------------
MAPSHORT

Scans VME short access space (64K), looking for non-FF values.  Prints a map of
the space.
----------------------------------------------------------------------------- */

void mapShort( void )
{
	int i, j, k;
	uchr *fred;

/*	outportb( IBM_NMI_ENA, DIS_NMI );*/	/* disable machine NMI */
/*	SHORT;				*/	/* point to short access space */

	fred = (uchr *)0xFFFF0000;
	printf( "Scanning short access space\n\n" );
	printf( "Registers found in the following paragraphs:\n    " );

#ifdef BROKEN
	k = 0;
	do
	{
		for( i = j = 0; i < 16; ++i )
			if( *fred++ != 0xFF )
				++j;/* count a register in this paragraph */

		if( j != 0 )	/* register(s) was found */
		{
			printf( "%04X    ", (uint)fred - 16 );
			if( ++k == 8 )
				k = 0, printf( "\n    " );
		}

	} while( (uint)fred != 0 );
#endif
/*	outportb( IBM_NMI_ENA, ENA_NMI );*/	/* re-enable machine NMI */
}

/* -------------------------------- M A I N --------------------------------- */

void vmeinst(
	uint shortBase,	  /* adapter short access space base address (registers) */
	long standardBase,/* adapter standard access space base address (RAM) */
	uint statusId,	  /* value to be loaded into interrupt status/ID register */
	uchr *fName)
{
	FILE *fp1;
	uchr *p;
	uint *z80MemW;	/* 16-bit word pointer to DPRAM */
	int i, c, d, e;


		/* assign defaults for the options */

	runModule = TRUE;		/* cleared to FALSE by -n (norun) */
	sil =				/* set to TRUE by -q (quiet) */
	touch = FALSE;		/* set to TRUE when pointer set up */
	fp1 = NULL;

/* check for 1KB boundary of short base address */

	if( (shortBase & 0x03FF) != 0 )
		abend( "VMEINST: short access base address (registers) is selected by the switch\n         setting.  It must be on a 1KB boundary.\n", INV_PARAM );

/* check for 64KB boundary of standard base address */

	if( (standardBase & 0x0000FFFFL) != 0 )
		abend( "VMEINST: standard access base address must be on a 64KB boundary.\n", INV_PARAM );

	if( statusId > 0x00FF )
		abend( "VMEINST: interrupt status/ID data is only 8 bits wide.\n         Data must be between 0 & FF.\n", INV_PARAM );

/* put the extension onto the simple filename */

			if( (p = strchr( fName, '.' )) == NULL )
				strcat( fName, ext );
			else
				strcpy( p, ext );

			if( (fp1 = fopen( fName, "rb" )) == NULL )
			{
				IF_NOT_SILENT fprintf( stderr, "VMEINST: unable to open file \"%s\" for read.\n", fName );
				exit( MODULE_NF );
			}

	if( fName == NULL )			/* no load file specified */
		abend( "VMEINST: no load file specified.\n", MODULE_NF );

		/* disable machine NMI, since VMEBus bus errors use it */

/*	outportb( IBM_NMI_ENA, DIS_NMI ); */

	IF_NOT_SILENT
	{
		printf( identification, VERSION );
		printf( "5136-SD-VME: RAM window at %06lX, registers at %04X\n\n", standardBase, shortBase );
	}

/* Xycom real mode window is at E000 */

	z80Mem = (uchr *)0xFFFF0000;	/* RAM pointer (standard space) */
	ctrlReg = (uchr *)(0xFFFF0000|(shortBase+1));	/* control register pointer (short space) */
	statIdReg = (uchr *)(0xFFFF0000|(shortBase+3));	/* interrupt status/ID register pointer (short space) */
	memBaseReg = (uchr *)(0xFFFF0000|(shortBase+5));/* memory base register pointer (short space) */


/* make a rudimentary test of adapter registers */

	*ctrlReg = IDLE_NORAM;		/* card idle, DPRAM off */
	taskDelay( 2 );
	*ctrlReg = RUN;
	taskDelay( 2 );
	*ctrlReg = IDLE_NORAM;
	for( i = 0; i < 256; ++i )
	{
		*statIdReg = (uchr)i;
		*memBaseReg = (uchr)~i;
		if( *statIdReg != (uchr)i )
			abend( "VMEINST: register error (interrupt status/ID register)\n", BAD_PORT );
		else if( *memBaseReg != (uchr)~i )
			abend( "VMEINST: register error (memory base register)\n", BAD_PORT );
	}
	*statIdReg = (uchr)statusId;	/* set adapter interrupt status/ID */
	*memBaseReg = (uchr)(standardBase >> 16);/* set upper 8 bits of standard address */
	*ctrlReg = IDLE;	/* processor held, LED on, DPRAM enabled */
	touch = TRUE;		/* pointers have now been set */

	taskDelay( 10 );

	z80Mem = (uchr *)(0xF0000000|standardBase|0x8000);
	*z80Mem = Z180_BUSY;			/* preset cpu busy flag */
	if( *z80Mem != Z180_BUSY )
		abend( "VMEINST: adapter memory access error!\n", MEMORY_ERR );

	for( i = 0, z80MemW = (uint *)(0xF0000000|standardBase|0x0000); ; ++i )
	{
		if( (c = getByte( fp1 )) == EOF )
			break;

		d = getByte( fp1 );
#ifdef INTEL
		e = (c & 0x00FF) | d << 8;
#else
		e = (d & 0x00FF) | c << 8;
#endif
		if ((VMEINST_verbose)&&(*z80MemW != e))
		  printf ("%d: Mismatch %x != %x\r\n",i,*z80MemW,e);
		*z80MemW = e;

		if( *z80MemW++ != e )
			abend( "VMEINST: adapter memory error!\n", MEMORY_ERR );

		if( d == EOF )
			break;
		else
			++i;
	}
	fclose( fp1 );
	IF_NOT_SILENT printf( "Loaded %u bytes from \"%s\"\n", i, fName );

	if( runModule )
	{
		*ctrlReg = RUN;		/* LED off, release Z180 to run */

		taskDelay( 10 );

		c = 70;			/* 7 second time out */
		while( ( *z80Mem == Z180_BUSY ) && c-- )
			taskDelay( 100 );

		if( *z80Mem == Z180_BUSY )
			abend( "VMEINST: Z180 did not run!\n", CPU_FAIL );

		if( *z80Mem != Z180_OK )	/* Z180 ran, but had problems */
		{
			if( *z80Mem++ == Z180_ERR )/* error message is in DPRAM */
			{
				IF_NOT_SILENT
				{
					z80Mem += 2;	/* skip CR/LF */
					fputs( "VMEINST: ", stderr );
					for( i = 9; i < 80 && *z80Mem; ++z80Mem )
					{
						putc( *z80Mem, stderr );
						if( *z80Mem == '\r' )
							i = 0;
						else
							i++;
					}
				}

				abend( "", CARD_ERR );
			}
			else			/* undetermined error */
				abend( "VMEINST: Z180 failure!\n", CPU_FAIL );
		}
	}
	else
		puts( "Z180 remains held, module not running" );
}
