/* mv162.h - Motorola MVME162 CPU board header */

/*
modification history
--------------------
01a,04jan93,ccc	 written from mv167.h.
*/

/*
This file contains I/O addresses and related constants for the
Motorola MVME162.
*/

#ifndef	INCmv162h
#define	INCmv162h

#include "drv/serial/z8530.h"
#include "drv/multi/mcchip.h"
#include "drv/vme/vmechip2.h"
#include "drv/timer/timerDev.h"
#include "drv/mem/memDev.h"
#include "drv/scsi/ncr710.h"

#define BUS		VME_BUS
#define CPU		MC68040

#define N_SIO_CHANNELS	2		/* Number of serial I/O channels */

/* Local I/O address map */

#define	BBRAM_ADRS	((char *) 0xfffc0000)	/* MK48T08 battery backup ram */
#define	BBRAM_SIZE	0x1ff8			/* number of bytes for BBRAM  */
#define	BB_CPU_SPEED	((char *) 0xfffc1f28)	/* CPU clock speed */
#define	BB_ENET		((char *) 0xfffc1f2c)	/* factory ethernet address   */
#define	TOD_CLOCK 	((char *) 0xfffc1ff8)	/* MK48T08 bb time of day clk */

#define	MCC_BASE_ADRS	(0xfff42000)		/* PCC registers base address */
#define	VMECHIP2_BASE_ADRS (0xfff40000) 	/* VMEchip LCSR registers     */
#define	SCC_BASE_ADRS	(0xfff45000)		/* Z85230 (Serial Comm Contr) */
#define	SERIAL_SCC	SCC_BASE_ADRS

/* interrupt vector locations */

#define MCC_INT_VEC_BASE	0x40	/* MCC interrupt vector base number */
					/* any multiple of 0x10             */
#define UTIL_INT_VEC_BASE0	0x50	/* VMEchip2 utility interrupt       */
					/* vector base number               */
					/* any multiple of 0x10             */
#define	UTIL_INT_VEC_BASE1	0x60	/* VMEchip2 utility interrupt       */
					/* vector base number               */
					/* any multiple of 0x10             */

#define	INT_VEC_SCC		0x90
#define INT_VEC_SCC_A_WR	INT_VEC_SCC + 0x08
#define	INT_VEC_SCC_A_EX	INT_VEC_SCC + 0x0a
#define INT_VEC_SCC_A_RD	INT_VEC_SCC + 0x0c
#define INT_VEC_SCC_A_SP	INT_VEC_SCC + 0x0e

#define	INT_VEC_SCC_B_WR	INT_VEC_SCC + 0x00
#define	INT_VEC_SCC_B_EX	INT_VEC_SCC + 0x02
#define	INT_VEC_SCC_B_RD	INT_VEC_SCC + 0x04
#define	INT_VEC_SCC_B_SP	INT_VEC_SCC + 0x06

#define	LANC_IRQ_LEVEL		3	/* LANC IRQ level             */
#define	SCC_IRQ_LEVEL		4	/* serial comm IRQ level      */
#define	SYS_CLK_LEVEL		6	/* interrupt level for sysClk */
#define AUX_CLK_LEVEL		5	/* interrupt level for auxClk */
#define	SCSI_IRQ_LEVEL		2	/* SCSI interrupt level       */
#define	ABORT_IRQ_LEVEL		7	/* ABORT interrupt level      */

/* board specific registers */

/* 82596CA */

#define	INT_VEC_EI		INT_VEC_LN
#define	EI_SYSBUS		0x6c		/* 82596 SYSBUS value */
						/* IRQ active low     */
#define	EI_POOL_ADRS		NONE

#define	I82596_PORT		((UINT32 *) 0xfff46000)
#define	I82596_CONTROL		((UINT32 *) 0xfff46004)

/* ncr710 */
#define	MV162_SIOP_BASE_ADRS	((UINT8 *) 0xfff47000)
#define	MV162_SIOP_FREQ		((UINT)NCR710_50MHZ)	/* 50MHz SCSI clock */
#define	MV162_SIOP_HW_REGS	{ 0,0,0,1,1,0,0,0,0,0,0,0,0,1,0 }	

#endif	/* INCmv162h */
