/************************************************************************/
/* Copyright 1993 MBARI							*/
/************************************************************************/
/* $Header$		*/
/* Summary  : Microtime functions for MVME-162 CPU under vxWorks	*/
/* Filename : microtime.c						*/
/* Author   : Bob Herlien (rah)						*/
/* Project  : Tiburon ROV						*/
/* $Revision$							*/
/* Created  : 02/28/91							*/
/************************************************************************/
/* Modification History:						*/
/* $Log$
 * Revision 1.2  2000/01/26 08:31:06  ekinney
 * Charlie's edits 25 Jan 2000
 *
/* Revision 1.1.1.1  1999/04/14 14:59:56  yanny
/* inital rev
/*
 * Revision 1.1.1.1  1999/04/14 05:33:46  spock
 * First entry of the MCP into CVS by matt newcomb
 * I made an attempt to cleanup the origional mcp directory.  It was hard
 * to tell what code was actually being used, so I could have made a mistake.
 * Please check to make sure everything is there, and if there is something that
 * you don't need, please remove it.
 *
*/
/* 23nov93 rah, created	for MVME-162					*/
/************************************************************************/

#include <types.h>		/* MBARI standard types		    */
#include <vxWorks.h>			/* VxWorks includes		    */

#include <mv162.h>			/* Hardware defns for MVME-162	    */
#include <drv/multi/mcchip.h>		/* MCC chip definitions		    */

/********************************/
/*	External Data		*/
/********************************/

extern unsigned long	hz;			/* Clock rate rtnd by sysClkRateGet()*/


/************************************************************************/
/* Function    : microtime						*/
/* Purpose     : Return microseconds since last clock tick		*/
/* Input       : None							*/
/* Outputs     : Microseconds since last clock tick			*/
/************************************************************************/
	unsigned long
microtime( void )
{
    unsigned long	cnt;

    cnt = *MCC_TIMER1_CNT;

    if ( cnt > 1000000/hz )
	return( 0 );
    return( cnt );

} /* microtime() */
