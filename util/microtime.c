/************************************************************************/
/* Copyright 1993 MBARI							*/
/************************************************************************/
/* Summary  : Microtime functions for MVME-162 CPU under vxWorks	*/
/* Filename : microtime.c						*/
/* Author   : Bob Herlien (rah)						*/
/* Project  : Tiburon ROV						*/
/* Created  : 02/28/91							*/
/************************************************************************/
/* Modification History:						
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
