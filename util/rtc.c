/************************************************************************/
/* Copyright 1990 - 1993 MBARI						*/
/************************************************************************/
/* $Header$			*/
/* Summary  : Functions to read time from MK48T08 chip on MVME-162 CPU	*/
/*		under vxWorks						*/
/* Filename : rtc.c							*/
/* Author   : Bob Herlien (rah)						*/
/* Project  : Tiburon ROV						*/
/* $Revision$							*/
/* Created  : 02/28/91							*/
/************************************************************************/
/* Modification History:						*/
/* $Log$
 * Revision 1.2  2000/01/26 08:31:07  ekinney
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
 * Revision 1.2  91/09/23  09:38:59  09:38:59  hebo (Bob Herlien 408-647-3748)
 * Minor bug fixes, added -DMICROTIME
 * 
 * Revision 1.1  91/09/06  16:05:35  16:05:35  hebo (Bob Herlien 408-647-3748)
 * Initial revision
 * 
 * Revision 1.0  91/08/09  11:10:33  11:10:33  hebo (Bob Herlien 408-647-3748)
 * Initial revision
 * 	*/
/* 28feb91 rah, created							*/
/************************************************************************/
/* The Unix Epoch (day 0) is Jan 1, 1970.  The real-time clock chip	*/
/* only gives 2 decimal digits of year.  Also, a Unix time_t value	*/
/* (signed long) can only go +/- 68 years from 1970.  Therefore, these	*/
/* routines assume that the year is between 1970 and 2070.  Note that	*/
/* the leap-year exception for centuries not divisible by 400 is not a	*/
/* factor in this period.						*/
/************************************************************************/

#include <types.h>		/* MBARI standard types		    */
#include <vxWorks.h>			/* VxWorks includes		    */
#include <time.h>			/* time structure definitions	    */
#include <usrTime.h>			/* time structure definitions	    */
#include <taskLib.h>			/* VxWorks task definitions	    */
#include <sysLib.h>			/* VxWorks driver definitions	    */

#include "mv162.h"			/* Hardware defns for MVME-162	    */


/* Defines for MK48T08 Real-Time Clock Chip on MVME-162	*/

#define RTC(x)		((unsigned char *)(TOD_CLOCK + x))

#define RTC_CTRL 	RTC(0)		/* Control byte of RTC chip	    */
#define RTC_SECS 	RTC(1)		/* Seconds on RTC chip	    */
#define RTC_MINS	RTC(2)		/* Minutes on RTC chip	    */
#define RTC_HR		RTC(3)		/* Hours on RTC chip		    */
#define RTC_DAY		RTC(4)		/* Day of week (1 - 7) on RTC chip */
#define RTC_DATE	RTC(5)		/* Date (1 - 31) on RTC chip	    */
#define RTC_MONTH	RTC(6)		/* Month on RTC chip		    */
#define RTC_YR		RTC(7)		/* Year (00 - 99) on RTC chip    */

#define STOP_BIT	0x80		/* "Stop Clock" bit in seconds field*/
#define WRITE_BIT	0x80		/* "Write" bit in control byte	    */
#define READ_BIT	0x40		/* "Read" bit in control byte	    */


/********************************/
/*	external Functions	*/
/********************************/

/************************************************************************/
/* Function    : bcd_to_hex						*/
/* Purpose     : BCD to hex conversion					*/
/* Input       : BCD byte						*/
/* Outputs     : Hex byte						*/
/************************************************************************/
	long
bcd_to_hex( long byte )
{
    return( (10 * (byte>>4)) + (byte & 0xf) );

} /* bcd_to_hex() */


/************************************************************************/
/* Function    : hex_to_bcd						*/
/* Purpose     : Hex to BCD conversion					*/
/* Input       : Hex byte						*/
/* Outputs     : BCD byte						*/
/************************************************************************/
	long
hex_to_bcd( long byte ) 
{
    return( ((byte / 10) << 4) + (byte % 10) );

} /* hex_to_bcd() */


/************************************************************************/
/* function    : rtc_on							*/
/* purpose     : Make sure rtc chip is running				*/
/* inputs      : None							*/
/* outputs     : None							*/
/************************************************************************/
	void
rtc_on( void )
{
    if ( *RTC_SECS & STOP_BIT )		/* Clock stopped?		*/
	*RTC_SECS &= ~STOP_BIT;		/* Start it			*/

} /* rtc_on() */


/************************************************************************/
/* Function    : rtc_read						*/
/* Purpose     : Read the RTC chip, put the results in struct tm *tp	*/
/* Inputs      : Pointer to struct tm to put current time into		*/
/* Outputs     : Time in time_t format					*/
/************************************************************************/
	time_t
rtc_read( struct tm *tp )
{
    struct tm	temp_tm;

    rtc_on();				/* Make sure RTC is running	*/

    if ( tp == (struct tm *)NULL )	/* If no tm, use temp one	*/
	tp = &temp_tm;

    *RTC_CTRL |= READ_BIT;		/* Stop updating clock		*/
    tp->tm_sec = bcd_to_hex(*RTC_SECS & 0x7f);
    tp->tm_min = bcd_to_hex(*RTC_MINS & 0x7f);
    tp->tm_hour = bcd_to_hex(*RTC_HR & 0x3f);
    tp->tm_mday = bcd_to_hex(*RTC_DATE & 0x3f);
    tp->tm_mon = bcd_to_hex(*RTC_MONTH & 0x1f) - 1;
    tp->tm_year = bcd_to_hex(*RTC_YR);
    if ( tp->tm_year < YEAR0 )		/* RTC_YR 00 is actual year 2000*/
	tp->tm_year += 100;
    tp->tm_wday = (*RTC_DAY & 7) - 1;
    *RTC_CTRL &= ~(WRITE_BIT | READ_BIT); /* Restart clock updating	*/
    return (mktime(tp));

} /* rtc_read() */


/************************************************************************/
/* Function    : rtc_write						*/
/* Purpose     : Write the RTC chip from the values of struct tm *tp	*/
/* Inputs      : Pointer to struct tm that contains current time	*/
/* Outputs     : None							*/
/************************************************************************/
	void
rtc_write( const struct tm *tp )
{
    rtc_on();				/* Make sure RTC is running	*/
    *RTC_CTRL |= WRITE_BIT;		/* Stop the clock		*/

    *RTC_SECS = (char)(hex_to_bcd(tp->tm_sec) & 0x7f);
    *RTC_MINS = (char)(hex_to_bcd(tp->tm_min) & 0x7f);
    *RTC_HR   = (char)(hex_to_bcd(tp->tm_hour) & 0x3f);
    *RTC_DAY  = (char)((tp->tm_wday + 1) & 7);
    *RTC_DATE = (char)(hex_to_bcd(tp->tm_mday) & 0x3f);
    *RTC_MONTH = (char)(hex_to_bcd(tp->tm_mon + 1) & 0x1f);
    *RTC_YR   = (char)hex_to_bcd(tp->tm_year % 100);

    *RTC_CTRL &= ~(WRITE_BIT | READ_BIT); /* Restart clock 		*/

} /* rtc_write() */
