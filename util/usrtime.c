/************************************************************************/
/* Copyright 1990 MBARI							*/
/************************************************************************/
/* $Header$		*/
/* Summary  : Unix-compatible Time Routines for SBE VPU-30 under vxWorks*/
/* Filename : usrTime.c							*/
/* Author   : Bob Herlien (rah)						*/
/* Project  : New ROV							*/
/* $Revision$							*/
/* Created  : 02/28/91							*/
/************************************************************************/
/* Modification History:						*/
/************************************************************************/
/* The Unix Epoch (day 0) is Jan 1, 1970.  The real-time clock chip	*/
/* only gives 2 decimal digits of year.  Also, a Unix time_t value	*/
/* (signed long) can only go +/- 68 years from 1970.  Therefore, these	*/
/* routines assume that the year is between 1970 and 2070.  Note that	*/
/* the leap-year exception for centuries not divisible by 400 is not a	*/
/* factor in this period.						*/
/************************************************************************/

#include <types.h>		/* MBARI standard types		    */
#include <vxWorks.h>			/* VxWorks types		    */
#include <time.h>
#include <timers.h>
#include <stdioLib.h>			/* VxWorks standard I/O library	    */
#include <sysLib.h>			/* VxWorks Driver defines	    */
#include <string.h>			/* VxWorks string types		    */
#include <limits.h>			/* VxWorks integer limits	    */
#include <taskLib.h>
#include "usrtime.h"			/* time structure definitions	    */

#define ASCTM_SIZE	32		/* Size of buffer for asctime()	    */
#define LCLOFS		28800L		/* Offset in seconds from GMT to PST*/
#define LCLST		"PST"		/* Local timezone is PST	    */
#define LCLDST		"PDT"		/* Local daylight savings zone is PDT*/
#define ADJTIME_MAX	((INT_MAX / USECS_PER_SEC) - 1)
					/* Max seconds in adjtime call	    */
#define CLKRATE		60		/* Default clock tick rate	    */
#define CLK_GRANULARITY  1		/* Granularity of system clock in usec*/
					/* Used to round down # usecs/tick  */
					/* On a VPU-30, PIT gets 8 MHz clk, */
					/*  & it prescales by 32, thus 4 usec*/
					/* To defeat rounding, set to 1	    */
					/* See usrTimeInit()		    */

/********************************/
/*	External Functions	*/
/********************************/

extern time_t rtc_read( struct tm *tp );	/* From rtc.c		    */
extern void rtc_write( const struct tm *tp );	/* Ditto		    */
#define MICROTIME	1
#ifdef MICROTIME
extern unsigned long microtime();
#endif


/********************************/
/*	Global Data		*/
/********************************/
unsigned long   hz = CLKRATE;                   /* Clock rate */

/********************************/
/*	Module Local Data	*/
/********************************/

static struct timeval gm_time;		/* Time, kept as GMT time	    */
static long	adj_time;		/* Time left to adjust		    */

struct aux_tod {
	unsigned short int year;
	unsigned short int month;
	unsigned short int day;
	unsigned short int hour;
	unsigned short int minute;
	unsigned short int second;
	unsigned short int hundreths;
};
struct aux_tod *get_aux_time(unsigned short node, struct aux_tod *aux_t);
int set_aux_time(struct aux_tod *aux_t);
long gmtset( long hhmmss, long yymmdd );

/************************************************************************/
/* Function    : adjtime						*/
/* Purpose     : Clock tick handler for usrTime module			*/
/* Inputs      : Ptr to timval struct to adjust, ptr to amount left from*/
/*		  last adjustment					*/
/* Outputs     : Zero for OK, -1 if exceeds ADJTIME_MAX (2146 seconds)	*/
/************************************************************************/
	long
adjtime( struct timeval *delta, struct timeval *olddelta )
{
    olddelta->tv_sec = (adj_time / USECS_PER_SEC);
    olddelta->tv_usec = (adj_time % USECS_PER_SEC);

    if ( (delta->tv_sec > ADJTIME_MAX) || (delta->tv_sec < -ADJTIME_MAX) )
	return( -1 );

    adj_time = (delta->tv_sec * USECS_PER_SEC) + delta->tv_usec;

    return( 0 );

} /* adjtime() */


/************************************************************************/
/* Function    : stime							*/
/* Purpose     : Unix-compatible set time function			*/
/* Inputs      : Ptr to time in seconds from 1/1/70			*/
/* Outputs     : 0							*/
/************************************************************************/
int stime( time_t *tp )
{
    gm_time.tv_sec = *tp;
    gm_time.tv_usec = 0;
    rtc_write( gmtime((time_t *)tp) );
    return( 0 );

} /* stime() */


/************************************************************************/
/* Function    : date							*/
/* Purpose     : User interface function to show time and date		*/
/* Inputs      : None							*/
/* Outputs     : 0							*/
/* Comment     : Displays time and date to stdout			*/
/************************************************************************/
long set_date_hdw ( void )
{
    struct timeval	tp;
    struct tm 		t;

  rtc_read (&t);
  printf (" mon=%d day=%d, year=%d %d:%d:%d\r\n",
        t.tm_mon,t.tm_mday,t.tm_year,t.tm_hour,t.tm_min,t.tm_sec);
  tp.tv_sec=mktime(&t);
  tp.tv_usec=0;
  printf (" sec=%d, nano_sec=%d\r\n",tp.tv_sec,tp.tv_usec);
  clock_settime(CLOCK_REALTIME,(struct timespec *)&tp);
  return 0;
}

long set_date ( void )
{
    struct timeval	tp;
    struct tm 		t;

  rtc_read (&t);
  printf (" mon=%d day=%d, year=%d %d:%d:%d\r\n",
        t.tm_mon,t.tm_mday,t.tm_year,t.tm_hour,t.tm_min,t.tm_sec);
  tp.tv_sec=mktime(&t);
  tp.tv_usec=0;
  printf (" sec=%d, nano_sec=%d\r\n",tp.tv_sec,tp.tv_usec);
  clock_settime(CLOCK_REALTIME,(struct timespec *)&tp);
  return 0;
}

/************************************************************************/
/* Function    : date							*/
/* Purpose     : User interface function to show time and date		*/
/* Inputs      : None							*/
/* Outputs     : 0							*/
/* Comment     : Displays time and date to stdout			*/
/************************************************************************/
	long
date( void )
{
    char	*s;
    time_t	timer;
    struct timespec ts;
    
    time( &timer );
    s = asctime( gmtime(&timer) );
    s[strlen(s)-1] = '\0';
    printf( "%s VxWorks\n", s );

    get_tod_stamp((timer_t)(CLOCK_REALTIME+1),&ts);
    s = asctime( gmtime((time_t *)&ts.tv_sec) );
    s[strlen(s)-1] = '\0';
    printf( "%s MK48T08\n", s );
    return( 0 );

} /* date() */

char *get_date()
{
    time_t	timer;    
    
    time( &timer );
    return (asctime( gmtime(&timer) ));
}

/************************************************************************/
/* Function    : gmt_help						*/
/* Purpose     : Print usage message for gmtset				*/
/* Inputs      : Return code to return to user				*/
/* Outputs     : Return code						*/
/************************************************************************/
	long
gmt_help( long rtn )
{
    printf("gmthelp: set and get from hardware TOD\n");
    printf("  gmtset hhmmss, yymmdd\n");
    printf("  gmtget (struct tm *)\n");

    return( rtn );

} /* gmt_help() */


/************************************************************************/
/* Function    : gmtset							*/
/* Purpose     : User interface function to set system time, date in GMT*/
/* Inputs      : Time in hhmmss format, (optional) date in yymmdd format*/
/* Outputs     : 0 if success, -1 if failure				*/
/* Comment     : System time is kept as GMT, not local time zone	*/
/************************************************************************/
	long
gmtset( long hhmmss, long yymmdd )
{
    struct tm	*tp;
    time_t	curtime;
    long	rem;

    time( &curtime );			/* Get current time		*/
    tp = gmtime( &curtime );		/* Fill in *tp			*/

    if ( yymmdd )			/* If date is present,		*/
    {					/*   set date			*/
	tp->tm_year = yymmdd / 10000;	/* Get year field		*/
	if ( tp->tm_year < YEAR0 )	/* If < 70, we're in 21st century*/
	    tp->tm_year += 100;
	rem = yymmdd % 10000;
	tp->tm_mon = (rem / 100) - 1;	/* Get month			*/
	tp->tm_mday = rem % 100;	/* Get day			*/
	if ( (tp->tm_mon > 11) || (tp->tm_mday > 31) )
	    return( gmt_help(-1) );	/* Check for valid date		*/
    }
    else if ( hhmmss == 0 )
	return( gmt_help(-1) );	/* If no parms, print usage	*/

    tp->tm_hour = hhmmss / 10000;	/* Get hour			*/
    rem = hhmmss % 10000;
    tp->tm_min = rem / 100;		/* Get minute			*/
    tp->tm_sec = rem % 100;		/* Get second			*/
    if ( (tp->tm_hour > 23) || (tp->tm_min > 59) || (tp->tm_sec > 59) )	
	return( gmt_help(-1) );	/* Check for valid time		*/

    curtime = mktime(tp);
    stime( &curtime );			/* Write the time & date	*/
    return( 0 );

} /* gmtset() */


/************************************************************************/
/* Function    : gmtget							*/
/* Purpose     : User interface function to get system time, date in GMT*/
/* Inputs      : */
/* Outputs     : 				*/
/* Comment     : System time is kept as GMT, not local time zone	*/
/************************************************************************/
	long
gmtget( struct tm *tp )
{

  rtc_read (tp);
  printf (" mon=%d day=%d, year=%d %d:%d:%d\r\n",
        tp->tm_mon,tp->tm_mday,tp->tm_year,tp->tm_hour,tp->tm_min,tp->tm_sec);
  return( 0 );

} /* gmtget() */


/************************************************************************/
/* Function    : get_tod_stamp						*/
/* Purpose     : tod stamp as accurate as possible			*/
/* Inputs      : 0 RTC, 1 MK48T08; address to place time_t in nanosecs	*/
/*			with usec resolution				*/
/* Outputs     : 0 if success, -1 if failure				*/
/************************************************************************/
	long
get_tod_stamp( timer_t timerid, struct timespec *ts )
{
    struct tm 		t;

  switch ((int)timerid)
  {
    default:
    case CLOCK_REALTIME:
	clock_gettime(CLOCK_REALTIME,ts);	/* get TOD to within the tick */
	ts->tv_nsec += (microtime()*1000);	/* add the usec from last tick */
	break;
    case CLOCK_REALTIME+1:
	clock_gettime(CLOCK_REALTIME,ts);	/* get tick precision */
	ts->tv_nsec += (microtime()*1000);	/* get usec from last tick */
  	rtc_read (&t);				/* get hardware TOD */
  	ts->tv_sec=mktime(&t);			/* convert to within a sec */
	break;
  }
  return( 0 );

} /* get_tod_stamp() */

void test_tod()
{
	int i;
	struct timespec tp[20];
	
  for (i=0;i<20;i++)
	get_tod_stamp(CLOCK_REALTIME,&tp[i]);
  for (i=0;i<20;i++)
    printf ("\nsecs=%d, nsecs=%d",tp[i].tv_sec,tp[i].tv_nsec);
  for (i=0;i<20;i++)
	get_tod_stamp((timer_t)(CLOCK_REALTIME+1),&tp[i]);
  for (i=0;i<20;i++)
    printf ("\nsecs=%d, nsecs=%d",tp[i].tv_sec,tp[i].tv_nsec);
}
