/************************************************************************/
/* $Header$		*/
/* Summary  : Unix-compatible Time Definitions and Declarations		*/
/* Filename : usrTime.h							*/
/* Author   : Bob Herlien (rah)						*/
/* Project  : ROV 1.5							*/
/* $Revision$							*/
/* Created  : 02/28/91							*/
/************************************************************************/
/* Modification History:						*/
/* $Log$
 * Revision 1.2  2000/01/26 08:31:07  ekinney
 * Charlie's edits 25 Jan 2000
 *
/* Revision 1.1.1.1  1999/04/14 14:59:57  yanny
/* inital rev
/*
 * Revision 1.1.1.1  1999/04/14 05:33:46  spock
 * First entry of the MCP into CVS by matt newcomb
 * I made an attempt to cleanup the origional mcp directory.  It was hard
 * to tell what code was actually being used, so I could have made a mistake.
 * Please check to make sure everything is there, and if there is something that
 * you don't need, please remove it.
 *
 * Revision 1.1  91/09/06  16:05:01  16:05:01  hebo (Bob Herlien 408-647-3748)
 * Various minor fixes, found while porting ntp
 * 
 * Revision 1.0  91/08/09  11:10:57  11:10:57  hebo (Bob Herlien 408-647-3748)
 * Initial revision
 * 	*/
/* 28feb91 rah, Ported from HP-UX <sys/time.h> and vxWorks <systime.h>	*/
/************************************************************************/
/* Note that, with all the conditionals below, we're trying not to	*/
/* conflict with: vxWorks utime.h, systime.h, types.h; HP-UX time.h or	*/
/* sys/types.h; or SUN types.h.  We may or may not have succeeded.	*/
/************************************************************************/

#if !defined(INCusrtimeh) && !defined(_SYS_TIME_INCLUDED)
#define INCusrtimeh	1

#ifndef NULL
#define NULL (char *)0
#endif

#define SECS_PER_DAY	86400L		/* Seconds per day		    */
#define USECS_PER_SEC	1000000L	/* Microseconds per second	    */

#define YEAR0		70		/* Unix Epoch year (1970)	    */
#define DAY0		4		/* 1jan1970 was a Thursday	    */

#if !defined(_TYPES_) && !defined(__sys_types_h) && !defined(_TIME_T) && \
    !defined(__INCvxTypesh)
typedef long time_t;
#endif
#ifdef WIPEOUT
struct tm				/* Structure returned by gmtime()   */
{					/*   localtime() calls		    */
    int	tm_sec;
    int	tm_min;
    int	tm_hour;
    int	tm_mday;
    int	tm_mon;
    int	tm_year;
    int	tm_wday;
    int	tm_yday;
    int	tm_isdst;
};
#endif
#if !defined(INCtimeh) && !defined(__INCtimesh)
#define _TIME_				/* Prevent conflict with sys/times.h*/
struct timeval 				/* Struct returned by gettimeofday()*/
{					/*    system call		    */
    long	tv_sec;			/* seconds			    */
    long	tv_usec;		/* microseconds			    */
};

struct timezone
{
    int	tz_minuteswest;			/* minutes west of Greenwich	    */
    int	tz_dsttime;			/* type of dst correction	    */
};					/* dst not implemented (always 0)   */
#endif /* INCtimeh */


#  ifdef __STDC__
extern int	  stime(time_t *);
extern int	  gettimeofday( struct timeval *tp, struct timezone *tzp );
extern int 	  settimeofday(struct timeval *, struct timezone *);
extern long	  gmtget( struct tm *);
extern long 	  get_tod_stamp( timer_t, struct timespec *);
#  else /* __STDC__ */
extern int	  stime();
extern int 	  gettimeofday();
extern int 	  settimeofday();
extern long	  gmtget();
extern long 	  get_tod_stamp();
#  endif /* __STDC__ */

extern long timezone;

#endif /* INCusrtimeh || _SYS_TIME_INCLUDED */



