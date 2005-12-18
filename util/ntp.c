#include <vxWorks.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <ctype.h>
#include <time.h>
#include <systime.h>
#include <socket.h>
#include <ioLib.h>
#include <inetLib.h>
#include <sockLib.h>
#include <sntpcLib.h>
#include <sysLib.h>
#include <hostLib.h>
#include <taskLib.h>
#include <in.h>
#include "tod_prototypes.h"
#include "mcpNtp.h"
#include "trace.h"

/*
 * Query the specified NTP server for the time
 *
 * If time is NULL set the time, otherwise return the time to the
 * caller leaving the clock unaffected
 */

int
setTimeFromNTP(const char *NTPserver_name, /* name of NTP server */
	       float timeout,		/* How long to wait for reply (seconds) */
	       unsigned long retryCnt,	/* Retry count (0 means none) */
	       int forceStep,		/* Force step adjustment? */
	       struct timeval *time)	/* if NULL set the time, otherwise
					   return it */
{
   struct timeval time_s;		/* the time to set it time == NULL */
   struct timespec timespec;		/* storage for retrieved time value */
   int    NTPtimeout = timeout * sysClkRateGet(); /* timeout in ticks */
   int    ret;

   /* Bleeping sntpcLib turns sub-second TimeGet timeouts to 0s. */
   if (NTPtimeout < sysClkRateGet()) NTPtimeout = sysClkRateGet();

   assert (forceStep > -100);		/* otherwise it's unused */

   do {
     ret = sntpcTimeGet((char *)NTPserver_name, NTPtimeout, &timespec);
   } while (ret != OK && retryCnt-- != 0);

   if (ret != OK) {
     TRACE(0, "Failed to get time from NTP server %s: %s",	
	   NTPserver_name, strerror(errno));	
     return(ERROR);
   }

   if(time == NULL) {
      time = &time_s;
   }
/*
 * convert timespec to timeval
 */
   time->tv_sec = timespec.tv_sec;
   time->tv_usec = timespec.tv_nsec/1000.0 + 0.5;

   if(time == &time_s) {		/* actually set the time */
      settimeofday(time, NULL);
   }

   return(OK);
}
