#include <vxWorks.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <ctype.h>
#include <systime.h>
#include <socket.h>
#include <ioLib.h>
#include <inetLib.h>
#include <sockLib.h>
#include <hostLib.h>
#include <taskLib.h>
#include <in.h>
#include "tod_prototypes.h"
#include "ntp.h"
#include "mcpNtp.h"
/*
 * Query the specified NTP server for the time
 *
 * If time is NULL set the time, otherwise return the time to the
 * caller leaving the clock unaffected
 */
int
setTimeFromNTP(const char *NTPserver_name, /* name of NTP server */
	       int retryDelay,		/* Delay between retries (ticks) */
	       unsigned long retryCnt,	/* Retry count (0 means none) */
	       int forceStep,		/* Force step adjustment? */
	       struct timeval *time)	/* if NULL set the time, otherwise
					   return it */
{
   int i;
   struct ntpdata NTPstamp;		/* the desired timestamp */
   struct sockaddr_in server;		/* server's address */
   int sock;				/* server socket */
   unsigned long NTPserver;		/* numerical IP address of NTP server*/
   struct timeval time_s;		/* the time to set it time == NULL */

   if(time == NULL) {
      time = &time_s;
   }
/*
 * Convert that server name; first dotted notation then a name
 */
   if((NTPserver = inet_addr(NTPserver_name)) == -1) {
      NTPserver = hostGetByName((char *)NTPserver_name);
   }
/*
 * create socket
 */
   if((sock = socket(PF_INET, SOCK_DGRAM, 0)) == -1) {
      fprintf(stderr,"Creating socket: %s", strerror(errno));
      return(-1);
   }
/*
 * setup local address
 */
   memset(&server, sizeof(server), '\0');
   server.sin_family = PF_INET;
   server.sin_port = htons(NTP_PORT);
   server.sin_addr.s_addr = htonl(NTPserver);
/*
 * connect to server
 */
   if(connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
      fprintf(stderr,"Failed to connect to NTP server %d: %s\n",
	      NTPserver, strerror(errno));
      return(-1);
   }
/*
 * setup the timestamp
 */
   memset(&NTPstamp, '\0', sizeof (NTPstamp));
   
   NTPstamp.status  = NO_WARNING | NTPVERSION_1 | MODE_CLIENT;
   NTPstamp.stratum = UNSPECIFIED;
   NTPstamp.ppoll   = 0;
/*
 * send request
 */
   if(send(sock, (void *)&NTPstamp, sizeof(NTPstamp), 0) < 0) {
      fprintf(stderr,"Failed to send to NTP server: %s\n",
	      strerror(errno));
      return(-1);
   }
/*
 * Receive our reply
 */
   for(i = 0; i < retryCnt; i++) {
      if(recvfrom(sock, (void *)&NTPstamp, sizeof(NTPstamp),
		  0, NULL, 0) > 0) {
	 break;
      }
      
      taskDelay(retryDelay);
   }
   close(sock);
   
   if(i == retryCnt) {
      fprintf(stderr,"Failed to recvfrom NTP server after %d tries: %s\n",
	      retryCnt, strerror(errno));
      return(-1);
   }
/*
 * Got an NTP timestamp.  Set the time
 *
 *   o   NTP timestamps consists of two 32-bit fields representing a fixed
 *       point number.  The first field is an integer count of seconds since
 *       Jan. 1, 1900.  The second is the fraction with a resolution of about
 *       232+ picoseconds.
 *
 *       Conversion from 232+ picoseconds units (32-bit fraction) to micro-
 *       seconds is done as:
 *                              6                6
 *                  ntpFrac * 10      ntpFrac * 5
 *          usec = --------------- = --------------
 *                         32               26
 *                        2                2
 */
   if(NTPstamp.xmt.int_part <= JAN_1970) { /* this is a problem */
      fprintf(stderr,"The time appears to be before Jan 1 1970\n");
      return(-1);
   }

   time->tv_sec  = NTPstamp.xmt.int_part - JAN_1970 + forceStep;
   time->tv_usec = (unsigned long)((NTPstamp.xmt.fraction*15625) >> 26);

   if(time == &time_s) {		/* actually set the time */
      settimeofday(time, NULL);
   }

   return(0);
}
