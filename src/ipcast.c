/*
 *	Broadcast a sdss frame as described in data_collection.h at 1 Hz.
 */
#include "vxWorks.h"
#include "stdio.h"
#include "intLib.h"
#include "iv.h"
#include "memLib.h"
#include "semLib.h"
#include "sigLib.h"
#include "etherLib.h"
#include "ifLib.h"
#include "string.h"
#include "inetLib.h"
#include "errnoLib.h"
#include "sockLib.h"
#include "socket.h"
#include "dscTrace.h"
#include "as2.h"

/*------------------------------------------------------------------------
**
** LOCAL DEFINITIONS
*/
static struct sockaddr_in cast_sockaddr;
static int cast_s=-1;
static char cast_adr[12];


/*****************************************************************************/
/*
 * Initialize socket for datagram broadcast
 *
 * Returns 0 or ERROR
 */
int
ipsdss_ini(void)
{
   int uid = 0, cid = 0;   
   int optval;
   int cast_port=0x6804;
   
   cast_s = socket(AF_INET, SOCK_DGRAM, 0);	/* get a udp socket */
   if(cast_s < 0) {
      NTRACE_2(0, uid, cid, "ipsdss_ini: creating socket: %d %s", errno, strerror(errno));
      return ERROR;
   }
   
   bzero ((char *)&cast_sockaddr, sizeof (cast_sockaddr));
   cast_sockaddr.sin_family      = AF_INET;
   cast_sockaddr.sin_port        = htons(cast_port);
   ifBroadcastGet("ei0",&cast_adr[0]);
   cast_sockaddr.sin_addr.s_addr=inet_addr(&cast_adr[0]);
   
   optval = 1;				/* turn on broadcast */
   if(setsockopt(cast_s, SOL_SOCKET, SO_BROADCAST,
		 (caddr_t)&optval, sizeof(optval)) < 0) {
      NTRACE_2(0, uid, cid, "ipsdss_ini: setsockopt: %d %s", errno, strerror(errno));
      return ERROR;
   }
   
   return 0;
}

/*****************************************************************************/
/*
 * Broadcast a message
 */
void
ipsdss_send(char *sdss_msg,		/* message to broadcast */
	    int sdss_size)		/* length of message */
{
    int uid = 0, cid = 0;   

    if(cast_s < 0) {			/* socket isn't open */
       return;
    }
    
    if(sendto(cast_s, sdss_msg, sdss_size, 0,
	      (struct sockaddr *)&cast_sockaddr, sizeof(cast_sockaddr)) < 0) {
       NTRACE_2(0, uid, cid, "couldn't broadcast sdss_msg: %d %s",
		errno, strerror(errno));
    }
}
