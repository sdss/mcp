#include "copyright.h"
/**************************************************************************
***************************************************************************
** FILE:
**      ipcast.c
**
** ABSTRACT:
**	Broadcast a sdss frame as described in data_collection.h at 1 Hz.
**
** ENTRY POINT          SCOPE   DESCRIPTION
** ----------------------------------------------------------------------
** ipsdss_ini		public	initialize ip port for broadcast
** ipsdss_send		public	broadcast a datagram
**
** ENVIRONMENT:
**      ANSI C.
**
** REQUIRED PRODUCTS:
**
** AUTHORS:
**      Creation date:  Aug 30, 1999
**      Charlie Briegel
**
***************************************************************************
***************************************************************************/
/************************************************************************/
/*   File:	ipcast.c						*/
/************************************************************************/
/*   Location:	Fermi National Accelerator Lab				*/
/*   Author:	Charlie Briegel, X4510, MS 360, ALMOND::[BRIEGEL]	*/
/*   Program:	Broadcast info (V1.00) : vxWorks			*/
/*   Modules:	*/
/*++ Version:
  1.00 - initial version
--*/
/*++ Description:
--*/
/*++ Notes:
--*/
/************************************************************************/

/*------------------------------*/
/*	includes		*/
/*------------------------------*/
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

/*========================================================================
**========================================================================
**
** LOCAL MACROS, DEFINITIONS, ETC.
**
**========================================================================
*/
/*------------------------------------------------------------------------
**
** LOCAL DEFINITIONS
*/
static struct sockaddr_in cast_sockaddr;
static int cast_s=-1;
static char cast_adr[12];
/*-------------------------------------------------------------------------
**
** GLOBAL VARIABLES
*/

/*=========================================================================
**=========================================================================
**
** ROUTINE: ipsdss_ini
**
** DESCRIPTION:
**      Initializes ethernet for datagram broadcast on port 0x6804
**
** RETURN VALUES:
**      int	zero or ERROR
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
int ipsdss_ini ()
{
    int optval;
    int cast_port=0x6804;

    cast_s = socket (AF_INET, SOCK_DGRAM, 0);	/* get a udp socket */
    if (cast_s < 0)
    {
      printf ("socket error (errno = %#x)\r\n", errnoGet ());
      return ERROR;
    }

    bzero ((char *)&cast_sockaddr, sizeof (cast_sockaddr));
    cast_sockaddr.sin_family      = AF_INET;
    cast_sockaddr.sin_port        = htons(cast_port);
    ifBroadcastGet("ei0",&cast_adr[0]);
    cast_sockaddr.sin_addr.s_addr=inet_addr(&cast_adr[0]);

    optval = 1;		/* turn on broadcast */
    if (setsockopt (cast_s, SOL_SOCKET, SO_BROADCAST,
                (caddr_t)&optval, sizeof(optval))<0)
    {
      printf("setsockopt error (errno =  %#x)\r\n", errnoGet ());
      return ERROR;
    }
    return 0;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: ipsdss_send
**
** DESCRIPTION:
**      Broadcasts a message
**
** RETURN VALUES:
**      int 	zero or ERROR
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
int ipsdss_send (char *sdss_msg, int sdss_size)
{
    int msglen;

    if (cast_s>=0) msglen = 
      sendto(cast_s, sdss_msg,sdss_size,0,
		(struct sockaddr*)&cast_sockaddr,sizeof(cast_sockaddr));
/*
    printf("sendto error (errno =  %#x)\r\n", errnoGet ());
    printf ("\r\n%p, %d",sdss_msg,msglen);  
*/
    return 0;	
}
