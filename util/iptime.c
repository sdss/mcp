/*%+copyright
*  Copyright (c) 1992 	Universities Research Association, Inc. 
*  			All Rights Reserved
* 
*				DISCLAIMER
* 
*  The software is licensed on an "as is" basis only.  Universities Research
*  Association, Inc. (URA) makes no representations or warranties, express
*  or implied.  By way of example but not of limitation, URA makes no
*  representations or WARRANTIES OF MERCHANTABILITY OR FITNESS FOR ANY
*  PARTICULAR PURPOSE, that the use of the licensed software will not
*  infringe any patent, copyright, or trademark, or as to the use (or the
*  results of the use) of the licensed software or written material in
*  terms of correctness, accuracy, reliability, currentness or otherwise.
*  The entire risk as to the results and performance of the licensed
*  software is assumed by the user.  In consideration for the licensed use 
*  of the software, each user agrees to indemnify and hold URA (and any of 
*  its trustees, overseers, directors, officers, employees, agents or 
*  contractors) and the US Government harmless for all damages, costs, and
*  expenses, including attorney's feed, arising from the commercialization 
*  and utilization of the licensed software, including, but not limited to,
*  the making, using, selling or exporting of products, processes, or 
*  services derived from the licensed software.
*
*  NOTICE: The Government is granted for itself and others acting on its behalf
*  a paid-up, nonexclusive, irrevocable worldwide license in this data to 
*  reproduce, prepare derivative works, perform publicly and display publicly.
*  Beginning five (5) years after April 15, 1992 the Government is granted 
*  for itself and others acting on its behalf a paid-up, nonexclusive, 
*  irrevocable worldwide license in this data to reproduce, prepare 
*  derivative works, distribute copies to the public, perform publicly and 
*  display publicly, and to permit others to do so.   NEITHER THE UNITED 
*  STATES NOR THE UNITED STATES DEPARTMENT OF ENERGY, NOR ANY OF THEIR 
*  EMPLOYEES, MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR ASSUMES ANY LEGAL 
*  LIABILITY OR RESPONSIBILITY FOR THE ACCURACY, COMPLETENESS, OR USEFULNESS 
*  OF ANY INFORMATION, APPARATUS, PRODUCT, OR PROCESS DISCLOSED, OR 
*  REPRESENTS THAT ITS USE WOULD NOT INFRINGE PRIVATELY OWNED RIGHTS.
-%*/
/************************************************************************/
/*   File:	IPIni.c						*/
/************************************************************************/
/*   Location:	Fermi National Accelerator Lab				*/
/*   Author:	Charlie Briegel, X4510, MS 360, ALMOND::[BRIEGEL]	*/
/*   Program:	ACNET UTI (V1.05) : MTOS				*/
/*   Modules:	ipini : 						*/
/*++ Version:
  1.00 - initial version - 29 Dec 92
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
#include "types.h"
/*#include "netdb.h"*/
#include "inetLib.h"
#include "stdioLib.h"
#include "strLib.h"
#include "sockLib.h"
#include "hostLib.h"
#include "ioLib.h"
#include "fioLib.h"
#include "semLib.h"
#include "usrLib.h"
#include "errnoLib.h"
#include "time.h"

struct sockaddr_in time_sockaddr;
char time_msg[100];
void iptimeGet ();

void iptimeGet (char *node)
{

    int time_s;
    int time_port=13;
    int msglen;


    time_s = socket (AF_INET, SOCK_STREAM, 0);	/* get a udp socket */
    if (time_s < 0)
    {
      printf ("socket error (errno = %#x)\r\n", errnoGet ());
      return;
    }

    bzero ((char *)&time_sockaddr, sizeof (time_sockaddr));
    time_sockaddr.sin_family      = AF_INET;
    time_sockaddr.sin_port        = time_port;

    if (bind (time_s, 
	(struct sockaddr *)&time_sockaddr, sizeof (time_sockaddr)) == ERROR)
    {
      printf ("time bind error (errno = %#x)\r\n", errnoGet ());
      return;
    }

    if ((time_sockaddr.sin_addr.s_addr = hostGetByName (node))==ERROR)
    {
      printf ("time host addr error (errno = %#x)\r\n", errnoGet ());
      return;
    }
    
    if (connect (time_s,
	(struct sockaddr *)&time_sockaddr,sizeof(time_sockaddr))==ERROR)
    {
      printf ("time connect error (errno = %#x)\r\n", errnoGet ());
      return;
    }

    msglen = read (time_s, &time_msg[0], sizeof(time_msg));

/*    
    msglen = recvfrom (time_s, &time_msg[0], sizeof(time_msg), 0,
			(struct sockaddr *)&time_sockaddr, &sockaddrLen);
*/
    printf ("\r\n%s, %d",&time_msg[0],msglen);
    
    close (time_s);

}
void iptimeSet (char *node, int tzadj)
{

    int time_s;
    int time_port=13;
    int msglen;
    struct timeval	tp;
    struct tm t;
    int dattim;


    time_s = socket (AF_INET, SOCK_STREAM, 0);	/* get a udp socket */
    if (time_s < 0)
    {
      printf ("socket error (errno = %#x)\r\n", errnoGet ());
      return;
    }

    bzero ((char *)&time_sockaddr, sizeof (time_sockaddr));
    time_sockaddr.sin_family      = AF_INET;
    time_sockaddr.sin_port        = time_port;

    if (bind (time_s, 
	(struct sockaddr *)&time_sockaddr, sizeof (time_sockaddr)) == ERROR)
    {
      printf ("time bind error (errno = %#x)\r\n", errnoGet ());
      return;
    }

    if ((time_sockaddr.sin_addr.s_addr = hostGetByName (node))==ERROR)
    {
      printf ("time host addr error (errno = %#x)\r\n", errnoGet ());
      return;
    }
    
    if (connect (time_s,
	(struct sockaddr *)&time_sockaddr,sizeof(time_sockaddr))==ERROR)
    {
      printf ("time connect error (errno = %#x)\r\n", errnoGet ());
      return;
    }

    msglen = read (time_s, &time_msg[0], sizeof(time_msg));

    printf ("\r\n%s, %d",&time_msg[0],msglen);
    
    close (time_s);
    
    sscanf(&time_msg[20],"%4d",&dattim);
    t.tm_year = dattim-1900;
    switch (time_msg[4]){
    case 'A':		/* APR AUG */
     if (time_msg[5]=='p') t.tm_mon=4;
     else t.tm_mon=8;
     break;
    case 'D':		/* DEC */
     t.tm_mon=12;
     break;
    case 'F':		/* FEB */
     t.tm_mon=2;
     break;
    case 'J':		/* JAN JUN JUL */
     if (time_msg[5]=='a') t.tm_mon=1;
     else if (time_msg[6]=='n') t.tm_mon=6;
     else t.tm_mon=7;
     break;
    case 'M':		/* MAR MAY */
     if (time_msg[5]=='r') t.tm_mon=3;
     else t.tm_mon=5;
     break;
    case 'N':		/* NOV*/
     t.tm_mon=11;
     break;
    case 'O':		/* OCT */
     t.tm_mon=10;
     break;
    case 'S':		/* SEP*/
     t.tm_mon=9;
     break;
    }
    t.tm_mon--;
    sscanf(&time_msg[8],"%2d",&dattim);
    t.tm_mday = dattim;
    sscanf(&time_msg[11],"%2d",&dattim);
    t.tm_hour = dattim;
    sscanf(&time_msg[14],"%2d",&dattim);
    t.tm_min = dattim;
    sscanf(&time_msg[17],"%2d",&dattim);
    t.tm_sec = dattim;
    tp.tv_sec=mktime(&t);
    tp.tv_sec += tzadj*3600;  /* for SDSS */     
    tp.tv_usec=0;
    printf (" sec=%d, nano_sec=%d\r\n",tp.tv_sec,tp.tv_usec);
    stime (&tp);
    clock_settime(CLOCK_REALTIME,(struct timespec *)&tp);
}
