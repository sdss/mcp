#include "copyright.h"
/************************************************************************/
/*   File:	bcast.c							*/
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
#include "intLib.h"
#include "iv.h"
#include "memLib.h"
#include "semLib.h"
#include "sigLib.h"
#include "etherLib.h"
/*#include "ifLib.h"*/
#include "socket.h"

struct ifnet *pIf=NULL;
SEM_ID semcast_1Hz=0;
struct TM {
	double pos[2];
	double vel;
	short adc;
};
struct IL {
	short strain_gage;
	short pos;
};
struct CW {
	short pos;
};
struct SDSS_FRAME {
	unsigned char vers;
#define VERSION		1
	unsigned char type;
#define DATA_TYPE	1
	unsigned short binary_len;
	unsigned long ctime;
	struct TM axis[3];
	struct IL inst;
	struct CW weight[4];
	unsigned short ascii_len;
#define ASCII_LEN	80
	unsigned char ascii[ASCII_LEN];
	
};
struct SDSS_FRAME tdt={DATA_TYPE,VERSION};

void bcast1Hz()
{
  struct EtherInit *etherinit;

  int status;
  int e_clock_size;
  int e_clock_hdr;
  STATUS e_status;
  BOOL ether_rcv_UCD();

  semcast_1Hz = semCCreate (SEM_Q_FIFO,0);

/*
  pIf = (struct ifnet *)ifunit ("ei0");
  printf (" pIf = %p\r\n",pIf);
  etherInputHookAdd (ether_rcv_UCD);
*/
  ipsdss_ini();

  FOREVER 
  {
    semTake (semcast_1Hz,WAIT_FOREVER);
    ipsdss_send (&tdt, sizeof(struct SDSS_FRAME));
  }
}

int promisc_rcv=0;
BOOL ether_rcv_UCD(struct ifnet *pIf, char *buffer, char length)
{
  extern unsigned short int Srcnod;
  extern unsigned long int ucd_int15hzcnt;
  int i;
  int byt;

  promisc_rcv++;
  if ((buffer[0]=(char)0xFF)&&
	(buffer[1]==(char)0xFF)&&
	(buffer[2]==(char)0xFF)&&
	(buffer[3]==(char)0xFF)&&
	(buffer[4]==(char)0xFF)&&
	(buffer[5]==(char)0xFF))
  {
      printf ("Broadcast message\r\n");
      printf (" buffer=%x, length=%x\r\n",buffer,length);  
      for (i=0;i<20;i++)
        printf ("%02x ",(short int)(buffer[i])&0xFF);
      printf ("\r\n");
  }
  return FALSE;
}
struct sockaddr_in time_sockaddr;
int time_ms[100];
char time_msg[80];

struct sockaddr_in cast_sockaddr;
int cast_s=-1;
char cast_adr[12];
int ipsdss_ini ()
{
    int optval;
    int cast_port=0x6804;

    cast_s = socket (AF_INET, SOCK_DGRAM, 0);	/* get a udp socket */
    if (cast_s < 0)
    {
      printf ("socket error (errno = %#x)\r\n", errnoGet ());
      return;
    }

    bzero (&cast_sockaddr, sizeof (cast_sockaddr));
    cast_sockaddr.sin_family      = AF_INET;
    cast_sockaddr.sin_port        = htons(cast_port);
    ifBroadcastGet("ei0",&cast_adr[0]);
    cast_sockaddr.sin_addr.s_addr=inet_addr(&cast_adr[0]);

    optval = 1;		/* turn on broadcast */
    if (setsockopt (cast_s, SOL_SOCKET, SO_BROADCAST,
                (caddr_t)&optval, sizeof(optval))<0)
    {
      printf("setsockopt error (errno =  %#x)\r\n", errnoGet ());
      return;
    }
}
int ipsdss_send (char *sdss_msg, int sdss_size)
{
    int msglen;

    if (cast_s>=0) msglen = 
      sendto(cast_s, sdss_msg,sdss_size,0,&cast_sockaddr,sizeof(cast_sockaddr));
/*
    printf("sendto error (errno =  %#x)\r\n", errnoGet ());
    printf ("\r\n%p, %d",sdss_msg,msglen);  
*/
}
