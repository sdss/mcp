#include "copyright.h"
/*
**	TCC Command handler both receives, distributes, and answers on
**	behalf of the action routines.
**	cmd_handler can be invoked from the serial driver or the shell
**	so access is arbitrated by a semaphore
*/
#include "vxWorks.h"
#include "stdio.h"
#include "intLib.h"
#include "iv.h"                                   
#include "semLib.h"                              
#include "memLib.h"
#include "string.h"
#include "ctype.h"
#include "wdLib.h"
#include "tyLib.h"
#include "ioLib.h"
#include "sysLib.h"
#include "taskLib.h"
#include "rebootLib.h"
#include "mv162IndPackInit.h"
#include "ipOctalSerial.h"
#include "serial.h"
#include "cmd.h"
#include "dscTrace.h"
#include "axis.h"

/*------------------------------------------------------------------------
**
** LOCAL DEFINITIONS
*/

struct SERIAL_CAN {
	int cancel;
	int active;
	int count;
	unsigned short tmo;
	unsigned short init_tmo;
	unsigned short min_tmo;
	int fd;
	FILE *stream;
};
#define SERIAL_DELAY	50
#define NPORT	4
struct BARCODE {
	short axis;
	short fiducial;
};
/*-------------------------------------------------------------------------
**
** GLOBAL VARIABLES
*/
struct SERIAL_CAN cancel[NPORT]= {
			{FALSE,FALSE,0,0,SERIAL_DELAY,0,NULL},
			{FALSE,FALSE,0,0,SERIAL_DELAY,0,NULL},
			{FALSE,FALSE,0,0,SERIAL_DELAY,0,NULL},
			{FALSE,FALSE,0,0,SERIAL_DELAY,0,NULL}
};
struct BARCODE barcode[256];
short barcodeidx=0;
int BARCODE_verbose=FALSE;
/*
 * prototypes
 */
void barcode_shutdown(int type);

/*=========================================================================
**=========================================================================
**
** ROUTINE: sdss_transmit
**
** DESCRIPTION:
**      Responds with original query plus any answer and terminated with OK.
**
** RETURN VALUES:
**      int 	always zero
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
int
sdss_transmit(FILE *output_stream,
	      unsigned char *const cmd,
	      unsigned char *const buffer)
{
   int status;

   /* Initialize if necessary */                 

   /* Transmit to the TCC */
   if (buffer==NULL)	/* if string is null, "\r\n%s" outputs "\r(null)" */
     status=fprintf ( output_stream, "%s  OK\n\r",  cmd );
   else
     status=fprintf ( output_stream, "%s\n\r%s  OK\n\r",  cmd,buffer );

   fflush ( output_stream);
   return 0;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: sdss_receive
**
** DESCRIPTION:
**      Receives string from TCC with appropriate termination.
**
** RETURN VALUES:
**      int
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
static int
sdss_receive(FILE *input_stream,
	     int port,
	     unsigned char *buffer,
	     int size)			/* size of buffer */
{
   int c;
   int status;
   unsigned char *ptr;
   int charcnt;

   /* Initialize if necessary */
   status = 0;

   /* Receive data from the TCC */
/*#define __USE_GETC__	1*/
#ifndef __USE_GETC__
   fgets(buffer, size, input_stream);
#else
   ptr    = buffer;
   charcnt=0;
   while ( ((c = getc(input_stream)) != EOF)&&(charcnt<size))
   {
      *ptr=c;
      if ( *ptr == '\015' )                 /* All done */
         break;
      ptr++;
      charcnt++;
   }
   if (charcnt>=size) return -1;
   while (*(--ptr)==0x20);
   ptr++;
   *ptr = '\0';
#endif          /* End __USE_GETC__ */

   /* Check the length */
#if 0
   if(strlen(buffer) < 1) {
      return -1;
   }
#endif

   return status;        
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tcc_serial
**
** DESCRIPTION:
**      Distributes string received over second serial port to command handler
**	and responds with answer.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**	cmd_handler
**	sdss_receive
**	sdss_transmit
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void
tcc_serial(int port)
{
  char serial_port[] = "/tyCo/x";
  FILE *stream;  
  int status;
  char command_buffer_in[256];
  char command_buffer[256];
  char *answer_buffer;

  sprintf(serial_port,"/tyCo/%d",port);
  stream = fopen (serial_port,"r+");
  if(stream == NULL) {
     TRACE(0, "Could **NOT** open port", 0, 0);
     exit (-1);
  }
  
  TRACE(1, "OPEN port %s", serial_port, 0);
  ioctl (fileno(stream),FIOBAUDRATE,9600);

  for(;;) {
     TRACE(16, "port %d", port, 0);

     command_buffer[0] = '\0';
     status = sdss_receive(stream, port, command_buffer, 256);
     {
	int lvl = 5;
	if(strstr(command_buffer, "STATUS") != NULL) {
	   lvl += 2;
	}
 	TRACE(lvl, "command from TCC: %s", command_buffer, 0);
	TRACE(16, "        cccccccc: 0x%08x%08x",
	      ((int *)command_buffer)[0], ((int *)command_buffer)[1]);
	TRACE(16, "        time = %f", sdss_get_time(), 0);
     }
     
     if(status == 0) {
	strcpy(command_buffer_in, command_buffer);
	answer_buffer = cmd_handler(1, command_buffer);	/* allow all cmds */
	status = sdss_transmit(stream, command_buffer_in, answer_buffer);
	if(status != 0) {
	   TRACE(2, "TCC **NOT** accepting response (status=%d)", status, 0);
	}
     } else {
	TRACE(2, "TCC **BAD** command %s (status=%d)\r\n", command_buffer, status);
	status = sdss_transmit(stream, command_buffer, "ERR: Bad Command");
	if(status != 0) {
	   TRACE(2, "TCC **NOT** accepting response (status=%d)", status, 0);
	}
     }
  }
}                                             
/*=========================================================================
**=========================================================================
**
** ROUTINE: barcode_init
**
** DESCRIPTION:
**      Initializes barcode serial port driver using 8-port serial IP.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**	Industry_Pack
**	octSerModuleInit
**	octSerDevCreate
**	VME_IP_Interrupt_Enable
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void barcode_init(unsigned char *ip_base, unsigned short model, 
		unsigned int vec, short ch)
{
  struct IPACK ip;
  char devName[32];
  OCT_SER_DEV *pOctSerDv;
  int i,j;
  unsigned long base;

  Industry_Pack (ip_base,model,&ip);
  for (i=0;i<MAX_SLOTS;i++)
  {
    if (ip.adr[i]!=0)
    {
      base = *((short *)(IP_MEM_BASE_BASE+(i*2)));
      printf ("\r\n base=%x, %p, %p",base, (char *)(base<<16),
		(char *)((long)base<<16));
      pOctSerDv=octSerModuleInit((char *)(((long)(base)<<16)+1),
		ip.adr[i],vec);
      printf ("\r\n pOctSerDv=%p, ip.adr[i]=%p",pOctSerDv,ip.adr[i]);
      if (octSerDrv() == ERROR)
        printf ("\r\n Driver installation failed");
      else
      {
        for (j=0;j<2/*N_CHANNELS*/;j++)
        {
          sprintf(devName,"/tyCo/%d", ch+j);
          if (octSerDevCreate(&pOctSerDv[j],devName,512,512))
          {
            printf("\r\n Device Creation failed, %s",devName);
            break;
          }
          else
            printf("\r\n pOctSerDv[j]=%p, %p",&pOctSerDv[j],&pOctSerDv[j].clkSelVal);
        }
      }
      VME_IP_Interrupt_Enable (ip.adr[i],5);
      sysIntEnable (5);
      break;
    }
  }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: barcode_open
**
** DESCRIPTION:
**      Open the serial port for the associated barcode.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**	barcode_shutdown
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
FILE *
barcode_open(int port)
{
  char serial_port[] = "/tyCo/x";
  FILE *stream;

  sprintf(serial_port,"/tyCo/%d",port);
  stream = fopen (serial_port,"r+");
  if (stream==NULL)
  {
    printf ("barcode_open: Could **NOT** open port\r\n");
    return NULL;
  }
  else
    printf ("barcode_open:  OPEN port %s, stream=%p\r\n",serial_port,stream);
  ioctl (fileno(stream),FIOBAUDRATE,2400);
  cancel[port].fd=fileno(stream);
  cancel[port].stream=stream;
  rebootHookAdd((FUNCPTR)barcode_shutdown);
  
  return stream;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: barcode_shutdown
**
** DESCRIPTION:
**      Shutdown routine for software reboot to close barcode serial ports.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void barcode_shutdown(int type)
{
  int i;

    printf("BARCODE OCTAL232 shutdown:\r\n");
    for (i=0;i<NPORT;i++) {
       if (cancel[i].fd!=NULL) {
	  close(cancel[i].fd);
	  cancel[i].fd=0;
	  printf ("\r\n close fd=%x",cancel[i].fd);
       }
    }
  
    taskDelay (30);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: barcode_serial
**
** DESCRIPTION:
**      Starts the query to read the barcode.  This is initiated when the 
**	reader is aligned with the tape and the UPC.
**
** RETURN VALUES:
**      int
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
int
barcode_serial(int port)
{
  FILE *stream;  
  int status;
  char barcode_buffer[256];
  int c;
  unsigned char *ptr;
  char *bf;
  int fiducial;
  int charcnt;
 
  if (cancel[port].fd==NULL)
    stream=barcode_open(port);
  else
    stream=cancel[port].stream;
  if (stream==NULL) return ERROR;

  taskLock();
  cancel[port].cancel=FALSE;
  cancel[port].active=TRUE;
  cancel[port].tmo=cancel[port].init_tmo;
  taskUnlock();

  status=fprintf ( stream, "\00221\003");  /* query string */
  if ( status != 4 )	/* four characters sent */
    printf (" BARCODE **NOT** accepting response (status=%d)\r\n",status);
  else
  {		/* receive answer */
    barcode_buffer[0]=NULL;
    ptr    = &barcode_buffer[0];
    charcnt=0;
    while ( ((c = getc(stream)) != EOF)&&(charcnt<256))
    {
      *ptr=c;
      if (cancel[port].cancel) return -1;
/*        printf ( "\n\tIn receive() -  character is %c\t%02x %02x",
		 *ptr, *ptr, c);*/
      if ( *ptr == '\003' )                 /* All done */
        break;
      ptr++;
      charcnt++;
    }
    while ( ((c = getc(stream)) != EOF)&&(charcnt<256))
    {
      *ptr=c;
      if (cancel[port].cancel) return -1;
/*        printf ( "\n\tIn receive() -  character is %c\t%02x %02x",
		 *ptr, *ptr, c);*/
      if ( *ptr == '\003' )                 /* All done */
        break;
      ptr++;
      charcnt++;
    }
    if (charcnt>=256) return -1;
    *ptr=NULL;
    if (cancel[port].tmo<cancel[port].min_tmo)
      cancel[port].min_tmo=cancel[port].tmo;
    cancel[port].active=FALSE;
    cancel[port].tmo=0;
    if (cancel[port].cancel) printf ("\r\n receive got canceled");
   
    if (BARCODE_verbose)
      printf ("\r\n%s",&barcode_buffer[0]);
    bf=strstr (&barcode_buffer[0],"ALT");
    if (bf!=NULL) 
    {
      sscanf (bf+3,"%3d",&fiducial);
      barcode[barcodeidx].axis=1;
    }
    bf=strstr (&barcode_buffer[0],"AZ");
    if (bf!=NULL)
    {
      sscanf (bf+2,"%3d",&fiducial);
      barcode[barcodeidx].axis=0;
    }
    barcode[barcodeidx].fiducial=fiducial;
    barcodeidx = (barcodeidx+1)%256;
    return fiducial;
  }
  barcode[barcodeidx].fiducial=0;
  barcode[barcodeidx].axis=-1;
  barcodeidx = (barcodeidx+1)%256;
  return -1;
}                                             
/*=========================================================================
**=========================================================================
**
** ROUTINE: print_barcode
**
** DESCRIPTION:
**      Diagnostic of barcode fiducials passed.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void
print_barcode(void)
{
  int i;

  printf ("\r\nBARCODE idx = %d", barcodeidx);
  for (i=0;i<256;i++) 
    printf ("\r\n  %d: axis=%d, fiducial=%d",i,barcode[i].axis,
		barcode[i].fiducial);
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: cancel_read
**
** DESCRIPTION:
**      If getc is used for serial communicaitons, then use a global timeout
**	for all 8 ports.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
#ifdef __USE_GETC__
void
cancel_read(void)
{
  int status;
  int i;

  for(i = 0; i < NPORT; i++) {
     cancel[i].cancel = FALSE;
     cancel[i].active = FALSE;
     cancel[i].min_tmo = cancel[i].init_tmo;
     cancel[i].tmo = 0;
     cancel[i].fd = 0;
  }

  for(;;) {
    taskDelay (1);

    for(i = 0; i < NPORT; i++) {
       TRACE(16, "port %d", i, 0);
       
       taskLock();
       TRACE(16, "task is locked", 0, 0);
       if(cancel[i].tmo > 0 && cancel[i].active) cancel[i].tmo--;
       
       if (cancel[i].tmo > 0 && !cancel[i].active) {
	  taskUnlock();
	  TRACE(4, "not active; port %d (tmo=%d)", i, cancel[i].tmo);
	  cancel[i].tmo = 0;
       } else {
	  taskUnlock();
       }
       TRACE(16, "task is unlocked", 0, 0);
       
       if(cancel[i].tmo == 1) {
	  status = ioctl(cancel[i].fd, FIOCANCEL, 0);
	  cancel[i].cancel = TRUE;
	  cancel[i].count++;
	  TRACE(4, "canceled port %d (status=0x%x)", i, status);
       } else {
	  TRACE(16, "port %d tmo = %d", i, cancel[i].tmo);
       }
    }
  }
}
#endif          /* End __USE_GETC__ */
