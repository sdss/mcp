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
#include "taskLib.h"
#include "mv162IndPackInit.h"
#include "ipOctalSerial.h"


char *cmd_handler(char *cmd);

/* Define non-printing (control) characters */

#ifndef __ASCII_MACROS__
#define SOH '\001'
#define STX '\002'
#define ACK '\006'
#define NAK '\025'
#define CR  '\015'
#define DLE '\020'
#endif
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
struct SERIAL_CAN cancel[NPORT]= {
			{FALSE,FALSE,0,0,SERIAL_DELAY,0,0},
			{FALSE,FALSE,0,0,SERIAL_DELAY,0,0},
			{FALSE,FALSE,0,0,SERIAL_DELAY,0,0},
			{FALSE,FALSE,0,0,SERIAL_DELAY,0,0}
};
void barcode_shutdown(int type);

/* Static variables */

int sdss_transmit ( FILE *output_stream, unsigned char * const cmd,
		unsigned char * const buffer )
{
   int status;

   /* Initialize if necessary */                 

   /* Transmit to the TCC */
   if (buffer==NULL)	/* if string is null, "\r\n%s" outputs "\r(null)" */
     status=fprintf ( output_stream, "%s  OK\n\r",  cmd );
   else
     status=fprintf ( output_stream, "%s\r\n%s  OK\n\r",  cmd,buffer );

   fflush ( output_stream);
   return 0;
                          
}

int sdss_receive ( FILE *input_stream, int port, unsigned char  *buffer)
{
   int i;
   int c;
   int status;
          unsigned char *ptr;


   /* Initialize if necessary */
   status = 0;

   /* Receive data from the TCC */
/*#define __USE_GETC__	1*/
#ifndef __USE_GETC__
/*   fscanf ( input_stream, "%s", buffer );*/
   fgets (buffer,40,input_stream);
#else
/*
   taskLock();
   cancel[port].cancel=FALSE;
   cancel[port].active=TRUE;
   cancel[port].tmo=cancel[port].init_tmo;
   taskUnlock();
   cancel[port].fd=fileno(input_stream);
*/
   ptr    = buffer;
   while ( ( c = getc ( input_stream ) ) != EOF )  
   {
      *ptr=c;
/*      if (cancel[port].cancel) */
/*        printf ( "\n\tIn receive() -  character is %c\t%02x %02x",
		 *ptr, *ptr, c);*/
      if ( *ptr == '\015' )                 /* All done */
         break;
      ++ptr;
   }
   while (*(--ptr)==0x20);
   ptr++;
   *ptr = '\0';
/*
   if (cancel[port].tmo<cancel[port].min_tmo) 
		cancel[port].min_tmo=cancel[port].tmo;
   cancel[port].tmo=0;
   cancel[port].active=FALSE;
   if (cancel[port].cancel) printf ("\r\n receive got canceled");
*/
#endif          /* End __USE_GETC__ */

   /* Check the length */
/*   if ( strlen(buffer) < 1 )
      return -1;*/

   return status;        

}

tcc_serial(int port)
{
  char *serial_port={"/tyCo/x"};
  FILE *stream;  
  int status;
  char command_buffer[120];
  char *answer_buffer;

  sprintf(serial_port,"/tyCo/%d",port);
  stream = fopen (serial_port,"r+");
  if (stream==NULL)
  {
    printf ("tcc_serial: Could **NOT** open port\r\n");
    exit (-1);
  }
  else
    printf ("tcc_serial:  OPEN port %s, stream=%p\r\n",serial_port,stream);
  ioctl (fileno(stream),FIOBAUDRATE,9600);

  FOREVER
  {
   command_buffer[0]=NULL;
   status = sdss_receive ( stream, port, &command_buffer[0]);
   if ( status==0 )
   {
     answer_buffer=cmd_handler(&command_buffer[0]);
     status = sdss_transmit (stream,&command_buffer[0],answer_buffer);
     if ( status != 0 )
       printf (" TCC **NOT** accepting response (status=%d)\r\n",status);
   }
   else
   {
     printf (" TCC **BAD** command %s (status=%d)\r\n",&command_buffer[0],status);
/*     status = sdss_transmit (stream,&command_buffer[0],NULL);
     if ( status != 0 )
       printf (" TCC **NOT** accepting response (status=%d)\r\n",status);*/
   }
  }                                             
}                                             
char barcode_buffer[256];
struct BARCODE {
	short axis;
	short fiducial;
};
struct BARCODE barcode[256];
short barcodeidx=0;
int BARCODE_verbose=FALSE;
barcode_init(unsigned char *ip_base, unsigned short model, short vec, short ch)
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
      printf ("\r\n pOctSerDv=%p, ip.adr[i]=%x",pOctSerDv,ip.adr[i]);
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
barcode_open(int port)
{
  char *serial_port={"/tyCo/x"};
  FILE *stream;
  int status;
  char command_buffer[120];
  char *answer_buffer;
  int c;
  unsigned char *ptr;
  char *az_bf;
  int az_fiducial;

  sprintf(serial_port,"/tyCo/%d",port);
  stream = fopen (serial_port,"r+");
  if (stream==NULL)
  {
    printf ("barcode_serial: Could **NOT** open port\r\n");
    return -1;
  }
  else
    printf ("barcode_serial:  OPEN port %s, stream=%p\r\n",serial_port,stream);
  ioctl (fileno(stream),FIOBAUDRATE,2400);
  cancel[port].fd=fileno(stream);
  cancel[port].stream=stream;
  rebootHookAdd(barcode_shutdown);
}
void barcode_shutdown(int type)
{
  int i;

    printf("BARCODE OCTAL232 shutdown:\r\n");
    for (i=0;i<NPORT;i++)
      if (cancel[i].fd!=NULL) 
      {
	close(cancel[i].fd);
        cancel[i].fd=0;
	printf ("\r\n close fd=%x",cancel[i].fd);
      }
    taskDelay (30);
}
int barcode_serial(int port)
{
  char *serial_port={"/tyCo/x"};
  FILE *stream;  
  int status;
  char command_buffer[120];
  char *answer_buffer;
  int c;
  unsigned char *ptr;
  char *bf;
  int fiducial;
 
  if (cancel[port].fd==NULL)
  {
    barcode_open(port);
/*
    sprintf(serial_port,"/tyCo/%d",port);
    stream = fopen (serial_port,"r+");
    if (stream==NULL)
    {
      printf ("barcode_serial: Could **NOT** open port\r\n");
      return -1;
    }
    else
      printf ("barcode_serial:  OPEN port %s, stream=%p\r\n",serial_port,stream);
    ioctl (fileno(stream),FIOBAUDRATE,2400);
    cancel[port].fd=fileno(stream);
    cancel[port].stream=stream;
*/
  }
  else
    stream=cancel[port].stream;

   taskLock();
   cancel[port].cancel=FALSE;
   cancel[port].active=TRUE;
   cancel[port].tmo=cancel[port].init_tmo;
   taskUnlock();

/*     status = sdss_transmit (stream,"\00221\003",NULL);*/
   status=fprintf ( stream, "\00221\003");
/*   fputc ('\002',stream);
   fputc ('2',stream);
   fputc ('1',stream);
   fputc ('\003',stream);*/
/*   fflush (stream);*/
   if ( status != 4 )
     printf (" BARCODE **NOT** accepting response (status=%d)\r\n",status);
   else
   {
     command_buffer[0]=NULL;
     ptr    = &barcode_buffer[0];
     while ( ( c = getc ( stream ) ) != EOF )  
     {
       *ptr=c;
       if (cancel[port].cancel) return -1;
/*        printf ( "\n\tIn receive() -  character is %c\t%02x %02x",
		 *ptr, *ptr, c);*/
       if ( *ptr == '\003' )                 /* All done */
         break;
       ++ptr;
     }
     while ( ( c = getc ( stream ) ) != EOF )  
     {
       *ptr=c;
       if (cancel[port].cancel) return -1;
/*        printf ( "\n\tIn receive() -  character is %c\t%02x %02x",
		 *ptr, *ptr, c);*/
       if ( *ptr == '\003' )                 /* All done */
         break;
       ++ptr;
     }
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
print_barcode()
{
  int i;

  printf ("\r\nBARCODE idx = %d", barcodeidx);
  for (i=0;i<256;i++) 
    printf ("\r\n  %d: axis=%d, fiducial=%d",i,barcode[i].axis,
		barcode[i].fiducial);
}
/* Test program */

#if defined (__TEST__)

int test_serial ( int port )
{

#endif          /* End __TEST__ */
#ifdef __USE_GETC__
void cancel_read ()
{
  int status;
  int i;

  for (i=0;i<NPORT;i++)
  {
    cancel[i].cancel=FALSE;
    cancel[i].active=FALSE;
    cancel[i].min_tmo=cancel[i].init_tmo;
    cancel[i].tmo=0;
    cancel[i].fd=0;
  }
  FOREVER
  {
    taskDelay (1);
    for (i=0;i<NPORT;i++)
    {
      taskLock();
      if ((cancel[i].tmo>0) && (cancel[i].active)) cancel[i].tmo--;
      if ((cancel[i].tmo>0) && (!cancel[i].active)) 
      {
        taskUnlock();
        printf ("\r\n Cancel_read not active port=%d (tmo=%d)",i,cancel[i].tmo);
        cancel[i].tmo=0;
      }
      else
        taskUnlock();
      if (cancel[i].tmo == 1)
      {
        status = ioctl (cancel[i].fd,FIOCANCEL,0);
        cancel[i].cancel=TRUE;
        cancel[i].count++;
        printf ("\r\n Cancel_read canceled port=%d (status=0x%x)",i, status);
      }
    }
  }
}
void set_cancel_tmo (int port, unsigned short tmo)
{
  cancel[port].init_tmo=tmo;
  cancel[port].min_tmo=cancel[port].init_tmo;
  cancel[port].count=0;
}
void set_all_cancel_tmo (unsigned short tmo)
{
  int i;

  for (i=0;i<NPORT;i++)
	set_cancel_tmo (i, tmo);
}
void print_cancel_tmo ()
{
  int i;

  for (i=0;i<NPORT;i++)
    printf ("\r\nCANCEL port %d:\r\n  cancel=%d, active=%d, count=%d, tmo=%d, init_tmo=%d, min_tmo=%d, fd=%d",
	i,cancel[i].cancel,cancel[i].active,cancel[i].count,cancel[i].tmo,
	cancel[i].init_tmo,cancel[i].min_tmo,cancel[i].fd);
}
#endif          /* End __USE_GETC__ */

