#include "copyright.h"
/*
**	TCC Command handler both receives, distributes, and answers on
**	behalf of the action routines.
*/
#include "vxWorks.h"
#include "stdio.h"
#include "intLib.h"
#include "iv.h"                                   
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
#include "data_collection.h"
#include "frame.h"
#include "axis.h"
#include "mcpUtils.h"
#include "as2.h"

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
			{FALSE,FALSE,0,0,SERIAL_DELAY,0,0,NULL},
			{FALSE,FALSE,0,0,SERIAL_DELAY,0,0,NULL},
			{FALSE,FALSE,0,0,SERIAL_DELAY,0,0,NULL},
			{FALSE,FALSE,0,0,SERIAL_DELAY,0,0,NULL}
};


#if USE_BARCODE
struct BARCODE barcode[256];
short barcodeidx=0;
int BARCODE_verbose=FALSE;
/*
 * prototypes
 */
void barcode_shutdown(int type);
#endif

/*
 * Send a string, possibly followed by a second string, then a \n\r to
 * the output stream.  Flushes output.
 */
int
sdss_transmit(FILE *output_stream,
	      const char *str,		/* string */
	      const char *str2)		/* second string to write too */
{
   int nchar;

   /* Initialize if necessary */                 

   /* Transmit to the TCC */
   if(str == NULL) {
      fprintf(stderr,"sdss_transmit saw NULL string\n");
      str = "";			/* shouldn't ever happen */
   }

   nchar = fprintf(output_stream, "%s%s\n\r", str, str2);

   fflush(output_stream);
   
   return((nchar < 0) ? -1 : 0);
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
	     char *buffer,
	     int size)			/* size of buffer */
{
   /* Receive data from the TCC */
   int status = 0;

#define USE_GETC 1
#if !USE_GETC
   int uid = 0, cid = 0;   

   if(fgets(buffer, size, input_stream) == NULL) {
      NTRACE_2(2, uid, cid, "failed to read buffer from serial port %d: %s\n", port, strerror(errno));
      return -1;
   }
#else
   int c;
   char *ptr;
   int charcnt;

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
#endif

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
long tcc_taskId = 0;			/* The TCC task's ID */
int tcc_may_release_semCmdPort = 0;	/* If all axes are idle, may the
					   may release semCmdPort? */
#define DEBUG_DELAY 1
#if DEBUG_DELAY
extern double sdss_delta_time(double t2, double t1);
#endif

#define DEBUG_TCC 1
#if DEBUG_TCC
extern SEM_ID semCMD;
#endif

void
tcc_serial(int port)
{
  int uid = 0, cid = 0;   
  char serial_port[] = "/tyCo/x";
  FILE *stream;  
  int status;
  char command_buffer[256];		/* input command */
  int cmd_type;				/* type of command from symb. table */
  char *answer_buffer;			/* reply from command parser */
#if DEBUG_DELAY
  float sdsstime_in;			/* time that command was received */
#endif

  tcc_taskId = taskNameToId("TCC");	/* save us looking it up */

  sprintf(serial_port,"/tyCo/%d",port);
  stream = fopen (serial_port,"r+");
  if(stream == NULL) {
     NTRACE(0, uid, cid, "Could **NOT** open port");
     taskSuspend(0);
  }
  
  NTRACE_1(1, uid, cid, "OPEN port %s", serial_port);
  ioctl (fileno(stream),FIOBAUDRATE,9600);

  new_ublock(0, 1, OLD_TCC_PROTOCOL, "TCC"); /* task-specific UBLOCK */
  log_mcp_command(CMD_TYPE_MURMUR, "TCC connected");

  for(;;) {
     NTRACE_1(16, uid, cid, "port %d", port);

     command_buffer[0] = '\0';
     status = sdss_receive(stream, port, command_buffer, 256);

     {
	int lvl = 5;
	if(strstr(command_buffer, "STATUS") != NULL) {
	   lvl += 2;
	}
 	NTRACE_1(lvl, uid, cid, "command from TCC: %s", command_buffer);
     }
     
     if(status != 0) {
	NTRACE_2(2, uid, cid, "TCC **BAD** command %s (status=%d)\r\n", command_buffer, status);
	answer_buffer = "ERR: Bad read from TCC";

	status = sdss_transmit(stream, answer_buffer, " OK");
	if(status != 0) {
	   NTRACE_1(2, uid, cid, "TCC **NOT** accepting response (status=%d)", status);
	}
	return;
     }

     if(command_buffer[0] == '\0') {
	status = sdss_transmit(stream, command_buffer, " OK");
	if(status != 0) {
	   NTRACE_1(2, uid, cid, "TCC **NOT** accepting echo (status=%d)", status);
	}
	continue;
     }
     
     status = sdss_transmit(stream, command_buffer, "");
#if DEBUG_DELAY
     sdsstime_in = sdss_get_time();
#endif
     
     if(status != 0) {
	NTRACE_1(2, uid, cid, "TCC **NOT** accepting echo (status=%d)", status);
     }
     
#if DEBUG_TCC
     {
	int ids[10], nblock;
	int j;

	for(j = 0; (nblock = semInfo(semCMD, ids, 10)) > 0; j++) {
	   int i;
	   for(i = 0; i < nblock; i++) {
	      NTRACE_2(3, uid, cid, "tcc_serial blocks on TID 0x%x [%d]\n", ids[i], j);	
	   }
	   
	   taskDelay(10);
	}
     }
#endif

/*
 * write logfile of murmurable commands
 */
     log_mcp_command(cmd_type, command_buffer);

     answer_buffer =
	 cmd_handler((getSemTaskId(semCmdPort) == taskIdSelf() ? 1 : 0), 0, 0, command_buffer, &cmd_type);
#if DEBUG_DELAY
/*
 * Stop task tracing if delay exceeds 4s
 */
     if(sdss_delta_time(sdsstime_in, sdss_get_time()) > 4.0) {
	NTRACE_1(0, uid, cid, "Too long delay %f; disabling trace",
		 sdss_delta_time(sdsstime_in, sdss_get_time()));
	traceMode(traceModeGet() & ~0x1);
     }
#endif
     if(answer_buffer == NULL) {
	fprintf(stderr,"RHL Command %s returned NULL\n", command_buffer);
	answer_buffer = "RHL";
     }

     status = sdss_transmit(stream, answer_buffer, " OK");
     if(status != 0) {
	NTRACE_1(2, uid, cid, "TCC **NOT** accepting response (status=%d)", status);
     }

#if DEBUG_DELAY
/*
 * Stop task tracing if delay exceeds 4s
 */
     if(sdss_delta_time(sdsstime_in, sdss_get_time()) > 4.0) {
	NTRACE_1(0, uid, cid, "Too long delay %f after log_mcp_command; disabling trace",
		 sdss_delta_time(sdsstime_in, sdss_get_time()));
	traceMode(traceModeGet() & ~0x1);
     }
#endif
  }
}                                             


#if USE_BARCODE
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
		  int vec, short ch)
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
      printf ("\r\n base=%lx, %p, %p",base, (char *)(base<<16),
		(char *)((long)base<<16));
      pOctSerDv=octSerModuleInit((unsigned char *)(((long)(base)<<16)+1),
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
       if (cancel[i].fd != 0) {
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
#if 0
int
barcode_serial(int port)
{
  FILE *stream;  
  int status;
  char barcode_buffer[256];
  int c;
  char *ptr;
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
#else
int
barcode_serial(int port)
{
   FILE *stream;  
   int status;
   char barcode_buffer[256];
   int i;
   char *ptr;
   char *bf;
   int fiducial;
   int charcnt;
   
   if(cancel[port].fd == 0) {
      stream = barcode_open(port);
   } else {
      stream = cancel[port].stream;
   }
   if (stream==NULL) return ERROR;
   
   taskLock();
   cancel[port].cancel = FALSE;
   cancel[port].active = TRUE;
   cancel[port].tmo = cancel[port].init_tmo;
   taskUnlock();
   
   status = fprintf(stream, "\00221\003");  /* query string */
   if(status != 4) {			/* four characters sent */
      printf (" BARCODE **NOT** accepting response (status=%d)\r\n",status);
   } else {				/* receive answer */
      barcode_buffer[0] = '\0';
      ptr = &barcode_buffer[0];
      charcnt=0;
/*
 * Read barcode.  Charlie Briegel tried reading twice; why?
 */
      for(i = 0; i < 2; i++) {
	 while((*ptr = getc(stream)) != EOF && charcnt < 256) {
	    if(cancel[port].cancel) return -1;

	    if(*ptr == '\003') {		/* All done */
	       break;
	    }
	    
	    ptr++; charcnt++;
	 }
      }

      if(charcnt >= 256) {
	 return -1;
      }
      
      *ptr = '\0';
      if(cancel[port].tmo < cancel[port].min_tmo) {
	 cancel[port].min_tmo = cancel[port].tmo;
      }
      cancel[port].active = FALSE;
      cancel[port].tmo = 0;
      if(cancel[port].cancel) {
	 printf("receive got canceled\n");
      }
	
      if(BARCODE_verbose) {
	 printf ("\r\n%s",&barcode_buffer[0]);
      }
      
      bf = strstr(barcode_buffer, "ALT");
      if(bf != NULL) {
	 bf += 3;			/* skip "ALT" */
	 sscanf(bf,"%3d", &fiducial);
	 barcode[barcodeidx].axis = 1;
      }
      
      bf = strstr(barcode_buffer, "AZ");
      if(bf != NULL) {
	 bf += 2;			/* skip "AZ" */
	 sscanf(bf, "%3d", &fiducial);
	 barcode[barcodeidx].axis=0;
      }

      barcode[barcodeidx].fiducial=fiducial;
      barcodeidx = (barcodeidx+1)%256;
      
      return fiducial;
   }
   
   barcode[barcodeidx].fiducial = 0;
   barcode[barcodeidx].axis = -1;
   barcodeidx = (barcodeidx+1)%256;

   return -1;
}                                             
#endif

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
**      If getc is used for serial communications, then use a global timeout
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
#if USE_GETC
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
       NTRACE_1(16, uid, cid, "port %d", i);
       
       taskLock();
       NTRACE(16, uid, cid, "task is locked");
       if(cancel[i].tmo > 0 && cancel[i].active) cancel[i].tmo--;
       
       if (cancel[i].tmo > 0 && !cancel[i].active) {
	  taskUnlock();
	  NTRACE_2(4, uid, cid, "barcode not active; port %d (tmo=%d)", i, cancel[i].tmo);
	  cancel[i].tmo = 0;
       } else {
	  taskUnlock();
       }
       NTRACE(16, uid, cid, "task is unlocked");
       
       if(cancel[i].tmo == 1) {
	  status = ioctl(cancel[i].fd, FIOCANCEL, 0);
	  cancel[i].cancel = TRUE;
	  cancel[i].count++;
	  NTRACE_2(4, uid, cid, "barcode canceled port %d (status=0x%x)", i, status);
       } else {
	  NTRACE_2(16, uid, cid, "port %d tmo = %d", i, cancel[i].tmo);
       }
    }
  }
}
#endif
#endif
