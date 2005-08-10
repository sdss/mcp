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
#include "axis.h"
#include "mcpUtils.h"

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
   if(fgets(buffer, size, input_stream) == NULL) {
      TRACE(2, "failed to read buffer from serial port %d: %s\n", port,
			strerror(errno), 0, 0);
      return -1;
   }
#else
   int c;
   char *ptr;
   int charcnt;
   FILE *dbg = fopen_logfile("serial.out", "a");
   if(dbg == NULL) {
      dbg = stderr;
   }

   ptr    = buffer;
   charcnt=0;
   while ( ((c = getc(input_stream)) != EOF)&&(charcnt<size))
   {
#if 1
      fprintf(dbg, "C: %03o %c\n", c, c); fflush(dbg);
#endif

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

   if(dbg != stderr) {
      fclose(dbg); dbg = NULL;
   }
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

void
tcc_serial(int port)
{
  char serial_port[] = "/tyCo/x";
  FILE *stream;  
  int status;
  char command_buffer[256];		/* input command */
  int cmd_type;				/* type of command from symb. table */
  char *answer_buffer;			/* reply from command parser */
#define DEBUG_DELAY 1
#if DEBUG_DELAY
  extern double sdss_delta_time(double t2, double t1);
  float sdsstime_in;			/* time that command was received */
#endif

  tcc_taskId = taskNameToId("TCC");	/* save us looking it up */

  sprintf(serial_port,"/tyCo/%d",port);
  stream = fopen (serial_port,"r+");
  if(stream == NULL) {
     TRACE(0, "Could **NOT** open port", 0, 0, 0, 0);
     taskSuspend(0);
  }
  
  TRACE(1, "OPEN port %s", serial_port, 0, 0, 0);
  ioctl (fileno(stream),FIOBAUDRATE,9600);

  new_ublock(0, "TCC");			/* task-specific UBLOCK */
  log_mcp_command(CMD_TYPE_MURMUR, "TCC connected");

  for(;;) {
     TRACE(16, "port %d", port, 0, 0, 0);

     command_buffer[0] = '\0';
     status = sdss_receive(stream, port, command_buffer, 256);

     {
	int lvl = 5;
	if(strstr(command_buffer, "STATUS") != NULL) {
	   lvl += 2;
	}
 	TRACE(lvl, "command from TCC: %s", command_buffer, 0, 0, 0);
	TRACE(16, "        cccccccc: 0x%08x%08x",
	      ((int *)command_buffer)[0], ((int *)command_buffer)[1], 0, 0);
	TRACE(16, "        time = %f", sdss_get_time(), 0, 0, 0);
     }
     
     if(status != 0) {
	TRACE(2, "TCC **BAD** command %s (status=%d)\r\n", command_buffer, status, 0, 0);
	answer_buffer = "ERR: Bad read from TCC";

	status = sdss_transmit(stream, answer_buffer, " OK");
	if(status != 0) {
	   TRACE(2, "TCC **NOT** accepting response (status=%d)", status, 0, 0, 0);
	}
	return;
     }

     if(command_buffer[0] == '\0') {
	status = sdss_transmit(stream, command_buffer, " OK");
	if(status != 0) {
	   TRACE(2, "TCC **NOT** accepting echo (status=%d)", status, 0, 0, 0);
	}
	continue;
     }
     
     status = sdss_transmit(stream, command_buffer, "");
#if DEBUG_DELAY
     sdsstime_in = sdss_get_time();
#endif
     
     if(status != 0) {
	TRACE(2, "TCC **NOT** accepting echo (status=%d)", status, 0, 0, 0);
     }
     
#if 1
     {
	extern SEM_ID semCMD;
	int ids[10], nblock;
	int j;
	
	for(j = 0; (nblock = semInfo(semCMD, ids, 10)) > 0; j++) {
	   int i;
	   fprintf(stderr, "semCMD: ");
	   for(i = 0;i < nblock; i++) {
	      fprintf(stderr, "tcc_serial blocks on 0x%x (%s) [%d]\n",
		      ids[i], taskName(ids[i]), j);
	      TRACE(3, "tcc_serial blocks on %s [%d]",
					taskName(ids[i]), j, 0, 0);
	   }
	   fprintf(stderr,"\n");
	   
	   taskDelay(1);
	}
     }
#endif
     answer_buffer =
       cmd_handler((getSemTaskId(semCmdPort) == taskIdSelf() ? 1 : 0),
		   command_buffer, &cmd_type);
#if DEBUG_DELAY
/*
 * Stop task tracing if delay exceeds 4s
 */
     if(sdss_delta_time(sdsstime_in, sdss_get_time()) > 4.0) {
	TRACE(0, "Too long delay %f; disabling trace",
	      sdss_delta_time(sdsstime_in, sdss_get_time()), 0, 0, 0);
	traceMode(traceModeGet() & ~0x1);
     }
#endif
     if(answer_buffer == NULL) {
	fprintf(stderr,"RHL Command %s returned NULL\n", command_buffer);
	answer_buffer = "RHL";
     }

     status = sdss_transmit(stream, answer_buffer, " OK");
     if(status != 0) {
	TRACE(2, "TCC **NOT** accepting response (status=%d)", status, 0, 0, 0);
     }
/*
 * write logfile of murmurable commands
 */
     log_mcp_command(cmd_type, command_buffer);

#if DEBUG_DELAY
/*
 * Stop task tracing if delay exceeds 4s
 */
     if(sdss_delta_time(sdsstime_in, sdss_get_time()) > 4.0) {
	TRACE(0, "Too long delay %f after log_mcp_command; disabling trace",
	      sdss_delta_time(sdsstime_in, sdss_get_time()), 0, 0, 0);
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
       TRACE(16, "port %d", i, 0, 0, 0);
       
       taskLock();
       TRACE(16, "task is locked", 0, 0, 0, 0);
       if(cancel[i].tmo > 0 && cancel[i].active) cancel[i].tmo--;
       
       if (cancel[i].tmo > 0 && !cancel[i].active) {
	  taskUnlock();
	  TRACE(4, "barcode not active; port %d (tmo=%d)", i, cancel[i].tmo, 0, 0);
	  cancel[i].tmo = 0;
       } else {
	  taskUnlock();
       }
       TRACE(16, "task is unlocked", 0, 0, 0, 0);
       
       if(cancel[i].tmo == 1) {
	  status = ioctl(cancel[i].fd, FIOCANCEL, 0);
	  cancel[i].cancel = TRUE;
	  cancel[i].count++;
	  TRACE(4, "barcode canceled port %d (status=0x%x)", i, status, 0, 0);
       } else {
	  TRACE(16, "port %d tmo = %d", i, cancel[i].tmo, 0, 0);
       }
    }
  }
}
#endif
#endif
