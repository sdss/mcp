#include "vxWorks.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <assert.h>
#include <sysLib.h>
#include <stat.h>
#include <ctype.h>
#include "iv.h"
#include "intLib.h"
#include "mv162.h"
#include "vmechip2.h"
#include "mcpUtils.h"
#include "dscTrace.h"

#define SRAM_BASE_ADRS		0xFFE00000L

unsigned long VMEC2_software_interrupt_init (int interrupt, 
	int level,
		void ((*routine)()) );
unsigned long VMEC2_software_interrupt_set (int interrupt);
unsigned long VMEC2_software_interrupt_clear (int interrupt);
void (*sft_int_routines[8])()={NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
void VME2_sft_int(int interrupt);
void test_routine();
void test_interrupt();
unsigned long MCC_timer_read(int timer);
unsigned long MCC_timer_start(int timer);
unsigned long MCC_timer_stop(int timer);
unsigned long VMEC2_timer_read(int timer);
unsigned long VMEC2_timer_start(int timer);
unsigned long VMEC2_timer_stop(int timer);

unsigned long VMEC2_software_interrupt_init (int interrupt, 
		int level,
		void ((*routine)()) )
{
  int vecno;
  STATUS s;

  if ((interrupt<0)||(interrupt>7)) return -1;
  if ((level<0)||(level>7)) level=3;
  sft_int_routines[interrupt] = routine;
/*vector*/
  vecno = (( (((unsigned long)*VMECHIP2_IOCR)>>24) &0xF)<<4)+interrupt+8;
/*interrupt handler */
  printf ("\r\nSoftware Vector number=%x %lx",
		vecno,(unsigned long)*VMECHIP2_IOCR);
  s = intConnect(INUM_TO_IVEC(vecno), VME2_sft_int, interrupt); 
  *VMECHIP2_ILR3 = ((unsigned long)*VMECHIP2_ILR3 & (~(0xF<<(interrupt*4)) )) | 
		(unsigned long)(level<<(interrupt*4));
  VMEC2_software_interrupt_clear (interrupt);
/*enable */
  *VMECHIP2_LBIER = (unsigned long)*VMECHIP2_LBIER | 
			(unsigned long)(1<<(interrupt+8));
  sysIntEnable (level);  
  return 0;
}
unsigned long VMEC2_software_interrupt_set (int interrupt)
{
  if ((interrupt<0)||(interrupt>7)) return -1;
  *VMECHIP2_SISR = (unsigned long)(1<<(interrupt+8));
  return 0;
}
unsigned long VMEC2_software_interrupt_clear (int interrupt)
{
  if ((interrupt<0)||(interrupt>7)) return -1;
  *VMECHIP2_ICLR = (unsigned long)(1<<(interrupt+8));
  return 0;
}
void VME2_sft_int(int interrupt)
{
  TRACE0(16, "VME2_sft_int", 0, 0);

  if (sft_int_routines[interrupt]!=NULL)
    (*sft_int_routines[interrupt])();
  VMEC2_software_interrupt_clear (interrupt);
}
void VME2_pre_scaler (unsigned long adjust)
{
  *VMECHIP2_TIMEOUTCR = (unsigned long)((*VMECHIP2_TIMEOUTCR&0xFFFFFF00)|adjust);
}

void test_interrupt()
{
  VMEC2_software_interrupt_init (5,6,test_routine);
  VMEC2_software_interrupt_set (5);
}
void test_routine()
{
   TRACE(1, "Test Interrupt Routine fired, status=0x%lx level=0x%lx",
	 (long)*VMECHIP2_LBISR, (long)*VMECHIP2_ILR3);
}

unsigned long MCC_timer_read(int timer)
{
  switch (timer)
  {
    case 3:
	return (*MCC_TIMER3_CNT);
    case 4:
	return (*MCC_TIMER4_CNT);
    default:
	return -1;
  }
  return 0;
}
unsigned long MCC_timer_start(int timer)
{
  switch (timer)
  {
    case 3:
	*MCC_TIMER3_CNT = 0;
	*MCC_TIMER3_CR = TIMER3_CR_CEN;
	break;
    case 4:
	*MCC_TIMER4_CNT = 0;
	*MCC_TIMER4_CR = TIMER4_CR_CEN;
	break;
    default:
	return -1;
  }
  return 0;
}
unsigned long MCC_timer_stop(int timer)
{
  switch (timer)
  {
    case 3:
	*MCC_TIMER3_CR = TIMER3_CR_DIS;
	return (*MCC_TIMER3_CNT);
    case 4:
	*MCC_TIMER4_CR = TIMER4_CR_DIS;
	return (*MCC_TIMER4_CNT);
    default:
	return -1;
  }
  return 0;
}
unsigned long VMEC2_timer_read(int timer)
{
  switch (timer)
  {
    case 3:
	return (*VMECHIP2_TTCOUNT1);
    case 4:
	return (*VMECHIP2_TTCOUNT2);
    default:
	return -1;
  }
  return 0;
}
unsigned long VMEC2_timer_start(int timer)
{
  switch (timer)
  {
    case 3:
        *VMECHIP2_TTCOUNT1 = 0;                 /* clear the counter */
        *VMECHIP2_TIMERCR |= TIMERCR_TT1_EN;    /* enable the timer */
	break;
    case 4:
        *VMECHIP2_TTCOUNT2 = 0;                 /* clear the counter */
        *VMECHIP2_TIMERCR |= TIMERCR_TT2_EN;    /* enable the timer */
	break;
    default:
        return -1;
  }
  return 0;
}

unsigned long VMEC2_timer_stop(int timer)
{
  switch (timer)
  {
    case 3:
        *VMECHIP2_TIMERCR &= ~TIMERCR_TT1_EN;   /* disable the timer */
        return (*VMECHIP2_TTCOUNT1);
    case 4:
        *VMECHIP2_TIMERCR &= ~TIMERCR_TT2_EN;   /* disable the timer */
        return (*VMECHIP2_TTCOUNT2);
    default:
        return -1;
  }
  return 0;
}
unsigned long timer_read(int timer)
{
  switch (timer)
  {
    case 1:
	return (*VMECHIP2_TTCOUNT1);
    case 2:
	return (*VMECHIP2_TTCOUNT2);
    case 3:
	return (*MCC_TIMER3_CNT);
    case 4:
	return (*MCC_TIMER4_CNT);
    default:
	return -1;
  }
  return 0;
}
unsigned long timer_start(int timer)
{
  switch (timer)
  {
    case 1:
        *VMECHIP2_TTCOUNT1 = 0;                 /* clear the counter */
        *VMECHIP2_TIMERCR |= TIMERCR_TT1_EN;    /* enable the timer */
	break;
    case 2:
        *VMECHIP2_TTCOUNT2 = 0;                 /* clear the counter */
        *VMECHIP2_TIMERCR |= TIMERCR_TT2_EN;    /* enable the timer */
	break;
    case 3:
	*MCC_TIMER3_CNT = 0;
	*MCC_TIMER3_CR = TIMER3_CR_CEN;
	break;
    case 4:
	*MCC_TIMER4_CNT = 0;
	*MCC_TIMER4_CR = TIMER4_CR_CEN;
	break;
    default:
        return -1;
  }
  return 0;
}
unsigned long timer_stop(int timer)
{
  switch (timer)
  {
    case 1:
        *VMECHIP2_TIMERCR &= ~TIMERCR_TT1_EN;   /* disable the timer */
        return (*VMECHIP2_TTCOUNT1);
    case 2:
        *VMECHIP2_TIMERCR &= ~TIMERCR_TT2_EN;   /* disable the timer */
        return (*VMECHIP2_TTCOUNT2);
    case 3:
	*MCC_TIMER3_CR = TIMER3_CR_DIS;
	return (*MCC_TIMER3_CNT);
    case 4:
	*MCC_TIMER4_CR = TIMER4_CR_DIS;
	return (*MCC_TIMER4_CNT);
    default:
        return -1;
  }
  return 0;
}

/*****************************************************************************/
/*
 * Return the name of a logfile in /mcptpm/<mjd>, making sure that
 * the directory exists.
 */
#define FILESIZE 100			/* size of filename buffer */

FILE *
fopen_logfile(const char *file,		/* desired file */
	      const char *fmode)	/* mode as for fopen */
{
   int s_errno;				/* saved errno */
   int fd;				/* file's file descriptor */
   FILE *fil;				/* the returned FILE */
   char filename[100];
   int mjd;				/* current MJD */
   int mode;				/* mode for open() */
   int trace_open_lvl = 5;		/* level for TRACE on successful open*/
   struct stat status;			/* information about the directory */
/*
 * Create directory if it doesn't exist
 */
   mjd = get_mjd();
   if(mjd < 0) {
      TRACE(1, "Cannot determine MJD for %s; assuming MJD == 0", file, 0);
      mjd = 0;
   }
   sprintf(filename, "/mcptpm/%d", mjd); /* directory */
   assert(strlen(filename) < FILESIZE);

   if(stat(filename, &status) == ERROR) { /* doesn't exist */
      trace_open_lvl = 1;		/* watch the open being tried */
      (void)mkdir(filename); s_errno = errno;

      if(stat(filename, &status) == ERROR) { /* still doesn't exist */
	 TRACE(0, "Can't create %s: %s", filename, strerror(s_errno));
	 return(NULL);
      }
   }
      
   if(!S_ISDIR(status.st_mode)) {
      TRACE(0, "%s isn't a directory", filename, 0);
      return(NULL);
   }
/*
 * We have the desired directory; on to the file itself. We open it
 * using open rather than fopen so as to be able to specify the
 * desired mode bits (as vxWorks doesn't support umask)
 */
   strncat(filename, "/", sizeof(filename));
   strncat(filename, file, sizeof(filename));
   TRACE(trace_open_lvl, "Opening %s", filename, 0);
/*
 * Translate an fopen() mode into an open() mode; ignore any '+' modifiers
 */
   if(*fmode == 'a') {
      mode = O_WRONLY;
      if(stat(filename, &status) == ERROR) { /* no such file */
	 mode |= O_CREAT;
      }
   } else if(*fmode == 'r') {
      mode = O_RDONLY;
   } else if(*fmode == 'w') {
      mode = O_WRONLY | O_CREAT;
      if(stat(filename, &status) != ERROR) { /* file already exists*/
	 (void)unlink(filename);	/* the open() call doesn't truncate */
      }
   } else {
      TRACE(0, "Unknown mode for fopen_logfile: %s", fmode, 0);
   }

   fd = open(filename, mode, 0664);
   if(fd < 0) {
      TRACE(0, "Open failed: %d %d", errno, strerror(errno));
      return(NULL);
   }

   fil = fdopen(fd, fmode);
   if(fil == NULL) {
      TRACE(0, "Fdopen failed: %d %s", errno, strerror(errno));
   }

   return(fil);
}

/*****************************************************************************/
/*
 * Return the MJD, as modified by the SDSS to roll over at 9:48am
 */
extern void slaCldj(int, int, int, double*, int*);

int
get_mjd(void)
{
   int status;
   double ldj;
   time_t t;
   struct tm Time;

   (void)time(&t);
   (void)gmtime_r(&t, &Time);

   slaCldj(Time.tm_year + 1900, Time.tm_mon + 1, Time.tm_mday,
	   &ldj, &status);

   if(status) {
      char buff[100];
      sprintf(buff, "%d %d %d",
	      Time.tm_year + 1900, Time.tm_mon + 1, Time.tm_mday);
      TRACE(2, "MJD: %d (%s)", status, buff);
   
      return(-1);
   } else {
      char buff[100];
      sprintf(buff, "%d %d %d %d:%d:%d",
	      Time.tm_year + 1900, Time.tm_mon + 1, Time.tm_mday,
	      Time.tm_hour, Time.tm_min, Time.tm_sec);

      ldj += (Time.tm_hour + (Time.tm_min + Time.tm_sec/60.0)/60.0)/24.0;
      ldj += 0.3;

      TRACE(3, "MJD: %s  LDJ %d", buff, (int)ldj);
	      
      return((int)ldj);
   }
}

/*****************************************************************************/
/*
 * return the MCP version
 */
char *
mcpVersion(char *version,		/* string to fill out, or NULL */
	   int len)			/* dimen of string */
{
   static char buff[100 + 1];		/* buffer if version == NULL */
   int i;
   int print = (version == NULL) ? 1 : 0; /* should I print the version? */
   const char *ptr;			/* scratch pointer */
   const char *tag = version_cmd("");	/* CVS tagname + compilation time */

   if(version == NULL) {
      version = buff; len = 101;
   }

   version[len - 1] = '\a';		/* check for string overrun */

   ptr = strchr(tag, ':');
   if(ptr == NULL) {
      strncpy(version, tag, len - 1);
   } else {
      ptr++;
      while(isspace(*ptr)) ptr++;
      
      if(*ptr != '$') {			/* a CVS tag */
	 for(i = 0; ptr[i] != '\0' && !isspace(ptr[i]) && i < 100; i++) {
	    version[i] = ptr[i];
	 }
	 version[i] = '\0';
      } else {
	 if(*ptr == '$') ptr++;
	 if(*ptr == '|') ptr++;
	 
	 sprintf(version, "NOCVS:%s", ptr);
	 version[strlen(version) - 2] = '\0'; /* trim "double quote\n" */
      }
   }

   assert(version[len - 1] == '\a');	/* no overrun */
   if(print) {
      printf("mcpVersion: %s\n", version);
   }

   return(version);
}

/*****************************************************************************/
/*
 * Return the task ID of the process holding a semaphore
 */
long
getSemTaskId(SEM_ID sem)
{
   if(sem == NULL) {
      return(0);
   } else {
      return((long)sem->state.owner);
   }
}

/*****************************************************************************/
/*
 * Calculate a CRC for a string of characters. You can call this routine
 * repeatedly to build up a CRC. Only the last 16bits are significant.
 *
 * e.g.
 *   crc = phCrcCalc(0, buff, n) & 0xFFFF;
 * or
 *   crc = 0;
 *   crc = phCrcCalc(crc, buff0, n);
 *   crc = phCrcCalc(crc, buff1, n);
 *   crc = phCrcCalc(crc, buff2, n);
 *   crc &= 0xFFFF;
 *
 * Routine taken from photo/src/utils.c
 */
long
phCrcCalc(long crc,			/* initial value of CRC (e.g. 0) */
	  const char *buff,		/* buffer to be CRCed */
	  int n)			/* number of chars in buff */
{
   register long c;
   static long crcta[16] = {
      0L, 010201L, 020402L, 030603L, 041004L,
      051205L, 061406L, 071607L, 0102010L,
      0112211L, 0122412L, 0132613L, 0143014L,
      0153215L, 0163416L, 0173617L
   };					/* CRC generation tables */
   static long crctb[16] = {
      0L, 010611L, 021422L, 031233L, 043044L,
      053655L, 062466L, 072277L, 0106110L,
      0116701L, 0127532L, 0137323L, 0145154L,
      0155745L, 0164576L, 0174367L
   };
   const char *ptr = buff;		/* pointers to buff */
   const char *const end = buff + n;	/*        and to end of desired data */

   for(;ptr != end;ptr++) {
      c = crc ^ (long)(*ptr);
      crc = (crc >> 8) ^ (crcta[(c & 0xF0) >> 4] ^ crctb[c & 0x0F]);
   }

   return(crc);
}
