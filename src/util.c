#include "vxWorks.h"
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <assert.h>
#include <sysLib.h>
#include <taskLib.h>
#include <stat.h>
#include <ctype.h>
#include "iv.h"
#include "intLib.h"
#include "mv162.h"
#include "vmechip2.h"
#include "mcpUtils.h"
#include "dscTrace.h"
#include "cmd.h"
#include "as2.h"

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
   printf("Test Interrupt Routine fired, status=0x%lx level=0x%lx\n",
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
   int uid = 0, cid = 0;
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
      NTRACE_1(1, uid, cid, "Cannot determine MJD for %s; assuming MJD == 0", file);
      mjd = 0;
   }
   sprintf(filename, "/mcptpm/%d", mjd); /* directory */
   assert(strlen(filename) < FILESIZE);

   if(stat(filename, &status) == ERROR) { /* doesn't exist */
      trace_open_lvl = 1;		/* watch the open being tried */
      (void)mkdir(filename); s_errno = errno;

      if(stat(filename, &status) == ERROR) { /* still doesn't exist */
	 NTRACE_2(0, uid, cid, "Can't create %s: %s", filename, strerror(s_errno));
	 return(NULL);
      }
   }

   if(!S_ISDIR(status.st_mode)) {
      NTRACE_1(0, uid, cid, "%s isn't a directory", filename);
      return(NULL);
   }
/*
 * We have the desired directory; on to the file itself. We open it
 * using open rather than fopen so as to be able to specify the
 * desired mode bits (as vxWorks doesn't support umask)
 */
   strncat(filename, "/", sizeof(filename));
   strncat(filename, file, sizeof(filename));
   NTRACE_1(trace_open_lvl, uid, cid, "Opening %s", filename);
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
      NTRACE_1(0, uid, cid, "Unknown mode for fopen_logfile: %s", fmode);
   }

   fd = open(filename, mode, 0664);
   if(fd < 0) {
      NTRACE_2(0, uid, cid, "Open failed: %d %s", errno, strerror(errno));
      return(NULL);
   }

   fil = fdopen(fd, fmode);
   if(fil == NULL) {
      NTRACE_2(0, uid, cid, "Fdopen failed: %d %s", errno, strerror(errno));
   }

   return(fil);
}

/*****************************************************************************/
/*
 * Return the MJD, as modified by the SDSS to roll over at 9:48am
 */
/*
**  - - - - - - - -
**   s l a C l d j
**  - - - - - - - -
**
**  Gregorian calendar to Modified Julian Date.
**
**  Given:
**     iy,im,id     int    year, month, day in Gregorian calendar
**
**  Returned:
**     *djm         double Modified Julian Date (JD-2400000.5) for 0 hrs
**     *j           int    status:
**                           0 = OK
**                           1 = bad year   (MJD not computed)
**                           2 = bad month  (MJD not computed)
**                           3 = bad day    (MJD computed)
**
**  The year must be -4699 (i.e. 4700BC) or later.
**
**  The algorithm is derived from that of Hatcher 1984 (QJRAS 25, 53-55).
**
**  Last revision:   29 August 1994
**
**  Copyright P.T.Wallace.  All rights reserved.
*/
static void
s_slaCldj ( int iy, int im, int id, double *djm, int *j )
{
   long iyL, imL;

/* Month lengths in days */
   int mtab[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/* Validate year */
   if ( iy < -4699 ) { *j = 1; return; }

/* Validate month */
   if ( ( im < 1 ) || ( im > 12 ) ) { *j = 2; return; }

/* Allow for leap year */
   mtab[1] = ( ( ( iy % 4 ) == 0 ) &&
             ( ( ( iy % 100 ) != 0 ) || ( ( iy % 400 ) == 0 ) ) ) ?
             29 : 28;

/* Validate day */
   *j = ( id < 1 || id > mtab[im-1] ) ? 3 : 0;

/* Lengthen year and month numbers to avoid overflow */
   iyL = (long) iy;
   imL = (long) im;

/* Perform the conversion */
   *djm = (double)
        ( ( 1461L * ( iyL - ( 12L - imL ) / 10L + 4712L ) ) / 4L
        + ( 306L * ( ( imL + 9L ) % 12L ) + 5L ) / 10L
        - ( 3L * ( ( iyL - ( 12L - imL ) / 10L + 4900L ) / 100L ) ) / 4L
        + (long) id - 2399904L );
}

int
get_mjd(void)
{
   int status;
   double ldj;
   time_t t;
   struct tm Time;
/*
 * I should not have to do this.  I suspect that the Time struct isn't
 * being put on the stack, and thus that its values change behind my back
 *
 * Certainly the value of ldj returned from s_slaCldj was occasionally
 * changed before being used, and the only way that I can see that
 * happening is if the Time struct was being modified.
 */
   int tm_year; int tm_mon; int tm_mday;
   int tm_hour; int tm_min; int tm_sec;

#define LOCK 1
#if LOCK
   taskLock();
#endif
   (void)time(&t);
   (void)gmtime_r(&t, &Time);

   tm_year = Time.tm_year; tm_mon = Time.tm_mon; tm_mday = Time.tm_mday;
   tm_hour = Time.tm_hour; tm_min = Time.tm_min; tm_sec = Time.tm_sec;
#if LOCK
   taskUnlock();
#endif

   s_slaCldj(tm_year + 1900, tm_mon + 1, tm_mday, &ldj, &status);

   if(status) {
      static char buff[100];
      sprintf(buff, "%d %d %d", tm_year + 1900, tm_mon + 1, tm_mday);
      /* OTRACE(6, "MJD: %d (%s)", status, buff); */

      return(-1);
   } else {
      ldj += (tm_hour + (tm_min + tm_sec/60.0)/60.0)/24.0;
      ldj += 0.3;

      return((int)ldj);
   }
}

/*****************************************************************************/
/*
 * return the MCP version.  Print to console if version == NULL && len == 0
 */
char *
mcpVersion(char *version,		/* string to fill out, or NULL */
	   int len)			/* dimen of string */
{
   int print = (version == NULL && len == 0) ? 1 : 0; /* should I print the version? */
   char *tag = version_cmd(0, 0, NULL); /* CVS tagname + compilation time */

   if(print) {
      printf("mcpVersion: %s\n", tag);
   }

   if (version != NULL) {
      strncpy(version, tag, len - 1);
      version[len - 1] = '\0';
   }

   return tag;
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

/*
 * We just got a timer message, not a proper MCP_MSG, so unpack uid/cid for us.
 *
 * We assume that the timer was sent with e.g.
 *   timerSendArgWithUidCid(MSG_TYPE, tmr_e_add, TIME, msg.uid, msg.cid, QUEUE)
 */
#define MID_SCALE_FACTOR 100		/* amount to scale the msg.type in constructing the mid */

void
get_uid_cid_from_tmr_msg(const MCP_MSG *msg, int *uid, unsigned long *cid)
{
   struct s_tmr_msg_fmt *tmsg = (struct s_tmr_msg_fmt *)msg;
   *uid = tmsg->u.tmr.arg%MAX_CONNECTIONS;
   *cid = tmsg->u.tmr.arg/MAX_CONNECTIONS;
}

/*
 * A wrapper for timerSendArg that properly encodes the uid/cid.  You should ALWAYS call this, and never
 * call the raw timerSendArg
 *
 * This routine packs msg_type/uid into the timerSendArg mid. We need to do this
 * (despite the fact that the MSG_TYPE is also passed to timerSendArg) because
 * tmr_e_abort_ns looks at the mid and thread ID to determine which messages to
 * abort, and we only want to abort messages of the given msg.type
 */
STATUS
timerSendArgWithUidCid(int msg_type,	/* type of message */
		       int cmd,		/* the command, e.g. tmr_e_add */
		       int tick_cnt,	/* delay in ticks */
		       int uid,		/* UID */
		       unsigned long cid, /* CID */
		       MSG_Q_ID return_q) /* queue to push msg onto, if requested by cmd */
{
   return timerSendArg(msg_type, cmd, tick_cnt, abs(msg_type), uid + MAX_CONNECTIONS*cid, return_q);
}

