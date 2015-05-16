#include <vxWorks.h>
#include <assert.h>
#include <semLib.h>
#include <taskLib.h>
#include <rebootLib.h>
#include <intLib.h>
#include <sysLib.h>
#include <systime.h>
#include <timers.h>
#include <time.h>
#include <errno.h>
#include "mcpNtp.h"
#include "tod_prototypes.h"
#include "iv.h"
#include "gendefs.h"
#include "mcpTimers.h"
#include "dscTrace.h"
#include "data_collection.h"
#include "frame.h"
#include "axis.h"
#include "mcpMsgQ.h"
#include "cmd.h"

int tm_DID48;
int DID48_Init = FALSE;
int tm_DIO316;
int DIO316_Init = FALSE;

SEM_ID ppsSem;

/*=========================================================================
**=========================================================================
**
** ROUTINE: sdss_get_time
**	    get_time - prints time as well; used as a diagnostic.
**
** DESCRIPTION:
**	Get SDSS time based on seconds-in-a-day plus us since last 1 Hz
**	interrupt.
**	The timer is scaled due to an incorrect setting in the BSP for
**	VME162.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**	timer_read
**
** GLOBALS REFERENCED:
**	SDSStime
**
**=========================================================================
*/
/*
 * turn on 1 Hz interrupt time returns else use local clock
 */
#define CLOCK_INT 1

#ifdef CLOCK_INT
double
sdss_get_time(void)
{
   unsigned long micro_sec;
   const long SDSStime0 = SDSStime;	/* initial value of SDSStime */
   long SDSSnow;			/* current value of SDSStime */
   
#if 0
   micro_sec = (unsigned long)(1.0312733648*timer_read (1));
#else
   micro_sec = timer_read (1);
#endif
   SDSSnow = SDSStime;

   if(SDSSnow != SDSStime0) {		/* a GPS interrupt arrived */
      if(micro_sec > 500000) {		/* micro_sec was read before int. */
	 SDSSnow = SDSStime0;		/* set SDSSnow to before interrupt */
      }
   }

   return(SDSSnow + 1e-6*micro_sec);
}
#else
double
sdss_get_time(void)
{
  	  struct timespec tp;
  	  unsigned long micro_sec;

  	  clock_gettime(CLOCK_REALTIME,&tp);
          micro_sec = timer_read (1);
          return ((double)(tp.tv_sec%ONE_DAY)+((micro_sec%1000000)/1000000.));
}
#endif

double
get_time(void)
{
   double t = sdss_get_time();
   printf("SDSS time = %f\n", t);

   return(t);
}

/*=========================================================================
**
** Determine the interval between two timestamps, given in seconds since
** midnight UTC ~ 5pm MST.
**
** The routine assumes that within 400s of this roll-over time the smaller
** time really belongs to the following day, and acts accordingly
**
**=========================================================================
*/
double
sdss_delta_time(double t2, double t1)
{
  if(t2 >= 0 && t2 < 400 && t1 > 86000 && t1 < 86400) {
     return((86400 - t1) + t2);
  } else if(t1 >= 0 && t1 < 400 && t2 > 86000 && t2 < 86400) {
     return((t2 - 86400) - t1);
  } else {
     return(t2 - t1);
  }
}

int
test_dt(int t2,int t1)
{
  printf ("dt=%f\n",sdss_delta_time((double)t2,(double)t1));
  return (int)sdss_delta_time((double)t2,(double)t1);
}

/*
 *
 *	SET.TIME date - sets time to TAI with fractional part from GPS
 *
 * The date specification is ignored in favour of an NTP server
 */
long SDSStime = -1;			/* integral part of TAI */

int
setSDSStimeFromNTP(int quiet)
{
   int uid = 0, cid = 0;
   const char *taiServer = "tai-time.apo.nmsu.edu";
   const char *utcServer = "utc-time.apo.nmsu.edu";
   struct tm tm;
   time_t t;
   struct timeval tai;			/* TAI from NTP */
   int    tries = 3;
   int    ret = ERROR;

/*
 * Find how many seconds it is after midnight TAI, and set SDSStime accordingly
 */
   
   while (tries-- > 0 && ret != OK) {
     /* Give ourselves the largest good window by syncing to the next 1PPS tick. */
     semTake(ppsSem, NO_WAIT);	/* semClear is not declared, the sillies. */
     if (semTake(ppsSem, sysClkRateGet() * 2) < 0) {
	NTRACE(0, uid, cid, "NO 1PPS !!!");
	return -1;
     }
     /* And now wait long enough to get over the ntp inaccuracies. */
     taskDelay(sysClkRateGet() * 0.2);

     taskLock();

     /* I know, that 1.0 timeout should be < 1.0, but there's a bug in sntpcLib */
     ret = setTimeFromNTP(taiServer, 1.0, 0, 0, &tai);
     if (ret != OK) {
       logMsg("failed to get time from %s: %s\n", 
	      (int)taiServer, (int)strerror(errno),0,0,0,0);
       taskUnlock();
       continue;
     }

     t = tai.tv_sec;
     tm = *localtime(&t);

     SDSStime = tm.tm_sec + 60*(tm.tm_min + 60*tm.tm_hour);

     if(!quiet) {
       fprintf(stderr,"RHL setting:  %d %d %d %d %d %f\n",
	       tm.tm_mon + 1, tm.tm_mday, 1900 + tm.tm_year,
	       tm.tm_hour, tm.tm_min,
	       tm.tm_sec + 1e-6*timer_read(1));
     }

     taskUnlock();
   }

   if (ret == ERROR) {
     logMsg("failed to get time from %s: %s\n", 
	    (int)taiServer, (int)strerror(errno),0,0,0,0);
     NTRACE_2(0, uid, cid, "failed to get time from %s: %s", taiServer, strerror(errno));
     return -1;
   }

   axis_stat[AZIMUTH][1].clock_not_set = 
     axis_stat[ALTITUDE][1].clock_not_set =
     axis_stat[INSTRUMENT][1].clock_not_set = 0;

/*
 * synchronise the system clock to UTC; this seems as good a place as any
 */
   taskLock();

   if(setTimeFromNTP(utcServer, 1.0, 3, 0, NULL) < 0) {
      NTRACE_2(0, uid, cid, "failed to get time from %s: %s", utcServer, strerror(errno));
   }

   taskUnlock();

   return 0;
}

char *
set_time_cmd(int uid, unsigned long cid, char *cmd)
{
   static int quiet = 1;		/* be quiet? */

   if (setSDSStimeFromNTP(quiet) < 0) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 0, "command", "set_time");
   } else {
      sendStatusMsg_S(uid, cid, FINISHED_CODE, 0, "command", "set_time");
   }

   return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: time_cmd
**
** DESCRIPTION:
**	TIME? - gets date and time.
**
** RETURN VALUES:
**	return date as "m d y seconds-of-day"
**
** CALLS TO:
**	sdss_get_time
**
** GLOBALS REFERENCED:
**	SDSStime
**
**=========================================================================
*/
char *
time_cmd(int uid, unsigned long cid, char *cmd)
{
  static struct tm *t;
  struct timespec tp;

  clock_gettime(CLOCK_REALTIME,&tp);
  printf(" sec=%ld, nano_sec=%ld\r\n",tp.tv_sec,tp.tv_nsec);

  t = localtime(&tp.tv_sec);
  sprintf(ublock->buff,"%d %d %d %f",
	  t->tm_mon+1, t->tm_mday, t->tm_year+1900, sdss_get_time());

  sendStatusMsg_S(uid, cid, FINISHED_CODE, 0, "command", "time");
  
  return(ublock->buff);
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: DIO316_initialize
**
** DESCRIPTION:
**	Setup the DIO316 for axis motion.
**
** RETURN VALUES:
**	return 0 or ERROR
**
** CALLS TO:
**	Industry_Pack
**
** GLOBALS REFERENCED:
**	tm_DIO316
**
**=========================================================================
*/
int
DIO316_initialize(unsigned char *addr, unsigned short vecnum)
{
   int uid = 0, cid = 0;
   STATUS stat;
   int i;
   struct IPACK ip;
   
   Industry_Pack (addr,SYSTRAN_DIO316,&ip);
   for(i = 0; i < MAX_SLOTS; i++) { 
      if (ip.adr[i]!=NULL) {
	 OTRACE(30, "Found at %d, %p", i, ip.adr[i]);
	 tm_DIO316=DIO316Init((struct DIO316 *)ip.adr[i], vecnum);
	 break;
      }
   }
   if(i >= MAX_SLOTS) {
      NTRACE_1(0, uid, cid, "****Missing DIO316 at %p****", addr);
      return ERROR;
   }
   
   DIO316_Init=TRUE;
   DIO316_Read_Reg(tm_DIO316,0xA,&vecnum);
   DIO316_Interrupt_Enable_Control (tm_DIO316,0,DIO316_INT_DIS);
   DIO316_Interrupt_Enable_Control (tm_DIO316,1,DIO316_INT_DIS);
   DIO316_Interrupt_Enable_Control (tm_DIO316,2,DIO316_INT_DIS);
   DIO316_Interrupt_Enable_Control (tm_DIO316,3,DIO316_INT_DIS);
#if 0
   if(vecnum==0) vecnum = DIO316_VECTOR;
#endif
   stat = intConnect (INUM_TO_IVEC(vecnum),
		      (VOIDFUNCPTR)DIO316_interrupt, DIO316_TYPE);
   assert(stat == OK);
   OTRACE(30, "DIO316 vector = %d, interrupt address = %p\n",
	  vecnum, DIO316_interrupt);
   rebootHookAdd((FUNCPTR)axis_DIO316_shutdown);

#if 0
   DIO316_Interrupt_Configuration(tm_DIO316,0,DIO316_INT_HIGH_LVL);           
   DIO316_Interrupt_Enable_Control(tm_DIO316,0,DIO316_INT_ENA);
#endif
   DIO316_Interrupt_Configuration(tm_DIO316,1,
				  DIO316_INT_FALL_EDGE/*ON_CHANGE*/);
   DIO316_Interrupt_Enable_Control(tm_DIO316,1,DIO316_INT_ENA);
   DIO316_Interrupt_Configuration(tm_DIO316,2,DIO316_INT_FALL_EDGE);
   DIO316_Interrupt_Enable_Control(tm_DIO316,2,DIO316_INT_ENA);
   DIO316_Interrupt_Configuration(tm_DIO316,3,DIO316_INT_FALL_EDGE);
   DIO316_Interrupt_Enable_Control(tm_DIO316,3,DIO316_INT_ENA);
   DIO316_OE_Control(tm_DIO316,3,DIO316_OE_ENA);
   DIO316_OE_Control(tm_DIO316,2,DIO316_OE_ENA);
   
   IP_Interrupt_Enable(&ip,DIO316_IRQ);
   sysIntEnable(DIO316_IRQ);
   
  return 0;
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: axis_DIO316_shutdown
**
** DESCRIPTION:
**	Software reboot shutdown of DIO316 hardware.  Disables interrupts.
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	tm_DIO316
**
**=========================================================================
*/
void
axis_DIO316_shutdown(int type)
{
   printf("AXIS DIO316 Shutdown:  4 interrupts %d\n",tm_DIO316);
   if(tm_DIO316 != -1) {
      DIO316_Interrupt_Enable_Control(tm_DIO316,0,DIO316_INT_DIS);
      DIO316_Interrupt_Enable_Control(tm_DIO316,1,DIO316_INT_DIS);
      DIO316_Interrupt_Enable_Control(tm_DIO316,2,DIO316_INT_DIS);
      DIO316_Interrupt_Enable_Control(tm_DIO316,3,DIO316_INT_DIS);
   }
   taskDelay(30);
}

/*
 * Interrupt handler. Used for monitoring a crossing of a fiducial
 * and triggers a task via a message queue
 */
int illegal_NIST = 0;

void
DIO316_interrupt(int type)
{
   unsigned char dio316int_bit = 0;

   DIO316ReadISR(tm_DIO316, &dio316int_bit);

   if(dio316int_bit & NIST_INT) {
      illegal_NIST++;
      DIO316ClearISR(tm_DIO316);
   } else {
/*
 * disable all axis interrupts until we've read the MEI's latched positions
 */
      DIO316_Interrupt_Enable_Control(tm_DIO316, 1, DIO316_INT_DIS);
      DIO316_Interrupt_Enable_Control(tm_DIO316, 2, DIO316_INT_DIS);
      DIO316_Interrupt_Enable_Control(tm_DIO316, 3, DIO316_INT_DIS);

      {
	 MCP_MSG msg;
	 STATUS stat;

	 msg.type = latchCrossed_type;
	 msg.u.latchCrossed.time = timer_read(2);
	 msg.u.latchCrossed.dio316int_bit = dio316int_bit;
	 msg.uid = 0;
	 msg.cid = 0;

	 stat = msgQSend(msgLatched, (char *)&msg, sizeof(msg),
			 NO_WAIT, MSG_PRI_NORMAL);
      }
   }
}

/*****************************************************************************/
/*
**
** Shutdown the DID48 hardware which is used to field the 1 Hz interrupt
** Timer for SDSStime
**
** GLOBALS REFERENCED:
**	tm_DID48
**
*/
void
axis_DID48_shutdown(int type)
{
   printf("AXIS DID48 shutdown: TOD interrupt %d\n", tm_DID48);
   if(tm_DID48 != -1) {
      DID48_Interrupt_Enable_Control (tm_DID48,5,DID48_INT_DIS);
   }
   taskDelay(30);
}

/*=========================================================================
**
** DESCRIPTION:
**	Interrupt handler for the 1 Hz tick interrupt.  Monitors timeliness
**	of interrupt and restarts 1us timer for interval between interrupts.
**
** GLOBALS REFERENCED:
**	NIST_sec
**	SDSS_cnt
**	SDSStime
**	axis_stat
**
**=========================================================================
*/
unsigned long NIST_sec;
unsigned char did48int_bit;
unsigned long SDSS_cnt=0;             /* number of times SDSStime was
                                         incremented */

void
DID48_interrupt(int type)
{
   int dt = 300;			/* maximum allowed fuzz in arrival
					   of GPS pulse; microseconds */

   DID48_Read_Port(tm_DID48,5,&did48int_bit);
   if(did48int_bit & NIST_INT) {
      /* Wake the setSDSStimeFromNTP routine up at the tick. */
      semGive(ppsSem);

      if(SDSStime < 0) {		/* we haven't set time yet */
	 axis_stat[AZIMUTH][0].clock_not_set = 
	   axis_stat[ALTITUDE][0].clock_not_set =
	   axis_stat[INSTRUMENT][0].clock_not_set = 1;
	    
	 axis_stat[AZIMUTH][1].clock_not_set = 
	   axis_stat[ALTITUDE][1].clock_not_set =
	   axis_stat[INSTRUMENT][1].clock_not_set = 1;

	 TRACE0(0, "DID48 interrupt with no set time", 0, 0);
	 timer_start(1);		/* reset microsecond timer */

	 /* Keep the task from spinning. Clears interrupts? */
	 DID48_Write_Reg (tm_DID48,4,0x20);
	 
	 return;
      }
      SDSS_cnt++;
/*
 * N.b. taskLock() is not callable from interrupt routines
 */
      SDSStime = (SDSStime + 1)%ONE_DAY;
      NIST_sec = timer_read(1);
      timer_start(1);			/* reset microsecond timer */

      if(NIST_sec < 1000000 - dt) {
	 if(SDSS_cnt > 1) {		/* not just the first partial second */
	    TRACE0(0, "Extra GPS pulse? NIST_sec = %d", NIST_sec, 0);
	 }
      } else if(NIST_sec > 1000000 + dt) {
	 if(SDSS_cnt > 1) {		/* not just the first partial second */
	    TRACE0(0, "Lost GPS? NIST_sec = %d", NIST_sec, 0);
	    
	    axis_stat[AZIMUTH][0].clock_loss_signal = 
	      axis_stat[ALTITUDE][0].clock_loss_signal =
		axis_stat[INSTRUMENT][0].clock_loss_signal = 1;
	    
	    axis_stat[AZIMUTH][1].clock_loss_signal = 
	      axis_stat[ALTITUDE][1].clock_loss_signal =
		axis_stat[INSTRUMENT][1].clock_loss_signal = 1;
	 }
      } else {
	 axis_stat[AZIMUTH][1].clock_loss_signal = 
	   axis_stat[ALTITUDE][1].clock_loss_signal =
	     axis_stat[INSTRUMENT][1].clock_loss_signal = 0;
      }
   }

   DID48_Write_Reg (tm_DID48,4,0x20);
}

/*=========================================================================
**
** Setup the DID48 for 1 Hz diffential interrupt (approximately +-8 volts)
**
** RETURN VALUES:
**	return 0 or ERROR
**
** CALLS TO:
**	Industry_Pack
**
** GLOBALS REFERENCED:
**	tm_DID48
**
**=========================================================================
*/
int
DID48_initialize(unsigned char *addr, unsigned short vecnum)
{
   int uid = 0, cid = 0;
   STATUS stat;
   int i;
   struct IPACK ip;
   
   Industry_Pack (addr,SYSTRAN_DID48,&ip);
   for(i = 0; i < MAX_SLOTS; i++) {
      if(ip.adr[i] != NULL) {
	 OTRACE(30, "Found at %d, %p", i, ip.adr[i]);
	 tm_DID48 = DID48Init((struct DID48 *)ip.adr[i], vecnum);
	 break;
      }
   }

   if(i == MAX_SLOTS) {
      NTRACE_1(0, uid, cid, "****Missing DID48 at %p****", addr);
      return ERROR;
   }
   
   ppsSem = semBCreate(SEM_Q_FIFO, SEM_EMPTY);

   DID48_Init = TRUE;
#if 0
   DID48_Read_Reg(tm_DID48,0x6,&vecnum);
   if (vecnum==0) vecnum = DID48_VECTOR;
#endif
   
   stat = intConnect(INUM_TO_IVEC(vecnum),
		     (VOIDFUNCPTR)DID48_interrupt, DID48_TYPE);
   assert(stat == OK);
   OTRACE(30, "DID48 vector = %d, interrupt address = %p",
	  vecnum, DID48_interrupt);
   
   rebootHookAdd((FUNCPTR)axis_DID48_shutdown);
   
   IP_Interrupt_Enable(&ip, DID48_IRQ);
   sysIntEnable(DID48_IRQ);                                
   DID48_Write_Reg (tm_DID48,3,0x3); /* disable debounce for all byte lanes */
   DID48_Interrupt_Enable_Control(tm_DID48,5,DID48_INT_ENA);

   return 0;
}

/*****************************************************************************/
/*
 * A hook to add time commands
 */
void
timeInit(void)
{
/*
 * Define time-related commands
 */
   define_cmd("SET_TIME",     set_time_cmd, -1, 0, 0, 1, "");
   define_cmd("TIME?",        time_cmd,      0, 0, 0, 1, "");
}
