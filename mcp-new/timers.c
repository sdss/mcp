#include <vxWorks.h>
#include <assert.h>
#include <semLib.h>
#include <taskLib.h>
#include <rebootLib.h>
#include <intLib.h>
#include <sysLib.h>
#include <timers.h>
#include <time.h>
#include <errno.h>
#include "iv.h"
#include "gendefs.h"
#include "mcpTimers.h"
#include "dscTrace.h"
#include "data_collection.h"
#include "axis.h"
#include "mcpMsgQ.h"
#include "cmd.h"

int tm_DID48;
int DID48_Init = FALSE;
int tm_DIO316;
int DIO316_Init = FALSE;

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
   
#if 0
   micro_sec = (unsigned long)(1.0312733648*timer_read (1));
#else
   micro_sec = timer_read (1);
#endif
   
   if(micro_sec > 1000000) micro_sec = 999999;

   return(double)(SDSStime+((micro_sec%1000000)/1000000.));
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
   double time = sdss_get_time();
   printf("SDSS time = %f\n", time);

   return(time);
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

/*=========================================================================
**=========================================================================
**
** ROUTINE: set_time_cmd
**	    print_time_changes - diagnostic
**
** DESCRIPTION:
**	SET.TIME date - sets time to WWVB.
**	Date can be specified as either "month day year hour minute second" or
**	"month day year seconds-in-day".
**
** RETURN VALUES:
**	NULL string or "ERR:..."
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	SDSStime
**	time1, time2
**
**=========================================================================
*/
static float time1, time2;
long SDSStime = -1;

char *
set_time_cmd(char *cmd)
{
   float t1,t2,t3;
   int cnt;
   struct timespec tp;
   struct tm t;
   int extrasec;
   
   cnt = sscanf(cmd, "%d %d %d %f %f %f",
		&t.tm_mon, &t.tm_mday, &t.tm_year, &t1, &t2, &t3);
   switch (cnt) {
    case 6:				/* date -> m d y h m s */
      t.tm_hour = t1;
      t.tm_min = t2;
      t.tm_sec = t3;
      break;		
      
    case 4:				/* date -> m d y s  where s is
					   in floating pt precision */
      t.tm_hour = t1/3600;
      t.tm_min = (t1 - t.tm_hour*3600)/60;
      t3 = t1 - t.tm_hour*3600 - t.tm_min*60;
      t.tm_sec = t3;
      break;		
      
    default:
      return "ERR: wrong number of args";
   }

   if(t3 - (int)t3 > 0.75) {
      taskDelay(20);			/* 1/3 sec */
      extrasec = 1;
   } else {
      extrasec = 0;
   }
   
   t.tm_year -= 1900;
   t.tm_mon -= 1;
   tp.tv_sec = mktime(&t) + extrasec;
   tp.tv_nsec = 0;
   time1 = sdss_get_time();		/* before and after for diagnostic */
   SDSStime = tp.tv_sec%ONE_DAY;
   time2 = sdss_get_time();
   clock_settime(CLOCK_REALTIME, &tp);
   
#if 0
  printf("\r\nt3=%f (extrasec=%d)",t3,extrasec);
  printf (" mon=%d day=%d, year=%d %d:%d:%d\r\n",
	t.tm_mon,t.tm_mday,t.tm_year,t.tm_hour,t.tm_min,t.tm_sec);
  printf (" sec=%d, nano_sec=%d\r\n",tp.tv_sec,tp.tv_nsec);
#endif
  
  return "";
}

void
print_time_changes(void)
{
   printf ("SDSS time1=%f time2=%f\n", time1, time2);
}

/*=========================================================================
**
**	TICKLOST @ . - Returns if receiving 1 Hz ticks.
**
** RETURN VALUES:
**	return "0" or undefined if not receiving ticks so i return NULL string.
**
** CALLS TO:
**	sdss_get_time
**	sdss_delta_time
**
*/
char *
ticklost_cmd(char *cmd)			/* NOTUSED */
{
   double tick = sdss_get_time();
   
   taskDelay(65);

   if(sdss_delta_time(sdss_get_time(), tick) > 1.0) {
      return "0";
   } else {
      return "";
   }
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
static char *time_ans={"01 31 1996 86400.000                "};	/* */
char *time_cmd(char *cmd)
{
  static struct tm *t;
  struct timespec tp;

/*  printf (" TIME? command fired\r\n");*/
  clock_gettime(CLOCK_REALTIME,&tp);
  printf (" sec=%d, nano_sec=%d\r\n",tp.tv_sec,tp.tv_nsec);
  t = localtime(&tp.tv_sec);
/*  printf ("t=%p, mon=%d day=%d, year=%d %d:%d:%d\r\n",
	t,t->tm_mon,t->tm_mday,t->tm_year,t->tm_hour,t->tm_min,t->tm_sec);*/
  sprintf (time_ans,"%d %d %d %f",t->tm_mon+1,t->tm_mday,t->tm_year+1900,
	sdss_get_time());
  return time_ans;
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
   STATUS stat;
   int i;
   struct IPACK ip;
   
   Industry_Pack (addr,SYSTRAN_DIO316,&ip);
   for(i = 0; i < MAX_SLOTS; i++) { 
      if (ip.adr[i]!=NULL) {
	 TRACE(30, "Found at %d, %p", i, ip.adr[i]);
	 tm_DIO316=DIO316Init((struct DIO316 *)ip.adr[i], vecnum);
	 break;
      }
   }
   if(i >= MAX_SLOTS) {
      TRACE(0, "****Missing DIO316 at %p****", addr, 0);
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
   TRACE(30, "DIO316 vector = %d, interrupt address = %p\n",
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

   TRACE0(16, "DIO316_interrupt", 0, 0);	 

   DIO316ReadISR(tm_DIO316, &dio316int_bit);

   if(dio316int_bit & NIST_INT) {
      TRACE(0, "NIST_INT bit is set: %d", dio316int_bit, 0);
      illegal_NIST++;
      DIO316ClearISR(tm_DIO316);
   } else {
      if(dio316int_bit & AZIMUTH_INT) {
	 DIO316_Interrupt_Enable_Control(tm_DIO316, 1, DIO316_INT_DIS);
      }
      if(dio316int_bit & ALTITUDE_INT) {
	 DIO316_Interrupt_Enable_Control(tm_DIO316, 2, DIO316_INT_DIS);
      }
      if(dio316int_bit & INSTRUMENT_INT) {
	 DIO316_Interrupt_Enable_Control(tm_DIO316, 3, DIO316_INT_DIS);
      }

      {
	 MCP_MSG msg;
	 STATUS stat;

	 msg.type = latchCrossed_type;
	 msg.u.latchCrossed.time = timer_read(2);
	 msg.u.latchCrossed.dio316int_bit = dio316int_bit;

	 stat = msgQSend(msgLatched, (char *)&msg, sizeof(msg),
			 NO_WAIT, MSG_PRI_NORMAL);
	 if(stat != OK) {
	    TRACE0(0, "Failed to send msg to msgLatched: %d", errno, 0);
	 }
      }
      TRACE0(8, "Sent message to msgLatched at %d", time, 0);
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
**	NIST_cnt
**	SDSS_cnt
**	SDSStime
**	axis_stat
**	persistent_axis_stat
**
**=========================================================================
*/
unsigned long NIST_sec;
unsigned char did48int_bit;
unsigned long NIST_cnt=0;
unsigned long SDSS_cnt=0;

void
DID48_interrupt(int type)
{
   TRACE0(16, "DID48_interrupt", 0, 0);
   
   DID48_Read_Port(tm_DID48,5,&did48int_bit);
   NIST_cnt++;
   if(did48int_bit & NIST_INT) {
      SDSS_cnt++;
      if(SDSStime >= 0) {
	 SDSStime = (SDSStime + 1)%ONE_DAY;
      }
      
      NIST_sec=timer_read(1);
      if(NIST_sec > 1000100) {
	 axis_stat[AZIMUTH].clock_loss_signal = 1;
	 persistent_axis_stat[AZIMUTH].clock_loss_signal = 1;
      } else {
	 axis_stat[AZIMUTH].clock_loss_signal = 0;
      }
      
      axis_stat[INSTRUMENT].clock_loss_signal =
	axis_stat[ALTITUDE].clock_loss_signal =
	  axis_stat[AZIMUTH].clock_loss_signal;
      
      persistent_axis_stat[INSTRUMENT].clock_loss_signal =
	persistent_axis_stat[ALTITUDE].clock_loss_signal =
	  persistent_axis_stat[AZIMUTH].clock_loss_signal;
      
      timer_start(1);
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
   STATUS stat;
   int i;
   struct IPACK ip;
   
   Industry_Pack (addr,SYSTRAN_DID48,&ip);
   for(i = 0; i < MAX_SLOTS; i++) {
      if(ip.adr[i] != NULL) {
	 TRACE(30, "Found at %d, %p", i, ip.adr[i]);
	 tm_DID48 = DID48Init((struct DID48 *)ip.adr[i], vecnum);
	 break;
      }
   }

   if(i == MAX_SLOTS) {
      TRACE(0, "****Missing DID48 at %p****", addr, 0);
      return ERROR;
   }
   
   DID48_Init = TRUE;
#if 0
   DID48_Read_Reg(tm_DID48,0x6,&vecnum);
   if (vecnum==0) vecnum = DID48_VECTOR;
#endif
   
   stat = intConnect(INUM_TO_IVEC(vecnum),
		     (VOIDFUNCPTR)DID48_interrupt, DID48_TYPE);
   assert(stat == OK);
   TRACE(30, "DID48 vector = %d, interrupt address = %p",
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
   define_cmd("SET.TIME",     set_time_cmd, -1, 1, 1);
   define_cmd("TICKLOST @ .", ticklost_cmd,  0, 0, 1);
   define_cmd("TIME?",        time_cmd,      0, 0, 1);
}
