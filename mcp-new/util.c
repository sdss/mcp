#include "vxWorks.h"
#include "stdlib.h"
#include "stdio.h"
#include <time.h>
#include <assert.h>
#include "sysLib.h"
#include "logLib.h"
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
  logMsg ("Test Interrupt Routine fired, status=%lx level=%lx\r\n",
	(long)*VMECHIP2_LBISR,
	(long)*VMECHIP2_ILR3,0,0,0,0);
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
 * Open a logfile in /mcptpm/<mjd>
 */
int
open_log(const char *file)
{
   int fd;
   char filename[100];

   sprintf(filename, "/mcptpm/%d/%s", mjd(), file);
   assert(strlen(filename) < sizeof(filename));

   printf("Opening %s\n", filename);

   fd = open(filename, O_RDWR|O_CREAT, 0666);

   return(fd);
}

/*****************************************************************************/
/*
 * Return the MJD, as modified by the SDSS to roll over at 10am
 */
#if 1
extern void slaCldj(int, int, int, double*, int*);

int
mjd(void)
{
   int stat;
   double ldj;
   time_t t;
   struct tm *Time;

   (void)time(&t);
   Time = gmtime(&t);
   assert(Time != NULL);

   slaCldj(Time->tm_year + 1900, Time->tm_mon + 1, Time->tm_mday, &ldj, &stat);

   if(stat) {
      fprintf(stderr,"slaCldj returns %d (Y m d == %d %d %d)\n",
	      stat, Time->tm_year + 1900, Time->tm_mon + 1, Time->tm_mday);
   
      return(-1);
   } else {
      fprintf(stderr,"slaCldj returns %d (Y m d == %d %d %d)\n",
	      stat, Time->tm_year + 1900, Time->tm_mon + 1, Time->tm_mday);
   
      return((int)(ldj + 0.3));
   }
}
#endif
