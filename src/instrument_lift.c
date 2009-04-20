#include "vxWorks.h"                            
#include "stdio.h"
#include "sysLib.h"
#include "taskLib.h"
#include "usrLib.h"
#include "semLib.h"
#include "sigLib.h"
#include "sysSymTbl.h"
#include "tickLib.h"
#include "logLib.h"
#include "inetLib.h"
#include "rebootLib.h"
#include "in.h"
#include "tyLib.h"
#include "ioLib.h"
#include "timers.h"
#include "time.h"
#include "gendefs.h"
#include "dio316dr.h"
#include "dio316lb.h"
#include "ad12f1lb.h"
#include "da128vlb.h"
#include "da128vrg.h"
#include "ip480.h"
#include "mv162IndPackInit.h"
#include "cw.h"
#include "tm.h"
#include "data_collection.h"
#include "abdh.h"
#include "instruments.h"
#include "axis.h"
#include "dscTrace.h"
#include "acromag.h"
#include "as2.h"

#define IL_ENABLE	3
#define IL_DISABLED	0xDF

#define	IL_POSITION	4		/*	      9,10 */
#define	IL_STRAIN_GAGE	5		/*	     11,12 */

#define IL_WD		2		/* WD channel    11 */
#define TM_WD		4		/* WD channel    20 */
#define CW_WD		6		/* WD channel    29 */

struct conf_blk sbrd;
int il_DIO316=-1;
int il_ADC128F1=-1;
int il_DAC128V=-1;

static void
il_enable_motion_status(void)
{
   unsigned char val;
   
   DIO316_Read_Port(il_DIO316,IL_ENABLE,&val);
}

static void
il_disable_motion(void)
{
   unsigned char val;
   
   DIO316_Read_Port (il_DIO316, IL_ENABLE, &val);
   DIO316_Write_Port (il_DIO316, IL_ENABLE, val & IL_DISABLED);
   /*	il_motion_up (0);*/
   il_enable_motion_status();
}

int
shutdown_wd(int type)
{
  printf("WD IP480 shutdown: 3 Interrupts\r\n");
  
  SetInterruptEnable(&sbrd, IL_WD, IntDisable);
  SetInterruptEnable(&sbrd, TM_WD, IntDisable);
  SetInterruptEnable(&sbrd, CW_WD, IntDisable);

  taskDelay(30);
  
  return 0;
}

int wdog=0;

void
wd_isr(struct conf_blk *cblk)
{
   int i,j;
   UWORD i_stat;
   
   i_stat = inpw((UWORD *)(cblk->brd_ptr + InterruptPending));
   if(cblk->num_chan == 2) {		/* check if it's a 2 or 6 channel bo */
      i_stat &= 0x0300;			/* and clear the unused upper bits */
      j = 2;
   } else {
      i_stat &= 0x3F00;			/* clear unused bits and save the */
      j = 6;
   }

   if(i_stat) {				/* any */
      cblk->event_status |= (BYTE)(i_stat >> 8);   /* update event */

        /* service the hardware */
      OTRACE(16, "IP480 ABORT fired %d, istat=%d", wdog++, i_stat);
        /* check each bit for an interrupt pending state */
      for(i = 0; i < j; i++) {		/* check each c */
	 if(i_stat & (1 << (i + 8))) {	/* build interr */
	    i_stat &= ~(1 << (i + 8));
          } else {
	     i_stat |= (1 << (i + 8));
	  }
      }
      outpw((UWORD *)(cblk->brd_ptr + InterruptPending),
	    i_stat);			/* write interrupt pe */
   }
}

int
setup_wd(char *addr,
	 unsigned char vec,
	 int irq)
{
   sbrd.brd_ptr=(BYTE *)addr;
   
   SetInterruptVector(&sbrd,vec);
   attach_ihandler(0,sbrd.m_InterruptVector,0,(FUNCPTR)wd_isr,
		   (struct handler_data *)&sbrd);
   rebootHookAdd((FUNCPTR)shutdown_wd);

   return 0;
}

int
il_setup_wd(void)
{
   SetMode(&sbrd,IL_WD,Watchdog);
   SetDebounce(&sbrd,IL_WD,DebounceOff);
   SetInterruptEnable(&sbrd,IL_WD,IntEnable);
   SetCounterSize(&sbrd,IL_WD,CtrSize32);
   SetCounterConstant(&sbrd,IL_WD,60000);
   SetClockSource(&sbrd,IL_WD,InC1Mhz);
   SetTriggerSource(&sbrd,IL_WD,InTrig);
   SetWatchdogLoad(&sbrd,IL_WD,WDIntLd);
   SetOutputPolarity(&sbrd,IL_WD,OutPolHi);
   ConfigureCounterTimer(&sbrd,IL_WD);
   
  return 0;
}

void
il_data_collection(void)
{
   unsigned short adc;
   
   if(il_ADC128F1 != -1) {
      ADC128F1_Read_Reg(il_ADC128F1,IL_POSITION,&adc);
      if((adc&0x800) == 0x800) {
	 sdssdc.inst.pos = adc | 0xF000;
      } else {
	 sdssdc.inst.pos = adc & 0xFFF;
      }
      
      ADC128F1_Read_Reg(il_ADC128F1,IL_STRAIN_GAGE,&adc);
      
      if((adc&0x800) == 0x800) {
	 sdssdc.inst.strain_gage = adc | 0xF000;
      } else {
	 sdssdc.inst.strain_gage = adc & 0xFFF;
      }
   }
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: lift_initialize
**
** DESCRIPTION:
**      Initialize the hardware utilized for the instrument lift.  Shares 
**	resources with the counter-weight system.  Checks to see if already
**	initialized by cw and then uses the same handle if TRUE.
**
** RETURN VALUES:
**	
**
** CALLS TO:
**	Industry_Pack
**	setup_wd
**	il_setup_wd
**
** GLOBALS REFERENCED:
**	cw_ADC128F1
**	il_ADC128F1
**	cw_DAC128V
**	il_DAC128V
**	cw_DIO316
**	il_DIO316
**
**=========================================================================
*/
int lift_initialize(unsigned char *addr)
{
  int i;
  unsigned short val;
  struct IPACK *ip;

  ip = (struct IPACK *)malloc (sizeof(struct IPACK));
  if (ip==NULL) return ERROR;
  if (cw_ADC128F1==-1)
  {
    Industry_Pack (addr,SYSTRAN_ADC128F1,ip);
    for (i=0;i<MAX_SLOTS;i++)
      if (ip->adr[i]!=NULL)
      {
        il_ADC128F1 = ADC128F1Init((struct ADC128F1 *)ip->adr[i]);
        break;
      }
    if (i>=MAX_SLOTS)
    {
      printf ("\r\n****Missing ADC128F1 at %p****\r\n",addr);
      free (ip);
      return ERROR;
    }
    ADC128F1_CVT_Update_Control(il_ADC128F1,ENABLE);
    cw_ADC128F1=il_ADC128F1;
  }
  else
    il_ADC128F1=cw_ADC128F1;

  if (cw_DAC128V==-1)
  {
    Industry_Pack (addr,SYSTRAN_DAC128V,ip);
    for (i=0;i<MAX_SLOTS;i++)
      if (ip->adr[i]!=NULL)
      {
        il_DAC128V = DAC128VInit((struct DAC128V *)ip->adr[i]);
        break;
      }
    if (i>=MAX_SLOTS)
    {
      printf ("\r\n****Missing DAC128V at %p****\r\n",addr);
      free (ip);
      return ERROR;
    }
    for (i=0;i<DAC128V_CHANS;i++)
    {
      DAC128V_Read_Reg(il_DAC128V,i,&val);
      if ((val&0xFFF) != 0x800)
                printf ("\r\nDAC128V Chan %d Init error %x",
                        i,val);
    }
    cw_DAC128V=il_DAC128V;
  }
  else
    il_DAC128V=cw_DAC128V;

  if (cw_DIO316==-1)
  {
    Industry_Pack (addr,SYSTRAN_DIO316,ip);
    for (i=0;i<MAX_SLOTS;i++)
      if (ip->adr[i]!=NULL)
      {
        il_DIO316 = DIO316Init((struct DIO316 *)ip->adr[i], 0);
        break;
      }
    if (i>=MAX_SLOTS)
    {
      printf ("\r\n****Missing DIO316 at %p****\r\n",addr);
      free (ip);
      return ERROR;
    }
    cw_DIO316=il_DIO316;
  }
  else
    il_DIO316=cw_DIO316;
/*  DIO316_Write_Port(il_DIO316,2,0xFF);*/
  DIO316_OE_Control(il_DIO316,2,DIO316_OE_ENA);
/*  DIO316_Write_Port(il_DIO316,3,0x0);*/
  il_disable_motion();
  DIO316_OE_Control(il_DIO316,3,DIO316_OE_ENA);

  Industry_Pack (addr,MODEL_IP_480_6,ip);
  for (i=0;i<MAX_SLOTS;i++)
    if (ip->adr[i]!=NULL)
    {
      setup_wd((char *)ip->adr[i], 0xB4, 3);
      il_setup_wd();
 /* VIPC610_IP_Interrupt_Enable (addr, 0, irq);*/
/*  VMESC5_IP_Interrupt_Enable (addr, 0, irq);*/
      IP_Interrupt_Enable (ip, 3);
      sysIntEnable (3);
      break;
    }
  if (i>=MAX_SLOTS)
  {
    printf ("\r\n****Missing IP480 at %p****\r\n",addr);
    free (ip);
    return ERROR;
  }
    
  free (ip);
  il_disable_motion();

#if 0
  for (i=0;i<sizeof(il_inst)/(sizeof(struct IL_LOOP)*2);i++)
  {
    il_inst[IL_UP][i].pos_current=il_inst[IL_UP][i].pos_error=0;
    il_inst[IL_UP][i].start_decel_position=50;/* start position to begin deceleration */
    il_inst[IL_UP][i].stop_pos_error=2;	/* stop position error allowed */
    il_inst[IL_UP][i].stop_count=6;		/* stop polarity swing counts allowed */
    il_inst[IL_DN][i].pos_current=il_inst[IL_DN][i].pos_error=0;
    il_inst[IL_DN][i].start_decel_position=50;/* start position to begin deceleration */
    il_inst[IL_DN][i].stop_pos_error=2;	/* stop position error allowed */
    il_inst[IL_DN][i].stop_count=6;		/* stop polarity swing counts allowed */
  }
#endif
  return 0;
}
