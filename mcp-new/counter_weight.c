#include "copyright.h"
/**************************************************************************
***************************************************************************
** FILE:
**      counter_weight.c
**
** ABSTRACT:
	Balance the telescope utilizing the four counter weights.  The 
control is by moving a motor through an DAC to a position decoded by an
absolute encoded (similar to a potentiometer) which is read back through 
an ADC.  The motor is slowed as it approaches the known balance point of the
axis and stopped upon arrival. The telescope is then finely balanced by 
monitoring the motor currents of the motion controls.
Hardware Requirements:
	2 DO bits to select the CW (0-3)
	1 DO bit to enable/inhibit the drive - watchdog timer
	4*2 DI bits for limit switch status
	1 DI bit interlock status

DIO316 3*16
  port	0 bi-directional
	1 bi-directional
	2 output
	3 output
		CW_SELECT
			bit 0-1 pin 32-31
		CW_DIRECTION		
			bit 2   pin 30
		CW_POWER
			bit 4   pin 28
		DC_INTERRUPT
			bit 7   pin 25
	4 input
		CW_LIMIT_INT
			bit 0   pin 40
			bit 0-3 interruptable   pin 40-37
		CW_INTERLOCK
			bit 4	pin 36	
		CW_LCLRMT
			bit 5   pin 35
	5 input
		CW_LIMIT_STATUS
			bit 0   pin 48
			bit 1   pin 47
			bit 2   pin 46
			bit 3   pin 45
			bit 4   pin 44
			bit 5   pin 43
			bit 6   pin 42
			bit 7   pin 41
DAC128V
	1 DAC to drive the selected CW
		CW_MOTOR	0
ADC128F1
	4 ADC to readback the CW position
		CW_POS_0	0
		CW_POS_1	1
		CW_POS_2	2
		CW_POS_3	3

	+/-10 volts is the range for the DAC (direction based on sign)
	5:1 gear box
	10 volts is 2500 RPM or 500 RPM after the gear reduction
**
** ENTRY POINT          SCOPE   DESCRIPTION
** ----------------------------------------------------------------------
** cw_DIO316_shutdown
** cw_DIO316_interrupt
** balance_initialize
** kbd_input
** balance_weight
** balance_cmd
** balance_init
** balance 
** cw_pos
** cw_position
** cw_calc 
** cw_DIO316_interrupt
** cw_DIO316_shutdown
** read_ADC 
** read_all_ADC 
** cw_brake_on
** cw_brake_off
** cw_power_on
** cw_power_off
** cw_select
** cw_status
** cw_abort
** cw_motor_raw
** cw_motor
** cw_list 
** cw_read_position
** cw_set_position 
** cw_set_params 
** cw_set_const 
** CW_help
** CW_Verbose
** CW_Quiet
** cw_data_collection	public	counter-weight data collection
**
** ENVIRONMENT:
**      ANSI C.
**
** REQUIRED PRODUCTS:
**
** AUTHORS:
**      Creation date:  Aug 30, 1999
**      Charlie Briegel
**
***************************************************************************
***************************************************************************/

/************************************************************************/
/* Project: 	SDSS - Sloan Digital Sky Survey				*/
/* 		Counter Weight Control					*/
/*   File:	counter_weight.c							*/
/************************************************************************/
/*   Location:	Fermi National Accelerator Lab				*/
/*   Author:	Charlie Briegel, X4510, MS 360, ALMOND::[BRIEGEL]	*/
/*   Program:	counter_weight : VxWorks				*/
/*   Modules:	balance : 	    					*/	
/*++ Version:
  1.00 - initial --*/
/*++ Description:
--*/
/*++ Notes:
--*/
/************************************************************************/
/*------------------------------*/
/*	includes		*/
/*------------------------------*/
#include "vxWorks.h"                            
#include "semLib.h"
#include "sigLib.h"
#include "taskLib.h"
#include "sysLib.h"
#include "stdio.h"
#include "logLib.h"
#include "logLib.h"
#include "tickLib.h"
#include "inetLib.h"
#include "in.h"
#include "tyLib.h"
#include "ioLib.h"
#include "timers.h"
#include "time.h"
#include "iv.h"
#include "intLib.h"
#include "string.h"
#include "rebootLib.h"
#include "gendefs.h"
#include "dio316dr.h"
#include "dio316lb.h"
#include "mv162IndPackInit.h"
#include "cw.h"
#include "ad12f1lb.h"
#include "did48lb.h"
#include "da128vlb.h"
#include "da128vrg.h"
#include "ip480.h"
#include "data_collection.h"
#include "io.h"

/*========================================================================
**========================================================================
**
** LOCAL MACROS, DEFINITIONS, ETC.
**
**========================================================================
*/
/*------------------------------------------------------------------------
**
** LOCAL DEFINITIONS
*/
/* below is a place for DIAGNOStic flag for turning off the feature
#define DIAGNOS 0
*/
#define NULLFP (void(*)()) 0
#define NULLPTR ((void *) 0)
#define ONCE if (YES)

/*-------------------------------------------------------------------------
**
** GLOBAL VARIABLES
*/
/*
  Brake is applied: If positive direction, go below stop limit (i.e. 0) but
not less than moving limit.  If negative direction, go above stop limit but 
not greater than moving limit.    
Negative Moving Limit      0     Stop Limit      Positive Moving Limit
---------|-----------------|---------|---------------------|--------------------
          Positive Brake............. Negative Brake.......
*/
#define STOP_LIM	10

#define INST_CAMERA	0
#define INST_FIBER	1
#define INST_EMPTY	2
#define INST_4		4
#define INST_5		5
#define INST_6		6
#define INST_7		7
#define INST_8		8
#define INST_9		9
#define INST_10		10
#define INST_11		11
#define INST_12		12
#define INST_13		13
#define INST_14		14
#define INST_15		15
#define INST		17
char *inst_name[]={"CAMERA","FIBER","EMPTY","INST3",
			"INST4","INST5","INST6","INST7",
			"INST8","INST9","INST10","INST11",
			"INST12","INST13","INST14","INST15",
			"INSTDEF"};
#define CW_0		0
#define CW_1		1
#define CW_2		2
#define CW_3		3
#define CW_MAX		4

#define POS_DIRECTION TRUE
#define NEG_DIRECTION FALSE

/* 10 volts = 2500 RPM, with a 5:1 gear ratio 				*/
/* therefore, 10 volts = 500 RPM, 1 volt = 50 RPM, .02 volts = 1 RPM	*/
#define RPM_IN_VOLTS	.02	/* voltage for one RPM */
#define RPM_PER_COUNT	.244140625	/* 500./2048. */
#define INCHES_PER_RPM	.1	/* 24 inches of total travel */
#define POS_PER_INCH  	66.5770 /* position in adc units per inch */

struct CW_LOOP {
	short pos_setting[CW_MAX];	/* position for a given instrucment */
	short pos_current[CW_MAX];	/* current position */
	short pos_error[CW_MAX];	/* error for the given instrument */
	short updates_per_sec;		/* update rate of the loop */
	float accel_rpm;		/* user specified acceleration */
	float vel_rpm;			/* user specified velocity */
	float decel_rpm;		/* user specified deceleration */
	float stop_vel_rpm;		/* user specified stop velocity */
	short velocity;			/* velocity in units for DAC */
	short accel;			/* acceleration in units for DAC */
	short decel;			/* deceleration in units for DAC */
	short stop_velocity;			/* stopping velocity in units for DAC */
	short start_decel_position;	/* start position to begin deceleration */
	short stop_pos_error;		/* stop position error allowed */
	short stop_count;		/* stop polarity swing counts allowed */
};
/*                                  1      2     3     4                */
struct CW_LOOP	cw_inst[] = {	{   50,    50,   50,   50},	/*CAMERA*/
				{0x400, 0x400,0x400,0x400},	/*FIBER	*/
				{ 1432,  1470, 1470, 1470},	/*EMPTY	*/
				{0x200, 0x200,0x200,0x200},	/*INST3	*/
				{0x200, 0x200,0x200,0x200},	/*INST4 */
				{0x200, 0x200,0x200,0x200},	/*INST5 */
				{0x200, 0x200,0x200,0x200},	/*INST6 */
				{0x200, 0x200,0x200,0x200},	/*INST7 */
				{0x200, 0x200,0x200,0x200},	/*INST8 */
				{0x200, 0x200,0x200,0x200},	/*INST9 */
				{0x200, 0x200,0x200,0x200},	/*INST10*/
				{0x200, 0x200,0x200,0x200},	/*INST11*/
				{0x200, 0x200,0x200,0x200},	/*INST12*/
				{0x200, 0x200,0x200,0x200},	/*INST13*/
				{0x200, 0x200,0x200,0x200},	/*INST14*/
				{0x200, 0x200,0x200,0x200},	/*INST15*/
				{0x200, 0x200,0x200,0x200}	/*DEFAULT*/
};
int CW_verbose=FALSE;
int CW_limit_abort=FALSE;
unsigned char cwLimit;
/* counter-weight ip hardware index reference */
int cw_DIO316=-1;
int cw_ADC128F1=-1;
int cw_DAC128V=-1;

/* Prototypes */
void cw_DIO316_shutdown(int type);
void cw_DIO316_interrupt(int type);
int balance_initialize(unsigned char *addr, unsigned short vecnum);
int kbd_input();
char *balance_weight(int inst);
char *balance_cmd(char *cmd);
int balance_init();
void balance (int cw, int inst);
void cw_pos(int cw, float *pos);
void cw_position(int cw, double pos);
void cw_calc (struct CW_LOOP *cw);
void cw_DIO316_interrupt(int type);
void cw_DIO316_shutdown(int type);
void read_ADC (int chan, int cnt);
void read_all_ADC (int cnt);
int cw_brake_on();
int cw_brake_off();
int cw_power_on();
int cw_power_off();
int cw_select(int cw);
int cw_status();
int cw_abort ();
void cw_motor_raw (int cw, int vel);
void cw_motor (int cw, double vel_rpm);
void cw_list (int inst);
void cw_read_position (int cnt);
void cw_set_position (int inst, double p1, double p2, double p3, double p4);
void cw_set_params (int inst, double accel, double vel, double decel, double stop_vel);
void cw_set_const (int inst, int upd, int start_decel_pos, int stop_pos_err, int stop_cnt);
void CW_help();
void CW_Verbose();
void CW_Quiet();
void cw_data_collection();

/*=========================================================================
**=========================================================================
**
** ROUTINE: balance_weight   executed from the shell for testing
**	    balance_cmd      executed from the TCC
**
** DESCRIPTION:
**      Balances for the instrument all four counter-weights in series.
**
**
** RETURN VALUES:
**      char *       error msg
**
** CALLS TO:
**      balance
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
char *balance_weight(int inst)
{
  int cw;

  printf ("\r\nBALANCE WEIGHT\r\n");
  for (cw=CW_0;cw<CW_MAX;cw++)
    balance (cw,inst);
  return "";
}
char *balance_cmd(char *cmd)
{
  int cw,inst;

  printf ("\r\nBALANCE command fired\r\n");
  if ((inst=cw_get_inst(cmd))!=-1)
  {
    for (cw=CW_0;cw<CW_MAX;cw++)
      balance (cw,inst);
    return "";
  }
  else
    return "BAD NAME";
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: balance_initialize   initialize the hardware
**
** DESCRIPTION:
**      Initializes all the hardware needed for the counter_weight system.
**
**
** RETURN VALUES:
**      int 	ERROR or zero
**
** CALLS TO:
**      Industry_Pack
**      ADC128F1Init
**	ADC128F1_CVT_Update_Control
**      DAC128VInit
**	DAC128V_Read_Reg
**	DAC128V_Write_Reg
**      DIO316Init
**	IP_Interrupt_Enable
**	DIO316_OE_Control
**	DIO316_Interrupt_Configuration
**	DIO316_Write_Reg
**	DIO316_Interrupt_Enable_Control
**	cw_power_off
**
** GLOBALS REFERENCED:
**	cw_ADC128F1
**	cw_DAC128V
**	cw_DIO316
**
**=========================================================================
*/
int balance_initialize(unsigned char *addr, unsigned short vecnum)
{
  int i,ii;                          
  short val;
  STATUS stat;                               
  struct IPACK *ip;

  ip = (struct IPACK *)malloc (sizeof(struct IPACK));
  if (ip==NULL) return ERROR;

/*  Initialize the ADC */
  Industry_Pack (addr,SYSTRAN_ADC128F1,ip);
  for (i=0;i<MAX_SLOTS;i++)
    if (ip->adr[i]!=NULL)
    {
      cw_ADC128F1 = ADC128F1Init((struct ADC128F1 *)ip->adr[i]);
      break;
    }
  if (i>=MAX_SLOTS)
  {
    printf ("\r\n****Missing ADC128F1 at %p****\r\n",addr);
    free (ip);
    return ERROR;
  }
  ADC128F1_CVT_Update_Control(cw_ADC128F1,ENABLE);

/*  Initialize the DAC */
  Industry_Pack (addr,SYSTRAN_DAC128V,ip);
  for (i=0;i<MAX_SLOTS;i++)
    if (ip->adr[i]!=NULL)
    {
      cw_DAC128V = DAC128VInit((struct DAC128V *)ip->adr[i]);
      break;
    }
  if (i>=MAX_SLOTS)
  {
    printf ("\r\n****Missing DAC128V at %p****\r\n",addr);
    free (ip);
    return ERROR;
  }
/* check if voltages are zero */
  for (i=0;i<DAC128V_CHANS;i++) 
  {
    DAC128V_Read_Reg(cw_DAC128V,i,&val);
    if ((val&0xFFF) != 0x800) 
      printf ("\r\nDAC128V Chan %d Init error %x",i,val);
  }

/*  Initialize the DIO316 */
  Industry_Pack (addr,SYSTRAN_DIO316,ip);
  for (i=0;i<MAX_SLOTS;i++)
    if (ip->adr[i]!=NULL)
    {
      cw_DIO316 = DIO316Init((struct DIO316 *)ip->adr[i], vecnum);
      break;
    }
  if (i>=MAX_SLOTS)
  {
    printf ("\r\n****Missing DIO316 at %p****\r\n",addr);
    free (ip);
    return ERROR;
  }
  stat = intConnect (INUM_TO_IVEC(vecnum),
                                (VOIDFUNCPTR)cw_DIO316_interrupt,
                                DIO316_TYPE);
  printf ("CW vector = %d, interrupt address = %p, result = %8x\r\n",
                vecnum,cw_DIO316_interrupt,stat);
  rebootHookAdd((FUNCPTR)cw_DIO316_shutdown);

  IP_Interrupt_Enable(ip,DIO316_IRQ);
  DIO316_OE_Control (cw_DIO316,3,DIO316_OE_ENA);
  DIO316_Interrupt_Configuration (cw_DIO316,0,DIO316_INT_FALL_EDGE);
  sysIntEnable(DIO316_IRQ);
  DIO316_Write_Reg(cw_DIO316,6,0xF);

/* Turned off the interrupts for the limits - unreliable */
/*  DIO316_Interrupt_Enable_Control (cw_DIO316,0,DIO316_INT_ENA);*/
  cw_power_off();

/* zero the DAC - there is a 2048 offset on the 12 bit DAC */
  DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,0x800);

/* Initialize the data structures for nominal operation */
  for (i=0;i<sizeof(cw_inst)/sizeof(struct CW_LOOP);i++)
  {
    for (ii=0;ii<CW_MAX;ii++)
      cw_inst[i].pos_current[ii]=cw_inst[i].pos_error[ii]=0;
    cw_inst[i].updates_per_sec=10;
    cw_inst[i].accel_rpm=12000.;	/* user specified acceleration */
    cw_inst[i].vel_rpm=500.;		/* user specified velocity */
    cw_inst[i].decel_rpm=6000.;	/* user specified deceleration */
    cw_inst[i].stop_vel_rpm=220.;	/* user specified stop velocity */
    cw_inst[i].start_decel_position=58;/* start position to begin deceleration */
    cw_inst[i].stop_pos_error=2;	/* stop position error allowed */
    cw_inst[i].stop_count=6;		/* stop polarity swing counts allowed */
    cw_calc (&cw_inst[i]);
  }

  free (ip);
  return 0;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: balance
**
** DESCRIPTION:
**      Balances one counter-weight according to its data structure for the 
**	instrument.
**      This enables a log file 'cwp.log' to track the motion by redirecting
**	all std out and error to the file.
**
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**	DAC128V_Write_Reg
**	cw_power_on
**	cw_brake_off
**	cw_status
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void balance (int cw, int inst)
{
  int fd;
  int i;
  short vel,pos,delta,direction;
#ifdef FAKE_IT
  short dpos;
#endif
  short last_direction;
  int totcnt, cnt, last_error;

  printf ("\r\nBALANCE CW %d: for instrument %d",cw,inst);
  fd=open ("cwp.log",O_RDWR|O_CREAT,0666);
  ioTaskStdSet(0,1,fd);
  ioTaskStdSet(0,2,fd);

/* set mux for specified counter-weight */
  cw_select (cw);

/* iterate until good or exceed stop count */
  DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,0x800);
  cw_power_on();
  cw_brake_off();
  CW_limit_abort=FALSE;

  for (i=0;i<cw_inst[inst].stop_count;i++)
  {
    totcnt=cnt=0;
    ADC128F1_Read_Reg(cw_ADC128F1,cw,&pos);
    if ((pos&0x800)==0x800) pos |= 0xF000;
    else pos &= 0xFFF;
    delta = abs(pos-cw_inst[inst].pos_setting[cw]);
    last_error=delta;
    cw_inst[inst].pos_current[cw]=pos;
    cw_inst[inst].pos_error[cw]=delta;
    direction = pos<cw_inst[inst].pos_setting[cw]?POS_DIRECTION:NEG_DIRECTION;
    last_direction = direction;
/* iterate until position is adequate */
    while (delta>cw_inst[inst].stop_pos_error)
    {
      taskDelay(60/cw_inst[inst].updates_per_sec);
      if (CW_limit_abort)
      {
        printf ("\r\nLIMIT ABORT");
	cw_status();
	return;
      }
      cnt++;
      if (cnt%(cw_inst[inst].updates_per_sec*4)==0)
      {
        if (delta>(last_error-4))
        {
	  cw_abort();
	  printf ("\r\nNot Closing in on position, cw_abort the balance");
	  cw_status();
	  return;
        }
        else
          last_error=delta;
      }
/* deceleration & coast to position */
      DAC128V_Read_Reg(cw_DAC128V,CW_MOTOR,&vel);
      if (delta<cw_inst[inst].start_decel_position)
      {
/* still decelerating */
	if ((abs(vel-0x800)-cw_inst[inst].decel)>cw_inst[inst].stop_velocity) 
	{
	  if (direction)
	    DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,vel-cw_inst[inst].decel);
	  else
	    DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,vel+cw_inst[inst].decel);
	  if (CW_verbose) printf ("\r\n DECEL: ");
	}
	else
	{
/* coast with stop velocity */
	  if (direction)
	    DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,0x800+cw_inst[inst].stop_velocity);
	  else
	    DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,0x800-cw_inst[inst].stop_velocity);
	  if (CW_verbose) printf ("\r\n STOP VEL: ");
        }    
      }    
      else
      {
/* accelerate to velocity */
        if ((abs(vel-0x800)+cw_inst[inst].accel)<cw_inst[inst].velocity)
	{
	  if (direction)
	    DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,vel+cw_inst[inst].accel);
	  else
	    DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,vel-cw_inst[inst].accel);
	  if (CW_verbose) printf ("\r\n ACCEL: ");
        }    
	else
	{
/* maintain velocity */
	  if (direction)
	    DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,0x800+cw_inst[inst].velocity);
	  else
	    DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,0x800-cw_inst[inst].velocity);
	  if (CW_verbose) printf ("\r\n ACCEL VEL: ");
        }    
      }
/* fake moving the weight */
#ifdef FAKE_IT
#define FAKE_IT	1
      DAC128V_Read_Reg(cw_DAC128V,CW_POS+cw,&dpos);
      DAC128V_Write_Reg(cw_DAC128V,CW_POS+cw,dpos+((vel-0x800)/10));
#endif
      ADC128F1_Read_Reg(cw_ADC128F1,CW_POS+cw,&pos);
      if ((pos&0x800)==0x800) pos |= 0xF000;
      else pos &= 0xFFF;
      delta = abs(pos-cw_inst[inst].pos_setting[cw]);
      cw_inst[inst].pos_current[cw]=pos;
      cw_inst[inst].pos_error[cw]=delta;
      last_direction = direction;
      direction = pos<cw_inst[inst].pos_setting[cw]?POS_DIRECTION:NEG_DIRECTION;
      if (direction!=last_direction) break;
      if (CW_verbose) 
	printf ("vel=%x,pos=%x,delta=%x,direction=%d",
		vel-0x800,pos,delta,direction);
      printf ("\r\n  vel=%f rpm ,pos=%6.4f\", %4.2f ",
		(vel-0x800)*RPM_PER_COUNT,(24*pos)/(2048*0.7802),(10*pos)/2048.);
    }
    totcnt += cnt;
    cw_brake_on();
    DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,0x800);
    cw_brake_off();
    if (CW_verbose) printf ("\r\n SWITCH: ");
  }
  cw_brake_on();
  cw_power_off();
  if (CW_verbose) printf ("\r\n STOP: ");
  printf ("\r\n .................time=%d secs\r\n",totcnt/cw_inst[inst].updates_per_sec);
  close (fd);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_posv	- differs in argument passing
**	    cw_positionv
**
** DESCRIPTION:
**      Set the DEFAULT instrument to a position specified in volts
**	Deprecated functions implemented inches across the table (0-24).
**
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**	cw_set_positionv
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void cw_pos(int cw, float *pos)
{
  cw_set_position (INST_DEFAULT, (double)*pos, (double)*pos, (double)*pos, (double)*pos);
  balance (cw, INST_DEFAULT);
}
void cw_position(int cw, double pos)
{
  cw_set_position (INST_DEFAULT, pos, pos, pos, pos);
  balance (cw, INST_DEFAULT);
}
void cw_posv(int cw, short *pos)
{
  cw_set_positionv (INST_DEFAULT, *pos, *pos, *pos, *pos);
  balance (cw, INST_DEFAULT);
}
void cw_positionv(int cw, short pos)
{
  cw_set_positionv (INST_DEFAULT, pos, pos, pos, pos);
  balance (cw, INST_DEFAULT);
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_calc
**
** DESCRIPTION:
**      Based on the instruments specifications, safe limits are imposed
**	for velocity, acceleration, and deceleration.
**	Numbers are pre-calculated into DAC units from an rpm specification.
**	The time to slow down is appoximated for a limit.
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
/* DAC is set for +1 10volts where 0=-10 volts;0x800=0 volts; 0xFFF=+10 volts */
/* pos and neg voltages provide direction control */
#define ONE_VLT		204.8		/* 0x800/10.0 */
#define MAX_VEL		8*204	/* absolute ranges */
#define MIN_VEL		2*204	/* 400-50 rpm range */
#define MAX_ACCEL	100	/* absolute step change */
#define MIN_ACCEL	10
#define MAX_STOP_VEL	4*204	/* absolute ranges */
#define MIN_STOP_VEL	2*204	/* 100-2.4 rpm range */
void cw_calc (struct CW_LOOP *cw)
{
  float val;
  short stop_guess;

  val = (cw->vel_rpm*RPM_IN_VOLTS)*ONE_VLT;
  if (val>MAX_VEL)
    cw->velocity=MAX_VEL;		/* velocity in units for DAC */
  else
  {
    if (val<MIN_VEL)
      cw->velocity=MIN_VEL;
    else
      cw->velocity=val;/* velocity in units for DAC */
  }

  val=(cw->accel_rpm/(60*cw->updates_per_sec))*
		RPM_IN_VOLTS*ONE_VLT;	/* acceleration in units for DAC */
  if (val>MAX_ACCEL)
    cw->accel=MAX_ACCEL;		/* acceleration in units for DAC */
  else
  {
    if (val<MIN_ACCEL)
      cw->accel=MIN_ACCEL;
    else
      cw->accel=val;/* velocity in units for DAC */
  }

  val=cw->decel_rpm/(60*cw->updates_per_sec)*
		RPM_IN_VOLTS*ONE_VLT;	/* deceleration in units for DAC */
  if (val>MAX_ACCEL)
    cw->decel=MAX_ACCEL;		/* deceleration in units for DAC */
  else
  {
    if (val<MIN_ACCEL)
      cw->decel=MIN_ACCEL;
    else
      cw->decel=val;			/* deceleration in units for DAC */
  }

  val = (cw->stop_vel_rpm*RPM_IN_VOLTS)*ONE_VLT;/* stopping velocity in units for DAC */
  if (val>MAX_STOP_VEL)
    cw->stop_velocity=MAX_STOP_VEL;	/* stop velocity in units for DAC */
  else
  {
    if (val<MIN_STOP_VEL)
      cw->stop_velocity=MIN_STOP_VEL;
    else
      cw->stop_velocity=val;		/* stop velocity in units for DAC */
  }

  stop_guess = 
    (((cw->velocity-cw->stop_velocity)/cw->decel)+1)/(float)cw->updates_per_sec*
               ((cw->vel_rpm/2.)/60.)*
               INCHES_PER_RPM*POS_PER_INCH;
  if (stop_guess>cw->start_decel_position) 
  {
    printf ("\r\nstart_decel_position increased to estimate 0x%x from 0x%x",
              stop_guess, cw->start_decel_position);
    cw->start_decel_position = stop_guess;
   }

}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_DIO316_interrupt
**
** DESCRIPTION:
**      Handles interrupt for hight/low limits of all four counter_weights.
**	Provides a software abort of motion in appropriate direction.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**	DIO316ReadISR
**	DAC128V_Read_Reg
**	DIO316_Read_Port
**	cw_rdselect
**	cw_abort
**	DIO316ClearISR
**
** GLOBALS REFERENCED:
**	CW_limit_abort
**
**=========================================================================
*/
void cw_DIO316_interrupt(int type)
{
  unsigned char limit;
  unsigned char int_bit[4];
  short vel;
  int cw;

  DIO316ReadISR (cw_DIO316,&int_bit[0]);
  DAC128V_Read_Reg(cw_DAC128V,CW_MOTOR,&vel);
  vel-=0x800;
  DIO316_Read_Port (cw_DIO316,CW_LIMIT_STATUS,&limit);
  cw=cw_rdselect();
/* Low Limit Set and going Negative - ABORT */
  if ((vel<0)&&(((limit>>((cw*2)+1))&0x1)==0))
  {
    cw_abort();
    CW_limit_abort=TRUE;
  }
/* Upper Limit Set and going Positive - ABORT */
  if ((vel>0)&&(((limit>>(cw*2))&0x1)==0))
  {
    cw_abort();
    CW_limit_abort=TRUE;
  }
  logMsg ("\r\nCW_INTERRUPT fired: cw=%d Limit %x vel=%d ISR %x abort=%d\r\n",
		cw,(unsigned long)limit,vel,(unsigned long)int_bit[0],
		CW_limit_abort,0);
  DIO316ClearISR (cw_DIO316);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_DIO316_shutdown
**
** DESCRIPTION:
**      Rundown of DIO316
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**	DIO316_Interrupt_Enable_Control
**
** GLOBALS REFERENCED:
**	cw_DIO316
**
**=========================================================================
*/
void cw_DIO316_shutdown(int type)
{

    printf("CW DIO316 shutdown: 4 interrupts for IP %d\r\n",cw_DIO316);
    if (cw_DIO316!=-1)
    {
      DIO316_Interrupt_Enable_Control (cw_DIO316,0,DIO316_INT_DIS);
      DIO316_Interrupt_Enable_Control (cw_DIO316,1,DIO316_INT_DIS);
      DIO316_Interrupt_Enable_Control (cw_DIO316,2,DIO316_INT_DIS);
      DIO316_Interrupt_Enable_Control (cw_DIO316,3,DIO316_INT_DIS);
    }
    taskDelay(30);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: read_ADC
**	    read_all_ADC
**
** DESCRIPTION:
**      Diagnositc functions to read ADCs in raw counts and volts a 
**	specified number of times in succession.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**	ADC128F1_Read_Reg
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void read_ADC (int chan, int cnt)
{
  int i;
  short adc;

  if (cnt==0) cnt=1;
  for (i=0;i<cnt;i++)
  {
      ADC128F1_Read_Reg(cw_ADC128F1,chan,&adc);
      if ((adc&0x800)==0x800) adc |= 0xF000;
      else adc &= 0xFFF;
      printf ("\r\nADC %d: %x %f volts",chan,adc,adc/2048.);
  }
}
void read_all_ADC (int cnt)
{
  int i,ii;
  short adc;

  if (cnt==0) cnt=1;
  printf ("\r\n");
  for (i=0;i<8;i++)
    printf (" ADC %d ",i);
  for (ii=0;ii<cnt;ii++)
  {
    printf ("\r\n");
    for (i=0;i<8;i++)
    {
      ADC128F1_Read_Reg(cw_ADC128F1,i,&adc);
      if ((adc&0x800)==0x800) adc |= 0xF000;
      else adc &= 0xFFF;
      printf (" %04x  ",adc);
    }
  }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_brake_on
**	    cw_brake_off
**
** DESCRIPTION:
**      Brake is applied: If positive direction, go below stop limit (i.e. 0)
**	but not less than moving limit.  If negative direction, go above stop
**	limit but not greater than moving limit.
**	Negative Moving Limit      0     Stop Limit      Positive Moving Limit
**	---------|-----------------|---------|---------------------|----------
**	          Positive Brake............. Negative Brake.......
**
** RETURN VALUES:
**      int 	always zero
**
** CALLS TO:
**      DAC128V_Read_Reg
**      DAC128V_Write_Reg
**
** GLOBALS REFERENCED:
**	cw_DAC128V
**
**=========================================================================
*/
int cw_brake_on()
{
	short vel;

	if (cw_DAC128V==-1) return ERROR;
        DAC128V_Read_Reg(cw_DAC128V,CW_MOTOR,&vel);
	if (vel>=0)
	  DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,0x800);
	else
	  DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,0x800+STOP_LIM);
	return 0;
}
/* not required, but specified for uniformity and was used at one time */
int cw_brake_off()
{
	return 0;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_power_on
**	    cw_power_off
**
** DESCRIPTION:
**	Enable/disable for motion.  Should be replaced with watchdog timer.
**
** RETURN VALUES:
**      int 	always zero
**
** CALLS TO:
**      DIO316_Read_Reg
**      DIO316_Write_Reg
**
** GLOBALS REFERENCED:
**	cw_DIO316
**
**=========================================================================
*/
int cw_power_on()
{
	unsigned char val;

	if (cw_DIO316==-1) return ERROR;
	DIO316_Read_Port (cw_DIO316,CW_POWER,&val);
	DIO316_Write_Port (cw_DIO316,CW_POWER,val|CW_POWER_ON);
        taskDelay (60);
	return 0;
}
int cw_power_off()
{
	unsigned char val;

	if (cw_DIO316==-1) return ERROR;
	DIO316_Read_Port (cw_DIO316,CW_POWER,&val);
	DIO316_Write_Port (cw_DIO316,CW_POWER,val&CW_POWER_OFF);
	return 0;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_select
**	    cw_rdselect
**
** DESCRIPTION:
**	Select which counter-weight to manipulate.
**	Read which counter-weight is currently selected.
**
** RETURN VALUES:
**      int 	always zero
**
** CALLS TO:
**      DIO316_Read_Port
**      DIO316_Write_Port
**
** GLOBALS REFERENCED:
**	cw_DIO316
**
**=========================================================================
*/
int cw_select(int cw)
{
	unsigned char val;

	if (cw_DIO316==-1) return ERROR;
	DIO316_Read_Port (cw_DIO316,CW_SELECT,&val);
	DIO316_Write_Port (cw_DIO316,CW_SELECT,(val&(~CW_SELECT))|cw);
	return 0;
}
int cw_rdselect()
{
	unsigned char val;

	if (cw_DIO316==-1) return ERROR;
	DIO316_Read_Port (cw_DIO316,CW_SELECT,&val);
	return (val&CW_SELECT);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_status
**
** DESCRIPTION:
**	Diagnostic to display current status of counter-weight system.
**
** RETURN VALUES:
**      int 	always zero
**
** CALLS TO:
**	DIO316_Read_Port
**	cw_read_position
**
** GLOBALS REFERENCED:
**	cw_DIO316
**
**=========================================================================
*/
int cw_status()
{
	unsigned char val;
	int i;

	if (cw_DIO316==-1) return ERROR;
	DIO316_Read_Port (cw_DIO316,CW_LCLRMT,&val);
	if ((val&CW_LOCAL)==CW_LOCAL) printf ("\r\nLOCAL:  ");
	else printf ("\r\nREMOTE:  ");

	DIO316_Read_Port (cw_DIO316,CW_SELECT,&val);
	printf ("CW %d Selected, ",val&0x3);

	DIO316_Read_Port (cw_DIO316,CW_INTERLOCK,&val);
	if ((val&CW_INTERLOCK_BAD)==CW_INTERLOCK_OK) 
	  printf ("Interlock OK and ignored\r\n");
	else 
	  printf ("Interlock BAD but ignored\r\n");

  	DIO316_Read_Port (cw_DIO316,CW_LIMIT_STATUS,&val);
	printf ("\r\nLimit Byte 0x%02x (1 implies OPEN or OK)",val);
	for (i=0;i<4;i++)
	  printf ("\r\n  CW%d:  Upper Limit %d Lower Limit %d",
			i,(val>>(i*2))&0x1,(val>>((i*2)+1))&0x1);
	cw_read_position (1);
	return 0;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_abort
**
** DESCRIPTION:
**	Abort the counter_weight motion by turning the brake on and disabling
**	the motion.
**
** RETURN VALUES:
**      int 	ERROR or zero
**
** CALLS TO:
**	cw_brake_on
**	cw_power_off
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
int cw_abort ()
{
    if (cw_brake_on()==0) 
      if (cw_power_off()==0) return 0;
    return ERROR;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_motor_raw	volts specified
**	    cw_motor		rpm specified
**
** DESCRIPTION:
**	Diagnostic to drive the counter-weight motor similar to the routine 
**	called to move the counter-weight.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**	cw_select
**	DAC128V_Write_Reg
**	cw_power_on
**	cw_brake_off
**
** GLOBALS REFERENCED:
**	cw_DAC128V
**
**=========================================================================
*/
#define MOTOR_RAMP	40
void cw_motor_raw (int cw, int vel)
{
  int i;

  printf ("\r\n  Type any character to abort......");
  cw_select(cw);
  DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,0x800);
  cw_power_on();
  cw_brake_off();
  for (i=0;i<abs(vel)/MOTOR_RAMP;i++)
  {
    DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,0x800+(i*MOTOR_RAMP));
    if (kbd_input()) cw_abort();
    taskDelay (1);
  }
  DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,0x800+vel);
  while (!kbd_input());
  cw_abort();
}
void cw_motor (int cw, double vel_rpm)
{
  int vel;

  vel = (vel_rpm*RPM_IN_VOLTS)*ONE_VLT;
  cw_motor_raw (cw, vel);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_list
**
** DESCRIPTION:
**	Diagnostic to list the corresponding instrument's counter-weight 
**	parameters.
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
void cw_list (int inst)
{
  int i;

  if ((inst>=0)&&(inst<(sizeof(cw_inst)/sizeof(struct CW_LOOP))))
  {
    printf ("\r\nINST %d (%s):",inst, inst_name[inst]);
    for (i=0;i<4;i++) printf ("\r\n CW %d POS SETTING=%d (%6.4f\" %4.2fv)",i,
	cw_inst[inst].pos_setting[i],
	(24*cw_inst[inst].pos_setting[i])/(2048*0.7802),
	(10*cw_inst[inst].pos_setting[i])/2048.);
    for (i=0;i<4;i++) printf ("\r\n CW %d POS CURRENT=%d (%6.4f\" %4.2fv)",i,
    	cw_inst[inst].pos_current[i],
    	(24*cw_inst[inst].pos_current[i])/(2048*0.7802),
    	(10*cw_inst[inst].pos_current[i])/2048.);
    for (i=0;i<4;i++) printf ("\r\n CW %d POS ERROR=%d (%6.4f\" %4.2fv)",i,
    	cw_inst[inst].pos_error[i],
    	(24*cw_inst[inst].pos_error[i])/(2048*0.7802),
    	(10*cw_inst[inst].pos_error[i])/2048.);
    printf ("\r\n ACCEL RPM=%f (%d)",cw_inst[inst].accel_rpm,cw_inst[inst].accel);
    printf ("\r\n VEL RPM=%f (%d)",cw_inst[inst].vel_rpm,cw_inst[inst].velocity);
    printf ("\r\n DECEL RPM=%f (%d)",cw_inst[inst].decel_rpm,cw_inst[inst].decel);
    printf ("\r\n STOP VEL RPM=%f (%d)",cw_inst[inst].stop_vel_rpm,cw_inst[inst].stop_velocity);
    printf ("\r\n Updates per Sec=%d",cw_inst[inst].updates_per_sec);
    printf ("\r\n Start Decel Position=%d",cw_inst[inst].start_decel_position);
    printf ("\r\n Stop Pos Error=%d",cw_inst[inst].stop_pos_error);
    printf ("\r\n Stop Count=%d",cw_inst[inst].stop_count);
    printf ("\r\n");
  }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_read_position
**
** DESCRIPTION:
**	Diagnostic to read the positions a specified number of consecutive
**	times.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**      ADC128F1_Read_Reg
**
** GLOBALS REFERENCED:
**	cw_ADC128F1
**
**=========================================================================
*/
void cw_read_position (int cnt)
{
  int i, ii;
  short adc;

  if (cnt==0) cnt=1;
  printf ("\r\n");
  for (ii=0;ii<CW_MAX;ii++)
    printf ("       CW %d       ",ii);
  for (i=0;i<cnt;i++)
  {
    printf ("\r\n");
    for (ii=0;ii<CW_MAX;ii++)
    {
      ADC128F1_Read_Reg(cw_ADC128F1,ii,&adc);
      if ((adc&0x800)==0x800) adc |= 0xF000;
      else adc &= 0xFFF;
      printf (" %6.4f\"  %4.2fv  ",(24*adc)/(2048*0.7802),(10*adc)/2048.);
    }
  }
  printf ("\r\n");
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_set_position	inches
**	    cw_set_positionv	volts
**
** DESCRIPTION:
**	Sets the the four desired positions for a selected instrument
**	The inches is deprecated, but not deleted.
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
void cw_set_position (int inst, double p1, double p2, double p3, double p4)
{
  if ((inst>=0)&&(inst<(sizeof(cw_inst)/sizeof(struct CW_LOOP))))
  {
     if ((p1<0.0)||(p1>24.)) return;
     if ((p2<0.0)||(p2>24.)) return;
     if ((p3<0.0)||(p3>24.)) return;
     if ((p4<0.0)||(p4>24.)) return;
     printf ("\r\nINST %d: p1=%f, p2=%f, p3=%f, p4=%f",inst,p1,p2,p3,p4);
     cw_inst[inst].pos_setting[0]=(short)((p1/24.)*(2048*0.7802));
     cw_inst[inst].pos_current[0]=0;
     cw_inst[inst].pos_error[0]=0;
     cw_inst[inst].pos_setting[1]=(short)((p2/24.)*(2048*0.7802));
     cw_inst[inst].pos_current[1]=0;
     cw_inst[inst].pos_error[1]=0;
     cw_inst[inst].pos_setting[2]=(short)((p3/24.)*(2048*0.7802));
     cw_inst[inst].pos_current[2]=0;
     cw_inst[inst].pos_error[2]=0;
     cw_inst[inst].pos_setting[3]=(short)((p4/24.)*(2048*0.7802));
     cw_inst[inst].pos_current[3]=0;
     cw_inst[inst].pos_error[3]=0;
  }
}
void cw_set_positionv (int inst, short p1, short p2, short p3, short p4)
{
  if ((inst>=0)&&(inst<(sizeof(cw_inst)/sizeof(struct CW_LOOP))))
  {
     if ((p1<0)||(p1>1000)) return;
     if ((p2<0)||(p2>1000)) return;
     if ((p3<0)||(p3>1000)) return;
     if ((p4<0)||(p4>1000)) return;
     printf ("\r\nINST %d: p1=%d, p2=%d, p3=%d, p4=%d",inst,p1,p2,p3,p4);
     cw_inst[inst].pos_setting[0]=(short)(p1*2.048);
     cw_inst[inst].pos_current[0]=0;
     cw_inst[inst].pos_error[0]=0;
     cw_inst[inst].pos_setting[1]=(short)(p2*2.048);
     cw_inst[inst].pos_current[1]=0;
     cw_inst[inst].pos_error[1]=0;
     cw_inst[inst].pos_setting[2]=(short)(p3*2.048);
     cw_inst[inst].pos_current[2]=0;
     cw_inst[inst].pos_error[2]=0;
     cw_inst[inst].pos_setting[3]=(short)(p4*2.048);
     cw_inst[inst].pos_current[3]=0;
     cw_inst[inst].pos_error[3]=0;
  }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_set_params
**
** DESCRIPTION:
**	Set the instruments acceleration, deceleration, velocity, and stop 
**	velocity values.  Always recalculates the instrument for these changes.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**	cw_calc
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void cw_set_params (int inst, double accel, double vel, double decel, double stop_vel)
{
  if ((inst>=0)&&(inst<(sizeof(cw_inst)/sizeof(struct CW_LOOP))))
  {
    cw_inst[inst].accel_rpm=(float)accel;	/* user specified acceleration */
    cw_inst[inst].vel_rpm=(float)vel;		/* user specified velocity */
    cw_inst[inst].decel_rpm=(float)decel;	/* user specified deceleration */
    cw_inst[inst].stop_vel_rpm=(float)stop_vel;	/* user specified stop velocity */
    cw_calc (&cw_inst[inst]);
  }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_set_const
**
** DESCRIPTION:
**	Set the instrument's update rate, deceleration position, stop position
**	error acceptable, and retry count.
**	Always recalculates the instrument for these changes.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	cw_calc
**
**=========================================================================
*/
void cw_set_const (int inst, int upd, int start_decel_pos, int stop_pos_err, int stop_cnt)
{
  if ((inst>=0)&&(inst<(sizeof(cw_inst)/sizeof(struct CW_LOOP))))
  {
    cw_inst[inst].updates_per_sec=upd;
    cw_inst[inst].start_decel_position=start_decel_pos;/* start position to begin deceleration */
    cw_inst[inst].stop_pos_error=stop_pos_err;	/* stop position error allowed */
    cw_inst[inst].stop_count=stop_cnt;		/* stop polarity swing counts allowed */
    cw_calc (&cw_inst[inst]);
  }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: CW_help
**
** DESCRIPTION:
**	Help facility for counter-weight.
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
char *help_CW[]={
        "CW_help",
	"CW_Verbose, CW_Quiet",
	"char *balance_cmd(char *cmd); cmd=CAMERA,FIBER,INST2...INST15",
	"int balance_init()",
	"int balance(int cw, int inst); cw=0-3; inst=0-15",
	"int cw_position(int cw, double pos)",
	"void cw_calc(struct CW_LOOP *cw)",
	"int read_ADC(int chan, int cnt)",
	"int read_all_ADC (int cnt)",
	"int cw_brake_on(), int cw_brake_off()",
	"int cw_power_on(), int cw_power_off()",
	"int cw_select(int cw)",
	"int cw_status()",
	"int cw_abort()",
	"int cw_motor_raw(int cw, int vel)",
	"int cw_motor(int cw, double vel_rpm)",
	"void cw_list(int inst), int cw_get_inst(char *cmd)",
	"void cw_read_position(int cnt)",
	"void cw_set_position(int inst, double p1, double p2, double p3, double p4)",
	"void cw_set_params(int inst,double accel,double vel,double decel,double stop_vel)",
	"void cw_set_const(int inst,int upd,int start_decel_pos,int stop_pos_err, int stop_cnt)",
""
};                                                         
void CW_help()
{
  int i;

  for (i=0;i<sizeof(help_CW)/sizeof(char *);i++)
    printf ("%s\r\n",help_CW[i]);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: CW_Verbose
**	    CW_Quiet
**
** DESCRIPTION:
**	Turns off/on diagnostic verbosity.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	CW_verbose
**
**=========================================================================
*/
void CW_Verbose()
{
	CW_verbose=TRUE;
}
void CW_Quiet()
{
	CW_verbose=FALSE;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: kbd_input
**
** DESCRIPTION:
**	An ugly routine to check for any input to stop cw_motor motion.
**
** RETURN VALUES:
**      TRUE or FALSE depending on if any key is hit.
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
int kbd_input()
{
  int bytes;

  ioctl (ioTaskStdGet(0,0),FIONREAD,(int)&bytes);
  if (bytes!=0) return TRUE;
  return FALSE;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_get_inst
**
** DESCRIPTION:
**	Searches the instrument names for a match and returns index.
**
** RETURN VALUES:
**      int 	instrument index or ERROR
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	inst_name
**
**=========================================================================
*/
int cw_get_inst(char *cmd)
{
  int inst;

  for (inst=0;inst<sizeof(inst_name)/sizeof(char *);inst++)
    if (!strncmp(cmd,inst_name[inst],strlen(inst_name[inst])))
    {
      return inst;
    }
  return ERROR;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: set_DAC
**
** DESCRIPTION:
**	Diagnostic to positively ramp up and down the DAC value for any channel.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**      DAC128V_Read_Port
**
** GLOBALS REFERENCED:
**	cw_DAC128V
**
**=========================================================================
*/
void set_DAC (int chan)
{
  if (chan==CW_MOTOR) return;
  if ((chan<0)||(chan>7)) return;
  for (;;)
  {
    taskDelay (60);
    DAC128V_Write_Reg(cw_DAC128V,chan,0x800+(tickGet()%0x800));
  }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_data_collection
**
** DESCRIPTION:
**	Read positions and limit status for sdssdc structure.
**	parameters.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**      ADC128F1_Read_Reg(cw_ADC128F1,ii,&adc);
**      DIO316_Read_Port
**	cw_read_position
**
** GLOBALS REFERENCED:
**	cw_ADC128F1
**	cw_DIO316
**	sdssdc
**
**=========================================================================
*/
void cw_data_collection()
{
  extern struct SDSS_FRAME sdssdc;
  short adc;
  int ii;

  if (cw_ADC128F1!=-1)
  {
    for (ii=0;ii<CW_MAX;ii++)
    {
      ADC128F1_Read_Reg(cw_ADC128F1,ii,&adc);
      if ((adc&0x800)==0x800) sdssdc.weight[ii].pos=adc|0xF000;
      else sdssdc.weight[ii].pos = adc&0xFFF;
    }
  }
  if (cw_DIO316!=-1)
    DIO316_Read_Port (cw_DIO316,CW_LIMIT_STATUS,&cwLimit);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_power_disengage
**	    cw_power_engage
**
** DESCRIPTION:
**	Enable/disable motion based on the watchdog support.
**
** RETURN VALUES:
**      int 	always zero
**
** CALLS TO:
**	StopCounter
**	WriteCounterConstant
**	StartCounter
**
** GLOBALS REFERENCED:
**	sbrd
**
**=========================================================================
*/
int cw_power_disengage()
{
  extern struct conf_blk sbrd;

  StopCounter (&sbrd,CW_WD);
  return 0;
}
int cw_power_engage()
{
  extern struct conf_blk sbrd;

  WriteCounterConstant (&sbrd,CW_WD);         /* 2 Sec */
  StartCounter (&sbrd,CW_WD);
  return 0;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_setup_wd
**
** DESCRIPTION:
**	Setup ACROMAG IP for watchdog support.
**
** RETURN VALUES:
**      int 	always zero
**
** CALLS TO:
**	SetCounterSize
**	SetCounterConstant
**	SetMode
**	SetDebounce
**	SetInterruptEnable
**	SetClockSource
**	SetTriggerSource
**	SetWatchdogLoad
**	SetOutputPolarity
**	ConfigureCounterTimer
**
** GLOBALS REFERENCED:
**	sbrd
**
**=========================================================================
*/
int cw_setup_wd ()
{
  extern struct conf_blk sbrd;

  SetCounterSize (&sbrd,CW_WD,CtrSize32);
  SetCounterConstant (&sbrd,CW_WD,2000000);             /* 2 Sec */
  SetMode (&sbrd,CW_WD,Watchdog);
  SetDebounce (&sbrd,CW_WD,DebounceOff);
  SetInterruptEnable(&sbrd,CW_WD,IntEnable);
  SetClockSource (&sbrd,CW_WD,InC1Mhz);
  SetTriggerSource (&sbrd,CW_WD,InTrig);
  SetWatchdogLoad (&sbrd,CW_WD,WDIntLd);
  SetOutputPolarity (&sbrd,CW_WD,OutPolHi);
  ConfigureCounterTimer(&sbrd,CW_WD);
  return 0;
}
