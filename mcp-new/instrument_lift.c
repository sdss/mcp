#include "copyright.h"
/**************************************************************************
***************************************************************************
** FILE:
**      instrument_lift.c
**
** ABSTRACT:
**	Routines to provide motion and rules for instrument lift via
**	FSM.
**
** ENTRY POINT          SCOPE   DESCRIPTION
** ----------------------------------------------------------------------
** 
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
/* 		Instrument Lift Control					*/
/*   File:	instrument_lift.c							*/
/************************************************************************/
/*   Location:	Fermi National Accelerator Lab				*/
/*   Author:	Charlie Briegel, X4510, MS 360, ALMOND::[BRIEGEL]	*/
/*   Program:	instrument_lift : VxWorks				*/
/*   Modules:	 : 	    					*/	
/*++ Version:
  1.00 - initial --*/
/*++ Description:
Requirements:
	4 DO bits positive direction motion
	4 DO bits negative direction motion
	1 DO bit to enable/disable motion
	1 DO bit for high force
	1 DO bit for pump on/off
	1 DO bit for solenoid engage
	2 DI bits for cart position
	4 DI bit proximity switches for instrument id
	1 DI bit proximity switches for lift on floor
	2 DI bit tank low or high
DIO316 3*16
  port	0 bi-directional
	    input
		IL_FIBER_INSERT
			bit 0	pin 8
		IL_FIBER_REMOVE
			bit 1	pin 7
		IL_LCLRMT
			bit 2	pin 6
		IL_STRAIN_HI
			bit 3	pin 5
		IL_STRAIN_LO
			bit 4	pin 4
  port	1 bi-directional
	    input
	    	IL_PLATE_ENGAGE
	    		bit 0	pin 16
	    	IL_CLAMP_FORCE
	    		bit 1	pin 15
		IL_INST_ID
			bit 2-3	pin 14-13
		IL_FLOOR
			bit 4	pin 12
		IL_PUMP_STATUS
			bit 7   pin 9
	2 output
		IL_DOWN_VEL
			bit 0-3 pin 24-21
		IL_UP_VEL
			bit 4-7 pin 20-17
	3 output
		CW_SELECT
			bit 0-1 pin 32-31
		CW_DIRECTION		
			bit 2   pin 30
		IL_PUMP
			bit 4   pin 29
		IL_ENABLE
			bit 5   pin 27
		IL_HIFORCE
			bit 6   pin 26
		IL_SOLENOID
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
ADC128F1
	1 ADC to readback the IL position
	1 ADC to readback the force
		IL_POSITION	4
		IL_STRAIN_GAGE	5
--*/
/*++ Notes:
--*/
/************************************************************************/

/*------------------------------*/
/*	includes		*/
/*------------------------------*/
#include "vxWorks.h"                            
#include "stdio.h"
#include "sysLib.h"
#include "taskLib.h"
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
#include "data_collection.h"
#include "abdh.h"

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

#define CW_SELECT		3
#define CW_SELECT_0	0x0
#define CW_SELECT_1	0x1
#define CW_SELECT_2	0x2
#define CW_SELECT_3	0x3

#define CW_DIRECTION		3
#define CW_POS_DIRECTION	0x4
#define CW_NEG_DIRECTION	0

#define STOP_LIM	10

#define CW_POWER		3
#define CW_POWER_ON	0x10
#define CW_POWER_OFF	0xEF

#define CW_LIMIT_INT		4
#define CW_LIMIT	0x1

#define	CW_INTERLOCK		4
#define CW_INTERLOCK_OK 0x10
#define CW_INTERLOCK_BAD 0xEF
#define	CW_LCLRMT		4
#define CW_LOCAL	0x20
#define CW_REMOTE	0xDF

#define	CW_LIMIT_STATUS		5
#define	CW_LOWER_LIMIT_0	0x01
#define	CW_UPPER_LIMIT_0	0x02
#define	CW_LOWER_LIMIT_1	0x04
#define	CW_UPPER_LIMIT_1	0x08
#define	CW_LOWER_LIMIT_2	0x10
#define	CW_UPPER_LIMIT_2	0x20
#define	CW_LOWER_LIMIT_3	0x40
#define	CW_UPPER_LIMIT_3	0x80
/*...............................C1 BURNDY # pin.DIO316 pin....*/
/* DI - status */
#ifdef MISKA
#define	IL_CART		0
#define	IL_FIBER_INSERT	0x1		/* 17T		8 */
#define	IL_FIBER_REMOVE	0x2		/* 16S		7 */
#define IL_FIBER_MASK	0x3

#define	IL_LCLRMT	0
#define	IL_LOCAL	0xFB		/* 22Y		6 */
#define	IL_REMOTE	0x04

#define IL_STRAIN	0
#define	IL_STRAIN_HI	0x8		/* 23Z		5 */
#define	IL_STRAIN_LO	0x10		/* 24a		4 */

#define IL_PLATE	1
#define IL_PLATE_ENGAGE	0x1		/* 11L		16 */

#define IL_FORCE	1
#define IL_CLAMP_FORCE	0x2		/* 12M		15 */

#define IL_INST_ID	1
#define IL_FIBER_INST	0x4		/* 14P		13 */
#define IL_DUMMY_INST	0x8		/* 13N		14 */
#define IL_EMPTY_INST	0xF3
#define IL_MASK_INST	0xC

#define IL_FLOOR	1
#define IL_OFF_FLOOR	0x10		/* 15R		12 */
#define IL_ON_FLOOR	0xEF

#define IL_PUMPST	1
#define IL_PUMPST_OFF	0x80		/* 10K		9 */
#define IL_PUMPST_ON	0x7F
/* DO - control */
#define IL_VELOCITY	2
#define IL_DOWN_VEL	0x0F		/* 1A,2B,3C,4D	21,22,23,24 (Speed 4-1) */
#define IL_UP_VEL	0xF0		/* 6F,7G,8H,9J	20,19,18,17 (Speed 1-4) */
#define UP_DIRECTION	TRUE
#define DOWN_DIRECTION	FALSE
#define DEFAULT		16

#define IL_PUMP		3
#define IL_PUMP_ON	0x8		/* 18U		29 */
#define IL_PUMP_OFF	0xF7

#define IL_ENABLE	3
#define IL_ENABLED	0x20		/* 5E		27 */
#define IL_DISABLED	0xDF

#define IL_HIFORCE	3
#define IL_HIFORCE_ON	0x40		/* 19V		26 */
#define IL_HIFORCE_OFF	0xBF

#define IL_SOLENOID	3
#define IL_SOLENOID_ON	0x80		/* 20W		25 */
#define IL_SOLENOID_OFF	0x7F
#define IL_WD		2		/* WD channel    11 */
#define TM_WD		4		/* WD channel    20 */
#define CW_WD		6		/* WD channel    29 */

#define	IL_POSITION	4		/*	      9,10 */
#define	IL_STRAIN_GAGE	5		/*	     11,12 */
#define IL_FAKE_POS	5
#define IL_FAKE_STRAIN	6
#else			/* CZARAPATA */
#define IL_ENABLE	3
#define IL_ENABLED	0x20		/* 5E		27 */
#define IL_DISABLED	0xDF
#define IL_WD		2		/* WD channel    11 */
#define TM_WD           4               /* WD channel    20 */
#define CW_WD           6               /* WD channel    29 */

#define	IL_POSITION	4		/*	      9,10 */
#define	IL_STRAIN_GAGE	5		/*	     11,12 */
#define IL_FAKE_POS	5
#define IL_FAKE_STRAIN	6

#define UP_DIRECTION	TRUE
#define DOWN_DIRECTION	FALSE
#define DEFAULT		0
#endif

struct IL_LOOP {
	short pos_setting;	/* position for a given instrucment */
	short pos_current;	/* current position */
	short pos_error;	/* error for the given instrument */
	short updates_per_sec;	/* update rate of the loop, zero->once
				   driven by fsm */
	short base_strain;	/* calculated at INIT state */
	short start_strain;	/* */
	short slew_strain;	/* offset from INIT state which is base_strain */
	short stop_strain;	/* up implies added to start_strain */
	short final_strain;	/* dn implies subtracted from start_strain */
	char start_velocity;		/* start velocity in units for DIO */
	char velocity;			/* velocity in units for DIO */
	char stop_velocity;		/* stopping velocity in units for DIO */
	char final_velocity;		/* final velocity in units for DIO */
	short start_accel_position;	/* start position to begin acceleration */
	short start_decel_position;	/* start position to begin deceleration */
	short stop_pos_error;		/* stop position error allowed */
	short stop_count;		/* stop polarity swing counts allowed */
};

/*-------------------------------------------------------------------------
**
** GLOBAL VARIABLES
*/
struct IL_LOOP	il_inst[16][2] = {
{{0x0400,0,0,20, 0x400,5,5,5,2, 0x7,0xF,0x3,0x1, 0,40, 2,4},	/* UP INST 0*/
 {0x0100,0,0,20, 0x400,40,10,10,5, 0x2,0xF,0x3,0x2, 40,0, 2,4}},/* DOWN */
{{0x0400,0,0,0, 0x400,40,40,100,140, 0xF,0x8F,0x2,0x1, 0,40, 2,4},	/* UP INST 1*/
 {0x0100,0,0,0, 0x400,140,40,20,20,  0x4,0x8F,0x2,0x2, 40,0, 2,4}},	/* DOWN */
{{0x0400,0,0,20, 110,40,20,20,40, 0x7,0xF,0x3,0x1, 0,40, 2,4},	/* UP INST 0*/
 {0x0000,0,0,20, 110,50,30,20,20, 0x7,0xF,0x3,0x3, 40,0, 2,4}},/* DOWN */
{{0x0400,0,0,20, 110,40,20,20,40, 0x7,0x8F,0x3,0x1, 0,40, 2,4},	/* UP INST 1*/
 {0x0000,0,0,20, 110,50,20,20,10, 0x7,0x8F,0x3,0x3, 40,0, 2,4}},	/* DOWN */
};
/*
	UP	state	DN	state
	_____________   _____________
	^	final	|	start
	|		|--
	|--		|
	|	slow	|	slew
	|--		|
	|		|
	|	slew	|
	|		|
	|		|
	|		|
	|		|
	|		|
	|		|
	|--		|--
	|	start	|	slow
	|		^
	_____________	_____________
*/
#define	IL_UP	0
#define	IL_DN	1
char *motion_type[]={"UP","DOWN"};
#define HI_FORCE_MASK 0x80
int IL_verbose=FALSE;
/*                     0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 */
/* ticks between checks */
short moving_rate[16]={120,120,120,60,40,40,40,40,30,30,30,30,30,30,30,30};
/* position change per check */
short moving_pos[16]= {-2, 1, 1, 4, 6, 6, 6, 6, 8, 8, 8, 8, 8, 8, 8, 8};
#define ILLEGAL_STATE		-1
#define SUCCESS			0
#define	INIT			1
#define START_SLEW		2
#define	SLEW			3
#define	SLOW_SLEW		4
#define	FINAL_SLEW		5
#define DELAYED			6
#define CONTINUE		7
#define EXIT			8

#define ABORTS			EXIT+1
#define	ABORT_HI_STRAIN		ABORTS+1
#define	ABORT_LO_STRAIN		ABORTS+2
#define	ABORT_NOT_MOVING	ABORTS+3
#define	ABORT_UP		ABORTS+4
#define ABORT_NO_STRAIN		ABORTS+5

static int state,new_state,last_state,delayed_state,last_error,last_tick;
static short last_direction;
static int cnt;

#define ABORT_FIBER		-1
#define FIBER_INIT		1
#define ENGAGE_PLATE		2
#define PLATE_EMPTY		3
#define	ENGAGE_FIBER		4
#define GET_FIBER		5
#define PLATE_FULL		6
#define ENGAGE_TELESCOPE	7
#define PUT_FIBER		8
#define RETURN			9
#define FIBER_EXIT		10

#define PLATE_ENGAGE	0x300
#define FIBER_ENGAGE	0x120
static fiber_state,last_fiber_state,new_fiber_state;
struct conf_blk sbrd;
int il_DIO316=-1;
int il_ADC128F1=-1;
int il_DAC128V=-1;
static short il_pos;
static short il_action;
static int il_state;
static int il_instrument;
struct IL_RULES {
	int (*rule)();
	short *var[2];
	unsigned short true_state;
	unsigned short false_state;
};
struct IL_STATES {
	char *name;
	int (*init)();
	int (*action)();
	struct IL_RULES rules[4];
};
struct IL_HISTORY {
	unsigned char state;
	unsigned char rule_fired;
	unsigned short cnt;
	time_t time;
};
struct IL_INST {
	unsigned short current_state;
	struct IL_STATES *fsm;
	int history_enabled;
	unsigned short history_idx;
#define HISTORY_IDX_MAX	200
	struct IL_HISTORY *history;
};

/* Prototypes */
int lift_initialize(unsigned char *addr);
char *liftup_cmd(char *cmd);
char *liftdown_cmd(char *cmd);
int lift_initialize(unsigned char *addr);
int fiberGet();
int fiberPut();
#define IL_GET		0
#define IL_PUT		1
int fiber_fsm (int inst, int action);
int fiber_put(struct IL_STATES *sm);
int fiber_msg(struct IL_STATES *sm);
int is_at_zenith();
int is_cart_get_position();
int is_cart_put_position();
int is_pump_on();
int is_plate_engage();
int is_plate_empty();
int is_plate_full();
int is_fiber_inst();
int is_dummy_inst();
int is_clamping_force();
int liftUp (int inst);
int liftDn (int inst);
int lift_fsm (int inst, int motion,int new_state);
void il_trace (int inst, int cnt);
void il_position(double pos);
void il_calc (struct IL_LOOP *il);
int il_enable_motion();
int il_enable_motion_status();
int il_disable_motion();
int il_hiforce_on();
int il_hiforce_off();
int il_force_on();
int il_force_off();
int il_solenoid_engage();
int il_solenoid_disengage();
int setup_wd (char *addr, char vec, int irq);
int il_setup_wd ();
void wd_isr(struct conf_blk *cblk);
int il_pump_on();
int il_pump_off();
void il_status();
void il_abort ();
int il_motion_up (char vel);
int il_motion_raw_up (char vel);
int il_motion_dn (char vel);
int il_motion_raw_dn (char vel);
int il_motor (double vel_rpm);
void il_list (int inst);
void il_read_position (int cnt);
void il_read_strain_gage (int cnt);
void il_set_upposition (int inst, double pos);
void il_set_dnposition (int inst, double pos);
void il_set_upparams (int inst, char start_vel, char vel,
		char stop_vel, char final_vel);
void il_set_dnparams (int inst, char start_vel, char vel,
		char stop_vel, char final_vel);
void il_set_upconst (int inst, int upd, 
	 int start_accel_pos, int start_decel_pos,
	int stop_pos_err, int stop_cnt);
void il_set_dnconst (int inst, int upd, 
	 int start_accel_pos, int start_decel_pos,
	int stop_pos_err, int stop_cnt);
void il_set_upstrain (short inst, short base_strain, 
	short start_strain, short slew_strain, 
	short stop_strain, short final_strain);
void il_set_dnstrain (short inst, short base_strain, 
	short start_strain, short slew_strain, 
	short stop_strain, short final_strain);
int il_umbilical_move_pos(int pos);
void IL_help();
void IL_Verbose();
void IL_Quiet();

/*=========================================================================
**=========================================================================
**
** ROUTINE: liftup_cmd
**	    liftdown_cmd
**
** DESCRIPTION:
**      Send lift up or down for the specified instrument.  These are motion
**	commands utilizing the il_inst structure and corresponding FSM
**
** RETURN VALUES:
**     null string
**
** CALLS TO:
**	cw_get_inst
**	lift_fsm
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
char *liftup_cmd(char *cmd)
{
  int inst;

  printf ("\r\nLIFT command fired\r\n");
  if ((inst=cw_get_inst(cmd))!=-1)
      lift_fsm (inst,IL_UP,INIT);
  return "";
}
char *liftdown_cmd(char *cmd)
{
  int inst;

  printf ("\r\nLIFT command fired\r\n");
  if ((inst=cw_get_inst(cmd))!=-1)
      lift_fsm (inst,IL_DN,INIT);
  return "";
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
  short val;
  struct IPACK *ip;
  extern int cw_DIO316;
  extern int cw_ADC128F1;
  extern int cw_DAC128V;

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
      setup_wd(ip->adr[i], 0xB4,3);
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
  return 0;
}

int fiberGet ()
{
  printf ("\r\nFIBER GET:");
  return (fiber_fsm(cw_get_inst("FIBER"),IL_GET));
}
int fiberPut ()
{
  printf ("\r\nFIBER PUT:");
  return (fiber_fsm(cw_get_inst("FIBER"),IL_PUT));
}
int fiber_fsm (int inst, int action)
{
  short pos;
  int lift_state,motion;
  
  new_fiber_state=FIBER_INIT;
  lift_state=INIT;
  if ((il_inst[inst][IL_UP].updates_per_sec!=0)||
  	(il_inst[inst][IL_DN].updates_per_sec!=0)) return ERROR;
  while (new_fiber_state!=FIBER_EXIT)
  {
    ADC128F1_Read_Reg(il_ADC128F1,IL_POSITION,&pos);
    if ((pos&0x800)==0x800) pos |= 0xF000;
    else pos &= 0xFFF;
    
    last_fiber_state=fiber_state;
    fiber_state=new_fiber_state;    
    switch (fiber_state)
    {
      case FIBER_INIT:
	if (last_fiber_state!=FIBER_INIT)
	  if (IL_verbose) printf ("\r\n CART INIT: ");
        last_fiber_state=FIBER_INIT;
        if (!is_at_zenith()) 
           printf ("\r\nTelescope is NOT at zenith and locked");
        while (!is_at_zenith()) 
          taskDelay (1*sysClkRateGet());
        printf ("\r\nTelescope is at ZENITH and LOCKED");
        if (action==IL_GET)
        {
          if (!is_cart_get_position()) 
           printf ("\r\nTelescope is NOT at retrieval position and locked");
          while (!is_cart_get_position()) 
            taskDelay (1*sysClkRateGet());
          printf ("\r\nCART is at CORRECT RETRIEVAL POSITION and LOCKED");
	}
        if (action==IL_PUT)
        {
          if (!is_cart_put_position()) 
           printf ("\r\nTelescope is NOT at insertion position and locked");
          while (!is_cart_put_position()) 
            taskDelay (1*sysClkRateGet());
          printf ("\r\nCART is at CORRECT INSERTION POSITION and LOCKED");
	}
        il_pump_on();
        if (!is_pump_on()) 
           printf ("\r\nPump is NOT on");
        while (!is_pump_on()) 
          taskDelay (1*sysClkRateGet());
        printf ("\r\nPump is ON");

  	motion=IL_UP;
  	lift_state=INIT;
        new_fiber_state=ENGAGE_PLATE;
	
        break;

      case ENGAGE_PLATE:
	if (last_fiber_state!=ENGAGE_PLATE)
	  if (IL_verbose) printf ("\r\n ENGAGE PLATE: ");
        lift_state=lift_fsm(inst,motion,lift_state);
        if (is_plate_engage())
        {
          if(action==IL_GET) new_fiber_state=PLATE_EMPTY;
	  else new_fiber_state=PLATE_FULL;
	  break;
	}
	if (pos<PLATE_ENGAGE) new_fiber_state = ABORT_FIBER;
        break;
        
      case PLATE_EMPTY:
	if (last_fiber_state!=PLATE_EMPTY)
	{
	  printf ("\r\nPlate is ENGAGED");
          if (IL_verbose) printf ("\r\n PLATE EMPTY: ");
        }
        lift_state=lift_fsm(inst,motion,lift_state);
        if (is_plate_empty()) new_fiber_state=ENGAGE_FIBER;
        break;
        
      case ENGAGE_FIBER:
	if (last_fiber_state!=ENGAGE_FIBER)
          if (IL_verbose) printf ("\r\n ENAGE FIBER: ");
        lift_state=lift_fsm(inst,motion,lift_state);
        if ((is_plate_full())&&(pos<(FIBER_ENGAGE+20)))
          new_fiber_state=GET_FIBER;
	if (pos<FIBER_ENGAGE) new_fiber_state = ABORT_FIBER;
        break;
        
      case GET_FIBER:
	if (last_fiber_state!=GET_FIBER)
	  if (IL_verbose) printf ("\r\n GET FIBER ");
        lift_state=lift_fsm(inst,motion,lift_state);
        if (is_clamping_force())
        { 
          lift_state=INIT;
          motion=IL_DN;
          new_fiber_state=RETURN;
        }
        break;
        
      case PLATE_FULL:
        break;
        
      case ENGAGE_TELESCOPE:
        break;
        
      case PUT_FIBER:
        break;
        
      case RETURN:
	if (last_fiber_state!=RETURN)
	  if (IL_verbose) printf ("\r\n RETURN ");
        if ((lift_state=lift_fsm(inst,motion,lift_state))>=ABORTS) 
          new_fiber_state=FIBER_EXIT;
        if (lift_state==EXIT) new_fiber_state=FIBER_EXIT;
        break;
        
      case FIBER_EXIT:
	if (IL_verbose) printf ("\r\n FIBER EXIT: ");
	il_abort();
        break;

      case ABORT_FIBER:
	if (IL_verbose) printf ("\r\n ABORT_FIBER:  ");
        lift_state=lift_fsm(inst,motion,EXIT);
	printf ("ABORT due to no plate in lift");
        return ABORT_FIBER;

      default:
	return ILLEGAL_STATE;
    }
  }
  if (IL_verbose) printf ("\r\n FIBER_EXIT: ");
  return SUCCESS;
}
int fiber_put(struct IL_STATES *sm)
{
  il_action=IL_DN;
  fiber_msg (sm);
  return 0;
}
int fiber_msg(struct IL_STATES *sm)
{
  if (IL_verbose) printf ("\r\nFSM: %s",sm->name);
  return 0;
}
int is_action_get()
{
	if (il_action==IL_GET) return TRUE;
	else return FALSE;
}
int fiberini(struct IL_STATES *sm)
{
        if (!is_at_zenith()) 
           printf ("\r\nTelescope is NOT at zenith and locked");
        while (!is_at_zenith()) 
          taskDelay (1*sysClkRateGet());
        printf ("\r\nTelescope is at ZENITH and LOCKED");

        if (is_action_get())
        {
          if (!is_cart_get_position()) 
           printf ("\r\nTelescope is NOT at retrieval position and locked");
          while (!is_cart_get_position()) 
            taskDelay (1*sysClkRateGet());
          printf ("\r\nCART is at CORRECT RETRIEVAL POSITION and LOCKED");
	}
	else
        {
          if (!is_cart_put_position()) 
           printf ("\r\nTelescope is NOT at insertion position and locked");
          while (!is_cart_put_position()) 
            taskDelay (1*sysClkRateGet());
          printf ("\r\nCART is at CORRECT INSERTION POSITION and LOCKED");
	}

        il_pump_on();
        if (!is_pump_on()) 
           printf ("\r\nPump is NOT on");
        while (!is_pump_on()) 
          taskDelay (1*sysClkRateGet());
        printf ("\r\nPump is ON");

        il_state=INIT;
        il_action=IL_UP;
	return 0;
}
int il_move (struct IL_STATES *sm)
{
        il_state=lift_fsm(il_instrument,il_action,il_state); 
	return 0;
}
int il_stop (struct IL_STATES *sm)
{
        il_state=lift_fsm(il_instrument,il_action,EXIT); 
	return 0;
}
int is_true (short *v1,short *v2)
{
	return TRUE;
}
int is_false (short *v1,short *v2)
{
	return FALSE;
}
int is_less_than (short *v1,short *v2)
{
	if (v1<v2) return TRUE;
	else return FALSE;
}
int is_motion_done (short *v1,short *v2)
{
	if (il_state==EXIT) return TRUE;
	else return FALSE;	
}
int leave (short *v1,short *v2)
{
	return TRUE;
	
}
short fiber_pos=100;
short plate_pos=50;
#define START_FSM	0x00
#define CONTINUE_FSM	0xFE
#define EXIT_FSM	0xFF
#define FIBER_INI	0
#define FIBER_ENGAG	1
#define FIBER_PLAT	2
#define FIBER_GET	3
#define FIBER_PUT	4
#define FIBER_RETURN	5
#define FIBER_ABORT	6
#define FIBER_FINISH	7
struct IL_STATES fiberfsm[]={
	{"Fiber_ini",fiberini,NULL,
	  {{is_true,NULL,NULL,FIBER_ENGAG,NULL},
	  {NULL},{NULL},{NULL}}},
	{"Fiber_Engage",fiber_msg,il_move,
	  {{is_plate_engage,NULL,NULL,FIBER_PLAT,CONTINUE_FSM},
	  {is_less_than,&il_pos,&plate_pos,CONTINUE_FSM,FIBER_ABORT},
	  {NULL},{NULL}}},
	{"Fiber_Plate",fiber_msg,il_move,
	  {{is_action_get,NULL,NULL,FIBER_GET,FIBER_PUT},
	  {NULL},{NULL},{NULL}}},
	{"Fiber_Get",fiber_msg,il_move,
	  {{is_clamping_force,NULL,NULL,FIBER_RETURN,FIBER_PUT},
	  {NULL},{NULL},{NULL}}},
	{"Fiber_Put",fiber_put,il_move,
	  {{is_clamping_force,NULL,NULL,FIBER_RETURN,FIBER_PUT},
	  {is_less_than,&il_pos,&fiber_pos,FIBER_ENGAG,FIBER_ABORT},
	  {NULL},{NULL}}},
	{"Fiber_Return",fiber_msg,il_move,
	  {{is_motion_done,NULL,NULL,FIBER_FINISH,FIBER_RETURN},
	  {NULL},{NULL},{NULL}}},
	{"Fiber_Abort",fiber_msg,il_stop,
	  {{leave,NULL,NULL},
	  {NULL},{NULL},{NULL}}},
	{"Fiber_Finish",fiber_msg,il_stop,
	  {{leave,NULL,NULL},
	  {NULL},{NULL},{NULL}}}
};
struct IL_STATES testfsm[]={

	{"Fiber_ini",NULL,NULL,
	  {{is_true,NULL,NULL,FIBER_ENGAG,NULL},
	  {NULL},{NULL},{NULL}}},
	{"Fiber_Engage",fiber_msg,NULL,
	  {{is_plate_engage,NULL,NULL,FIBER_PLAT,CONTINUE_FSM},
	  {is_less_than,&il_pos,&plate_pos,CONTINUE_FSM,FIBER_ABORT},
	  {NULL},{NULL}}},
	{"Fiber_Plate",fiber_msg,NULL,
	  {{is_action_get,NULL,NULL,FIBER_GET,FIBER_PUT},
	  {NULL},{NULL},{NULL}}},
	{"Fiber_Get",fiber_msg,NULL,
	  {{is_clamping_force,NULL,NULL,FIBER_RETURN,FIBER_PUT},
	  {NULL},{NULL},{NULL}}},
	{"Fiber_Put",fiber_put,NULL,
	  {{is_clamping_force,NULL,NULL,FIBER_RETURN,FIBER_PUT},
	  {is_less_than,&il_pos,&fiber_pos,FIBER_ENGAG,FIBER_ABORT},
	  {NULL},{NULL}}},
	{"Fiber_Return",fiber_msg,NULL,
	  {{is_motion_done,NULL,NULL,FIBER_FINISH,FIBER_RETURN},
	  {NULL},{NULL},{NULL}}},
	{"Fiber_Abort",fiber_msg,NULL,
	  {{leave,NULL,NULL},
	  {NULL},{NULL},{NULL}}},
	{"Fiber_Finish",fiber_msg,NULL,
	  {{leave,NULL,NULL,EXIT_FSM,EXIT_FSM},
	  {NULL},{NULL},{NULL}}}

};
struct IL_INST instfsm[]={
		{0,&fiberfsm[0],FALSE,0,NULL},
		{0,&testfsm[0] ,TRUE, 0,NULL}
};
int fsm (int inst,int action)
{
  int i;
  struct IL_STATES *sm;
  unsigned char new_state,last_state,state;
  
  il_action=action;
  il_instrument=inst;
  sm=instfsm[inst].fsm;
  new_state=START_FSM;
  last_state=CONTINUE_FSM;	/* force initial state to be different */
  if ((il_inst[inst][IL_UP].updates_per_sec!=0)||
  	(il_inst[inst][IL_DN].updates_per_sec!=0)) return 0;
  	
  if ((instfsm[inst].history_enabled)&&(instfsm[inst].history==NULL))
  {
    instfsm[inst].history=
      (struct IL_HISTORY *)malloc (sizeof(struct IL_HISTORY)*HISTORY_IDX_MAX);
    instfsm[inst].history_idx=0;
  }

  if (instfsm[inst].history!=NULL)
  {
    (instfsm[inst].history+instfsm[inst].history_idx)->state=new_state;
    (instfsm[inst].history+instfsm[inst].history_idx)->time=time(NULL);
    (instfsm[inst].history+instfsm[inst].history_idx)->cnt=0;
  }
  while (new_state!=EXIT_FSM)
  {
    ADC128F1_Read_Reg(il_ADC128F1,IL_POSITION,&il_pos);
    if ((il_pos&0x800)==0x800) il_pos |= 0xF000;
    else il_pos &= 0xFFF;
    
    state=new_state;
    if ((*(sm+state)->init!=NULL)&&
    		(last_state!=new_state)) (*(sm+state)->init)(sm+state);
    last_state=state;
    
    if (instfsm[inst].history!=NULL)
      (instfsm[inst].history+instfsm[inst].history_idx)->cnt++;
    if (*(sm+state)->action!=NULL) (*(sm+state)->action)(sm+state);
    for (i=0;i<4;i++)
    {
      if ((sm+state)->rules[i].rule!=NULL) 
      {
	if ((*(sm+state)->rules[i].rule) ((sm+state)->rules[i].var[0],
	(sm+state)->rules[i].var[1]) )
	{
	  new_state=(sm+state)->rules[i].true_state;
	}
	else
	{ 
	  new_state=(sm+state)->rules[i].false_state;
	}
	if (new_state==CONTINUE_FSM) new_state=state;
	if (new_state!=state) 
	{
	  if (instfsm[inst].history!=NULL)
	  {
	    (instfsm[inst].history+instfsm[inst].history_idx)->rule_fired=i;
            instfsm[inst].history_idx = 
       		(instfsm[inst].history_idx+1)%HISTORY_IDX_MAX;
	    (instfsm[inst].history+instfsm[inst].history_idx)->state=
	    	new_state;
	    (instfsm[inst].history+instfsm[inst].history_idx)->time=
	    	time(NULL);
	    (instfsm[inst].history+instfsm[inst].history_idx)->cnt=0;

	  }
	  break;
	}
      }
    }
    taskDelay (1);
  }
  return 0;
}
void il_trace (int inst, int cnt)
{
  int i, idx;
  char symname[MAX_SYS_SYM_LEN+1];
  SYM_TYPE symtype;
  int symval;
  struct IL_STATES *sm;
  struct IL_HISTORY *h;
  
  if (instfsm[inst].history!=NULL)
  {

    printf ("\r\n IL FSM History for instrument %d",inst);
    printf ("\r\n   STATE NAME         CNT  RULE        TIME\r\n");
    idx=instfsm[inst].history_idx;
    for (i=0;i<cnt;i++)
    {
      if (idx<=0)idx=HISTORY_IDX_MAX;
      idx = (idx-1)%HISTORY_IDX_MAX;
      h=instfsm[inst].history+idx;
      sm=instfsm[inst].fsm;        
      printf ("%d: %d %s       %d  %d %s",idx,
        h->state,
        (sm+h->state)->name,
        h->cnt,
        h->rule_fired,
        ctime(&h->time));
        
      if (h->rule_fired<4)
      {
        symFindByValue(sysSymTbl,       
          (unsigned long)(sm+h->state)->rules[h->rule_fired].rule,
          &symname[0],&symval,&symtype);

        printf ("  rule @%p: %s %d\r\n",
          (sm+h->state)->rules[h->rule_fired].rule,
          symname,symtype);
      }
    }
  }  
}
int is_at_zenith()
{
  return TRUE;
}
int is_cart_get_position()
{
  extern struct SDSS_FRAME sdssdc;
  return TRUE;
	
  return (sdssdc.status.i1.il0.fiber_cart_pos1);
}
int is_cart_put_position()
{
  extern struct SDSS_FRAME sdssdc;
  return TRUE;
	
  return (sdssdc.status.i1.il0.fiber_cart_pos2);
}
int is_pump_on()
{	
  extern struct SDSS_FRAME sdssdc;
  return (sdssdc.status.i1.il0.inst_lift_pump_on);
}
int is_plate_engage()
{
  extern struct SDSS_FRAME sdssdc;
  return TRUE;
	
  return (sdssdc.status.i1.il0.inst_lift_sw1);
}
int is_plate_full()
{
  extern struct SDSS_FRAME sdssdc;
  return TRUE;
	
  return (sdssdc.status.i1.il0.inst_lift_sw3||sdssdc.status.i1.il0.inst_lift_sw4);
}
int is_plate_empty()
{
  return TRUE;
	
  return (!is_plate_full());
}
int is_fiber_inst()
{
  extern struct SDSS_FRAME sdssdc;
	
  return (sdssdc.status.i1.il0.inst_lift_sw4);
}
int is_dummy_inst()
{
  extern struct SDSS_FRAME sdssdc;
	
  return (sdssdc.status.i1.il0.inst_lift_sw3);
}
int is_clamping_force()
{
  extern struct SDSS_FRAME sdssdc;
	
  return (sdssdc.status.i1.il0.inst_lift_sw2);
}

int liftUp (int inst)
{
   printf ("\r\nLIFT IL: for instrument %d in %s position",
  		inst,motion_type[IL_UP]);
  new_state=state=INIT;
  return (lift_fsm(inst,IL_UP,INIT));
}
int liftDn (int inst)
{
   printf ("\r\nLIFT IL: for instrument %d in %s position",
  		inst,motion_type[IL_DN]);
  new_state=state=INIT;
  return(lift_fsm(inst,IL_DN,INIT));
}
int lift_fsm (int inst, int motion, int new_state)
{
  short force,pos,delta,direction,abort_pos,abort_force;
  unsigned char vel,idxvel;
#ifdef FAKE_IT
  extern SEM_ID semSLC;
  short dstrain,dvel,dpos;
  struct B10_0 il_ctrl;   
#endif
  if (il_inst[inst][motion].updates_per_sec!=0) new_state=state=INIT;
  last_tick=tickGet();
  last_error=0;
  vel=0;
  while (new_state!=EXIT)
  {
    cnt++;
    ADC128F1_Read_Reg(il_ADC128F1,IL_POSITION,&pos);
    if ((pos&0x800)==0x800) pos |= 0xF000;
    else pos &= 0xFFF;
    ADC128F1_Read_Reg(il_ADC128F1,IL_STRAIN_GAGE,&force);
    if ((force&0x800)==0x800) force |= 0xF000;
    else force &= 0xFFF;
    delta = abs(pos-il_inst[inst][motion].pos_setting);
    il_inst[inst][motion].pos_current=pos;
    il_inst[inst][motion].pos_error=delta;
    last_direction = direction;
    direction = pos<il_inst[inst][motion].pos_setting?DOWN_DIRECTION:UP_DIRECTION;

    if (force<0x40) new_state=ABORT_NO_STRAIN;		/* ABORT */
    if ((vel&0xF)!=0) idxvel=vel&0xF;
    else idxvel=((vel>>4)&0xF);
    if ((tickGet()-last_tick)>moving_rate[idxvel])
    {
      if (delta>(last_error-moving_pos[idxvel]))
	new_state=ABORT_NOT_MOVING;
      last_error=delta;
      last_tick=tickGet();
    }

    last_state=state;
    state=new_state;
    switch (state)
    {
      case INIT:
        cnt=0;
        last_state=INIT;
        last_error=0x7FFF;

/* dynamically adjust base-line of static force */
	if (motion==IL_UP)
	{
          ADC128F1_Read_Reg(il_ADC128F1,IL_STRAIN_GAGE,&force);
          if ((force&0x800)==0x800) force |= 0xF000;
          else force &= 0xFFF;
          il_inst[inst][motion].base_strain=force;
	}
        il_pump_on();
        il_solenoid_engage();
        last_tick=tickGet();
        if (delta>il_inst[inst][motion].stop_pos_error) new_state=START_SLEW;
	else new_state=SLOW_SLEW;
	if (IL_verbose) printf ("\r\n INIT: ");
        break;

      case START_SLEW:
/* start velocity */
	if (last_state!=START_SLEW)
	  if (IL_verbose) printf ("\r\n START SLEW: ");
        if (direction)
	{
          vel=il_inst[inst][motion].start_velocity&0xF;	/* DOWN */
	  if (force<(il_inst[inst][motion].base_strain-
	    il_inst[inst][motion].start_strain)) new_state=ABORT_LO_STRAIN;
 	  il_motion_dn (vel);
	}
        else
	{
          vel=il_inst[inst][motion].start_velocity&0xF;	/* UP */
	  if (force>(il_inst[inst][motion].base_strain+
	    il_inst[inst][motion].start_strain)) new_state=ABORT_HI_STRAIN;
 	  il_motion_up (vel);
	}
	if (il_inst[inst][motion].velocity&HI_FORCE_MASK) il_force_on();
        if (delta<il_inst[inst][motion].start_decel_position) new_state=SLOW_SLEW;
        if (delta>il_inst[inst][motion].start_accel_position) new_state=SLEW;
        break;

      case SLEW:
/* maintain velocity */
	if (last_state!=SLEW)
	  if (IL_verbose) printf ("\r\n SLEW: ");
        if (direction)
	{
          vel=il_inst[inst][motion].velocity&0xF;
	  if (force<(il_inst[inst][motion].base_strain-
	    il_inst[inst][motion].slew_strain)) new_state=ABORT_LO_STRAIN;
 	  il_motion_dn (vel);
	}
        else
	{
          vel=il_inst[inst][motion].velocity&0xF;
	  if (force>(il_inst[inst][motion].base_strain+
	    il_inst[inst][motion].slew_strain)) new_state=ABORT_HI_STRAIN;
 	  il_motion_up (vel);
	}
        if (delta<il_inst[inst][motion].start_decel_position) 
          new_state=SLOW_SLEW;
        break;

      case SLOW_SLEW:
/* coast with stop velocity */
        if (direction)
	{
          vel=il_inst[inst][motion].stop_velocity&0xF;
	  if (force<(il_inst[inst][motion].base_strain-
	    il_inst[inst][motion].stop_strain)) new_state=ABORT_LO_STRAIN;
 	  il_motion_dn (vel);
	}
        else
	{
          vel=il_inst[inst][motion].stop_velocity&0xF;
	  if (force>(il_inst[inst][motion].base_strain+
	    il_inst[inst][motion].stop_strain)) new_state=ABORT_HI_STRAIN;
 	  il_motion_up (vel);
	}
	if (last_state!=SLOW_SLEW)
	  if (IL_verbose) printf ("\r\n STOP SLEW: ");
        if ((delta<il_inst[inst][motion].stop_pos_error)&&
          (motion==IL_UP)) new_state=FINAL_SLEW;
        if ((delta<il_inst[inst][motion].stop_pos_error)&&
          (motion==IL_DN)) new_state=EXIT;
        break;

      case FINAL_SLEW:
	if (force>il_inst[inst][motion].base_strain+il_inst[inst][motion].final_strain) new_state=EXIT;
        if (delta>(il_inst[inst][motion].stop_pos_error+10))	
          new_state=ABORT_UP;				/* ABORT - don't go DN */
        vel=il_inst[inst][motion].final_velocity&0xF;	/* UP only */
 	il_motion_up (vel);
	if (last_state!=FINAL_SLEW)
	  if (IL_verbose) printf ("\r\n FINAL SLEW: ");
        break;

      case DELAYED:
	il_disable_motion();
	printf ("\r\nDelayed from state=%d, force=%d",last_state,force);
	if (last_state!=DELAYED) delayed_state=last_state;
	il_status();
        break;
        
      case CONTINUE:
	new_state=delayed_state;
	printf ("\r\nCONTINUE to state=%d",new_state);
        break;
      
      case ABORT_HI_STRAIN:
	il_disable_motion();
	abort_force=force;
/* Going up, go back down */
	abort_pos=pos-10;
	if (abort_pos<0) abort_pos=2;
	printf ("\r\nABORT due to HI STRAIN from state=%d, with force=%d, pos=%d",
		last_state,abort_force,abort_pos);
        vel=il_inst[inst][motion].stop_velocity&0xF;
	while (abort_pos<pos)
	{
    	  ADC128F1_Read_Reg(il_ADC128F1,IL_POSITION,&pos);
    	  if ((pos&0x800)==0x800) pos |= 0xF000;
    	  else pos &= 0xFFF;
 	  il_motion_dn (vel);
	  taskDelay(1);
    	  if ((force&0x800)==0x800) force |= 0xF000;
    	  else force &= 0xFFF;
	  printf ("\r\n                             backoff to force=%d, pos=%d",
		force,pos);
	}
	il_abort();
    	ADC128F1_Read_Reg(il_ADC128F1,IL_STRAIN_GAGE,&force);
	il_status();
        return ABORT_HI_STRAIN;

      case ABORT_LO_STRAIN:
	il_disable_motion();
	abort_force=force;
       /* Going down, go back up */
	abort_pos=pos+10;
	if (abort_pos>il_inst[inst][motion].pos_setting) 
	  abort_pos=il_inst[inst][motion].pos_setting-2;
	printf ("\r\nABORT due to LO STRAIN from state=%d, with force=%d, pos=%d",
		last_state,abort_force,abort_pos);
        vel=il_inst[inst][motion].stop_velocity&4;
	while (abort_pos>pos)
	{
    	  ADC128F1_Read_Reg(il_ADC128F1,IL_POSITION,&pos);
    	  if ((pos&0x800)==0x800) pos |= 0xF000;
    	  else pos &= 0xFFF;
 	  il_motion_up (vel);
	  taskDelay(1);
    	  ADC128F1_Read_Reg(il_ADC128F1,IL_STRAIN_GAGE,&force);
    	  if ((force&0x800)==0x800) force |= 0xF000;
    	  else force &= 0xFFF;
	  printf ("\r\n                             backoff to force=%d, pos=%d",
		force,pos);
	}
	il_abort();
	il_status();
        return ABORT_LO_STRAIN;

      case ABORT_NOT_MOVING:
	il_abort();
	printf ("\r\nABORT Not Closing in on position delta=%d, last error=%d",
		delta,last_error);
	printf ("\r\n   for idxvel=%d moving tick rate=%d, moving pos=%d",
		idxvel,moving_rate[idxvel],moving_pos[idxvel]);
	il_status();
        return ABORT_NOT_MOVING;
        
      case ABORT_UP:
	il_abort();
	printf ("\r\nABORT exceeding final position delta=%d, last error=%d",
		delta,last_error);
	il_status();
        return ABORT_NOT_MOVING;

      case ABORT_NO_STRAIN:
	il_abort();
	printf ("\r\nABORT not enough strain, string broke? force=%d",
		force);
	il_status();
        return ABORT_NOT_MOVING;

     case EXIT:
	il_abort();
	if (IL_verbose) printf ("\r\n EXIT: force=%d",force);
        break;

      default:
	return ILLEGAL_STATE;
    }
/* fake moving the weight */
#ifdef FAKE_IT
#define FAKE_IT	1
    if (semTake (semSLC,60)!=ERROR)
    {
      err = slc_read_blok(1,10,BIT_FILE,0,&ctrl,1);
      semGive (semSLC);
      if (err)
      {
        printf ("R Err=%04x\r\n",err);
        return err;
      }
    }
/* printf (" read ctrl = 0x%04x\r\n",ctrl);*/
   swab ((char *)&ctrl,(char *)&il_ctrl,2);
   if ((vel=il_ctrl.mcp_lift_dn)==0) vel=-il_ctrl.mcp_lift_up;

    if (state==FINAL_SLEW)
    {
      if (motion==IL_UP)
      {
        DAC128V_Read_Reg(il_DAC128V,IL_FAKE_STRAIN,&dstrain);
        DAC128V_Write_Reg(il_DAC128V,IL_FAKE_STRAIN,dstrain+10);
      }
    }
    else
    {
      DAC128V_Read_Reg(il_DAC128V,IL_FAKE_POS,&dpos);
      DAC128V_Write_Reg(il_DAC128V,IL_FAKE_POS,dpos+vel);
    }
    if (state==START_SLEW)
    {
      DAC128V_Read_Reg(il_DAC128V,IL_FAKE_STRAIN,&dstrain);
      if (dstrain>0x820)DAC128V_Write_Reg(il_DAC128V,IL_FAKE_STRAIN,dstrain-32);
    }
#else
    printf ("\r\n");
#endif
    if (IL_verbose) 
	printf ("pos=%2x,delta=%x,direction=%d,force=%d,",
		pos,delta,direction,force);
    printf ("pos=%6.4f\", %4.2fvolts %4d strain=%6.4f lb, %4.2fvolts %4d ",
		(24*pos)/(2048*0.7802),(10*pos)/2048.,pos,
		force/.3,(10*force)/2048.,force);
    if (il_inst[inst][motion].updates_per_sec!=0)
      taskDelay(sysClkRateGet()/il_inst[inst][motion].updates_per_sec);
    else
       return new_state;
  }
  il_abort();
  if (IL_verbose) printf ("\r\n EXIT: ");
  if (il_inst[inst][motion].updates_per_sec!=0)
    printf ("\r\n .................time=%d secs, cnt=%d\r\n",
		cnt/il_inst[inst][motion].updates_per_sec,cnt);
  return SUCCESS;
}
void il_position(double pos)
{
  il_set_upposition (DEFAULT, pos);
  il_set_dnposition (DEFAULT, pos);
  liftDn (DEFAULT);
}

/* DAC is set for +1 10volts where 0=-10 volts;0x800=0 volts; 0xFFF=+10 volts */
/* pos and neg voltages provide direction control */
#define ONE_VLT		204.8		/* 0x800/10.0 */
#define MAX_VEL		8*204	/* absolute ranges */
#define MIN_VEL		2*204	/* 400-50 rpm range */
#define MAX_ACCEL	100	/* absolute step change */
#define MIN_ACCEL	10
#define MAX_STOP_VEL	4*204	/* absolute ranges */
#define MIN_STOP_VEL	2*204	/* 100-2.4 rpm range */
void il_calc (struct IL_LOOP *il)
{

}
short umbilical_offset=600;
int il_umbilical_move(int val) 
{
   int err;
   unsigned short ctrl;
   struct B10_1 il_ctrl;   
  extern SEM_ID semSLC;
             
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_read_blok(1,10,BIT_FILE,1,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("R Err=%04x\r\n",err);
       return err;
     }
   }
/* printf (" read ctrl = 0x%04x\r\n",ctrl);*/
   swab ((char *)&ctrl,(char *)&il_ctrl,2);
   il_ctrl.mcp_umbilical_up_dn_cmd = val;
/*   printf ("\r\n mcp_umbilical_up_dn_cmd=%d, ",il_ctrl.mcp_umbilical_up_dn_cmd);*/
   swab ((char *)&il_ctrl,(char *)&ctrl,2);
   printf ("ctrl=%x",ctrl);
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_write_blok(1,10,BIT_FILE,1,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("W Err=%04x\r\n",err);
       return err;
     }
   }
   return 0;
}
int il_umbilical_move_dn() 
{
   return (il_umbilical_move(1));
}
int il_umbilical_move_up() 
{
   return (il_umbilical_move(0));
}
int il_umbilical(int val) 
{
   int err;
   unsigned short ctrl;
   struct B10_1 il_ctrl;   
   extern SEM_ID semSLC;
             
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_read_blok(1,10,BIT_FILE,1,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("R Err=%04x\r\n",err);
       return err;
     }
   }
/* printf (" read ctrl = 0x%04x\r\n",ctrl);*/
   swab ((char *)&ctrl,(char *)&il_ctrl,2);
   il_ctrl.mcp_umbilical_on_off_cmd = val;
/*   printf ("\r\n mcp_umbilical_on_off_cmd=%d, ",il_ctrl.mcp_umbilical_on_off_cmd);*/
   swab ((char *)&il_ctrl,(char *)&ctrl,2);
   printf ("ctrl=%x",ctrl);
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_write_blok(1,10,BIT_FILE,1,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("W Err=%04x\r\n",err);
       return err;
     }
   }
   return 0;
}
int il_umbilical_on() 
{
    return (il_umbilical(1));
}
int il_umbilical_off() 
{
    return (il_umbilical(0));
}
short il_umbilical_position()
{
  int err;
  extern SEM_ID semSLC;
  short pos,position;

   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_read_blok(1,9,BIT_FILE,235,&pos,1);
     swab (&pos,&position,2);
     semGive (semSLC);
     if (err)
     {
       printf ("R Err=%04x\r\n",err);
       return err;
     }
   }
   return (position-umbilical_offset);
}
void print_umbilical_position ()
{  
  short position;

  position=il_umbilical_position();
  printf ("\r\nUmbilical Position %d (%2.2f inches)",
	position,
	(28.5*position)/29829.);
}
int il_umbilical_move_pos(int pos)
{
  short position;
  short lastpos;

  position=il_umbilical_position();
  printf ("\r\nUmbilical Position %d move to %d",
	position,pos);
  if (pos<position)	/* move down */
  {
    il_umbilical_move_dn();
    il_umbilical_on();
    taskDelay (4);
    lastpos=32767;
    while (((position=il_umbilical_position())-pos)>60) 
    {
      printf ("\r\nUmbilical Position %d moving down to %d",
	position,pos);
      if ((lastpos-position)<4)
      {
        il_umbilical_off();
	printf ("\r\nABORT UMBILICAL MOVE: not moving down %d",lastpos-position);
        return -1;
      }
      lastpos=position;
      taskDelay (1);
    }
  }
  else			/* move up */
  {
    il_umbilical_move_up();
    il_umbilical_on();
    taskDelay (4);
    lastpos=0;
    while ((pos-(position=il_umbilical_position()))>30) 
    {
      printf ("\r\nUmbilical Position %d moving up to %d",
	position,pos);
      if ((position-lastpos)<4)
      {
        il_umbilical_off();
	printf ("\r\nABORT UMBILICAL MOVE: not moving up %d",position-lastpos);
	return -1;
      }
      lastpos=position;
      taskDelay (1);
    }
  }
  il_umbilical_off();
  return 0;
}
int il_umbilical_offset(int offset)
{
  umbilical_offset=offset;
  return 0;
}
int il_umbilical_mgt()
{
  extern long *axis2pos;
  extern long *axis4pos;
  extern int umbil();
  short umbilical_pt;
  int umbpos;

  umbpos=umbil (*axis2pos,*axis4pos);
  umbpos = (short)(29.5*umbpos);
  umbilical_pt=il_umbilical_position();
  if ((umbpos<(umbilical_pt-60))||(umbpos>(umbilical_pt+30)))
  {
    il_umbilical_move_pos(umbpos);
  }
  return 0;
}
int test_umbilical_mgt(int alt, int rot)
{
  extern int umbilGet();
  short umbilical_pt;
  int umbpos;

  umbpos=umbilGet (alt,rot);
  umbpos = (short)(29.5*umbpos);
  umbilical_pt=il_umbilical_position();
  printf ("\r\n umpos move to %d from %d",umbpos,umbilical_pt); 
  if ((umbpos<(umbilical_pt-60))||(umbpos>(umbilical_pt+30)))
  {
    il_umbilical_move_pos(umbpos);
  }
  return 0;
}
int test_umbilical_move()
{
    print_umbilical_position();
    il_umbilical_move_up();
    il_umbilical_on();
    taskDelay(60*2);
    il_umbilical_off();
    print_umbilical_position();
    taskDelay(30);
    print_umbilical_position();

    taskDelay(60*10);

    il_umbilical_move_dn();
    il_umbilical_on();
    taskDelay(60*2);
    il_umbilical_off();
    print_umbilical_position();
    taskDelay(30);
    print_umbilical_position();
    printf ("\r\nUmbilical UP/DN test Done");    
    return 0;
}
int il_zenith_clamp(int val) 
{
   int err;
   unsigned short ctrl[2];
   struct B10_0 il_ctrl;   
   struct B10_1 il_ctrl_1;   
  extern SEM_ID semSLC;
             
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_read_blok(1,10,BIT_FILE,0,&ctrl[0],2);
     semGive (semSLC);
     if (err)
     {
       printf ("R Err=%04x\r\n",err);
       return err;
     }
   }
/* printf (" read ctrl = 0x%04x\r\n",ctrl);*/
   swab ((char *)&ctrl[0],(char *)&il_ctrl,2);
   swab ((char *)&ctrl[1],(char *)&il_ctrl_1,2);
   if (val==1)
   {
     il_ctrl.mcp_clamp_en_cmd = 1;
     il_ctrl_1.mcp_clamp_dis_cmd = 0;
   }
   if (val==0)
   {
     il_ctrl.mcp_clamp_en_cmd = 0;
     il_ctrl_1.mcp_clamp_dis_cmd = 1;
   }
   if (val==-1)
   {
     il_ctrl.mcp_clamp_en_cmd = 0;
     il_ctrl_1.mcp_clamp_dis_cmd = 0;
   }
   printf ("\r\n mcp_clamp_en_cmd=%d, ",il_ctrl.mcp_clamp_en_cmd);
   printf ("\r\n mcp_clamp_dis_cmd=%d, ",il_ctrl_1.mcp_clamp_dis_cmd);
   swab ((char *)&il_ctrl,(char *)&ctrl[0],2);
   swab ((char *)&il_ctrl_1,(char *)&ctrl[1],2);
   printf ("ctrl=%x %x",ctrl[0],ctrl[1]);
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_write_blok(1,10,BIT_FILE,0,&ctrl[0],2);
     semGive (semSLC);
     if (err)
     {
       printf ("W Err=%04x\r\n",err);
       return err;
     }
   }
   return 0;
}
int il_zenith_clamp_engage() 
{
    il_zenith_clamp(1);
    taskDelay (60*2);
    il_zenith_clamp(-1);
    return 0;
}
int il_zenith_clamp_disengage() 
{
    il_zenith_clamp(0);
    taskDelay (60*2);
    il_zenith_clamp(-1);
    return 0;
}
int il_disable_motion()
{
	unsigned char val;

	DIO316_Read_Port (il_DIO316,IL_ENABLE,&val);
	DIO316_Write_Port (il_DIO316,IL_ENABLE,val&IL_DISABLED);
/*	il_motion_up (0);*/
        il_enable_motion_status();
	return 0;
}
int il_enable_motion()
{
	unsigned char val;

	DIO316_Read_Port (il_DIO316,IL_ENABLE,&val);
	DIO316_Write_Port (il_DIO316,IL_ENABLE,val|IL_ENABLED);
        il_enable_motion_status();
	return 0;
}
int il_enable_motion_status()
{
	unsigned char val;

	DIO316_Read_Port (il_DIO316,IL_ENABLE,&val);
	printf ("\r\nENABLE MOTION status=%x",val);
	return 0;
}
int il_pump(short val) 
{
   int err;
   unsigned short ctrl;
   struct B10_0 il_ctrl;   
  extern SEM_ID semSLC;
             
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_read_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("R Err=%04x\r\n",err);
       return err;
     }
   }
/* printf (" read ctrl = 0x%04x\r\n",ctrl);*/
   swab ((char *)&ctrl,(char *)&il_ctrl,2);
   il_ctrl.mcp_pump_on = val;
   swab ((char *)&il_ctrl,(char *)&ctrl,2);
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_write_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("W Err=%04x\r\n",err);
       return err;
     }
   }
   else 
     return ERROR;
   return 0;
}
int il_pump_on()
{
  int status;

    status=il_pump (1);
    taskDelay (4*60);
    return status;
}
int il_pump_off()
{
    return il_pump (0);
}
int il_force(short val) 
{
   int err;
   unsigned short ctrl;
   struct B10_0 il_ctrl;   
  extern SEM_ID semSLC;
             
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_read_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("R Err=%04x\r\n",err);
       return err;
     }
   }
   printf (" read ctrl = 0x%04x\r\n",ctrl);
   swab ((char *)&ctrl,(char *)&il_ctrl,2);
   il_ctrl.mcp_lift_high_psi = val;
   printf("\r\n mcp_lift_high_psi=%d, ",il_ctrl.mcp_lift_high_psi);
   swab ((char *)&il_ctrl,(char *)&ctrl,2);
   printf("ctrl=%x",ctrl);
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_write_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("W Err=%04x\r\n",err);
       return err;
     }
   }
   else
     return -1;
   return 0;
}
int il_force_on()
{
    return il_force (1);
}
int il_force_off()
{
    return il_force (0);
}
int il_solenoid(short val) 
{
   int err;
   unsigned short ctrl;
   struct B10_0 il_ctrl;   
  extern SEM_ID semSLC;
             
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_read_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("R Err=%04x\r\n",err);
       return err;
     }
   }
/* printf (" read ctrl = 0x%04x\r\n",ctrl);*/
   swab ((char *)&ctrl,(char *)&il_ctrl,2);
   il_ctrl.mcp_solenoid_enable = val;
   printf ("\r\n mcp_solenoid_enable=%d, ",il_ctrl.mcp_solenoid_enable);
   swab ((char *)&il_ctrl,(char *)&ctrl,2);
   printf ("ctrl=%x",ctrl);
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_write_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("W Err=%04x\r\n",err);
       return err;
     }
   }
   else
     return -1;
   return 0;
}
int il_solenoid_disengage()
{
    StopCounter (&sbrd,IL_WD);
    return il_solenoid (0);
}
int il_solenoid_engage()
{
    WriteCounterConstant (&sbrd,IL_WD);		/* 60 ms */
    StartCounter (&sbrd,IL_WD);
    return il_solenoid (1);
}
int il_solenoid_maintain()
{
    SetCounterConstant (&sbrd,IL_WD,60000);		/* 60 ms */
    return 0;
}
int shutdown_wd (int type)
{
  printf("WD IP480 shutdown: 3 Interrupts\r\n");
  SetInterruptEnable(&sbrd,IL_WD,IntDisable);
  SetInterruptEnable(&sbrd,TM_WD,IntDisable);
  SetInterruptEnable(&sbrd,CW_WD,IntDisable);
  taskDelay (30);
  return 0;
}
int setup_wd (char *addr, char vec, int irq)
{
  sbrd.brd_ptr=(BYTE *)addr;

  SetInterruptVector (&sbrd,&vec);
  attach_ihandler (0,sbrd.m_InterruptVector,0,wd_isr,
  		(struct handler_data *)&sbrd);
  rebootHookAdd ((FUNCPTR)shutdown_wd);
  return 0;
}
int il_setup_wd ()
{
  SetMode (&sbrd,IL_WD,Watchdog);
  SetDebounce (&sbrd,IL_WD,DebounceOff);
  SetInterruptEnable(&sbrd,IL_WD,IntEnable);
  SetCounterSize (&sbrd,IL_WD,CtrSize32);
  SetCounterConstant (&sbrd,IL_WD,60000);
  SetClockSource (&sbrd,IL_WD,InC1Mhz);
  SetTriggerSource (&sbrd,IL_WD,InTrig);
  SetWatchdogLoad (&sbrd,IL_WD,WDIntLd);
  SetOutputPolarity (&sbrd,IL_WD,OutPolHi);
  ConfigureCounterTimer(&sbrd,IL_WD);
  return 0;
}
int wdog=0;
void wd_isr(struct conf_blk *cblk)
{
int i,j;
UWORD i_stat;

 i_stat = inpw(cblk->brd_ptr + InterruptPending);
  if(cblk->num_chan == 2)   /* check if it's a 2 or 6 channel bo */
   {
    i_stat &= 0x0300;       /* and off the unused upper bits */
    j = 2;
   }
  else
   {
    i_stat &= 0x3F00;        /* and off the unused bits and save the */
    j = 6;
   }


  if( i_stat )               /* any */
  {
           cblk->event_status |= (BYTE)(i_stat >> 8 );   /* update event */

        /* service the hardware */
    logMsg ("\r\nIP480 ABORT fired %d, istat=%d",wdog++,i_stat,0,0,0,0);
        /* check each bit for an interrupt pending state */
    for( i = 0; i < j; i++ )   /* check each c */
    {
          if( i_stat & (1 << (i + 8)) )        /* build interr */
                  i_stat &= (~(1 << (i + 8))); /* clear interr */
          else
                  i_stat |= (1 << (i + 8));    /* set */
    }
    outpw(cblk->brd_ptr + InterruptPending, i_stat);  /* write interrupt pe */
  }
}

void il_status()
{
	extern struct SDSS_FRAME sdssdc;

        if (sdssdc.status.i1.il0.inst_lift_man)
	  printf ("\r\nLOCAL/MANUAL:  ");
	else 
	  printf ("\r\nREMOTE:  ");
	printf ("  %p:  val=0x%hx, test bit=%d",
		&sdssdc.status.i1.il0,sdssdc.status.i1.il0,
		sdssdc.status.i1.il0.inst_lift_man);

	printf ("\r\nDUMMY CARTRIDGE = %hx  ",sdssdc.status.i1.il0.inst_lift_sw3);
	printf ("\r\nFIBER CARTRIDGE = %hx  ",sdssdc.status.i1.il0.inst_lift_sw4);

	if (sdssdc.status.i1.il0.inst_lift_dn) 
	  printf ("\r\nLIFT is DOWN on FLOOR");
	else
	  printf ("\r\nLIFT OFF FLOOR");

	if (sdssdc.status.i1.il0.inst_lift_low_force) printf ("\r\nSTRAIN LOW");
	if (sdssdc.status.i1.il0.inst_lift_high_force) printf ("\r\nSTRAIN HIGH");

	if (is_pump_on())
	  printf ("\r\nPUMP ON");
	else 
   	  printf ("\r\nPUMP OFF");

	if (sdssdc.status.i1.il0.fiber_cart_pos1) 
	  printf ("\r\nFIBER CART in REMOVE POSITION");
	if (sdssdc.status.i1.il0.fiber_cart_pos2) 
	  printf ("\r\nFIBER CART in INSERT POSITION");

	il_read_position (1);
}
void il_abort ()
{
    il_disable_motion();
    il_force_off();
    il_pump_off();
    il_solenoid_disengage();
}
static char bit_reversal[]={0,8,4,0xC, 2,0xA,6,0xE, 1,9,5,0xD, 3,0xB,7,0xF}; 
int il_motion_up(char vel)
{
   int err;
   unsigned short ctrl;
   struct B10_0 il_ctrl;   
  extern SEM_ID semSLC;
             
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_read_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("R Err=%04x\r\n",err);
       return err;
     }
   }
/* printf (" read ctrl = 0x%04x\r\n",ctrl);*/
   swab ((char *)&ctrl,(char *)&il_ctrl,2);
   vel=bit_reversal[vel];
   if (vel != il_ctrl.mcp_lift_up)
   {
      il_disable_motion();
      il_ctrl.mcp_lift_high_psi = 0;
      il_ctrl.mcp_lift_dn = 0;
      il_ctrl.mcp_lift_up = vel;
   }
   swab ((char *)&il_ctrl,(char *)&ctrl,2);
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_write_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("W Err=%04x\r\n",err);
       return err;
     }
   }
   il_enable_motion();
   SetCounterConstant (&sbrd,IL_WD,60000);		/* 60 ms */
   return 0;
 }
int stop_motion;
int il_motion_dn(char vel)
{
   int err;
   unsigned short ctrl;
   struct B10_0 il_ctrl;   
  extern SEM_ID semSLC;
             
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_read_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("R Err=%04x\r\n",err);
       return err;
     }
   }
/* printf (" read ctrl = 0x%04x\r\n",ctrl);*/
   swab ((char *)&ctrl,(char *)&il_ctrl,2);
   if (vel != il_ctrl.mcp_lift_dn)
   {
      il_disable_motion();
      il_force_off();
      il_ctrl.mcp_lift_high_psi = 0;
      il_ctrl.mcp_lift_up = 0;
      il_ctrl.mcp_lift_dn = vel;
   }
   swab ((char *)&il_ctrl,(char *)&ctrl,2);
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_write_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("W Err=%04x\r\n",err);
       return err;
     }
   }
   il_enable_motion();
   SetCounterConstant (&sbrd,IL_WD,60000);		/* 60 ms */
   return 0;
}
#define MOTOR_RAMP	40
int il_motion_raw_up (char vel)
{
   int err;
   unsigned short ctrl;
   struct B10_0 il_ctrl;   
  extern SEM_ID semSLC;

   printf ("\r\n  Type any character to abort......");             
   il_disable_motion();
   il_force_off();
   if (semTake (semSLC,60)!=ERROR)
   {
     semGive (semSLC);
     err = slc_read_blok(1,10,BIT_FILE,0,&ctrl,1);
     if (err)
     {
       printf ("R Err=%04x\r\n",err);
       return err;
     }
   }
/* printf (" read ctrl = 0x%04x\r\n",ctrl);*/
   swab ((char *)&ctrl,(char *)&il_ctrl,2);
   il_ctrl.mcp_lift_dn = 0;
   il_ctrl.mcp_lift_up = vel;
   swab ((char *)&il_ctrl,(char *)&ctrl,2);
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_write_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("W Err=%04x\r\n",err);
       return err;
     }  
   }
  stop_motion=FALSE;
  il_enable_motion();
/*  while (!kbd_input()) */
  while (!stop_motion) 
   SetCounterConstant (&sbrd,IL_WD,60000);		/* 60 ms */
  il_abort();
  return 0;
}
int il_motion_raw_dn (char vel)
{
   int err;
   unsigned short ctrl;
   struct B10_0 il_ctrl;   
  extern SEM_ID semSLC;
             
   printf ("\r\n  Type any character to abort......");
   il_disable_motion();
   il_force_off();
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_read_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("R Err=%04x\r\n",err);
       return err;
     }
   }
/* printf (" read ctrl = 0x%04x\r\n",ctrl);*/
   swab ((char *)&ctrl,(char *)&il_ctrl,2);
   il_ctrl.mcp_lift_up = 0;
   il_ctrl.mcp_lift_dn = vel;
   swab ((char *)&il_ctrl,(char *)&ctrl,2);
   if (semTake (semSLC,60)!=ERROR)
   {
     err = slc_write_blok(1,10,BIT_FILE,0,&ctrl,1);
     semGive (semSLC);
     if (err)
     {
       printf ("W Err=%04x\r\n",err);
       return err;
     }  
   }
  stop_motion=FALSE;
  il_enable_motion();
  while (!stop_motion) 
/*  while (!kbd_input())*/
   SetCounterConstant (&sbrd,IL_WD,60000);		/* 60 ms */
  il_abort();
  return 0;
}
int il_stop_motion()
{
  stop_motion=TRUE;
  return stop_motion;
}
void il_list (int inst)
{
  int i;
  extern char *inst_name[];

  if ((inst>=0)&&(inst<(sizeof(il_inst)/(sizeof(struct IL_LOOP)*2))))
  {
    printf ("\r\nINST %d (%s):",inst, inst_name[inst]);
    for (i=0;i<2;i++) 
    {
      printf ("\r\n%s............",motion_type[i]);
      printf ("\r\n IL POS SETTING=%d (%6.4f\" %4.2fv)",
	il_inst[inst][i].pos_setting,
	(24*il_inst[inst][i].pos_setting)/(2048*0.7802),
	(10*il_inst[inst][i].pos_setting)/2048.);
      printf (", CURRENT=%d (%6.4f\" %4.2fv)",
    	il_inst[inst][i].pos_current,
    	(24*il_inst[inst][i].pos_current)/(2048*0.7802),
    	(10*il_inst[inst][i].pos_current)/2048.);
      printf (", ERROR=%d (%6.4f\" %4.2fv)",
    	il_inst[inst][i].pos_error,
    	(24*il_inst[inst][i].pos_error)/(2048*0.7802),
    	(10*il_inst[inst][i].pos_error)/2048.);
      printf ("\r\n START VEL=0x%x",il_inst[inst][i].start_velocity);
      printf (" VELOCITY=0x%x",il_inst[inst][i].velocity);
      printf (" STOP VEL=0x%x",il_inst[inst][i].stop_velocity);
      printf (" FINAL VEL=0x%x",il_inst[inst][i].final_velocity);
      printf ("\r\n BASE STRAIN=%d",il_inst[inst][i].base_strain);
      printf (" START STRAIN=%d",il_inst[inst][i].start_strain);
      printf (" SLEW STRAIN=%d",il_inst[inst][i].slew_strain);
      printf (" STOP STRAIN=%d",il_inst[inst][i].stop_strain);
      printf (" FINAL STRAIN=%d",il_inst[inst][i].final_strain);
      printf ("\r\n Updates per Sec=%d",il_inst[inst][i].updates_per_sec);
      printf (" Start Accel Position=%d",il_inst[inst][i].start_accel_position);
      printf (" Start Decel Position=%d",il_inst[inst][i].start_decel_position);
      printf ("\r\n Stop Pos Error=%d",il_inst[inst][i].stop_pos_error);
      printf (" Stop Count=%d",il_inst[inst][i].stop_count);
      printf ("\r\n");
    }
  }
}
void il_read_position (int cnt)
{
  int i;
  short adc;

  if (cnt==0) cnt=1;
  printf ("\r\n    IL POSITION");
  for (i=0;i<cnt;i++)
  {
      ADC128F1_Read_Reg(il_ADC128F1,IL_POSITION,&adc);
      if ((adc&0x800)==0x800) adc |= 0xF000;
      else adc &= 0xFFF;
      printf ("\r\n %6.4f\"  %4.2fv %4d ",(24*adc)/(2048*0.7802),(10*adc)/2048.,adc);
  }
  printf ("\r\n");
}
void il_read_strain_gage (int cnt)
{
  int i;
  short adc;

  if (cnt==0) cnt=1;
  printf ("\r\n       IL STRAIN");
  for (i=0;i<cnt;i++)
  {
    ADC128F1_Read_Reg(il_ADC128F1,IL_STRAIN_GAGE,&adc);
    if ((adc&0x800)==0x800) adc |= 0xF000;
    else adc &= 0xFFF;
    printf ("\r\n %6.4f lb  %4.2fv %4d ",adc/.3,(10*adc)/2048.,adc);
  }
  printf ("\r\n");
}
void il_data_collection()
{
  extern struct SDSS_FRAME sdssdc;
  short adc;

  if (il_ADC128F1!=-1)
  {
    ADC128F1_Read_Reg(il_ADC128F1,IL_POSITION,&adc);
    if ((adc&0x800)==0x800) sdssdc.inst.pos=adc|0xF000;
    else sdssdc.inst.pos = adc&0xFFF;
    ADC128F1_Read_Reg(il_ADC128F1,IL_STRAIN_GAGE,&adc);
    if ((adc&0x800)==0x800) sdssdc.inst.strain_gage = adc|0xF000;
    else sdssdc.inst.strain_gage=adc&0xFFF;
  }
}
void il_set_upposition (int inst, double pos)
{
  if ((inst>=0)&&(inst<(sizeof(il_inst)/(sizeof(struct IL_LOOP)*2))))
  {
     if ((pos<0.0)||(pos>24.)) return;
     printf ("\r\nINST %d: UP pos=%f",inst,pos);
     il_inst[inst][IL_UP].pos_setting=(short)((pos/24.)*(2048*0.7802));
     il_inst[inst][IL_UP].pos_current=0;
     il_inst[inst][IL_UP].pos_error=0;
  }
}
void il_set_dnposition (int inst, double pos)
{
  if ((inst>=0)&&(inst<(sizeof(il_inst)/(sizeof(struct IL_LOOP)*2))))
  {
     if ((pos<0.0)||(pos>24.)) return;
     printf ("\r\nINST %d: DN pos=%f",inst,pos);
     il_inst[inst][IL_DN].pos_setting=(short)((pos/24.)*(2048*0.7802));
     il_inst[inst][IL_DN].pos_current=0;
     il_inst[inst][IL_DN].pos_error=0;
  }
}
void il_set_upparams (int inst, char start_vel, char vel,
		char stop_vel, char final_vel)
{
  if ((inst>=0)&&(inst<(sizeof(il_inst)/(sizeof(struct IL_LOOP)*2))))
  {
    il_inst[inst][IL_UP].start_velocity=start_vel;		/* user specified velocity */
    il_inst[inst][IL_UP].velocity=vel;		/* user specified velocity */
    il_inst[inst][IL_UP].stop_velocity=stop_vel;	/* user specified stop velocity */
    il_inst[inst][IL_UP].final_velocity=final_vel;	/* user specified stop velocity */
  }
}
void il_set_dnparams (int inst, char start_vel, char vel,
		char stop_vel, char final_vel)
{
  if ((inst>=0)&&(inst<(sizeof(il_inst)/(sizeof(struct IL_LOOP)*2))))
  {
    il_inst[inst][IL_DN].start_velocity=start_vel;		/* user specified velocity */
    il_inst[inst][IL_DN].velocity=vel;		/* user specified velocity */
    il_inst[inst][IL_DN].stop_velocity=stop_vel;	/* user specified stop velocity */
    il_inst[inst][IL_DN].final_velocity=final_vel;	/* user specified stop velocity */
  }
}
void il_set_upconst (int inst, int upd, 
	 int start_accel_pos, int start_decel_pos,
	int stop_pos_err, int stop_cnt)
{
  if ((inst>=0)&&(inst<(sizeof(il_inst)/(sizeof(struct IL_LOOP)*2))))
  {
    il_inst[inst][IL_UP].updates_per_sec=upd;
    il_inst[inst][IL_UP].start_accel_position=start_accel_pos;/* start position to begin deceleration */
    il_inst[inst][IL_UP].start_decel_position=start_decel_pos;/* start position to begin deceleration */
    il_inst[inst][IL_UP].stop_pos_error=stop_pos_err;	/* stop position error allowed */
    il_inst[inst][IL_UP].stop_count=stop_cnt;		/* stop polarity swing counts allowed */
  }
}
void il_set_dnconst (int inst, int upd, 
	 int start_accel_pos, int start_decel_pos,
	int stop_pos_err, int stop_cnt)
{
  if ((inst>=0)&&(inst<(sizeof(il_inst)/(sizeof(struct IL_LOOP)*2))))
  {
    il_inst[inst][IL_DN].updates_per_sec=upd;
    il_inst[inst][IL_DN].start_accel_position=start_accel_pos;/* start position to begin deceleration */
    il_inst[inst][IL_DN].start_decel_position=start_decel_pos;/* start position to begin deceleration */
    il_inst[inst][IL_DN].stop_pos_error=stop_pos_err;	/* stop position error allowed */
    il_inst[inst][IL_DN].stop_count=stop_cnt;		/* stop polarity swing counts allowed */
  }
}
void il_set_upstrain (short inst, short base_strain, 
	short start_strain, short slew_strain, 
	short stop_strain, short final_strain)
{
  if ((inst>=0)&&(inst<(sizeof(il_inst)/(sizeof(struct IL_LOOP)*2))))
  {
    il_inst[inst][IL_UP].base_strain=base_strain;
    il_inst[inst][IL_UP].start_strain=start_strain;
    il_inst[inst][IL_UP].slew_strain=slew_strain;
    il_inst[inst][IL_UP].stop_strain=stop_strain;
    il_inst[inst][IL_UP].final_strain=final_strain;
  }
}
void il_set_dnstrain (short inst, short base_strain, 
	short start_strain, short slew_strain, 
	short stop_strain, short final_strain)
{
  if ((inst>=0)&&(inst<(sizeof(il_inst)/(sizeof(struct IL_LOOP)*2))))
  {
    il_inst[inst][IL_DN].base_strain=base_strain;
    il_inst[inst][IL_DN].start_strain=start_strain;
    il_inst[inst][IL_DN].slew_strain=slew_strain;
    il_inst[inst][IL_DN].stop_strain=stop_strain;
    il_inst[inst][IL_DN].final_strain=final_strain;
  }
}

char *help_IL[]={
        "IL_help",
	"IL_Verbose, IL_Quiet",
	"int fiberGet(), int fiberPut()",
	"char *fiber_fsm(int inst, int action)",
	"liftUp (int inst), liftDn (int inst)",
	"char *liftup_cmd(char *cmd); cmd=CAMERA,FIBER,INST2...INST15",
	"char *liftdn_cmd(char *cmd); cmd=CAMERA,FIBER,INST2...INST15",
	"int lift_initialize(char *addr)",
	"int lift_fsm(int inst, int motion); inst=0-15; IL_UP=0, IL_DN=1",
	"int il_position(double pos)",
	"int read_ADC(int chan, int cnt)",
	"int read_all_ADC (int cnt)",
	"int il_enable_motion(), int il_disable_motion()",
	"int il_pump_on(), int il_pump_off()",
	"int il_solenoid_engage(), int il_solenoid_disengage()",
	"int il_force_on(), int il_force_off()",
	"int il_status()",
	"int il_abort()",
	"int il_motor_raw(int vel)",
	"void il_list(int inst), int cw_get_inst(char *cmd)",
	"void il_read_position(int cnt)",
	"void il_set_xxposition(int inst, double pos) where xx=up|dn",
	"void il_set_xxparams(int inst,int start_vel,int vel,int stop_vel,",
	"		intfinal_vel)",
	"void il_set_xxconst(int inst,int upd,int start_accel_pos",
	"		int start_decel_pos,int stop_pos_err, int stop_cnt)",
""
};                                                         
void IL_help()
{
  int i;

  for (i=0;i<sizeof(help_IL)/sizeof(char *);i++)
    printf ("%s\r\n",help_IL[i]);
}
void IL_Verbose()
{
	IL_verbose=TRUE;
}
void IL_Quiet()
{
	IL_verbose=FALSE;
}
