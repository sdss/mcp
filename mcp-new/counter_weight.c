#include "vxWorks.h"                            
#include <assert.h>
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
#include "instruments.h"
#include "io.h"
#include "mcpUtils.h"
#include "dscTrace.h"
#include "mcpMsgQ.h"
#include "mcpTimers.h"

/*****************************************************************************/
/*
 * Create the task and message queue for the counterweights
 */
#if USE_MSG_Q
void
tMoveCW(void)
{
   MCP_MSG msg;				/* message to read */
   int status;
   
   for(;;) {
/*
 * Wait for a message asking us to do something
 */
      status = msgQReceive(msgMoveCW, (char*)&msg, sizeof(msg), WAIT_FOREVER);
      assert(status != ERROR);

      TRACE(4, "tMoveCW: received message %d", msg.type, 0);
      
      assert(msg.type == moveCW_type);
      
      if(semTake(semMoveCWBusy, 0) != OK) { /* task is busy */
	 TRACE(0, "moveCW task is already already busy", 0, 0);
	 return;
      }
      
      set_counterweight(msg.u.moveCW.inst,
			msg.u.moveCW.cw, msg.u.moveCW.cwpos);
      
      semGive(semMoveCWBusy);
   }
}

void
tMoveCWInit(void)
{
   if(msgMoveCW != NULL) {
      return;
   }

   msgMoveCW = msgQCreate(40, sizeof(MCP_MSG), MSG_Q_FIFO);
   assert(msgMoveCW != NULL);

   msgMoveCWAbort = msgQCreate(40, sizeof(MCP_MSG), MSG_Q_FIFO);
   assert(msgMoveCWAbort != NULL);
   
   semMoveCWBusy = semBCreate(SEM_Q_PRIORITY, SEM_FULL);
   assert(semMoveCWBusy != NULL);
   
   if(taskSpawn("tMoveCW", 60, VX_FP_TASK, 10000,
		(FUNCPTR)tMoveCW, 0,0,0,0,0,0,0,0,0,0) == ERROR) {
      TRACE(0, "Failed to spawn tMoveCW: %s (%d)", strerror(errno), errno);
   }
}

void
tMoveCWFini(void)
{
   if(msgMoveCW == NULL) {
      return;
   }

   msgQDelete(msgMoveCW);
   msgMoveCW = NULL;

   msgQDelete(msgMoveCWAbort);
   msgMoveCWAbort = NULL;
   
   semDelete(semMoveCWBusy);
   semMoveCWBusy = NULL;
   
   taskDelete(taskIdFigure("tMoveCW"));
}
#endif

/*****************************************************************************/
/*
 * Abort counter weight motion
 */
int
mcp_cw_abort(void)
{
#if USE_MSG_Q
   MCP_MSG msg;
   STATUS stat;

   if(semTake(semMoveCWBusy, 0) == OK) { /* task is idle */
      semGive(semMoveCWBusy);
   } else {
      msg.type = moveCWAbort_type;
      msg.u.moveCWAbort.abort = 1;

      TRACE(1, "Sending abort msg to msgMoveCWAbort", 0, 0);
      stat = msgQSend(msgMoveCWAbort, (char *)&msg, sizeof(msg),
		      NO_WAIT, MSG_PRI_NORMAL);
      assert(stat == OK);
   }
#else
   taskDelete(taskIdFigure("moveCW"));
#endif
   
   cw_abort();

   return(0);
}

/*****************************************************************************/
/*
 * Set or balance the counter weights
 */
int
mcp_set_cw(int inst,			/* instrument to balance for */
	   int cw,			/* CW to move, or ALL_CW */
	   int cwpos,			/* desired position, or 0 to balance */
	   const char **errstr)		/* &error_string, or NULL  */
{
   if(inst < 0 || inst >= NUMBER_INST) {
      if(errstr != NULL) {
	 *errstr = "illegal choice of instrument";
      }
   }

   if(cw != ALL_CW && (cw < 0 || cw > NUMBER_CW)) {
      if(errstr != NULL) {
	 *errstr = "illegal choice of CW";
      }

      return(-1);
   }

   if(cwpos != 0 && (cwpos < 10 || cwpos > 800)) {
      if(errstr != NULL) {
	 *errstr = "ERR: Position out of Range (10-800)";
      }

      return(-1);
   }

   if(sdssdc.status.i9.il0.alt_brake_en_stat == 0) {
      if(errstr != NULL) {
	 *errstr = "ERR: Altitude Brake NOT Engaged";
      }
      
      return(-1);
   } else {
#if USE_MSG_Q
/*
 * send message requesting a counter weight motion
 */
      MCP_MSG msg;
      STATUS stat;

      if(semTake(semMoveCWBusy, 0) == OK) { /* task is idle */
	 semGive(semMoveCWBusy);
      } else {
	 if(errstr != NULL) {
	    *errstr = "ERR: CW or CWP task still active...be patient";
	 }
	 
	 return(-1);			/* CW or CWP task still active */
	 ;
      }
      
      msg.type = moveCW_type;
      msg.u.moveCW.inst = inst;
      msg.u.moveCW.cw = cw;
      msg.u.moveCW.cwpos = cwpos;
      
      stat = msgQSend(msgMoveCW, (char *)&msg, sizeof(msg),
		      NO_WAIT, MSG_PRI_NORMAL);
      assert(stat == OK);
#else
      if(taskIdFigure("moveCW") != ERROR) {
	 if(errstr != NULL) {
	    *errstr = "ERR: CW or CWP task still active...be patient";
	 }
	 
	 return(-1);			/* CW or CWP task still active */
      }

      taskSpawn("moveCW",60,VX_FP_TASK,10000,(FUNCPTR)set_counterweight,
		inst, cw, cwpos,
		0,0,0,0,0,0,0);
#endif
   }

   if(errstr != NULL) {
      *errstr = "";
   }

   return(0);
}

/**************************************************************************
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
	*/
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
#define INST_SPC_COR_CART		4
#define INST_SPC			5
#define INST_SPC_COR			6
#define INST_SPC_ENGCAM			7
#define INST_SPC_ENGCAM_COR		8
#define INST_SPC_IMGCAM			9
#define INST_10		10
#define INST_11		11
#define INST_12		12
#define INST_13		13
#define INST_14		14
#define INST_15		15
#define INST		17
/* Mneumonics are as follows:
	S=Spectograph
	C=Corrector
	F=Fiber cartridge
	E=Engineering camera
	I=Imager camera
*/
char *inst_name[]={"CAMERA","FIBER","EMPTY","SCF",
		     "S","SC","SE","SEC",
		     "SI","INST9","INST10","INST11",
		     "INST12","INST13","INST14","INST15",
		     "INSTDEF"};

#define POS_DIRECTION TRUE
#define NEG_DIRECTION FALSE

/* 10 volts = 2500 RPM, with a 5:1 gear ratio 				*/
/* therefore, 10 volts = 500 RPM, 1 volt = 50 RPM, .02 volts = 1 RPM	*/
#define RPM_IN_VOLTS	.02	/* voltage for one RPM */
#define RPM_PER_COUNT	.244140625	/* 500./2048. */
#define INCHES_PER_RPM	.1	/* 24 inches of total travel */
#define POS_PER_INCH  	66.5770 /* position in adc units per inch */

struct CW_LOOP {
	short pos_setting[NUMBER_CW];	/* position for a given instrucment */
	short pos_current[NUMBER_CW];	/* current position */
	short pos_error[NUMBER_CW];	/* error for the given instrument */
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

/*     1      2     3     4                */
struct CW_LOOP	cw_inst[NUMBER_INST] = {
   {   50,    50,   50,   50},		/*CAMERA*/
   {  600,   600,  600,  600},		/*FIBER	*/
   { 1432,  1470, 1470, 1470},		/*EMPTY	*/
   {  594,   594,  594,  594},		/*SCF	*/
   {  307,   307,  307,  307},		/*S     */
   {  450,   450,  450,  450},		/*SC    */
   {  235,   235,  235,  235},		/*SE    */
   {  870,   870,  870,  870},		/*SEC   */
   { 1413,  1413, 1413, 1413},		/*SI    */
   {0x200, 0x200,0x200,0x200},		/*INST9 */
   {0x200, 0x200,0x200,0x200},		/*INST10*/
   {0x200, 0x200,0x200,0x200},		/*INST11*/
   {0x200, 0x200,0x200,0x200},		/*INST12*/
   {0x200, 0x200,0x200,0x200},		/*INST13*/
   {0x200, 0x200,0x200,0x200},		/*INST14*/
   {0x200, 0x200,0x200,0x200},		/*INST15*/
   {0x200, 0x200,0x200,0x200}		/*DEFAULT*/
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
static void cw_calc(struct CW_LOOP *cw);
static int cw_brake_on(void);
static int cw_power_on(void);
static int cw_power_off(void);
static int cw_select(int cw);
static int cw_rdselect(void);
static void cw_status(void);
static void cw_read_position(int cnt);
static void cw_set_positionv(int inst, const short p[4]);

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
int
balance_initialize(unsigned char *addr,
		   unsigned short vecnum)
{
   int i,ii;                          
   short val;
   STATUS stat;                               
   struct IPACK ip;
/*
 * Initialize the ADC
 */
   Industry_Pack(addr, SYSTRAN_ADC128F1, &ip);
   
   for(i = 0; i < MAX_SLOTS; i++) {
      if(ip.adr[i] != NULL) {
	 cw_ADC128F1 = ADC128F1Init((struct ADC128F1 *)ip.adr[i]);
	 break;
      }
   }
   
   if(i >= MAX_SLOTS) {
      printf ("\r\n****Missing ADC128F1 at %p****\r\n",addr);
      return ERROR;
   }
   ADC128F1_CVT_Update_Control(cw_ADC128F1, ENABLE);
/*
 * Initialize the DAC
 */
   Industry_Pack(addr, SYSTRAN_DAC128V, &ip);
   for(i = 0; i < MAX_SLOTS; i++) {
     if (ip.adr[i]!=NULL) {
	cw_DAC128V = DAC128VInit((struct DAC128V *)ip.adr[i]);
	break;
     }
  }
  
  if(i >= MAX_SLOTS) {
     printf ("****Missing DAC128V at %p****\n",addr);
     return ERROR;
  }
/*
 * check if voltages are zero
 */
  for(i = 0; i < DAC128V_CHANS; i++) {
     DAC128V_Read_Reg(cw_DAC128V,i,&val);
     if((val&0xFFF) != 0x800) {
	printf ("\r\nDAC128V Chan %d Init error %x",i,val);
     }
  }
/*
 * Initialize the DIO316
 */
   Industry_Pack (addr,SYSTRAN_DIO316,&ip);
   for(i = 0; i < MAX_SLOTS; i++) {
      if (ip.adr[i]!=NULL) {
	 cw_DIO316 = DIO316Init((struct DIO316 *)ip.adr[i], vecnum);
	 break;
      }
   }
   
   if(i >= MAX_SLOTS) {
      printf ("\r\n****Missing DIO316 at %p****\r\n",addr);
      return ERROR;
   }
/*
 * set interrupt handlers
 */
   stat = intConnect(INUM_TO_IVEC(vecnum),
		      (VOIDFUNCPTR)cw_DIO316_interrupt,
		      DIO316_TYPE);
   printf ("CW vector = %d, interrupt address = %p, result = %8x\r\n",
	   vecnum,cw_DIO316_interrupt,stat);
   rebootHookAdd((FUNCPTR)cw_DIO316_shutdown);
   
   IP_Interrupt_Enable(&ip, DIO316_IRQ);
   DIO316_OE_Control(cw_DIO316, 3, DIO316_OE_ENA);
   DIO316_Interrupt_Configuration(cw_DIO316, 0, DIO316_INT_FALL_EDGE);
   sysIntEnable(DIO316_IRQ);
   DIO316_Write_Reg(cw_DIO316, 6, 0xF);

# if 0					/* Turned off the interrupts
					   for the limits - unreliable */
   DIO316_Interrupt_Enable_Control (cw_DIO316,0,DIO316_INT_ENA);
#endif
  cw_power_off();
/*
 * zero the DAC - there is a 2048 offset on the 12 bit DAC
 */
   DAC128V_Write_Reg(cw_DAC128V, CW_MOTOR, 0x800);
/*
 * Initialize the data structures for nominal operation
 */
   for(i = 0; i < NUMBER_INST; i++) {
      for(ii = 0; ii < NUMBER_CW; ii++) {
	 cw_inst[i].pos_current[ii] = cw_inst[i].pos_error[ii] = 0;
      }
    
      cw_inst[i].updates_per_sec=10;
      cw_inst[i].accel_rpm=12000.;	/* user specified acceleration */
      cw_inst[i].vel_rpm=500.;		/* user specified velocity */
      cw_inst[i].decel_rpm=6000.;	/* user specified deceleration */
      cw_inst[i].stop_vel_rpm=220.;	/* user specified stop velocity */
      cw_inst[i].start_decel_position=58;/* start position to begin decel. */
      cw_inst[i].stop_pos_error=2;	/* stop position error allowed */
      cw_inst[i].stop_count=6;		/* stop polarity swing counts allowed*/
      cw_calc(&cw_inst[i]);
   }
   
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
**	cw_status
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void
balance(int cw,				/* counter weight to move */
	int inst)			/* to position for this instrument */
{
   int cw0, cw1;			/* move counterweights cw0..cw1 incl.*/
   int CW_next;				/* go on to next counterweight */
   int i;
   short vel,pos,delta,direction;
   short last_direction;
   int totcnt = 0, cnt, last_error;
   MCP_MSG msg;				/* message to read */

   if(cw == ALL_CW) {
      cw0 = 0; cw1 = NUMBER_CW - 1;
   } else {
      cw0 = cw1 = cw;
   } 

   for(cw = cw0; cw <= cw1; cw++) {
      if(CW_verbose) printf("\r\nBALANCE CW %d: for instrument %d",
			    cw + 1, inst);
/*
 * set mux for specified counter-weight
 */
      cw_select(cw);
      
      TRACE(4, "balance cw = %d inst = %d", cw, inst);
/*
 * iterate until good or exceed stop count
 */
      DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,0x800);
      cw_power_on();
      CW_limit_abort = FALSE;
      
      for(CW_next = i = 0; !CW_next && i < cw_inst[inst].stop_count; i++) {
	 ADC128F1_Read_Reg(cw_ADC128F1,cw,&pos);
	 if((pos & 0x800) == 0x800) {
	    pos |= 0xF000;
	 } else {
	    pos &= 0xFFF;
	 }
	 delta = abs(pos - cw_inst[inst].pos_setting[cw]);
	 
	 TRACE(5, "balance i = %d delta = %d", i, delta);
	 
	 last_error = delta;
	 cw_inst[inst].pos_current[cw] = pos;
	 cw_inst[inst].pos_error[cw] = delta;
	 direction = (pos < cw_inst[inst].pos_setting[cw]) ?
						 POS_DIRECTION : NEG_DIRECTION;
/*
 * iterate until position is adequate
 */
	 cnt = 0;
	 while(delta > cw_inst[inst].stop_pos_error) {
	    TRACE(6, "balance delta = %d, cw_inst[inst].stop_pos_error = %d",
		  delta, cw_inst[inst].stop_pos_error);
	    
	    taskDelay(60/cw_inst[inst].updates_per_sec);
	    
	    if(msgQReceive(msgMoveCWAbort, (char*)&msg, sizeof(msg), NO_WAIT)
								    != ERROR) {
	       assert(msg.type == moveCWAbort_type);
	       TRACE(0, "Counter weight motion abort", 0, 0);
	       printf("CWABORT\n");
	       cw_abort();
	       return;
	    }
	 
	    if(CW_limit_abort) {
	       printf ("\r\nLIMIT ABORT");
	       cw_status();

	       CW_next = 1;

	       break;
	    }
	    
	    cnt++;
	    if(cnt%(cw_inst[inst].updates_per_sec*6) == 0) {
	       if(delta > last_error - 4) {
		  TRACE(2, "Not Closing in on position for CW %d; aborting",
								    cw + 1, 0);
		  cw_status();
		  
		  CW_next = 1;
		  break;
	       }

	       last_error = delta;
	    }
/*
 * deceleration & coast to position
 */
	    DAC128V_Read_Reg(cw_DAC128V, CW_MOTOR, &vel);
	    vel -= 0x800;
	    
	    if(delta < cw_inst[inst].start_decel_position) {
/*
 * still decelerating
 */
	       if(direction) {
		  vel -= cw_inst[inst].decel;
	       } else {
		  vel += cw_inst[inst].decel;
	       }
	       
	       if(abs(vel) >= cw_inst[inst].stop_velocity) {
		  if (CW_verbose) printf ("\r\n DECEL: ");
	       } else {			/* coast with stop velocity */
		  if(direction) {
		     vel =  cw_inst[inst].stop_velocity;
		  } else {
		     vel = -cw_inst[inst].stop_velocity;
		  }
		  if(CW_verbose) printf ("\r\n STOP VEL: ");
	       }    
	    } else {
/*
 * accelerate to velocity
 */
	       if(direction) {
		  vel += cw_inst[inst].accel;
	       } else {
		  vel -= cw_inst[inst].accel;
	       }
	       
	       if(abs(vel) <= cw_inst[inst].velocity) {
		  if (CW_verbose) printf ("\r\n ACCEL: ");
	       } else {			/* maintain velocity */
		  if(direction) {
		     vel =  cw_inst[inst].velocity;
		  } else {
		     vel = -cw_inst[inst].velocity;
		  }
		  if (CW_verbose) printf ("\r\n ACCEL VEL: ");
	       }    
	       
	       vel += 0x800;
	       DAC128V_Write_Reg(cw_DAC128V, CW_MOTOR, vel);
	    }
/*
 * where did we get to?
 */
	    ADC128F1_Read_Reg(cw_ADC128F1, CW_POS + cw, &pos);
	    if((pos & 0x800) == 0x800) {
	       pos |= 0xF000;
	    } else {
	       pos &= 0xFFF;
	    }
	    
	    delta = abs(pos - cw_inst[inst].pos_setting[cw]);
	    cw_inst[inst].pos_current[cw] = pos;
	    cw_inst[inst].pos_error[cw] = delta;
	    
	    last_direction = direction;
	    direction = (pos < cw_inst[inst].pos_setting[cw]) ?
						 POS_DIRECTION : NEG_DIRECTION;
	    if(direction != last_direction) break;
	    
	    if(CW_verbose) {
	       printf("vel=%x,pos=%x,delta=%x,direction=%d",
		      vel - 0x800, pos, delta, direction);
	       printf ("\r\n  vel=%f rpm ,pos=%6.4f\", %4.2f ",
		       (vel-0x800)*RPM_PER_COUNT,(24*pos)/(2048*0.7802),
		       (10*pos)/2048.);
	    }
	 }
	 totcnt += cnt;
	 cw_brake_on();
	 DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,0x800);
	 if(CW_verbose) printf("\r\n SWITCH: ");
      }
      
      TRACE(4, "CW %d done", cw + 1, 0);
   }

   cw_brake_on();
   cw_power_off();
   
   if (CW_verbose) {
      printf ("\r\n STOP: ");
      printf ("\r\n .................time=%d secs\r\n",
	      totcnt/cw_inst[inst].updates_per_sec);
   }
}

/*
 * Set a counterweight to a position specified in volts
 */
void
set_counterweight(int inst,		/* instrument to set for */
		  int cw,		/* counterweight to move, or ALL_CW */
		  short pos)		/* positions to go to */
{
   int i;
   short parray[4];			/* positions for all counterweights */
   
   for(i = 0; i < NUMBER_CW; i++) {
      parray[i] = (cw == ALL_CW || i == cw) ? pos : 0;
   }

   cw_set_positionv(inst, parray);
   
   balance(cw, inst);
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

static void
cw_calc(struct CW_LOOP *cw)
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

  TRACE0(16, "cw_DIO316_interrupt", 0, 0);
  
  DIO316ReadISR (cw_DIO316,&int_bit[0]);
  DAC128V_Read_Reg(cw_DAC128V,CW_MOTOR,&vel);
  vel-=0x800;
  DIO316_Read_Port (cw_DIO316,CW_LIMIT_STATUS,&limit);
  cw = cw_rdselect();
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
** ROUTINE: cw_brake_on
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
static int
cw_brake_on(void)
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
static int
cw_power_on(void)
{
	unsigned char val;

	if (cw_DIO316==-1) return ERROR;
	DIO316_Read_Port (cw_DIO316,CW_POWER,&val);
	DIO316_Write_Port (cw_DIO316,CW_POWER,val|CW_POWER_ON);
        taskDelay (60);
	return 0;
}

static int
cw_power_off(void)
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
static int
cw_select(int cw)
{
	unsigned char val;

	if (cw_DIO316==-1) return ERROR;
	DIO316_Read_Port (cw_DIO316,CW_SELECT,&val);
	DIO316_Write_Port (cw_DIO316,CW_SELECT,(val&(~CW_SELECT))|cw);
	return 0;
}

static int
cw_rdselect(void)
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
** CALLS TO:
**	DIO316_Read_Port
**	cw_read_position
**
** GLOBALS REFERENCED:
**	cw_DIO316
**
**=========================================================================
*/
static void
cw_status(void)
{
   unsigned char val;
   int i;
   
   if(cw_DIO316 == -1) {
      return;
   }
   
   DIO316_Read_Port(cw_DIO316,CW_LCLRMT,&val);
   if(val & CW_LOCAL) {
      printf ("\r\nLOCAL:  ");
   } else {
      printf ("\r\nREMOTE:  ");
   }
   
   DIO316_Read_Port(cw_DIO316, CW_SELECT, &val);
   printf("CW %d Selected, ", (val & 0x3) + 1);

   DIO316_Read_Port(cw_DIO316, CW_INTERLOCK, &val);
   if((val & CW_INTERLOCK_BAD) == CW_INTERLOCK_OK) {
      printf ("Interlock OK and ignored\n");
   } else {
      printf ("Interlock BAD but ignored\n");
   }

   DIO316_Read_Port (cw_DIO316,CW_LIMIT_STATUS,&val);
   printf ("\nLimit Byte 0x%02x (1 implies OPEN or OK)\n",val);
   for(i = 0; i < 4; i++) {
      printf ("  CW%d:  Upper Limit %d Lower Limit %d\n",
	      i + 1, (val >> 2*i) & 0x1, (val >> (2*i + 1)) & 0x1);
   }
   cw_read_position(1);
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
int
cw_abort(void)
{
    if (cw_brake_on()==0) 
      if (cw_power_off()==0) return 0;
    return ERROR;
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
static int
kbd_input(void)
{
  int bytes;

  ioctl (ioTaskStdGet(0,0),FIONREAD,(int)&bytes);
  if (bytes!=0) return TRUE;
  return FALSE;
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
**
** GLOBALS REFERENCED:
**	cw_DAC128V
**
**=========================================================================
*/
#define MOTOR_RAMP	40
static void
cw_motor_raw(int cw, int vel)
{
  int i;

  printf ("\r\n  Type any character to abort......");
  cw_select(cw);
  DAC128V_Write_Reg(cw_DAC128V,CW_MOTOR,0x800);
  cw_power_on();
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

void
cw_motor(int cw, double vel_rpm)
{
  int vel;

  vel = (vel_rpm*RPM_IN_VOLTS)*ONE_VLT;
  cw_motor_raw(cw, vel);
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
void
cw_list(int inst)
{
  int i;

  if ((inst>=0)&&(inst<(sizeof(cw_inst)/sizeof(struct CW_LOOP))))
  {
/*    printf ("\r\nINST %d (%s):",inst, inst_name[inst]);*/
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
void
cw_read_position(int cnt)
{
  int i, ii;
  short adc;

  if(cnt == 0) cnt=1;

  for(ii=0;ii<NUMBER_CW;ii++) {
     printf ("       CW %d       ",ii);
  }
  printf("\n");

  for(i=0;i<cnt;i++) {
     for(ii=0;ii<NUMBER_CW;ii++) {
	ADC128F1_Read_Reg(cw_ADC128F1,ii,&adc);
	if((adc&0x800) == 0x800) {
	   adc |= 0xF000;
	} else {
	   adc &= 0xFFF;
	}
	printf (" %6.4f\"  %4.2fv  ",(24*adc)/(2048*0.7802),(10*adc)/2048.);
     }
     printf("\n");
  }
}

/*=========================================================================
** Set the the four desired positions for a selected instrument in volts
**=========================================================================
*/
static void
cw_set_positionv(int inst,		/* instrument to set pos for */
		 const short p[4])	/* positions of counterweights (V) */
{
   int i;

   if(inst < 0 || inst >= sizeof(cw_inst)/sizeof(struct CW_LOOP)) {
      TRACE(0, "cw_set_positionv: invalid instrument %d", inst, 0);
      return;
   }

   for(i = 0; i < 4; i++) {
      if(p[i] != 0) {
	 if(p[i] < 0 || p[i] > 1000) {
	    TRACE(0, "cw_set_positionv: invalid position %d for cw %d",
		  p[i], i + 1);
	    return;
	 }
	 
	 cw_inst[inst].pos_setting[i] = (short)(p[i]*2.048);
	 cw_inst[inst].pos_current[i] = 0;
	 cw_inst[inst].pos_error[i] = 0;
      }
   }
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: cw_get_inst
**
** DESCRIPTION:
**	Searches the instrument names for a match and returns index. It's
** OK if the instrument string is simply the desired digit
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
int
cw_get_inst(char *cmd)
{
  int inst;
  const int ninst = sizeof(inst_name)/sizeof(char *);

  if(sscanf(cmd, "%d", &inst) == 1) {
     if(inst < 0 || inst >= ninst) {
	return ERROR;
     }
     return(inst);
  }

  for(inst = 0; inst < ninst; inst++) {
     if(strncmp(cmd, inst_name[inst], strlen(inst_name[inst])) == 0) {
	return(inst);
     }
  }
  
  return ERROR;
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
**
** GLOBALS REFERENCED:
**	cw_ADC128F1
**	cw_DIO316
**	sdssdc
**
**=========================================================================
*/
void
cw_data_collection(void)
{
  short adc;
  int ii;

  if(cw_ADC128F1 != -1) {
     for(ii = 0; ii < NUMBER_CW; ii++) {
	ADC128F1_Read_Reg(cw_ADC128F1,ii,&adc);
	if ((adc&0x800) == 0x800) {
	   sdssdc.weight[ii].pos = adc | 0xF000;
	} else {
	   sdssdc.weight[ii].pos = adc & 0xFFF;
	}
     }
  }
  if(cw_DIO316 != -1) {
     DIO316_Read_Port(cw_DIO316, CW_LIMIT_STATUS, &cwLimit);
  }
}

#if 0
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
void
set_DAC(int chan)
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
static int
cw_power_disengage()
{
  StopCounter (&sbrd,CW_WD);
  return 0;
}
static int
cw_power_engage()
{
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
static int
cw_setup_wd ()
{
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
void
cw_set_const(int inst,
	     int upd,
	     int start_decel_pos,
	     int stop_pos_err,
	     int stop_cnt)
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
void
cw_set_params(int inst,
	      double accel,
	      double vel,
	      double decel,
	      double stop_vel)
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
** DESCRIPTION:
**	Sets the the four desired positions for a selected instrument in inches
**	This is deprecated, but not deleted.
**=========================================================================
*/
void
cw_set_position(int inst, double p1, double p2, double p3, double p4)
{
  if ((inst>=0)&&(inst<(sizeof(cw_inst)/sizeof(struct CW_LOOP))))
  {
     if ((p1<0.0)||(p1>24.)) return;
     if ((p2<0.0)||(p2>24.)) return;
     if ((p3<0.0)||(p3>24.)) return;
     if ((p4<0.0)||(p4>24.)) return;
/*     printf ("\r\nINST %d: p1=%f, p2=%f, p3=%f, p4=%f",inst,p1,p2,p3,p4);*/
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
static void
read_ADC (int chan, int cnt)
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

static void
read_all_ADC (int cnt)
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
#endif
