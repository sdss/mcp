#include <vxWorks.h>
#include <stdio.h>
#include <errno.h>
#include <semLib.h>
#include <sigLib.h>
#include <tickLib.h>
#include <taskLib.h>
#include <usrLib.h>
#include <string.h>
#include <inetLib.h>
#include <rebootLib.h>
#include "in.h"
#include "timers.h"
#include "time.h"
#include "ms.h"
#include "abdh.h"
#include "idsp.h"
#include "pcdsp.h"
#include "gendefs.h"
#include "ad12f1lb.h"
#include "ip480.h"
#include "mv162IndPackInit.h"
#include "axis.h"
#include "frame.h"
#include "data_collection.h"
#include "tm.h"
#include "cw.h"
#include "instruments.h"
#include "dscTrace.h"
#include "mcpFiducials.h"

/*-------------------------------------------------------------------------
**
** GLOBAL VARIABLES
*/
short az1vlt,az1cur,az2vlt,az2cur;
short alt1vlt,alt1cur,alt2vlt,alt2cur;
short rot1vlt,rot1cur;
#define TM_ROT1VLT	6
#define TM_ROT1CUR	7
#define TM_AZ1VLT	0
#define TM_AZ1CUR	1
#define TM_AZ2VLT	4
#define TM_AZ2CUR	5
#define TM_ALT1VLT	2
#define TM_ALT1CUR	3
#define TM_ALT2VLT	6
#define TM_ALT2CUR	7
int tm_ADC128F1=-1;

/*
 * tm_mgt variables to enable safe operation if telescope is not in control
 */
int monitor_axis[3];	/* provides mechanism to prevent deadlocks at startup*/
int monitor_on[3];	/* overrides monitoring of axis...possible disable */

/*
 * keep watch dog alive based on all axis software active
 */
int axis_alive = 0;

/*=========================================================================
 *
 *      Telescope motion to the instrument change position.  The routine 
 *	returns immediately while the motion is taking place
 *
 * Instrument change position, and velocity and acceleration to get there
 */
double ilcpos[] = { 120.6, 90.0, 0,0 };
double ilcvel[] = { 200000, 200000, 500000};
double ilcacc[] = { 10000, 10000, 10000};

int
tm_move_instchange(void)
{
   int axis;
   
   if(semTake(semMEI, 60) == ERROR) {
      printf("Err: Could not take semMEI semphore\n");
      return ERROR;
   }

   for(axis = 0; axis < NAXIS; axis++) {
      sem_controller_run(2*axis);
      start_move(2*axis,
		 ilcpos[axis]*ticks_per_degree[axis],
		 ilcvel[axis], ilcacc[axis]);
   }

   semGive (semMEI); 

   return 0;
}

/*=========================================================================
**
**      Telescope motion to a specified position.  The routine returns to 
**	the caller immediately while the motion is being fulfilled.
**	Encaspulates the MEI function and converts args from ints to doubles.
**
*/
void
tm_start_move (int axis, int vel, int accel, int pos)
{
   semTake(semMEI,WAIT_FOREVER);
   start_move(axis,(double)pos,(double)vel,(double)accel);
   semGive (semMEI);
}

/*=========================================================================
**
**      Telescope motion to go over a fiducial point back-and-forth a
**	specified number of times.  Boroski spec.
**
*/
void
tm_bf(int axis,
      int vel,
      int accel,
      int pos1, int pos2,
      int times)
{
   int i;
   int status;
   
   printf("Pass ");
   for(i = 0; i < times; i++) {
      printf ("%d ",i);
      tm_controller_run(axis);
      tm_start_move(axis,vel,accel,pos1);

      status = FALSE;
      while(!status) {
	 taskDelay(60);

	 semTake(semMEI, WAIT_FOREVER);
	 status=motion_done(axis);
	 semGive(semMEI);
      }

      tm_start_move (axis,vel,accel,pos2);

      status = FALSE;
      while(!status) {
	 taskDelay(60);

	 semTake(semMEI, WAIT_FOREVER);
	 status = motion_done(axis);
	 semGive(semMEI);
      }
   }
   printf(" Done\n");
}

/*=========================================================================
**
**      Print the specified axis PID coefficients and sample rate.
*/
void
tm_print_coeffs(int axis)
{
   short coeff[COEFFICIENTS];
   short mode;
   short rate;
   
   semTake(semMEI,WAIT_FOREVER);
   
   get_filter(axis,(P_INT)coeff);
   get_integration(axis,&mode);
   rate = dsp_sample_rate();
   
   semGive (semMEI);
   
   printf("AXIS %d: P=%d, I=%d, D=%d\n", axis, coeff[0], coeff[1], coeff[2]);
   printf("         AFF=%d, VFF=%d, FFF=%d\n", coeff[3], coeff[4], coeff[9]);
   printf("         ILIMIT=%d, OFFSET=%d, OLIMIT=%d, SHIFT=%d\n",
	   coeff[5], coeff[6], coeff[7], coeff[8]);
   printf("integration mode is %d and sample is %d Hz\n", mode, rate);
}

/*=========================================================================
**
**      Set the specified axis coefficient where index is used to specify
**	one of 10 possible coefficients.
**		index=0(P),1(I),2(D),3(AFF),4(VFF),5(ILIM),6(OFF),7(DLIM)
**		8(SHIFT)(-5 is 1/32),9(FFF)
*/
void
tm_set_coeffs(int axis, int index, int val)
{
   short coeff[COEFFICIENTS];
   
   semTake(semMEI,WAIT_FOREVER);

   get_filter(axis,(P_INT)coeff);
   coeff[index] = val;
   set_filter(axis,(P_INT)coeff);

   semGive (semMEI);
}

/*=========================================================================
**=========================================================================
**
**      Telescope motion encapsulates with a semaphore the MEI function.
**
**=========================================================================
*/
void
tm_clear_pos(int axis)
{
   semTake(semMEI,WAIT_FOREVER);
   set_position (axis,0.0);
   semGive(semMEI);
}

void
tm_get_pos(int axis, double *position)
{
   semTake(semMEI,WAIT_FOREVER);
   get_position(axis,position);
   semGive(semMEI);
}

void
tm_get_vel(int axis,
	   double *velocity)
{
   semTake(semMEI,WAIT_FOREVER);
   get_velocity(axis,velocity);
   semGive(semMEI);
}

void
tm_set_sample_rate(unsigned short rate)
{
   semTake(semMEI,WAIT_FOREVER);
   set_sample_rate (rate);
   printf("Sample Rate=%d\n", (unsigned short)dsp_sample_rate());
   semGive(semMEI);
}

void
tm_reset_integrator(int axis)
{
   semTake(semMEI,WAIT_FOREVER);
   reset_integrator (axis);
   semGive(semMEI);
}

void
tm_set_pos(int axis,int pos)
{
   semTake(semMEI,WAIT_FOREVER);
   set_position (axis,(double)pos);
   semGive(semMEI);
}

int
tm_adjust_pos(int axis,			/* desired axis */
	      int offset)		/* how much to offset position */
{
   double position;			/* position of axis */

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      TRACE(0, "tm_adjust_pos: invalid axis %d", axis, 0);
      fprintf(stderr,"tm_adjust_pos: illegal axis %d\n", axis);
      
      return(-1);
   }

   if(semTake(semMEI,60) != OK) {
      TRACE(0, "adjusting position for axis %s: unable to take semaphore: %s",
	    axis_name(axis_select), strerror(errno));
      return(-1);
   }

   taskLock();
   
   if(get_position(2*axis, &position) != DSP_OK) {
      taskUnlock();
      semGive(semMEI);
      TRACE(0, "adjusting position for axis %s: unable to read position",
	    axis_name(axis_select), 0);
      return(-1);
   }
   
   position += offset;
   
   set_position(2*axis, position);
   set_position(2*axis + 1, position);	/* the second encoder in az/alt isn't
					   connected/doesn't exist */
   
   taskUnlock();
   semGive(semMEI);
   
   return(0);
}
  
void
tm_set_encoder(int axis)
{
   semTake(semMEI,WAIT_FOREVER);
   set_feedback(axis,FB_ENCODER);
   semGive(semMEI);
}

void
tm_dual_loop(int axis, int dual)
{
   semTake(semMEI,WAIT_FOREVER);
   set_dual_loop (axis,axis+1,dual);
   semGive(semMEI);
}

#if 0
/*=========================================================================
**
**      Telescope motion sets up the tachometer as the encoder.  No longer
**	used at this time.
*/
void
tm_set_analog_encoder(int axis, int channel)
{
   semTake(semMEI,WAIT_FOREVER);
   
   set_analog_channel(axis,channel,TRUE,TRUE);
   set_axis_analog (axis,TRUE);
   set_feedback(axis,FB_ANALOG);

   semGive(semMEI);
}

/*=========================================================================
**
**      Telescope motion sets analog channel as feedback to PID.  Not used.
*/
void
tm_set_analog_channel(int axis, int channel)
{
   semTake(semMEI,WAIT_FOREVER);

   set_analog_channel(axis,channel,TRUE,TRUE);
   set_axis_analog (axis,TRUE);

   semGive(semMEI);
}
#endif

/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_controller_run
**	    sem_controller_run (assumes the semaphore is already owned)
**
** DESCRIPTION:
**      Telescope motion encapsulates and ensures the MEI is in closed
**	loop before returning.  A nominal retry count is utilized to 
**	prevent a loop.  MEI required at one time numerous function calls
**	for this to take effect.  Subsequent software was suppose to fix
**	this, but I am not convinced.
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
tm_controller_run (int axis)
{
   semTake(semMEI,WAIT_FOREVER);
   sem_controller_run(axis);
   semGive(semMEI);
}

void
sem_controller_run(int axis)
{
   const int nretry = 12;
   int i;
   
   for(i = 0; axis_state(axis) > 2 && i < nretry; i++) {
      taskDelay(20);
      controller_run(axis);
   }
   
   if(i != nretry) {			/* success */
      monitor_axis[axis/2] = TRUE;
   }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_controller_idle
**	    sem_controller_idle (assumes the semaphore is already owned)
**
** DESCRIPTION:
**      Telescope motion encapsulates and ensures the MEI is out of closed
**	loop before returning.  A nominal retry count is utilized to 
**	prevent a loop.  MEI required at one time numerous function calls
**	for this to take effect.  Subsequent software was suppose to fix
**	this, but I am not convinced.
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
tm_controller_idle(int axis)
{
	int retry;

	retry=6;
	semTake(semMEI,WAIT_FOREVER);
  	while ((axis_state(axis)<=2)&&(--retry>0)) 
	  controller_idle (axis);
	semGive(semMEI);
}

void
sem_controller_idle(int axis)
{
	int retry;

	retry=6;
  	while ((axis_state(axis)<=2)&&(--retry>0)) 
	  controller_idle (axis);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_set_boot_filter
**
** DESCRIPTION:
**      Telescope motion makes the current coeffs active for next boot.
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
tm_set_boot_filter(int axis)
{
   short coeff[COEFFICIENTS];
   
   semTake(semMEI,WAIT_FOREVER);
   get_filter(axis,(P_INT)coeff);
   set_boot_filter(axis,(P_INT)coeff);
   semGive(semMEI);
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: ADC128F1_initialize
**
** DESCRIPTION:
**      Initialize the ADC for monitoring the current and voltages.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**	Industry_Pack
**
** GLOBALS REFERENCED:
**	tm_ADC128F1
**
**=========================================================================
*/
int
ADC128F1_initialize(unsigned char *addr, int occur)
{
  int i;
  struct IPACK ip;

  Industry_Pack (addr,SYSTRAN_ADC128F1,&ip);
  for (i=0;i<MAX_SLOTS;i++) 
    if (ip.adr[i]!=NULL)
    {
      if (occur==0)
      {	
        printf ("\r\nFound one at %d, %p",i,ip.adr[i]);
        tm_ADC128F1=ADC128F1Init((struct ADC128F1 *)ip.adr[i]);
	printf ("\r\n tm_ADC128F1=%d",tm_ADC128F1);
        ADC128F1_CVT_Update_Control(tm_ADC128F1,ENABLE);
	break;
      }
      else occur--;;
    }
  if (i>=MAX_SLOTS)
  {
    printf ("\r\n****Missing ADC128F1 at %p****\r\n",addr);
    return ERROR;
  }
  return 0;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_data_collection
**
** DESCRIPTION:
**      Collect the adc voltages and currents for the amplifiers.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	cw_ADC128F1
**	tm_ADC128F1
**
**=========================================================================
*/
void
tm_data_collection(void)
{
  short adc;

  if (cw_ADC128F1!=-1)
  {
    ADC128F1_Read_Reg(cw_ADC128F1,TM_ROT1VLT,&adc);
    if ((adc&0x800)==0x800) rot1vlt=adc|0xF000;
    else rot1vlt = adc&0xFFF;
    ADC128F1_Read_Reg(cw_ADC128F1,TM_ROT1CUR,&adc);
    if ((adc&0x800)==0x800) rot1cur=adc|0xF000;
    else rot1cur = adc&0xFFF;
  }
  if (tm_ADC128F1!=-1)
  {
    ADC128F1_Read_Reg(tm_ADC128F1,TM_AZ1VLT,&adc);
    if ((adc&0x800)==0x800) az1vlt=adc|0xF000;
    else az1vlt = adc&0xFFF;
    ADC128F1_Read_Reg(tm_ADC128F1,TM_AZ1CUR,&adc);
    if ((adc&0x800)==0x800) az1cur=adc|0xF000;
    else az1cur = adc&0xFFF;
    ADC128F1_Read_Reg(tm_ADC128F1,TM_AZ2VLT,&adc);
    if ((adc&0x800)==0x800) az2vlt=adc|0xF000;
    else az2vlt = adc&0xFFF;
    ADC128F1_Read_Reg(tm_ADC128F1,TM_AZ2CUR,&adc);
    if ((adc&0x800)==0x800) az2cur=adc|0xF000;
    else az2cur = adc&0xFFF;
      
    ADC128F1_Read_Reg(tm_ADC128F1,TM_ALT1VLT,&adc);
    if ((adc&0x800)==0x800) alt1vlt=adc|0xF000;
    else alt1vlt = adc&0xFFF;
    ADC128F1_Read_Reg(tm_ADC128F1,TM_ALT1CUR,&adc);
    if ((adc&0x800)==0x800) alt1cur=adc|0xF000;
    else alt1cur = adc&0xFFF;
    ADC128F1_Read_Reg(tm_ADC128F1,TM_ALT2VLT,&adc);
    if ((adc&0x800)==0x800) alt2vlt=adc|0xF000;
    else alt2vlt = adc&0xFFF;
    ADC128F1_Read_Reg(tm_ADC128F1,TM_ALT2CUR,&adc);
    if ((adc&0x800)==0x800) alt2cur=adc|0xF000;
    else alt2cur = adc&0xFFF;
  }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_read_all_adc
**
** DESCRIPTION:
**      Diagnostic which reads the two ADC's and 
**	print results a specified number of times.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	cw_ADC128F1
**	tm_ADC128F1
**
**=========================================================================
*/
void tm_read_all_adc(int cnt)
{
  int i,ii;
  short adc;

  for (i=0;i<cnt;i++)
  {
    for (ii=6;ii<8;ii++)
    {
      ADC128F1_Read_Reg(cw_ADC128F1,ii,&adc);
      if ((adc&0x800)==0x800) adc |= 0xF000;
      else adc &= 0xFFF;
      printf ("\r\nCW chan %d:  0x%x  %4.2fv  ",ii,adc,(10*adc)/2048.);
    }
    for (ii=0;ii<8;ii++)
    {
      ADC128F1_Read_Reg(tm_ADC128F1,ii,&adc);
      if ((adc&0x800)==0x800) adc |= 0xF000;
      else adc &= 0xFFF;
      printf ("\r\nTM chan %d:   0x%x  %4.2fv  ",ii,adc,(10*adc)/2048.);
    }
    printf ("\r\n");
  }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_az_brake
**	    tm_az_brake_on
**	    tm_az_brake_off
**	    tm_sp_az_brake_on
**	    tm_sp_az_brake_off
**
** DESCRIPTION:
**      Turn azimuth brakes off/on.  Function available to spawn the task
**	since return can take some time...useful for TCC which times out
**	reply.
**
** RETURN VALUES:
**      status
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	sdssdc
**	semSLC
**
**=========================================================================
*/
static int az_cnt;

int
tm_az_brake(short val) 
{
   int err;
   unsigned short ctrl;
   struct B10_0 tm_ctrl;   
             
   if (semTake (semSLC,60) == ERROR) {
      printf("tm_az_brake: unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,0,&ctrl,1);
   if(err) {
      printf ("R Err=%04x\r\n",err);
      semGive (semSLC);
      return err;
   }
   swab ((char *)&ctrl,(char *)&tm_ctrl,2);

   if(val==1) {
      tm_ctrl.mcp_az_brk_en_cmd = 1;
      tm_ctrl.mcp_az_brk_dis_cmd = 0;
   } else {
      tm_ctrl.mcp_az_brk_en_cmd = 0;
      tm_ctrl.mcp_az_brk_dis_cmd = 1;
   }

   swab ((char *)&tm_ctrl,(char *)&ctrl,2);
   err = slc_write_blok(1,10,BIT_FILE,0,&ctrl,1);
   semGive (semSLC);
   if(err) {
      printf ("W Err=%04x\r\n",err);
      return err;
   }
   
#ifdef TURNOFF
   if(val==1) {
      cnt=120;
      while(sdssdc.status.i9.il0.az_brake_en_stat == 0 && cnt > 0) {
	 taskDelay(1);
	 cnt--;
      }
/*   hold on the brake command */
   } else {
      cnt=60*6;
      while(sdssdc.status.i9.il0.az_brake_dis_stat == 0 && cnt > 0) {
	 taskDelay(1);
	 cnt--;
      }
      taskDelay(12*60);

      if(semTake(semSLC,60) == ERROR) {
	 printf("tm_az_brake: unable to take semaphore to write: %s",
		strerror(errno));
	 TRACE(0, "Unable to take semaphore: %d", errno, 0);
	 return(-1);
      }

      err = slc_read_blok(1,10,BIT_FILE,0,&ctrl,1);
      if(err) {
	 printf ("R Err=%04x\r\n",err);
	 semGive (semSLC);
	 return err;
      }
      swab ((char *)&ctrl,(char *)&tm_ctrl,2);
      
      tm_ctrl.mcp_az_brk_dis_cmd = 0;
      
      swab ((char *)&tm_ctrl,(char *)&ctrl,2);
      err = slc_write_blok(1,10,BIT_FILE,0,&ctrl,1);
      semGive (semSLC);
      if(err) {
	 printf("W Err=%04x\r\n",err);
	 return err;
      }
   }
   az_cnt=cnt;
#endif

   return 0;
}

void
tm_az_brake_on()
{
    tm_az_brake (1);
}

void
tm_az_brake_off()
{
    tm_az_brake (0);
}

void
tm_sp_az_brake_on()
{
  if (taskIdFigure("tmAzBrk")==ERROR)
    taskSpawn("tmAzBrk",90,0,2000,(FUNCPTR)tm_az_brake,1,0,0,0,0,0,0,0,0,0);
}

void
tm_sp_az_brake_off()
{
  if (taskIdFigure("tmAzBrk")==ERROR)
    taskSpawn("tmAzBrk",90,0,2000,(FUNCPTR)tm_az_brake,0,0,0,0,0,0,0,0,0,0);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_alt_brake
**	    tm_alt_brake_on
**	    tm_alt_brake_off
**	    tm_sp_alt_brake_on
**	    tm_sp_alt_brake_off
**
** DESCRIPTION:
**      Turn altitude brakes off/on.  Function available to spawn the task
**	since return can take some time...useful for TCC which times out
**	reply.
**
** RETURN VALUES:
**      status
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	sdssdc
**	semSLC
**
**=========================================================================
*/
int alt_cnt;
int tm_alt_brake(short val) 
{
   int err;
   unsigned short ctrl;
   struct B10_0 tm_ctrl;   

   TRACE(10, "Taking semaphore", 0, 0);
   if (semTake(semSLC,60) == ERROR) {
      printf("tm_alt_brake: unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }
   
   TRACE(10, "Reading blok", 0, 0);
   err = slc_read_blok(1,10,BIT_FILE,0,&ctrl,1);
   if(err) {
      printf ("R Err=%04x\r\n",err);
      semGive(semSLC);
      return err;
   }
   TRACE(10, "Read 0x%x", *(int *)&ctrl, 0);
   swab ((char *)&ctrl,(char *)&ctrl,2);
   TRACE(10, "Swapped to 0x%x", *(int *)&tm_ctrl, 0);

   if(val == 1) {
      tm_ctrl.mcp_alt_brk_en_cmd = 1;
      tm_ctrl.mcp_alt_brk_dis_cmd = 0;
   } else {
      tm_ctrl.mcp_alt_brk_en_cmd = 0;
      tm_ctrl.mcp_alt_brk_dis_cmd = 1;
   }

   TRACE(10, "Swapping back", 0, 0);

   swab ((char *)&tm_ctrl,(char *)&ctrl,2);
   err = slc_write_blok(1,10,BIT_FILE,0,&ctrl,1);
   semGive (semSLC);
   if(err) {
      printf ("W Err=%04x\r\n",err);
      return err;
   }

   TRACE(10, "returning", 0, 0);
#ifdef TURNOFF
   if(val == 1) {
      while(sdssdc.status.i9.il0.alt_brake_en_stat == 0 && cnt > 0) {
	 taskDelay(1);
	 cnt--;
      }
      /*   hold on the brake command */
   } else {
      cnt=60*4;
      while(sdssdc.status.i9.il0.alt_brake_dis_stat == 0 && cnt >0) {
	 taskDelay(1);
	 cnt--;
      }
      taskDelay(60*4);
      
      if(semTake (semSLC,60)== ERROR) {
	 printf("tm_alt_brake: unable to take semaphore for write: %s",
		strerror(errno));
	 TRACE(0, "Unable to take semaphore: %d", errno, 0);
	 return(-1);
      }
      
      err = slc_read_blok(1,10,BIT_FILE,0,&ctrl,1);
      if(err) {
	 printf ("R Err=%04x\r\n",err);
	 semGive (semSLC);
	 return err;
      }
      swab ((char *)&ctrl,(char *)&tm_ctrl,2);
      
      tm_ctrl.mcp_alt_brk_dis_cmd = 0;
      
      swab ((char *)&tm_ctrl,(char *)&ctrl,2);
      err = slc_write_blok(1,10,BIT_FILE,0,&ctrl,1);
      semGive(semSLC);
      if(err) {
	 printf ("W Err=%04x\r\n",err);
	 return err;
      }
   }
   alt_cnt=cnt;
#endif
   
   return 0;
}

void
tm_alt_brake_on()
{
    tm_alt_brake (1);
}

void
tm_alt_brake_off()
{
    tm_alt_brake (0);
}

void
tm_sp_alt_brake_on()
{
  if (taskIdFigure("tmAltBrk")==ERROR)
    taskSpawn("tmAltBrk",90,0,2000,(FUNCPTR)tm_alt_brake,1,0,0,0,0,0,0,0,0,0);
}

void
tm_sp_alt_brake_off()
{
   if (taskIdFigure("tmAltBrk")==ERROR)
     taskSpawn("tmAltBrk",90,0,2000,(FUNCPTR)tm_alt_brake,0,0,0,0,0,0,0,0,0,0);
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_brake_status
**
** DESCRIPTION:
**      Diagnostic for brake status.
**
** RETURN VALUES:
**      status
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	sdssdc
**	semSLC
**
**=========================================================================
*/
int
tm_brake_status()
{
  printf("\r\nAZ\tEngaged=%d\tDisengaged=%d, cnt=%d\n",
    sdssdc.status.i9.il0.az_brake_en_stat,
    sdssdc.status.i9.il0.az_brake_dis_stat,az_cnt);
  printf("\r\nALT\tEngaged=%d\tDisengaged=%d, cnt=%d\n",
    sdssdc.status.i9.il0.alt_brake_en_stat,
    sdssdc.status.i9.il0.alt_brake_dis_stat,alt_cnt);
  
#if 0
  int err;
  unsigned short ctrl;
  struct B10_0 tm_ctrl;   

  if(semTake(semSLC,60) == ERROR) {
     printf("tm_brake_status: unable to take semaphore: %s", strerror(errno));
     TRACE(0, "Unable to take semaphore: %d", errno, 0);
     return(-1);
  }
  
  err = slc_read_blok(1,10,BIT_FILE,0,&ctrl,1);
  semGive (semSLC);
  if(err) {
     printf ("R Err=%04x\r\n",err);
     return err;
  }
  swab((char *)&ctrl,(char *)&tm_ctrl,2);
#endif

  return 0;
}

/*=========================================================================
**
**      Check if amp is ok.
**
** GLOBALS REFERENCED:
**	sdssdc
**
**=========================================================================
*/
int
az_amp_ok(void)
{
   if((sdssdc.status.i6.il0.az_mtr_ccw_perm_in ||
				     sdssdc.status.i6.il0.az_mtr_cw_perm_in) &&
      sdssdc.status.i6.il0.az_plc_perm_in) {
      return TRUE;
   } else {
      return FALSE;
   }
}

int
alt_amp_ok(void)
{
   if((sdssdc.status.i6.il0.alt_mtr_dn_perm_in ||
				    sdssdc.status.i6.il0.alt_mtr_up_perm_in) &&
       sdssdc.status.i6.il0.alt_plc_perm_in) {
      return TRUE;
   } else {
      return FALSE;
   }
}

int
rot_amp_ok(void)
{
   if((sdssdc.status.i7.il0.rot_mtr_ccw_perm_in ||
				    sdssdc.status.i7.il0.rot_mtr_cw_perm_in) &&
      sdssdc.status.i7.il0.rot_plc_perm_in) {
      return TRUE;
   } else {
      return FALSE;
   }
}
/*=========================================================================
**
**     Shutdown the telescope by turning on the brakes due to a software reboot
**     Check the amplifiers to see if fault and need to turn brakes on.
**     Must allow for restart...reason for monitor boolean only turned on
**     by tm_controller_run.
**     Keep the watchdog timer on which enables the amplifiers.
**
** CALLS TO:
**	tm_amp_engage
**
** GLOBALS REFERENCED:
**	sddsdc
**	monitor_axis
**
*/
#define TM_WD		4		/* WD channel    15 */

void
mgt_shutdown(int type)
{
   printf("mgt: Safely halt the telescope by braking AZ and ALT\n");
   tm_az_brake_on();
   tm_alt_brake_on();
}

void
tm_amp_mgt(void)
{
   int state;
   
   monitor_axis[AZIMUTH] = monitor_axis[ALTITUDE] =
					      monitor_axis[INSTRUMENT] = FALSE;
   monitor_on[AZIMUTH] = monitor_on[ALTITUDE] = monitor_on[INSTRUMENT] = TRUE;
   rebootHookAdd((FUNCPTR)mgt_shutdown);
   
   for(;;) {
      taskDelay (30);
      tm_amp_engage();		/* keep amps alive */
      
      if(monitor_on[AZIMUTH]) {
	 if(monitor_axis[AZIMUTH] && sdssdc.status.i9.il0.az_brake_dis_stat) {
	    if((state = tm_axis_state(2*AZIMUTH)) > 2 && state != STOP_EVENT) {
	       TRACE(0, "MGT: bad az state %d", state, 0);
	       printf("MGT: bad az state %d\n", state);
	       tm_az_brake_on();
	       monitor_axis[AZIMUTH] = FALSE;
	    }
	    if(!az_amp_ok()) {
	       TRACE(0, "MGT: bad az amp", 0, 0);
	       printf("MGT: bad az amp\n");
	       tm_controller_idle(2*AZIMUTH);
	       tm_az_brake_on();
	       monitor_axis[AZIMUTH]=FALSE;
	    }
	 }
      }

      if(monitor_on[ALTITUDE]) {
	 if(monitor_axis[ALTITUDE] &&
	    sdssdc.status.i9.il0.alt_brake_dis_stat) {
	    if((state = tm_axis_state(2*ALTITUDE)) > 2 && state != STOP_EVENT){
	       TRACE(0, "MGT: bad alt state %d", state, 0);
	       printf("MGT: bad alt state %d\n", state);
	       tm_alt_brake_on();
	       monitor_axis[ALTITUDE] = FALSE;
	    }
	    if(!alt_amp_ok()) {
	       TRACE(0, "MGT: bad alt amp", 0, 0);
	       printf("MGT: bad alt amp\n");
	       tm_controller_idle(2*ALTITUDE);
	       tm_alt_brake_on();
	       monitor_axis[ALTITUDE] = FALSE;
	    }
	 }
      }
     
      if(monitor_on[INSTRUMENT]) {
	 if(monitor_axis[INSTRUMENT]) {
	    if(!rot_amp_ok()) {
	       TRACE(0, "MGT: bad rot amp", 0, 0);
	       printf("MGT: bad rot amp\n");
	       tm_controller_idle(2*INSTRUMENT);
	       monitor_axis[INSTRUMENT] = FALSE;
	    }
	 }
      }
   }
}

/*=========================================================================
**
**      Print the amp status.
**
*/
void
tm_print_amp_status(void)
{
    if(az_amp_ok()) {
       printf("AZ Amp OK\n");
    } else {
       printf ("Az Amp Disengaged: az_mtr_ccw_perm=%d,az_mtr_cw_perm=%d\n",
	       sdssdc.status.o11.ol0.az_mtr_ccw_perm,
	       sdssdc.status.o11.ol0.az_mtr_cw_perm);
    }

    if(alt_amp_ok()) {
       printf("ALT Amp OK\n");
    } else {
       printf("Alt Amp Disengaged: alt_mtr_dn_perm=%d,alt_mtr_up_perm=%d\n",
	      sdssdc.status.o11.ol0.alt_mtr_dn_perm,
	      sdssdc.status.o11.ol0.alt_mtr_up_perm);
    }

    if(rot_amp_ok()) {
       printf("ROT Amp OK\n");
    } else {
       printf("Rot Amp Disengaged: "
	      "rot_mtr_rdy=%d,rot_mtr_ccw_perm=%d,rot_mtr_cw_perm=%d\n",
	      sdssdc.status.i8.il0.rot_mtr_rdy,
	      sdssdc.status.o11.ol0.rot_mtr_ccw_perm,
	      sdssdc.status.o11.ol0.rot_mtr_cw_perm);
    }
}

/*=========================================================================
**
**      Enable/disable the amp utilizing the watchdog which is setup with
**	the initialization routine.
*/
void
tm_amp_disengage(void)
{
  StopCounter (&sbrd,TM_WD);
}

void
tm_amp_engage(void)
{
   if((axis_alive & 0x7) == 0x7) {
      WriteCounterConstant(&sbrd, TM_WD);		/* 2 Sec */
      StartCounter(&sbrd, TM_WD);
#if 0
      axis_alive = 0;			/* requires each task to actively
					   set the bits */
#endif
   }
}

void
tm_setup_wd(void)
{
   SetCounterSize(&sbrd, TM_WD, CtrSize32);
   SetCounterConstant(&sbrd, TM_WD, 2000000);		/* 2 Sec */
   SetMode(&sbrd, TM_WD, Watchdog);
   SetDebounce(&sbrd, TM_WD, DebounceOff);
   SetInterruptEnable(&sbrd, TM_WD, IntEnable);
   SetClockSource(&sbrd, TM_WD, InC1Mhz);
   SetTriggerSource(&sbrd, TM_WD, InTrig);
   SetWatchdogLoad(&sbrd, TM_WD, WDIntLd);
   SetOutputPolarity(&sbrd, TM_WD, OutPolLow);
   ConfigureCounterTimer(&sbrd, TM_WD);
}

/*=========================================================================
**
**      Telescope motion prints status of axis_status funciton.
**
*/
char const* const msg_axis_status[] = {
   "IN_SEQUENCE",
   "IN_POSITION",
   "IN_MOTION",
   "DIRECTION positive",
   "FRAMES_LEFT"
};

int
tm_axis_status(int axis)
{
   int value;
   
   semTake(semMEI,WAIT_FOREVER);
   value = axis_status(axis);
   semGive(semMEI);

   return value;
}

void
tm_print_axis_status(int axis)
{
   int i,value;
   
   value = tm_axis_status(axis);
   printf("AXIS STATUS: 0x%x",value);
   for(i = 0; i < sizeof(msg_axis_status)/sizeof(char *); i++) {
      if((value >> (i + 4)) & 1) {
	 printf("     %s\r\n",msg_axis_status[i]);
      }
  }
}

/*=========================================================================
**
**      Telescope motion prints status of axis_state funciton.
**
**=========================================================================
*/
char const* const msg_axis_state[] = {
   "NO_EVENT",
   "NEW_FRAME",
   "STOP_EVENT",
   "E_STOP_EVENT",
   "ABORT_EVENT",
   "Running???",
   "Undocumented Value"
};

int
tm_axis_state(int axis)
{
  int value;

  semTake(semMEI, WAIT_FOREVER);
  value = axis_state(axis);
  semGive(semMEI);

  return value;
}

void
tm_print_axis_state(int axis)
{
   int i,value;
   
   value = tm_axis_state(axis);
   printf("AXIS STATE: 0x%x", value);
   
   switch (value) {
    case NO_EVENT:
      i=0;
      break;
    case NEW_FRAME:
      i=1;
      break;
    case STOP_EVENT:
      i=2;
      break;
    case E_STOP_EVENT:
      i=3;
      break;
    case ABORT_EVENT:
      i=4;
      break;
    case 1:
      i=5;
      break;
    default:
      i=6;
   }

   printf ("     %s\n",msg_axis_state[i]);
}
/*=========================================================================
**
**      Telescope motion prints status of axis_source funciton.
**
**=========================================================================
*/
char const* const msg_axis_source[] = {
   "ID_NONE",
   "ID_HOME_SWITCH",
   "ID_POS_LIMIT",
   "ID_NEG_LIMIT",
   "ID_AMP_FAULT",
   "unused",
   "unused",
   "ID_X_NEG_LIMIT",
   "ID_X_POS_LIMIT",
   "ID_ERROR_LIMIT",
   "ID_PC_COMMAND",
   "ID_OUT_OF_FRAMES",
   "ID_TEMPO_PROBE_FAULT",
   "ID_AXIS_COMMAND"
};

void
tm_print_axis_source(int axis)
{
   int value;
   
   semTake(semMEI,WAIT_FOREVER);
   value=axis_source(axis);
   semGive(semMEI);
   printf ("AXIS SOURCE: %x",value);
   printf ("     %s\r\n",msg_axis_source[value]);
}                                                              
 
