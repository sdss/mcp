#include "copyright.h"
/************************************************************************/
/* Project: 	SDSS - Sloan Digital Sky Survey				*/
/* 		AXIS control						*/
/************************************************************************/

/*------------------------------*/
/*	includes		*/
/*------------------------------*/
#include "vxWorks.h"                            
#include "stdio.h"
#include <errno.h>
#include "semLib.h"
#include "sigLib.h"
#include "tickLib.h"
#include "taskLib.h"
#include "usrLib.h"
#include "string.h"
#include "inetLib.h"
#include "rebootLib.h"
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
/* tm_mgt variables to enable safe operation if telescope is not in control */
int monitor_axis[3];	/* provides mechanism to prevent deadlocks for startup */
int monitor_on[3];	/* overrides monitoring of axis...possible disable */
/* keep watch dog alive based on all axis software active */
int axis_alive=0;
double ilcpos[6]={120.6,0,90.0,0,0,0};
int ilcvel[6]={200000,0,200000,0,500000,0};
int ilcacc[6]={10000,10000,10000,10000,10000,10000};

/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_move_instchange
**
** DESCRIPTION:
**      Telescope motion to the instrument change position.  The routine 
**	returns to 
**	the caller immediately while the motion is being fulfilled.
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
int tm_move_instchange ()
{
  if (semTake (semMEI,60)!=ERROR)
  {
    sem_controller_run (0);
    sem_controller_run (2);
    sem_controller_run (4);
    start_move(0,(double)(ilcpos[0]*ticks_per_degree[0]),
		(double)ilcvel[0],(double)ilcacc[0]);
    start_move(2,(double)(ilcpos[2]*ticks_per_degree[1]),
		(double)ilcvel[2],(double)ilcacc[2]);
    start_move(4,(double)(ilcpos[4]*ticks_per_degree[2]),
		(double)ilcvel[4],(double)ilcacc[4]);
    semGive (semMEI); 
  }
  else
  {
    printf("Err: Could not take semMEI semphore     ");
    return ERROR;
  }
  return 0;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_start_move
**
** DESCRIPTION:
**      Telescope motion to a specified position.  The routine returns to 
**	the caller immediately while the motion is being fulfilled.
**	Encaspulates the MEI function and converts args from ints to doubles.
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
void tm_start_move (int axis, int vel, int accel, int pos)
{
	semTake(semMEI,WAIT_FOREVER);
	start_move(axis,(double)pos,(double)vel,(double)accel);
	semGive (semMEI);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_bf
**
** DESCRIPTION:
**      Telescope motion to go over a fiducial point back-and-forth a
**	specified number of times.
**	Boroski spec.
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
void tm_bf (int axis, int vel, int accel, int pos1,int pos2, int times)
{
  int i;
  int status;

  printf ("\r\nPass ");
  for (i=0;i<times;i++)
  {
    printf ("\r\n%d ",i);
    tm_controller_run (axis);
    tm_start_move (axis,vel,accel,pos1);
    status=FALSE;
    while (!status)
    {
      taskDelay(60);
      semTake(semMEI,WAIT_FOREVER);
      status=motion_done(axis);
      semGive (semMEI);
    }
    tm_start_move (axis,vel,accel,pos2);
    status=FALSE;
    while (!status)
    {
      taskDelay(60);
      semTake(semMEI,WAIT_FOREVER);
      status=motion_done(axis);
      semGive (semMEI);
    }
  }
  printf(" Done");
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_print_coeffs
**
** DESCRIPTION:
**      Print the specified axis PID coefficients and sample rate.
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
void tm_print_coeffs(int axis)
{
	short coeff[COEFFICIENTS];
	short mode;
	short rate;

	semTake(semMEI,WAIT_FOREVER);
	get_filter (axis,(P_INT)coeff);
	get_integration (axis,&mode);
	rate=dsp_sample_rate();
	semGive (semMEI);
	printf ("\r\n AXIS %d: P=%d, I=%d, D=%d",axis,
		coeff[0],coeff[1],coeff[2]);
	printf ("\r\n          AFF=%d, VFF=%d, FFF=%d",
		coeff[3],coeff[4],coeff[9]);
	printf ("\r\n          ILIMIT=%d, OFFSET=%d, OLIMIT=%d, SHIFT=%d",
		coeff[5],coeff[6],coeff[7],coeff[8]);
	printf ("\r\n integration mode is %d",mode);
	printf ("\r\n and sample is %d Hz",rate);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_set_coeffs 
**
** DESCRIPTION:
**      Set the specified axis coefficient where index is used to specify
**	one of 10 possible coefficients.
**		index=0(P),1(I),2(D),3(AFF),4(VFF),5(ILIM),6(OFF),7(DLIM)
**		8(SHIFT)(-5 is 1/32),9(FFF)
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
void tm_set_coeffs(int axis, int index, int val)
{
	short coeff[COEFFICIENTS];

	semTake(semMEI,WAIT_FOREVER);
	get_filter (axis,(P_INT)coeff);
	coeff[index]=val;
	set_filter (axis,(P_INT)coeff);
	semGive (semMEI);
}
/*=========================================================================
**=========================================================================
**
**      Telescope motion encapsulates with a semaphore the MEI function.
**
**=========================================================================
*/
void tm_clear_pos (int axis)
{
	semTake(semMEI,WAIT_FOREVER);
	set_position (axis,0.0);
	semGive(semMEI);
}
void tm_get_pos (int axis,double *position)
{
	semTake(semMEI,WAIT_FOREVER);
        get_position(axis,position);
	semGive(semMEI);
}
void tm_get_vel (int axis,double *velocity)
{
	semTake(semMEI,WAIT_FOREVER);
        get_velocity(axis,velocity);
	semGive(semMEI);
}
void tm_set_sample_rate (unsigned short rate)
{
	semTake(semMEI,WAIT_FOREVER);
	set_sample_rate (rate);
	printf("\r\n Sample Rate=%d",(unsigned short)dsp_sample_rate());
	semGive(semMEI);
}
void tm_reset_integrator (int axis)
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
  
void tm_set_encoder(int axis)
{
	semTake(semMEI,WAIT_FOREVER);
	set_feedback(axis,FB_ENCODER);
	semGive(semMEI);
}
void tm_dual_loop (int axis, int dual)
{
	semTake(semMEI,WAIT_FOREVER);
	set_dual_loop (axis,axis+1,dual);
	semGive(semMEI);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_set_analog_encoder
**
** DESCRIPTION:
**      Telescope motion sets up the tachometer as the encoder.  No longer
**	used at this time.
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
void tm_set_analog_encoder(int axis, int channel)
{
	semTake(semMEI,WAIT_FOREVER);
	set_analog_channel(axis,channel,TRUE,TRUE);
	set_axis_analog (axis,TRUE);
	set_feedback(axis,FB_ANALOG);
	semGive(semMEI);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_set_analog_channel
**
** DESCRIPTION:
**      Telescope motion sets analog channel as feedback to PID.  Not used.
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
void tm_set_analog_channel(int axis, int channel)
{
	semTake(semMEI,WAIT_FOREVER);
	set_analog_channel(axis,channel,TRUE,TRUE);
	set_axis_analog (axis,TRUE);
	semGive(semMEI);
}
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
void tm_controller_idle (int axis)
{
	int retry;

	retry=6;
	semTake(semMEI,WAIT_FOREVER);
  	while ((axis_state(axis)<=2)&&(--retry>0)) 
	  controller_idle (axis);
	semGive(semMEI);
}
void sem_controller_idle (int axis)
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
void tm_data_collection()
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
**=========================================================================
**
** ROUTINE: tm_clamp
**	    tm_clamp_on
**	    tm_clamp_off
**	    tm_sp_clamp_on
**	    tm_sp_clamp_off
**
** DESCRIPTION:
**      Turn on/off the instrument change clamp.
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
int clamp_cnt;
int
tm_clamp(short val) 
{
   int err;
   unsigned short ctrl[2];
   struct B10_0 tm_ctrl;   
   struct B10_1 tm_ctrl1;   
   int cnt;
             
   if(semTake(semSLC,60) == ERROR) {
      printf("Unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,0,&ctrl[0],2);
   if(err) {
      semGive (semSLC);
      printf ("R Err=%04x\r\n",err);
      return err;
   }
   swab ((char *)&ctrl[0],(char *)&tm_ctrl,2);
   swab ((char *)&ctrl[1],(char *)&tm_ctrl1,2);

   if(val == 1) {
      tm_ctrl.mcp_clamp_engage_cmd = 1;
      tm_ctrl1.mcp_clamp_disen_cmd = 0;
   } else {
      tm_ctrl.mcp_clamp_engage_cmd = 0;
      tm_ctrl1.mcp_clamp_disen_cmd = 1;
   }
   
   swab((char *)&tm_ctrl, (char *)&ctrl[0],2);
   swab((char *)&tm_ctrl1,(char *)&ctrl[1],2);
   err = slc_write_blok(1,10,BIT_FILE,0,&ctrl[0],2);
   semGive (semSLC);
   
   if(err) {
      printf ("W Err=%04x\r\n",err);
      return err;
   }
   
   if(val == 0) {
      return 0;
   }

   cnt=60*15;				/* wait 15s */
   while(sdssdc.status.i9.il0.clamp_en_stat == 0 && cnt > 0) {
      taskDelay(1);
      cnt--;
   }
   clamp_cnt = cnt;

   if (sdssdc.status.i9.il0.clamp_en_stat == 1) { /* success */
      return -1;
   }
/*
 * Failure; turn off clamp and disengage
 */
   printf ("\r\n Clamp did NOT engage...turning off and disengaging ");
   if(semTake(semSLC,60) == ERROR) {
      printf("Unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,0,&ctrl[0],2);
   if(err) {
      semGive (semSLC);
      printf ("R2 Err=%04x\r\n",err);
      return err;
   }
   swab ((char *)&ctrl[0],(char *)&tm_ctrl,2);
   swab ((char *)&ctrl[1],(char *)&tm_ctrl1,2);

   tm_ctrl.mcp_clamp_engage_cmd = 0;
   tm_ctrl1.mcp_clamp_disen_cmd = 1;
   
   swab((char *)&tm_ctrl,(char *)&ctrl[0],2);
   swab((char *)&tm_ctrl1,(char *)&ctrl[1],2);
   
   err = slc_write_blok(1,10,BIT_FILE,0,&ctrl[0],2);
   semGive(semSLC);
   if(err) {
      printf ("W Err=%04x\r\n",err);
      return err;
   }

   return 0;
}
void tm_clamp_on()
{
    tm_clamp (1);
}
void tm_clamp_off()
{
    tm_clamp (0);
}
void tm_sp_clamp_on()
{
  if (taskIdFigure("tmClamp")==ERROR)
    taskSpawn("tmClamp",90,0,2000,(FUNCPTR)tm_clamp,1,0,0,0,0,0,0,0,0,0);
}
void tm_sp_clamp_off()
{
  if (taskIdFigure("tmClamp")==ERROR)
    taskSpawn("tmClamp",90,0,2000,(FUNCPTR)tm_clamp,0,0,0,0,0,0,0,0,0,0);
}
int tm_clamp_status()
{
   int err;
   unsigned short ctrl[2],sctrl[2];
   
   if(semTake(semSLC,60) == ERROR) {
      printf("Unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,0,&ctrl[0],2);
   semGive (semSLC);
   if(err) {
      printf ("R Err=%04x\r\n",err);
      return err;
   }
   
   swab ((char *)&ctrl[0],(char *)&sctrl[0],2);
   swab ((char *)&ctrl[1],(char *)&sctrl[1],2);
   
   printf (" read ctrl = 0x%4x 0x%4x\r\n",sctrl[0],sctrl[1]);
   
   return 0;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_slit
**	    tm_slit_clear
**	    tm_slit_open
**	    tm_slit_close
**	    tm_sp_slit_open
**	    tm_sp_slit_close
**
** DESCRIPTION:
**      Open/close/clear the slit door.  Clear neither closes nor opens the
**	door, the actuator is inactive.  There are two spectographs so the
**	door must be specified.
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
tm_slit(short val) 
{
   int err;
   unsigned short ctrl[1];
   struct B10_1 tm_ctrl1;   
             
   if(semTake(semSLC,60) == ERROR) {
      printf("tm_slit: unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   if(err) {
      semGive (semSLC);
      printf ("R Err=%04x\r\n",err);
      return err;
   }
   swab ((char *)&ctrl[0],(char *)&tm_ctrl1,2);

   switch (val) {
    case 5:
      tm_ctrl1.mcp_slit_dr2_opn_cmd = 0;
      tm_ctrl1.mcp_slit_dr2_cls_cmd = 0;
      break;
    case 4:
      tm_ctrl1.mcp_slit_dr2_opn_cmd = 1;
      tm_ctrl1.mcp_slit_dr2_cls_cmd = 0;
      break;
    case 3:
      tm_ctrl1.mcp_slit_dr2_opn_cmd = 0;
      tm_ctrl1.mcp_slit_dr2_cls_cmd = 1;
      break;
    case 2:
      tm_ctrl1.mcp_slit_dr1_opn_cmd = 0;
      tm_ctrl1.mcp_slit_dr1_cls_cmd = 0;
      break;
    case 1:
      tm_ctrl1.mcp_slit_dr1_opn_cmd = 1;
      tm_ctrl1.mcp_slit_dr1_cls_cmd = 0;
      break;
    case 0:
      tm_ctrl1.mcp_slit_dr1_opn_cmd = 0;
      tm_ctrl1.mcp_slit_dr1_cls_cmd = 1;
      break;
   }

   swab((char *)&tm_ctrl1,(char *)&ctrl[0],2);

   err = slc_write_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   semGive (semSLC);
   if(err) {
      printf("W Err=%04x\r\n",err);
      return err;
   }

   return 0;
}
void tm_slit_clear(int door)
{
    tm_slit (2+(door*3));
}
void tm_slit_open(int door)
{
    tm_slit (1+(door*3));
}
void tm_slit_close(int door)
{
    tm_slit (0+(door*3));
}

void
tm_sp_slit_clear(int door)
{
   if(taskIdFigure("tmSlit") == ERROR) {
      taskSpawn("tmSlit",90,0,2000,
		(FUNCPTR)tm_slit_clear, door,
		0,0,0,0,0,0,0,0,0);
   }
}

void
tm_sp_slit_open(int door)
{
   if(taskIdFigure("tmSlit") == ERROR) {
      taskSpawn("tmSlit",90,0,2000,
		(FUNCPTR)tm_slit_open, door,
		0,0,0,0,0,0,0,0,0);
   }
}

void
tm_sp_slit_close(int door)
{
   if(taskIdFigure("tmSlit") == ERROR) {
      taskSpawn("tmSlit",90,0,2000,
		(FUNCPTR)tm_slit_close, door,
		0,0,0,0,0,0,0,0,0);
   }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_cart
**	    tm_cart_latch
**	    tm_cart_unlatch
**	    tm_sp_cart_latch
**	    tm_sp_cart_unlatch
*
** DESCRIPTION:
**      Latch/unlatch the fiber cartridge latch for the selected spectograph
**	specified by the door.
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
tm_cart(short val) 
{
   int err;
   unsigned short ctrl[1];
   struct B10_1 tm_ctrl1;   
             
   if(semTake (semSLC,60) == ERROR) {
      printf("tm_cart: unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   if(err) {
      semGive (semSLC);
      printf ("R Err=%04x\r\n",err);
      return err;
   }
   swab ((char *)&ctrl[0],(char *)&tm_ctrl1,2);

   switch (val) {
    case 3:
      tm_ctrl1.mcp_slit_latch2_cmd = 1;
      break;
    case 2:
      tm_ctrl1.mcp_slit_latch2_cmd = 0;
      break;
    case 1:
      tm_ctrl1.mcp_slit_latch1_cmd = 1;
      break;
    case 0:
      tm_ctrl1.mcp_slit_latch1_cmd = 0;
      break;
   }
   
   swab ((char *)&tm_ctrl1,(char *)&ctrl[0],2);
   err = slc_write_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   semGive (semSLC);
   if(err) {
      printf ("W Err=%04x\r\n",err);
      return err;
   }

   return 0;
}
void tm_cart_latch(int door)
{
    tm_cart (1+(door*2));
}
void tm_cart_unlatch(int door)
{
    tm_cart (0+(door*2));
}
void tm_sp_cart_latch(int door)
{
  if (taskIdFigure("tmCart")==ERROR)
    taskSpawn("tmCart",90,0,1000,(FUNCPTR)tm_cart_latch,door,0,0,0,0,0,0,0,0,0);
}
void tm_sp_cart_unlatch(int door)
{
  if (taskIdFigure("tmCart")==ERROR)
    taskSpawn("tmCart",90,0,1000,(FUNCPTR)tm_cart_unlatch,door,0,0,0,0,0,0,0,0,0);
}

int
tm_slit_status()
{
   int err;
   unsigned short ctrl[1];
   struct B10_1 tm_ctrl1;   

   if(semTake(semSLC,60) == ERROR) {
      printf("tm_slit_status: unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   semGive (semSLC);
   if (err) {
      printf ("R Err=%04x\r\n",err);
      return err;
   }
   swab ((char *)&ctrl[0],(char *)&tm_ctrl1,2);
   
  printf (" read ctrl = 0x%04x\r\n",(unsigned int)ctrl);
  printf ("\r\n mcp_slit_dr1_opn_cmd=%d, mcp_slit_dr1_cls_cmd=%d",
     tm_ctrl1.mcp_slit_dr1_opn_cmd,tm_ctrl1.mcp_slit_dr1_cls_cmd);
  printf ("\r\n mcp_slit_dr2_opn_cmd=%d, mcp_slit_dr2_cls_cmd=%d",
     tm_ctrl1.mcp_slit_dr2_opn_cmd,tm_ctrl1.mcp_slit_dr2_cls_cmd);
  printf ("\r\n mcp_slit_latch1_cmd=%d, mcp_slit_latch2_cmd=%d",
     tm_ctrl1.mcp_slit_latch1_cmd,tm_ctrl1.mcp_slit_latch2_cmd);

  printf ("\r\n slit_door1_opn=%d, slit_door1_cls=%d, cart_latch1_opn=%d",
	sdssdc.status.i1.il9.slit_head_door1_opn,
	sdssdc.status.i1.il9.slit_head_door1_cls,
	sdssdc.status.i1.il9.slit_head_latch1_ext);
  printf ("\r\n slit_door2_opn=%d, slit_door2_cls=%d, cart_latch2_opn=%d",
	sdssdc.status.i1.il9.slit_head_door2_opn,
	sdssdc.status.i1.il9.slit_head_door2_cls,
	sdssdc.status.i1.il9.slit_head_latch2_ext);
  printf ("\r\n slit_dr1_ext_perm=%d, slit_dr1_cls_perm=%d, slit_latch1_ext_perm=%d",
	sdssdc.status.o1.ol9.slit_dr1_opn_perm,
	sdssdc.status.o1.ol9.slit_dr1_cls_perm,
	sdssdc.status.o1.ol9.slit_latch1_ext_perm);
  printf ("\r\n slit_dr2_opn_perm=%d, slit_dr2_cls_perm=%d, slit_latch2_ext_perm=%d",
	sdssdc.status.o1.ol9.slit_dr2_opn_perm,
	sdssdc.status.o1.ol9.slit_dr2_cls_perm,
	sdssdc.status.o1.ol9.slit_latch2_ext_perm);

  return 0;
}

/*
 * Commands to control the spectrograph doors/latches
 */
int
mcp_slit_clear(int spec)
{
   if(spec != SPECTOGRAPH1 && spec != SPECTOGRAPH2) {
      return(-1);
   }
   
   tm_sp_slit_clear(spec);

   return(0);
}

int
mcp_slit_open(int spec)
{
   if(spec != SPECTOGRAPH1 && spec != SPECTOGRAPH2) {
      return(-1);
   }
   
   tm_sp_slit_open(spec);

   return(0);
}

int
mcp_slit_close(int spec)
{
   if(spec != SPECTOGRAPH1 && spec != SPECTOGRAPH2) {
      return(-1);
   }
   
   tm_sp_slit_close(spec);

   return(0);
}

int
mcp_slithead_latch_open(int spec)
{
   if(spec != SPECTOGRAPH1 && spec != SPECTOGRAPH2) {
      return(-1);
   }
   
   tm_cart_unlatch(spec);

   return(0);
}

int
mcp_slithead_latch_close(int spec)
{
   if(spec != SPECTOGRAPH1 && spec != SPECTOGRAPH2) {
      return(-1);
   }
   
   tm_cart_latch(spec);

   return(0);
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_ffs
**	    tm_ffs_open
**	    tm_ffs_close
**          tm_ffs_enable
**	    tm_sp_ffs_move
**	    tm_ffs_open_status
**	    tm_ffs_close_status
**
** DESCRIPTION:
**      Open/close the flat field screen
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
/*
 * Set the FFS control bits. If val is < 0, it isn't set
 */
static int
set_mcp_ffs_bits(int val,		/* value of mcp_ff_scrn_opn_cmd */
		 int enab)		/* value of mcp_ff_screen_enable */
{
   unsigned short ctrl[1];
   struct B10_1 tm_ctrl1;   
   int err;
             
   if(semTake(semSLC,60) == ERROR) {
      printf("Unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   if(err) {
      semGive (semSLC);
      printf("R Err=%04x\r\n",err);
      return err;
   }
   swab((char *)&ctrl[0], (char *)&tm_ctrl1, 2);

   if(val >= 0) {
      tm_ctrl1.mcp_ff_scrn_opn_cmd = val;
   }
   tm_ctrl1.mcp_ff_screen_enable = enab;

   swab ((char *)&tm_ctrl1,(char *)&ctrl[0],2);
   err = slc_write_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   semGive (semSLC);

   if(err) {
      printf ("W Err=%04x\r\n",err);
      return err;
   }

   return 0;
}

/*
 * Open or close the FF screen
 */
int
tm_ffs(short val)			/* FFS_CLOSE or FFS_OPEN */
{
   int cnt;
   int err;
   int failed = 0;			/* did operation fail? */
   int wait_time = 30;			/* FF screen timeout (seconds) */

   if(set_mcp_ffs_bits(val, 1) != 0) {
      return(err);
   }

   cnt=60*wait_time;
   printf("Waiting up to %ds for flat field to move\n", wait_time);
   if(val == 1) {
      while(!tm_ffs_open_status() && cnt > 0) {
	 taskDelay(1);
	 cnt--;
      }
      if(!tm_ffs_open_status()) {	/* did not work */
	 failed = 1;
	 TRACE(0, "FFS did NOT all open", 0, 0);
	 printf("\r\n FFS did NOT all open ");
      }
   } else {
      while(!tm_ffs_close_status() && cnt > 0) {
	 taskDelay(1);
	 cnt--;
      }
      if(!tm_ffs_close_status()) {	/* did not work */
	 failed = 1;
	 TRACE(0, "FFS did NOT all close", 0, 0);
	 printf("\r\n FFS did NOT all close ");
      }
   }
   
   if(failed) {
      return(set_mcp_ffs_bits(!val, 1)); /* don't leave command pending */
   } else {
      return 0;
   }
}

/*
 * Enable the flat field screen
 */
int
tm_ffs_enable(int val)
{
   return(set_mcp_ffs_bits(-1, 1));
}

void
tm_ffs_open(void)
{
   tm_ffs(FFS_OPEN);
}

void
tm_ffs_close(void)
{
   tm_ffs(FFS_CLOSE);
}

void
tm_sp_ffs_move(int open_close)		/* FFS_CLOSE or FFS_OPEN */
{
   if(taskIdFigure("tmFFS") == ERROR) {
      taskSpawn("tmFFS", 90, 0, 2000, (FUNCPTR)tm_ffs, open_close,
		0,0,0,0,0,0,0,0,0);
   } else {
      TRACE(0, "Task tmFFS is already active; not moving FF screen (%d)",
	    open_close, 0);
   }
}

int
tm_ffs_open_status(void)
{
  if ((sdssdc.status.i1.il13.leaf_1_open_stat)&&
	(sdssdc.status.i1.il13.leaf_2_open_stat)&&
	(sdssdc.status.i1.il13.leaf_3_open_stat)&&
	(sdssdc.status.i1.il13.leaf_4_open_stat)&&
	(sdssdc.status.i1.il13.leaf_5_open_stat)&&
	(sdssdc.status.i1.il13.leaf_6_open_stat)&&
	(sdssdc.status.i1.il13.leaf_7_open_stat)&&
	(sdssdc.status.i1.il13.leaf_8_open_stat))
    return TRUE;
  else 
    return FALSE;
}

int
tm_ffs_close_status(void)
{
  if ((sdssdc.status.i1.il13.leaf_1_closed_stat)&&
 	(sdssdc.status.i1.il13.leaf_2_closed_stat)&&
	(sdssdc.status.i1.il13.leaf_3_closed_stat)&&
	(sdssdc.status.i1.il13.leaf_4_closed_stat)&&
	(sdssdc.status.i1.il13.leaf_5_closed_stat)&&
	(sdssdc.status.i1.il13.leaf_6_closed_stat)&&
	(sdssdc.status.i1.il13.leaf_7_closed_stat)&&
	(sdssdc.status.i1.il13.leaf_8_closed_stat)) 
    return TRUE;
  else 
    return FALSE;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_ffl
**	    tm_ffl_on
**	    tm_ffl_off
**	    tm_sp_ffl_on
**	    tm_sp_ffl_off
**
** DESCRIPTION:
**      Turn on/off the incandescent lamps for the flat field.
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
tm_ffl(short val)
{
   int err;
   unsigned short ctrl[1];
   struct B10_1 tm_ctrl1;   
             
   if(semTake(semSLC,60) == ERROR) {
      printf("Unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   if(err) {
      semGive (semSLC);
      printf ("R Err=%04x\r\n",err);
      return err;
   }
   swab ((char *)&ctrl[0],(char *)&tm_ctrl1,2);

   tm_ctrl1.mcp_ff_lamp_on_cmd = val;

   swab ((char *)&tm_ctrl1,(char *)&ctrl[0],2);
   err = slc_write_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   semGive (semSLC);

   if(err) {
      printf ("W Err=%04x\r\n",err);
      return err;
   }

   return 0;
}
void tm_ffl_on()
{
   tm_ffl(1);
}
void tm_ffl_off()
{
   tm_ffl(0);
}
void tm_sp_ffl_on()
{
  if (taskIdFigure("tmFFL")==ERROR)
    taskSpawn("tmFFL",90,0,1000,(FUNCPTR)tm_ffl,1,0,0,0,0,0,0,0,0,0);
}
void tm_sp_ffl_off()
{
  if (taskIdFigure("tmFFL")==ERROR)
    taskSpawn("tmFFL",90,0,1000,(FUNCPTR)tm_ffl,0,0,0,0,0,0,0,0,0,0);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_neon
**	    tm_neon_on
**	    tm_neon_off
**	    tm_sp_neon_on
**	    tm_sp_neon_off
**
** DESCRIPTION:
**      Turn on/off the Neon lamps for the flat field.
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
tm_neon(short val)
{
   int err;
   unsigned short ctrl[1];
   struct B10_1 tm_ctrl1;   
             
   if(semTake(semSLC,60) == ERROR) {
      printf("Unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   if(err) {
      semGive (semSLC);
      printf ("R Err=%04x\r\n",err);
      return err;
   }
   swab ((char *)&ctrl[0],(char *)&tm_ctrl1,2);

   tm_ctrl1.mcp_ne_lamp_on_cmd = val;

   swab ((char *)&tm_ctrl1,(char *)&ctrl[0],2);
   err = slc_write_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   semGive (semSLC);
   
   if(err) {
      printf ("W Err=%04x\r\n",err);
      return err;
   }

   return 0;
}
void tm_neon_on()
{
    tm_neon (1);
}
void tm_neon_off()
{
    tm_neon (0);
}
void tm_sp_neon_on()
{
  if (taskIdFigure("tmNeon")==ERROR)
    taskSpawn("tmNeon",90,0,1000,(FUNCPTR)tm_neon,1,0,0,0,0,0,0,0,0,0);
}
void tm_sp_neon_off()
{
  if (taskIdFigure("tmNeon")==ERROR)
    taskSpawn("tmNeon",90,0,1000,(FUNCPTR)tm_neon,0,0,0,0,0,0,0,0,0,0);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_hgcd
**	    tm_hgcd_on
**	    tm_hgcd_off
**	    tm_sp_hgcd_on
**	    tm_sp_hgcd_off
**
** DESCRIPTION:
**      Turn on/off the Mercury Cadmium lamps for the flat field.
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
tm_hgcd(short val)
{
   int err;
   unsigned short ctrl[1];
   struct B10_1 tm_ctrl1;   
             
   if(semTake(semSLC,60) == ERROR) {
      printf("Unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   if(err) {
      semGive (semSLC);
      printf ("R Err=%04x\r\n",err);
      return err;
   }
   swab ((char *)&ctrl[0],(char *)&tm_ctrl1,2);

   tm_ctrl1.mcp_hgcd_lamp_on_cmd = val;

   swab ((char *)&tm_ctrl1,(char *)&ctrl[0],2);
   err = slc_write_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   semGive (semSLC);

   if(err) {
      printf ("W Err=%04x\r\n",err);
      return err;
   }

   return 0;
}
void tm_hgcd_on()
{
    tm_hgcd (1);
}
void tm_hgcd_off()
{
    tm_hgcd (0);
}
void tm_sp_hgcd_on()
{
  if (taskIdFigure("tmHgCd")==ERROR)
    taskSpawn("tmHgCd",90,0,1000,(FUNCPTR)tm_hgcd,1,0,0,0,0,0,0,0,0,0);
}
void tm_sp_hgcd_off()
{
  if (taskIdFigure("tmHgCd")==ERROR)
    taskSpawn("tmHgCd",90,0,1000,(FUNCPTR)tm_hgcd,0,0,0,0,0,0,0,0,0,0);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: az_amp_ok
**	    alt_amp_ok
**	    rot_amp_ok
**
** DESCRIPTION:
**      Check if amp is ok.
**
** RETURN VALUES:
**      T/F
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	sdssdc
**
**=========================================================================
*/
int az_amp_ok()
{
  if ((sdssdc.status.i6.il0.az_mtr_ccw_perm_in) &&
	(sdssdc.status.i6.il0.az_mtr_cw_perm_in) &&
	(sdssdc.status.i6.il0.az_plc_perm_in))
	return TRUE;
  else
    if (((sdssdc.status.i6.il0.az_mtr_ccw_perm_in) ||
	(sdssdc.status.i6.il0.az_mtr_cw_perm_in)) &&
	(sdssdc.status.i6.il0.az_plc_perm_in))
      return TRUE;
    else
	return FALSE;
}
int alt_amp_ok()
{
  if ((sdssdc.status.i6.il0.alt_mtr_dn_perm_in) &&
	(sdssdc.status.i6.il0.alt_mtr_up_perm_in) &&
	(sdssdc.status.i6.il0.alt_plc_perm_in))
	return TRUE;
  else
    if (((sdssdc.status.i6.il0.alt_mtr_dn_perm_in) ||
	(sdssdc.status.i6.il0.alt_mtr_up_perm_in)) &&
	(sdssdc.status.i6.il0.alt_plc_perm_in))
      return TRUE;
    else
	return FALSE;
}
int rot_amp_ok()
{
  if ((sdssdc.status.i7.il0.rot_mtr_ccw_perm_in) &&
        (sdssdc.status.i7.il0.rot_mtr_cw_perm_in) &&
	(sdssdc.status.i7.il0.rot_plc_perm_in))
	return TRUE;
  else
    if (((sdssdc.status.i7.il0.rot_mtr_ccw_perm_in) ||
	(sdssdc.status.i7.il0.rot_mtr_cw_perm_in)) &&
	(sdssdc.status.i7.il0.rot_plc_perm_in))
      return TRUE;
    else
	return FALSE;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: mgt_shutdown
**	    tm_amp_mgt	task 
**
** DESCRIPTION:
**      Shutdown the telescope by turning on the brakes due to a software reboot.
**	Check the amplifiers to see if fault and need to turn brakes on.
**	Must allow for restart...reason for monitor boolean only turned on
**	by tm_controller_run.
**	Keep the watchdog timer on which enables the amplifiers.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**	tm_amp_engage
**
** GLOBALS REFERENCED:
**	sddsdc
**	monitor_axis
**
**=========================================================================
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
**=========================================================================
**
** ROUTINE: tm_print_amp_status
**
** DESCRIPTION:
**      Print the amp status.
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
void tm_print_amp_status()
{
    if (!az_amp_ok())
      printf ("\r\nAz Amp Disengaged: az_mtr_ccw_perm=%d,az_mtr_cw_perm=%d",
	sdssdc.status.o11.ol0.az_mtr_ccw_perm,
	sdssdc.status.o11.ol0.az_mtr_cw_perm);
    else
      printf ("\r\nAZ Amp OK");
    if (!alt_amp_ok())
      printf ("\r\nAlt Amp Disengaged: alt_mtr_dn_perm=%d,alt_mtr_up_perm=%d",
	sdssdc.status.o11.ol0.alt_mtr_dn_perm,
	sdssdc.status.o11.ol0.alt_mtr_up_perm);
    else
      printf ("\r\nALT Amp OK");
    if (!rot_amp_ok())
      printf ("\r\nRot Amp Disengaged: rot_mtr_rdy=%d,rot_mtr_ccw_perm=%d,rot_mtr_cw_perm=%d",
	sdssdc.status.i8.il0.rot_mtr_rdy,
	sdssdc.status.o11.ol0.rot_mtr_ccw_perm,
	sdssdc.status.o11.ol0.rot_mtr_cw_perm);
    else
      printf ("\r\nROT Amp OK");
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_amp_disengage
**	    tm_amp_engage
**	    tm_setup_wd
**
** DESCRIPTION:
**      Enable/disable the amp utilizing the watchdog which is setup with
**	the initialization routine.
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
**=========================================================================
**
** ROUTINE: tm_axis_status
**	    tm_print_axis_status
**
** DESCRIPTION:
**      Telescope motion prints status of axis_status funciton.
**
** RETURN VALUES:
**	int	axis status
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
char const* const msg_axis_status[]=
	{"IN_SEQUENCE",
	 "IN_POSITION",
	 "IN_MOTION",
	 "DIRECTION positive",
	 "FRAMES_LEFT"};
int  tm_axis_status(int axis)
{
  int value;

  semTake(semMEI,WAIT_FOREVER);
  value=axis_status(axis);
  semGive(semMEI);
  return value;
}
void tm_print_axis_status(int axis)
{
  int i,value;

  value=tm_axis_status(axis);
  printf ("AXIS STATUS: %x",value);
  for (i=0;i<sizeof(msg_axis_status)/sizeof(char *);i++)
    if ((value>>(i+4))&1) printf ("     %s\r\n",msg_axis_status[i]);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_axis_state
**	    tm_print_axis_state
**
** DESCRIPTION:
**      Telescope motion prints status of axis_state funciton.
**
** RETURN VALUES:
**      int	state value
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
char const* const msg_axis_state[]=
	{"NO_EVENT",
	 "NEW_FRAME",
	 "STOP_EVENT",
	 "E_STOP_EVENT",
	 "ABORT_EVENT",
	 "Running???",
	 "Undocumented Value"};
int tm_axis_state(int axis)
{
  int value;

  semTake(semMEI,WAIT_FOREVER);
  value=axis_state(axis);
  semGive(semMEI);
  return value;
}
void tm_print_axis_state(int axis)
{
  int i,value;

  value=tm_axis_state(axis);
  printf ("AXIS STATE: %x",value);
  switch (value)
  {
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
  printf ("     %s\r\n",msg_axis_state[i]);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_print_axis_source
**
** DESCRIPTION:
**      Telescope motion prints status of axis_source funciton.
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
char const* const msg_axis_source[]=
	{"ID_NONE",
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
	 "ID_AXIS_COMMAND"};
void tm_print_axis_source(int axis)
{
  int value;

  semTake(semMEI,WAIT_FOREVER);
  value=axis_source(axis);
  semGive(semMEI);
  printf ("AXIS SOURCE: %x",value);
    printf ("     %s\r\n",msg_axis_source[value]);
}                                                              
 
