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
#include "cmd.h"

int tm_ADC128F1 = -1;

/*
 * tm_mgt variables to enable safe operation if telescope is not in control
 *
 * The monitor_axis array provides a mechanism to prevent deadlocks at startup
 */
int monitor_on[NAXIS];	/* overrides monitoring of axis...possible disable */
int monitor_axis[NAXIS];

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
      TRACE(0, "tm_move_instchange: could not take semMEI semphore", 0, 0);
      return ERROR;
   }

   for(axis = 0; axis < NAXIS; axis++) {
      sem_controller_run(2*axis);
      start_move_corr(2*axis, ilcpos[axis]*ticks_per_degree[axis],
		      ilcvel[axis], ilcacc[axis]);
   }

   semGive (semMEI); 

   return 0;
}

/*=========================================================================
**
**      Telescope motion to a specified position.  The routine returns to 
**	the caller immediately while the motion is being fulfilled.
**	Encapsulates the MEI function and converts args from ints to doubles.
**
*/
void
tm_start_move(int mei_axis, int pos, int vel, int accel)
{
   semTake(semMEI,WAIT_FOREVER);
   start_move_corr(mei_axis, (double)pos, (double)vel, (double)accel);
   semGive(semMEI);
}

# if 0
/*=========================================================================
**
**      Telescope motion to go over a fiducial point back-and-forth a
**	specified number of times.  Boroski spec.
**
*/
void
tm_bf(int axis,				/* axis to move (AZ, ALT, ROT) */
      int vel,
      int accel,
      int pos1, int pos2,
      int times)
{
   int i;
   int status;

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      fprintf(stderr,"Ilegal axis: %d\n", axis);
      return;
   }
   
   printf("Pass ");
   for(i = 0; i < times; i++) {
      printf ("%d ",i);
      tm_sem_controller_run(2*axis);
      tm_start_move(2*axis, pos1, vel, accel);

      status = FALSE;
      while(!status) {
	 taskDelay(60);

	 semTake(semMEI, WAIT_FOREVER);
	 status = motion_done(2*axis);
	 semGive(semMEI);
      }

      tm_start_move(2*axis, pos2, vel, accel);

      status = FALSE;
      while(!status) {
	 taskDelay(60);

	 semTake(semMEI, WAIT_FOREVER);
	 status = motion_done(2*axis);
	 semGive(semMEI);
      }
   }
   printf(" Done\n");
}
#endif

/*=========================================================================
**
**      Print the specified axis PID coefficients and sample rate.
*/
void
tm_print_coeffs(int mei_axis)
{
   short coeff[COEFFICIENTS];
   short mode;
   short rate;
   
   semTake(semMEI,WAIT_FOREVER);
   
   get_filter(mei_axis, (P_INT)coeff);
   get_integration(mei_axis, &mode);
   rate = dsp_sample_rate();
   
   semGive (semMEI);
   
   printf("AXIS %d: P=%d, I=%d, D=%d\n", mei_axis,
	  coeff[0], coeff[1], coeff[2]);
   printf("        ACCEL_FF=%d, VEL_FF=%d, FRICT_FF=%d\n",
	  coeff[3], coeff[4], coeff[9]);
   printf("        I_LIMIT=%d, OFFSET=%d, DAC_LIMIT=%d, SHIFT=%d\n",
	   coeff[5], coeff[6], coeff[7], coeff[8]);
   printf("integration mode is %d and sample is %d Hz\n", mode, rate);
}

/*=========================================================================
**
**      Set the specified axis coefficient where index is used to specify
**	one of 10 possible coefficients.
**		index=0(P),1(I),2(D),3(AFF),4(VFF),5(ILIM),6(OFF),7(DLIM)
**		8(SHIFT)(-5 is 1/32),9(FFF)
**
**  Not used in code; only for vxWorks experiments
*/
void
tm_set_filter_coeff(int mei_axis, int ind, int val)
{
   short coeff[COEFFICIENTS];
   
   semTake(semMEI,WAIT_FOREVER);

   get_filter(mei_axis, (P_INT)coeff);
   coeff[ind] = val;
   set_filter(mei_axis, (P_INT)coeff);

   semGive(semMEI);
}

/*=========================================================================
**=========================================================================
**
**      Telescope motion encapsulates with a semaphore the MEI function.
**
**=========================================================================
*/
void
tm_get_position(int mei_axis, double *position)
{
   semTake(semMEI, WAIT_FOREVER);
   get_position_corr(mei_axis, position);
   semGive(semMEI);
}

void
tm_get_velocity(int mei_axis,
		double *velocity)
{
   semTake(semMEI, WAIT_FOREVER);
   get_velocity(mei_axis, velocity);
   semGive(semMEI);
}

void
tm_reset_integrator(int mei_axis)
{
   semTake(semMEI, WAIT_FOREVER);
   reset_integrator(mei_axis);
   semGive(semMEI);
}

void
tm_set_position(int mei_axis, int pos)
{
   semTake(semMEI,WAIT_FOREVER);
   set_position_corr(mei_axis, pos);
   semGive(semMEI);
}

int
tm_adjust_position(int axis,			/* desired axis */
	      int offset)		/* how much to offset position */
{
   double position;			/* position of axis */

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      TRACE(0, "tm_adjust_position: invalid axis %d", axis, 0);
      
      return(-1);
   }

   if(semTake(semMEI,60) != OK) {
      TRACE(0, "adjusting axis %s position: unable to take semaphore: %s",
	    axis_name(axis), strerror(errno));
      return(-1);
   }
   
   taskLock();
   
   set_axis_encoder_error(axis, offset, 1); /* correct the current error */

   if(get_position_corr(2*axis, &position) != DSP_OK) {
      taskUnlock();
      semGive(semMEI);
      TRACE(0, "adjusting position for axis %s: unable to read position",
	    axis_name(axis), 0);
      return(-1);
   }
   
   write_fiducial_log("UPDATE_ENCODER", axis, 0, 0, position, 0,
		      get_axis_encoder_error(axis), 0);

   position += get_axis_encoder_error(axis); /* include software correction */
   
   set_position_corr(2*axis, position);
   set_position_corr(2*axis + 1, position); /* second az/alt encoder isn't
					       connected/doesn't exist */
/*
 * zero software correction term
 */
   set_axis_encoder_error(axis, -get_axis_encoder_error(axis), 0);

   taskUnlock();
   semGive(semMEI);

   return(0);
}

#if 0
void
tm_set_encoder(int mei_axis)
{
   semTake(semMEI, WAIT_FOREVER);
   set_feedback(mei_axis, FB_ENCODER);
   semGive(semMEI);
}
#endif

#if 0
/*=========================================================================
**
**      Telescope motion sets up the tachometer as the encoder.  No longer
**	used at this time.
*/
void
tm_set_analog_encoder(int mei_axis, int channel)
{
   semTake(semMEI,WAIT_FOREVER);
   
   set_analog_channel(mei_axis, channel, TRUE, TRUE);
   set_axis_analog(mei_axis, TRUE);
   set_feedback(mei_axis, FB_ANALOG);

   semGive(semMEI);
}

/*=========================================================================
**
**      Telescope motion sets analog channel as feedback to PID.  Not used.
*/
void
tm_set_analog_channel(int mei_axis, int channel)
{
   semTake(semMEI,WAIT_FOREVER);

   set_analog_channel(mei_axis,channel, TRUE, TRUE);
   set_axis_analog(mei_axis, TRUE);

   semGive(semMEI);
}
#endif

/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_sem_controller_run
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
tm_sem_controller_run(int mei_axis)
{
   semTake(semMEI,WAIT_FOREVER);
   sem_controller_run(mei_axis);
   semGive(semMEI);
}

void
sem_controller_run(int mei_axis)
{
   const int nretry = 12;
   int i;
   
   for(i = 0; axis_state(mei_axis) > 2 && i < nretry; i++) {
      taskDelay(20);
      controller_run(mei_axis);
   }
   
   if(i != nretry) {			/* success */
      monitor_axis[mei_axis/2] = TRUE;
   }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_sem_controller_idle
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
tm_sem_controller_idle(int mei_axis)
{
   semTake(semMEI, WAIT_FOREVER);
   sem_controller_idle(mei_axis);
   semGive(semMEI);
}

void
sem_controller_idle(int mei_axis)
{
   int retry = 6;
   
   while(axis_state(mei_axis) <= 2 && --retry > 0) {
      controller_idle(mei_axis);
   }
}

#if 0
/*
**
** Telescope motion makes the current coeffs active for next boot.
*/
void
tm_set_boot_filter(int mei_axis)
{
   short coeff[COEFFICIENTS];
   
   semTake(semMEI, WAIT_FOREVER);
   get_filter(mei_axis, (P_INT)coeff);
   set_boot_filter(mei_axis, (P_INT)coeff);
   semGive(semMEI);
}
#endif

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

#if 0
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
#endif

#if 0
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
#endif

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
az_amp_ok(int update)			/* update status before reporting? */
{
   if(update) {
      update_sdssdc_status_i6();
   }

   if((sdssdc.status.i6.il0.az_mtr_ccw_perm_in ||
				     sdssdc.status.i6.il0.az_mtr_cw_perm_in) &&
      sdssdc.status.i6.il0.az_plc_perm_in) {
      return TRUE;
   } else {
      return FALSE;
   }
}

int
alt_amp_ok(int update)			/* update status before reporting? */
{
   if(update) {
      update_sdssdc_status_i6();
   }
   
   if((sdssdc.status.i6.il0.alt_mtr_dn_perm_in ||
				    sdssdc.status.i6.il0.alt_mtr_up_perm_in) &&
       sdssdc.status.i6.il0.alt_plc_perm_in) {
      return TRUE;
   } else {
      return FALSE;
   }
}

int
rot_amp_ok(int update)			/* update status before reporting? */
{
   if(update) {
      unsigned short ctrl[2];

      if(semTake(semSLC,60) == ERROR) {
	 TRACE(0, "Unable to take semaphore: %s (%d)", strerror(errno), errno);
      } else {
	 const int offset = (char *)&sdssdc.status.i7 - (char *)&sdssdc.status;
	 int err = slc_read_blok(1, 9, BIT_FILE, offset/2,
						      &ctrl[0],sizeof(ctrl)/2);
	 if(err) {
	    TRACE(0, "az_amp_ok: error reading slc: 0x%04x", err, 0);
	 }
	 semGive(semSLC);

	 if(!err) {
	    swab((char *)&ctrl[0], (char *)&sdssdc.status.i7,
						     sizeof(sdssdc.status.i7));
	 }
      }
   }

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
**     by tm_sem_controller_run.
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
   TRACE(1, "Safely halting the telescope by braking AZ and ALT", 0, 0);
   mcp_set_brake(AZIMUTH);
   mcp_set_brake(ALTITUDE);
}

void
tm_amp_mgt(void)
{
   int amp_ok;
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
	       TRACE(0, "MGT: bad az state %s: %s",
		     axis_state_str(2*AZIMUTH), axis_source_str(2*AZIMUTH));
	       mcp_set_brake(AZIMUTH);
	       monitor_axis[AZIMUTH] = FALSE;
	    }
	    if(!az_amp_ok(0)) {
	       taskDelay(1);
	       amp_ok = az_amp_ok(0);
	       TRACE(2, "MGT: bad az amp (now %d %d)", amp_ok, az_amp_ok(1));
	       if(!az_amp_ok(1)) {
		  TRACE(0, "MGT: bad az amp; aborting", 0, 0);
		  tm_sem_controller_idle(2*AZIMUTH);
		  mcp_set_brake(AZIMUTH);
		  monitor_axis[AZIMUTH]=FALSE;
	       }
	    }
	 }
      }

      if(monitor_on[ALTITUDE]) {
	 if(monitor_axis[ALTITUDE] &&
	    sdssdc.status.i9.il0.alt_brake_dis_stat) {
	    if((state = tm_axis_state(2*ALTITUDE)) > 2 && state != STOP_EVENT){
	       TRACE(0, "MGT: bad alt state %s: %s", 
		     axis_state_str(2*ALTITUDE), axis_source_str(2*ALTITUDE));

	       mcp_set_brake(ALTITUDE);
	       monitor_axis[ALTITUDE] = FALSE;
	    }
	    if(!alt_amp_ok(0)) {
	       taskDelay(1);
	       amp_ok = alt_amp_ok(0);

	       TRACE(2, "MGT: bad alt amp (now %d %d)", amp_ok, alt_amp_ok(1));
	       if(!alt_amp_ok(1)) {
		  TRACE(0, "MGT: bad alt amp; aborting", 0, 0);
		  tm_sem_controller_idle(2*ALTITUDE);
		  mcp_set_brake(ALTITUDE);
		  monitor_axis[ALTITUDE] = FALSE;
	       }
	    }
	 }
      }
     
      if(monitor_on[INSTRUMENT]) {
	 if(monitor_axis[INSTRUMENT]) {
	    if((state = tm_axis_state(2*INSTRUMENT)) > 2 &&
							  state != STOP_EVENT){
	       TRACE(0, "MGT: bad rot state %s: %s", 
		     axis_state_str(2*INSTRUMENT),
		     axis_source_str(2*INSTRUMENT));
	       monitor_axis[INSTRUMENT] = FALSE;
	    }
	    if(!rot_amp_ok(0)) {
	       taskDelay(1);
	       amp_ok = alt_amp_ok(0);

	       TRACE(2, "MGT: bad rot amp (now %d %d)", amp_ok, rot_amp_ok(1));

	       if(!rot_amp_ok(1)) {
		  TRACE(0, "MGT: bad rot amp; aborting", 0, 0);
		  
		  tm_sem_controller_idle(2*INSTRUMENT);
		  monitor_axis[INSTRUMENT] = FALSE;
	       }
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
    if(az_amp_ok(1)) {
       printf("AZ Amp OK\n");
    } else {
       printf ("Az Amp Disengaged: az_mtr_ccw_perm=%d,az_mtr_cw_perm=%d\n",
	       sdssdc.status.o11.ol0.az_mtr_ccw_perm,
	       sdssdc.status.o11.ol0.az_mtr_cw_perm);
    }

    if(alt_amp_ok(1)) {
       printf("ALT Amp OK\n");
    } else {
       printf("Alt Amp Disengaged: alt_mtr_dn_perm=%d,alt_mtr_up_perm=%d\n",
	      sdssdc.status.o11.ol0.alt_mtr_dn_perm,
	      sdssdc.status.o11.ol0.alt_mtr_up_perm);
    }

    if(rot_amp_ok(1)) {
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
  StopCounter(&sbrd,TM_WD);
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
** Get/print status of current motion
**
*/
int
tm_axis_status(int mei_axis)
{
   int value;
   
   semTake(semMEI,WAIT_FOREVER);
   value = axis_status(mei_axis);
   semGive(semMEI);

   return value;
}

static void
print_axis_status(int mei_axis)
{
   if(in_sequence(mei_axis)) {
      printf(" IN_SEQUENCE");
   }
   if(in_position(mei_axis)) {
      printf(" IN_POSITION");
   } 
   if(in_motion(mei_axis)) {
      printf(" IN_MOTION");
      if(negative_direction(mei_axis)) {
	 printf(" NEGATIVE");
      } else {
	 printf(" POSITIVE");
      }
   }
   if(frames_left(mei_axis)) {
      printf(" FRAMES_LEFT");
   }
   printf("\n");
}

void
tm_print_axis_status(int mei_axis)
{
   semTake(semMEI, WAIT_FOREVER);

   printf("AXIS STATUS: 0x%x", axis_status(mei_axis));
   print_axis_status(mei_axis);

   semGive(semMEI);
}

/*=========================================================================
**
** Get and/or print the state of the axes
**
**=========================================================================
*/
int
tm_axis_state(int mei_axis)
{
  int value;

  semTake(semMEI, WAIT_FOREVER);
  value = axis_state(mei_axis);
  semGive(semMEI);

  return value;
}

const char *
axis_state_str(int mei_axis)
{
   int value = axis_state(mei_axis);
   
   switch (value) {
    case NO_EVENT:
      return("NO_EVENT");
      break;
    case NEW_FRAME:
      return("NEW_FRAME");
      break;
    case STOP_EVENT:
      return("STOP_EVENT");
      break;
    case E_STOP_EVENT:
      return("E_STOP_EVENT");
      break;
    case ABORT_EVENT:
      return("ABORT_EVENT");
      break;
    case 1:				/* not documented */
      return("Running???");
      break;
    default:
      value &= 0xffff;			/* don't overrun string */
      sprintf(ublock->buff, "Undocumented Value 0x%x", value);
      return(ublock->buff);
   }
}

void
tm_print_axis_state(int mei_axis)
{
   semTake(semMEI, WAIT_FOREVER);
   printf("AXIS STATE: %s\n", axis_state_str(mei_axis));
   semGive(semMEI);
}
/*=========================================================================
**
**      Telescope motion prints status of axis_source function.
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

const char *
axis_source_str(int mei_axis)
{
   int value;

   value = axis_source(mei_axis);
   
   if(value < 0 ||
      value >= sizeof(msg_axis_source)/sizeof(msg_axis_source[0])) {
            value &= 0xffff;		/* don't overrun string */
      sprintf(ublock->buff, "ID_MCP_UNKNOWN 0x%x", value);
      return(ublock->buff);
   } else {
      return(msg_axis_source[value]);
   }
}                                                              

void
tm_print_axis_source(int mei_axis)
{
   semTake(semMEI, WAIT_FOREVER);
   printf ("AXIS SOURCE: %s\n", axis_source_str(mei_axis));
   semGive(semMEI);
}                                                              

/*
 * A routine to provide all that axis information and more with one command;
 * an axis < 0 shows all axes
 */
void
tm_show_axis(int axis)
{
   int axis0, axis1;			/* range of axes to show */
   double pos, vel;

   if(axis < 0) {
      axis0 = AZIMUTH; axis1 = INSTRUMENT;
   } else {
      axis0 = axis1 = axis;
   }

   semTake(semMEI, WAIT_FOREVER);

   for(axis = axis0; axis <= axis1; axis++) {
      if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
	 fprintf(stderr,"Ilegal axis: %d\n", axis);
	 break;
      }
   
      get_position_corr(2*axis, &pos);
      get_velocity(2*axis, &vel);
      
      printf("Axis %-8s: p= %9ld (%9.4f deg), v = %9ld (%9.4f deg/s)\n",
	     axis_name(axis),
	     (long)pos, pos/axis_ticks_deg(axis),
	     (long)vel, vel/axis_ticks_deg(axis));
      
      printf("  axis state:  %s\n", axis_state_str(2*axis));
      if(axis_state(2*axis) >= STOP_EVENT) {
	 printf("  axis source: %s\n", axis_source_str(2*axis));
      } else {
	 printf("  axis status:");
	 print_axis_status(2*axis);
      }
   }   

   semGive(semMEI);
}

/*****************************************************************************/
/*
 * Clear the sticky versions of the bump switches
 */
void
clear_sticky_bumps(int axis, int which)
{
   switch (axis) {
    case AZIMUTH:
      axis_stat[AZIMUTH][which].bump_up_ccw_sticky = 0;
      axis_stat[AZIMUTH][which].bump_dn_cw_sticky = 0;
      break;
    case ALTITUDE:
      axis_stat[ALTITUDE][which].bump_up_ccw_sticky = 0;
      axis_stat[ALTITUDE][which].bump_dn_cw_sticky = 0;
      break;
    case INSTRUMENT:
      break;
    default:
      TRACE(0, "Illegal axis %d in clear_sticky_bumps", axis, 0);
   }
}

void
show_bump(void)
{
   printf("Windscreen bump switches:\n");

   printf("Azimuth: ");

   if(sdssdc.status.i1.il0.az_bump_cw) {
      printf(" in contact CW");
   } else if(axis_stat[AZIMUTH][0].bump_dn_cw_sticky) {
      printf(" touched CW   ");
   }

   if(sdssdc.status.i1.il0.az_bump_ccw) {
      printf(" in contact CCW");
   } else if(axis_stat[AZIMUTH][0].bump_up_ccw_sticky) {
      printf(" touched CCW   ");
   }
   printf("\n");

   printf("Altitude:");
   if(sdssdc.status.i1.il10.alt_bump_up) {
      printf(" in contact UP");
   } else if(axis_stat[ALTITUDE][0].bump_up_ccw_sticky) {
      printf(" touched UP   ");
   }

   if(sdssdc.status.i1.il10.alt_bump_dn) {
      printf(" in contact DOWN");
   } else if(axis_stat[ALTITUDE][0].bump_dn_cw_sticky) {
      printf(" touched DOWN   ");
   }
   printf("\n"); 
}
