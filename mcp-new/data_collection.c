#include "copyright.h"
/**************************************************************************
** ABSTRACT:
**	Collects data from MEI, AB, CW, and time.  Distributes to shared
**	memory and broadcasts at 1 Hz.
***************************************************************************/
/*------------------------------*/
/*	includes		*/
/*------------------------------*/
#include "vxWorks.h"
#include "string.h"
#include "intLib.h"
#include "iv.h"
#include "memLib.h"
#include "semLib.h"
#include "sigLib.h"
#include "etherLib.h"
#include "taskLib.h"
#include "logLib.h"
#include "inetLib.h"
#include "sockLib.h"
#include "sysLib.h"
#include "symLib.h"
#include "symbol.h"
#include "semLib.h"
#include "ioLib.h"
#include "tickLib.h"
#include "stdio.h"
#include "idsp.h"
#include "time.h"
#include "data_collection.h"
#include "gendefs.h"
#include "dio316dr.h"
#include "dio316lb.h"
#include "tm.h"
#include "io.h"
#include "abdh.h"
#include "ipcast.h"
#include "axis.h"
#include "cw.h"
#include "instruments.h"

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
#define SHARE_MEMORY	0x02800000
 
/*-------------------------------------------------------------------------
**
** GLOBAL VARIABLES
*/

SEM_ID semMEIDC=NULL;
SEM_ID semMEIUPD=NULL;
SEM_ID semSLCDC=NULL;
int rawtick=0;
/* Enables for axis 0, 2, 4: Azimuth(0), Altitude(2), Rotator(4) */
int MEIDC_Enable[]={TRUE,TRUE,TRUE};
int MEIDC_Rotate=0;
unsigned long DC_freq=0;
unsigned long mei_freq=1;
unsigned long slc_freq=100;
int BCAST_Enable=TRUE;
long *axis0pos=NULL;		/* AZ axis */
long *axis0cmd=NULL;
short *dac0out=NULL;
short *axis0err=NULL;
short *axis0velf=NULL;
long *axis0vel=NULL;
short *axis0accelf=NULL;
long *axis0accel=NULL;
long *axis0tim=NULL;
long *axis1pos=NULL;

long *axis2pos=NULL;		/* AL axis */
long *axis2cmd=NULL;
short *dac2out=NULL;
short *axis2err=NULL;
short *axis2velf=NULL;
long *axis2vel=NULL;
short *axis2accelf=NULL;
long *axis2accel=NULL;
long *axis2tim=NULL;
long *axis3pos=NULL;

long *axis4pos=NULL;		/* IR axis */
long *axis4cmd=NULL;
short *dac4out=NULL;
short *axis4err=NULL;
short *axis4velf=NULL;
long *axis4vel=NULL;
short *axis4accelf=NULL;
long *axis4accel=NULL;
long *axis4tim=NULL;
long *axis5pos=NULL;
float sdss_time_dc;

struct SDSS_FRAME sdssdc={SDSS_FRAME_VERSION,DATA_TYPE};
struct TM_M68K *tmaxis[3]={
   (struct TM_M68K *)&sdssdc.axis[0],
   (struct TM_M68K *)&sdssdc.axis[1],
   (struct TM_M68K *)&sdssdc.axis[2]
};
struct AXIS_STAT axis_stat[3]={0,0,0};
struct AXIS_STAT persistent_axis_stat[3]={0,0,0};
int meistatcnt=0;
#define	DATA_STRUCT(dsp, axis, offset)	(P_DSP_DM)((dsp)->data_struct+(DS_SIZE* (axis)) + offset)

/* prototypes */
void swapwords (register short *dp, register unsigned short len);
void mei_data_collection(unsigned long freq);
void print_mei_dc (int cnt);
void print_axis_dc (int axis);
void print_pos_dc (int axis);
void slc500_data_collection(unsigned long freq);
void DataCollectionTrigger();
int dc_interrupt();

/*=========================================================================
**=========================================================================
**
** ROUTINE: swapwords
**
** DESCRIPTION:
**	Swap words
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
void swapwords (register short *dp, register unsigned short len)
{
   short temp;

   while (len-- > 0)
   {
      temp=dp[0];
      dp[0] = dp[1];
      dp[1]=temp;
      dp+=2;
   }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: mei_data_collection
**
** DESCRIPTION:
**	Collects data from MEI controller.  The time to respond to a request
**	depends on the sample frequency of the controller.  Slowing the 
**	loop frequency slows the response.  The controller is also limited
**	in the size of the data returned and barely returns enough info
**	for the axis + some portion of the consecutive axis which is used
**	for a redundant postition.  To increase the frequency of a single 
**	axis one can disable the collection of the other two since the
**	collection is round-robined.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	sdssdc
**	semMEI
**	semMEIDC
**	semMEIUPD
**
**=========================================================================
*/
void mei_data_collection(unsigned long freq)
{
	int i;
	int rotate;
        void restore_pos();
	
	/*  ****************************************************  **
		put whatever type of motion you want to sample here.
	**  ****************************************************  */
  	if (semMEIDC==NULL) semMEIDC = semBCreate(SEM_Q_FIFO,SEM_EMPTY);
  	if (semMEIUPD==NULL) semMEIUPD = semMCreate(SEM_Q_PRIORITY|SEM_INVERSION_SAFE);
        restore_pos();
	mei_freq=freq;
	axis0pos=&tmaxis[0]->actual_position;
	axis0cmd=&tmaxis[0]->position;
	dac0out=&tmaxis[0]->voltage;
	axis0err=&tmaxis[0]->error;
	axis0velf=&tmaxis[0]->velocity_fractional;
	axis0vel=&tmaxis[0]->velocity;
	axis0accelf=&tmaxis[0]->acceleration_fractional;
	axis0accel=&tmaxis[0]->acceleration;
	axis0tim=&tmaxis[0]->time;
	axis1pos=&tmaxis[0]->actual_position2;

	axis2pos=&tmaxis[1]->actual_position;
	axis2cmd=&tmaxis[1]->position;
	dac2out=&tmaxis[1]->voltage;
	axis2err=&tmaxis[1]->error;
	axis2velf=&tmaxis[1]->velocity_fractional;
	axis2vel=&tmaxis[1]->velocity;
	axis2accelf=&tmaxis[1]->acceleration_fractional;
	axis2accel=&tmaxis[1]->acceleration;
	axis2tim=&tmaxis[1]->time;
	axis3pos=&tmaxis[1]->actual_position2;
	
	axis4pos=&tmaxis[2]->actual_position;
	axis4cmd=&tmaxis[2]->position;
	dac4out=&tmaxis[2]->voltage;
	axis4err=&tmaxis[2]->error;
	axis4velf=&tmaxis[2]->velocity_fractional;
	axis4vel=&tmaxis[2]->velocity;
	axis4accelf=&tmaxis[2]->acceleration_fractional;
	axis4accel=&tmaxis[2]->acceleration;
	axis4tim=&tmaxis[2]->time;
	axis5pos=&tmaxis[2]->actual_position2;
	
	FOREVER
	{
  	 if (semTake (semMEIDC,WAIT_FOREVER)!=ERROR)
  	 {
  	  if (semTake (semMEIUPD,60)!=ERROR)
          {
  	   if (semTake (semMEI,NO_WAIT)!=ERROR)
           {
	    sdss_time_dc=(float)sdss_get_time();
            time (&sdssdc.ctime);
            sdssdc.sdsstime=(long)(sdss_time_dc*1000);
            i=MEIDC_Rotate;
	    if (MEIDC_Enable[i])
            {
	      pcdsp_transfer_block(dspPtr,TRUE,FALSE,DATA_STRUCT(dspPtr,
	        i*2,DS_PREV_ENCODER) ,DS_SIZE+4,
	        (short *)tmaxis[i]);
	      if (dsp_error) 
	      {
	        tmaxis[i]->status = dsp_error;
	        tmaxis[i]->errcnt++;
	      }
	      else
	        meistatcnt++;
	      semGive(semMEI);
	      swapwords ((short *)(&tmaxis[i]->actual_position),1);
	      swapwords ((short *)(&tmaxis[i]->position),1);
	      swapwords ((short *)(&tmaxis[i]->time),1);
	      swapwords ((short *)(&tmaxis[i]->velocity),1);
	      swapwords ((short *)(&tmaxis[i]->acceleration),1);
	      swapwords ((short *)(&tmaxis[i]->actual_position2),1);
	    }
    	    for(i = 1; i < 4; i++)	/* find next enabled data collection */
	    {
	      rotate = (MEIDC_Rotate+i)%3;
	      if (MEIDC_Enable[rotate]) 
	      {
                 MEIDC_Rotate=rotate;
	         break;
	      }
	    }
	    tm_data_collection();
	   }
	   semGive(semMEIUPD);
          }
	 }
	}
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: print_mei_dc
**	    print_axis_dc
**	    print_pos_dc
**
** DESCRIPTION:
**	Diagnostic to print status from sdssdc structure.
**
** RETURN VALUES:
**      int 	always zero
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	sdssdc
**
**=========================================================================
*/
void print_mei_dc (int cnt)
{
	long *ap, *cp, *ap2;
	int ii;
	int i;

	logMsg("\r\n\tapos\tcpos\tapos2\tvoltage\tptr",0,0,0,0,0,0);
	  
    	for(ii = 0; ii < cnt; ii++)
	{
  	 if (semTake (semMEIUPD,60)!=ERROR)
  	 {
    	  for(i = 0; i < 3; i++)
          {
	    ap=(long *)&tmaxis[i]->actual_position;
	    cp=(long *)&tmaxis[i]->position;
	    ap2=(long *)&tmaxis[i]->actual_position2;
	    logMsg("\r\naxis %d:\t%ld\t%ld\t%ld\t%ld\t%x",i,
	      *ap, *cp,*ap2,tmaxis[i]->voltage,(long)tmaxis[i]);
	  }
	  logMsg ("\r\n%d raw tick, %f Secs",
		rawtick,((double)rawtick)/sysClkRateGet(),0,0,0,0);
	  semGive (semMEIUPD);
	 }
	 taskDelay (1);
	}
}
void print_axis_dc (int axis)
{
	long *ap, *cp, *ap2;
	int i;

	i=axis;
	logMsg("\r\n\tapos\tcpos\tapos2\tvoltage\tptr",0,0,0,0,0,0);
  	 if (semTake (semMEIUPD,60)!=ERROR)
  	 {
           ap=(long *)&tmaxis[i]->actual_position;
	   cp=(long *)&tmaxis[i]->position;
	   ap2=(long *)&tmaxis[i]->actual_position2;
           logMsg("\r\naxis %d:\t%ld\t%ld\t%ld\t%ld\t%x",i,
	      *ap, *cp,*ap2,tmaxis[i]->voltage,(long)tmaxis[i]);
	   semGive (semMEIUPD);
	 } else
	     logMsg("\r\n" __FUNCTION__ ": couldn't take semMEIUPD semaphore.",
		0,0,0,0,0,0);
}
void print_pos_dc (int axis)
{
	long *ap, *cp, *ap2;
	int i;

	i=axis;
  	 if (semTake (semMEIUPD,60)!=ERROR)
  	 {
	    ap=(long *)&tmaxis[i]->actual_position;
	    cp=(long *)&tmaxis[i]->position;
	    ap2=(long *)&tmaxis[i]->actual_position2;
	    logMsg("\r\naxis %d:\tap=%ld\tcp=%ld\tap2=%ld\t",i,
	      *ap, *cp,*ap2,0,0);
	    semGive (semMEIUPD);
	 } else
	     logMsg("\r\n" __FUNCTION__ ": couldn't take semMEIUPD semaphore.",
		0,0,0,0,0,0);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: slc_data_collection
**
** DESCRIPTION:
**	Collect data from the SLC504 AB via DH+.  Also, collection of 
**	counter-weight values from hardware, instrument lift hardware and
**	ctime.  Status of the MEI controllers is produced for the TCC.  If
**	the broadcast is enabled, the datagram is sent.  All this is 
**	done at the base rate of the SLC data collection.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**	slc_read_blok
**	cw_data_collection
**	il_data_collection 
**	time
**	ipsdss_send
**	check_stop_in
**	az_amp_ok
**	alt_amp_ok
**	rot_amp_ok
**
** GLOBALS REFERENCED:
**	sdssdc
**	semMEI
**	semSLC
**	semSLCDC
**	axis_stat[]
**
**=========================================================================
*/
void slc500_data_collection(unsigned long freq)
{
  char status[168*2];
  int stat;

  semSLCDC = semBCreate (SEM_Q_FIFO,SEM_EMPTY);
  slc_freq=freq;
    FOREVER
  {
    if (semTake (semSLCDC,WAIT_FOREVER)!=ERROR)
    {
      if (semTake (semSLC,60)!=ERROR)
      {
        stat = slc_read_blok(1,9,0x85,0,(uint *)&status[0],64);
        stat |= slc_read_blok(1,9,0x85,64,(uint *)&status[128],64);
        stat |= slc_read_blok(1,9,0x85,128,(uint *)&status[256],38);
	semGive (semSLC);
        if (stat==0)
        {
          taskLock();
          swab (&status[0],(char *)(&sdssdc.status.i1),166*2);
          taskUnlock();
        }
      }
    } else {
	logMsg("\r\n" __FUNCTION__ ": couldn't take semSLCDC semahore.",
		0,0,0,0,0,0);
	break;
    }
    cw_data_collection();
    il_data_collection(); 

    if (check_stop_in()) axis_stat[0].stop_ok=0;
    else axis_stat[0].stop_ok=1;
    axis_stat[2].stop_ok=axis_stat[1].stop_ok= axis_stat[0].stop_ok;
    if (az_amp_ok()) axis_stat[0].amp_ok=1;
    else axis_stat[0].amp_ok=0;
    if (alt_amp_ok()) axis_stat[1].amp_ok=1;
    else axis_stat[1].amp_ok=0;
    if (rot_amp_ok()) axis_stat[2].amp_ok=1;
    else axis_stat[2].amp_ok=0;
    if (semTake (semMEI,5)!=ERROR)
    {
      if ((sdssdc.axis_state[0]=axis_state(0))<=2) axis_stat[0].closed_loop=1;
      else axis_stat[0].closed_loop=0;
      if ((sdssdc.axis_state[1]=axis_state(2))<=2) axis_stat[1].closed_loop=1;
      else axis_stat[1].closed_loop=0;
      if ((sdssdc.axis_state[2]=axis_state(4))<=2) axis_stat[2].closed_loop=1;
      else axis_stat[2].closed_loop=0;
      semGive (semMEI);
    }

    if (BCAST_Enable)
      ipsdss_send ((char *)&sdssdc,sizeof(struct SDSS_FRAME));
  }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: DataCollectionTrigger
**
** DESCRIPTION:
**	Interrupt routine for data collection delivers binary semaphores
**	so collection is accomplished at task context.  Also, saves
**	sdssdc structure to shared memory for TPM.  Provides an output pulse
**	as an interrupt to TPM.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**	dc_interrupt
**
** GLOBALS REFERENCED:
**	sdssdc
**	DC_freq
**	slc_freq
**	mei_freq
**	rawtick
**	semMEIDC
**	semSLCDC
**
**=========================================================================
*/
int SM_COPY=TRUE;
void DataCollectionTrigger()
{
  DC_freq++;
  rawtick=tickGet();
  
  if ((semMEIDC!=NULL)&&(!(DC_freq%mei_freq))) semGive (semMEIDC);
  if ((semSLCDC!=NULL)&&(!(DC_freq%slc_freq))) semGive (semSLCDC);
  if (SM_COPY)
  {
    *(short *)SHARE_MEMORY = TRUE;
    bcopy ((char *)&sdssdc,(char *)(SHARE_MEMORY+2),sizeof(struct SDSS_FRAME));
    *(short *)SHARE_MEMORY = FALSE;
    dc_interrupt();
  }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: dc_interrupt
**
** DESCRIPTION:
**	Diagnostic to list the corresponding instrument's counter-weight 
**	parameters.
**
** RETURN VALUES:
**      int 	always zero
**
** CALLS TO:
**      DAC128V_Read_Port
**	cw_read_position
**
** GLOBALS REFERENCED:
**	cw_DIO316
**
**=========================================================================
*/
int dc_interrupt()
{
  unsigned char val;
  int ikey;

  ikey=intLock();
  DIO316_Read_Port (cw_DIO316,DC_INTERRUPT,&val);
  DIO316_Write_Port (cw_DIO316,DC_INTERRUPT,val|DC_INTPULSE);
  DIO316_Write_Port (cw_DIO316,DC_INTERRUPT,val);
  intUnlock(ikey);
  return 0;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: restore_pos
**
** DESCRIPTION:
**	Restore positions after a boot from shared memory
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	sdssdc
**
**=========================================================================
*/
void
restore_pos()
{
  struct SDSS_FRAME *save;
  struct TM_M68K *restore;
  int i;

  if (SM_COPY)
  {
    save=(struct SDSS_FRAME *)(SHARE_MEMORY+2);
    for (i=0;i<3;i++)
    { 
      restore=(struct TM_M68K *) &save->axis[i];
      tm_set_pos(i*2,restore->actual_position);
/*      if (i==2) tm_set_pos((i*2)+1,restore->actual_position2);*/
      if (i==2) tm_set_pos(3,restore->actual_position2);
      if (i==2) tm_set_pos(5,restore->actual_position2);
    }
  }
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: print_ab_status
**
** DESCRIPTION:
**	Print the sdssdc data structure.
**
** RETURN VALUES:
**      void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	sdssdc
**
**=========================================================================
*/
void print_ab_status()
{
printf ("\r\n AB status: status=%p, status.i1.il0=%p status.i1.il4=%p\n",
	&sdssdc.status.status,&sdssdc.status.i1.il0,&sdssdc.status.i1.il4);
printf("status=%d\n",sdssdc.status.status);
printf("errcnt=%d\n",sdssdc.status.errcnt);
printf("az_bump_ccw=%d\n",sdssdc.status.i1.il0.az_bump_ccw);
printf("az_bump_cw=%d\n",sdssdc.status.i1.il0.az_bump_cw);
printf("ops_cart_in_pos=%d\n",sdssdc.status.i1.il0.ops_cart_in_pos);
printf("fiber_cart_pos2=%d\n",sdssdc.status.i1.il0.fiber_cart_pos2);
printf("fiber_cart_pos1=%d\n",sdssdc.status.i1.il0.fiber_cart_pos1);
printf("inst_lift_low_force=%d\n",sdssdc.status.i1.il0.inst_lift_low_force);
printf("inst_lift_high_force=%d\n",sdssdc.status.i1.il0.inst_lift_high_force);
printf("inst_lift_man=%d\n",sdssdc.status.i1.il0.inst_lift_man);
printf("inst_lift_dn=%d\n",sdssdc.status.i1.il0.inst_lift_dn);
printf("inst_lift_sw4=%d\n",sdssdc.status.i1.il0.inst_lift_sw4);
printf("inst_lift_sw3=%d\n",sdssdc.status.i1.il0.inst_lift_sw3);
printf("inst_lift_sw2=%d\n",sdssdc.status.i1.il0.inst_lift_sw2);
printf("inst_lift_sw1=%d\n",sdssdc.status.i1.il0.inst_lift_sw1);
printf("inst_lift_pump_on=%d\n",sdssdc.status.i1.il0.inst_lift_pump_on);
printf("low_lvl_light_req=%d\n",sdssdc.status.i1.il0.low_lvl_light_req);
printf("optical_bench_cls=%d\n",sdssdc.status.i1.il0.optical_bench_cls);
printf("optical_bench_opn=%d\n",sdssdc.status.i1.il0.optical_bench_opn);
printf("ops_cart_in_house=%d\n",sdssdc.status.i1.il0.ops_cart_in_house);
printf("dog_house_door_cls=%d\n",sdssdc.status.i1.il0.dog_house_door_cls);
printf("dog_house_door_opn=%d\n",sdssdc.status.i1.il0.dog_house_door_opn);
printf("dog_house_ccw_pad=%d\n",sdssdc.status.i1.il0.dog_house_ccw_pad);
printf("dog_house_cw_pad=%d\n",sdssdc.status.i1.il0.dog_house_cw_pad);
printf("sad_latch_opn_cmd=%d\n",sdssdc.status.i1.il4.sad_latch_opn_cmd);
printf("sad_latch_cls_cmd=%d\n",sdssdc.status.i1.il4.sad_latch_cls_cmd);
printf("sec_latch_opn_cmd=%d\n",sdssdc.status.i1.il4.sec_latch_opn_cmd);
printf("sec_latch_cls_cmd=%d\n",sdssdc.status.i1.il4.sec_latch_cls_cmd);
printf("pri_latch_opn_cmd=%d\n",sdssdc.status.i1.il4.pri_latch_opn_cmd);
printf("pri_latch_cls_cmd=%d\n",sdssdc.status.i1.il4.pri_latch_cls_cmd);
printf("sad_man_valve_cls=%d\n",sdssdc.status.i1.il4.sad_man_valve_cls);
printf("sec_man_valve_cls=%d\n",sdssdc.status.i1.il4.sec_man_valve_cls);
printf("inst_man_valve_cls=%d\n",sdssdc.status.i1.il4.inst_man_valve_cls);
printf("ilcb_pres_good=%d\n",sdssdc.status.i1.il4.ilcb_pres_good);
printf("rot_pos_370_ccw=%d\n",sdssdc.status.i1.il4.rot_pos_370_ccw);
printf("rot_neg_190_cw=%d\n",sdssdc.status.i1.il4.rot_neg_190_cw);
printf("rot_inst_chg_b=%d\n",sdssdc.status.i1.il4.rot_inst_chg_b);
printf("rot_inst_chg_a=%d\n",sdssdc.status.i1.il4.rot_inst_chg_a);
printf("spec_lens2=%d\n",sdssdc.status.i1.il8.spec_lens2);
printf("spec_lens1=%d\n",sdssdc.status.i1.il8.spec_lens1);
printf("inst_id3_4=%d\n",sdssdc.status.i1.il8.inst_id3_4);
printf("inst_id3_3=%d\n",sdssdc.status.i1.il8.inst_id3_3);
printf("inst_id3_2=%d\n",sdssdc.status.i1.il8.inst_id3_2);
printf("inst_id3_1=%d\n",sdssdc.status.i1.il8.inst_id3_1);
printf("inst_id2_4=%d\n",sdssdc.status.i1.il8.inst_id2_4);
printf("inst_id2_3=%d\n",sdssdc.status.i1.il8.inst_id2_3);
printf("inst_id2_2=%d\n",sdssdc.status.i1.il8.inst_id2_2);
printf("inst_id2_1=%d\n",sdssdc.status.i1.il8.inst_id2_1);
printf("inst_id1_4=%d\n",sdssdc.status.i1.il8.inst_id1_4);
printf("inst_id1_3=%d\n",sdssdc.status.i1.il8.inst_id1_3);
printf("inst_id1_2=%d\n",sdssdc.status.i1.il8.inst_id1_2);
printf("inst_id1_1=%d\n",sdssdc.status.i1.il8.inst_id1_1);
printf("prot_bolt2_cls=%d\n",sdssdc.status.i1.il8.prot_bolt2_cls);
printf("prot_bolt2_opn=%d\n",sdssdc.status.i1.il8.prot_bolt2_opn);
printf("prot_bolt1_cls=%d\n",sdssdc.status.i1.il8.prot_bolt1_cls);
printf("prot_bolt1_opn=%d\n",sdssdc.status.i1.il8.prot_bolt1_opn);
printf("sec_latch3_cls=%d\n",sdssdc.status.i1.il8.sec_latch3_cls);
printf("sec_latch3_opn=%d\n",sdssdc.status.i1.il8.sec_latch3_opn);
printf("sec_latch2_cls=%d\n",sdssdc.status.i1.il8.sec_latch2_cls);
printf("sec_latch2_opn=%d\n",sdssdc.status.i1.il8.sec_latch2_opn);
printf("sec_latch1_cls=%d\n",sdssdc.status.i1.il8.sec_latch1_cls);
printf("sec_latch1_opn=%d\n",sdssdc.status.i1.il8.sec_latch1_opn);
printf("pri_latch3_cls=%d\n",sdssdc.status.i1.il8.pri_latch3_cls);
printf("pri_latch3_opn=%d\n",sdssdc.status.i1.il8.pri_latch3_opn);
printf("pri_latch2_cls=%d\n",sdssdc.status.i1.il8.pri_latch2_cls);
printf("pri_latch2_opn=%d\n",sdssdc.status.i1.il8.pri_latch2_opn);
printf("pri_latch1_cls=%d\n",sdssdc.status.i1.il8.pri_latch1_cls);
printf("pri_latch1_opn=%d\n",sdssdc.status.i1.il8.pri_latch1_opn);
printf("slit_head_2_in_place=%d\n",sdssdc.status.i1.il9.slit_head_2_in_place);
printf("slit_head_latch2_opn=%d\n",sdssdc.status.i1.il9.slit_head_latch2_opn);
printf("slit_head_door2_cls=%d\n",sdssdc.status.i1.il9.slit_head_door2_cls);
printf("slit_head_door2_opn=%d\n",sdssdc.status.i1.il9.slit_head_door2_opn);
printf("slit_head_1_in_place=%d\n",sdssdc.status.i1.il9.slit_head_1_in_place);
printf("slit_head_latch1_opn=%d\n",sdssdc.status.i1.il9.slit_head_latch1_opn);
printf("slit_head_door1_cls=%d\n",sdssdc.status.i1.il9.slit_head_door1_cls);
printf("slit_head_door1_opn=%d\n",sdssdc.status.i1.il9.slit_head_door1_opn);
printf("sad_mount2=%d\n",sdssdc.status.i1.il9.sad_mount2);
printf("sad_mount1=%d\n",sdssdc.status.i1.il9.sad_mount1);
printf("sad_latch2_cls=%d\n",sdssdc.status.i1.il9.sad_latch2_cls);
printf("sad_latch2_opn=%d\n",sdssdc.status.i1.il9.sad_latch2_opn);
printf("sad_latch1_cls=%d\n",sdssdc.status.i1.il9.sad_latch1_cls);
printf("sad_latch1_opn=%d\n",sdssdc.status.i1.il9.sad_latch1_opn);

printf("solenoid_engage_sw=%d\n",sdssdc.status.i1.il12.solenoid_engage_sw);
printf("inst_lift_auto=%d\n",sdssdc.status.i1.il12.inst_lift_auto);
printf("man_lift_up=%d\n",sdssdc.status.i1.il12.man_lift_up);
printf("man_lift_dn=%d\n",sdssdc.status.i1.il12.man_lift_dn);

printf("leaf_8_closed_stat=%d\n",sdssdc.status.i1.il13.leaf_8_closed_stat);
printf("leaf_8_open_stat=%d\n",sdssdc.status.i1.il13.leaf_8_open_stat);
printf("leaf_7_closed_stat=%d\n",sdssdc.status.i1.il13.leaf_7_closed_stat);
printf("leaf_7_open_stat=%d\n",sdssdc.status.i1.il13.leaf_7_open_stat);
printf("leaf_6_closed_stat=%d\n",sdssdc.status.i1.il13.leaf_6_closed_stat);
printf("leaf_6_open_stat=%d\n",sdssdc.status.i1.il13.leaf_6_open_stat);
printf("leaf_5_closed_stat=%d\n",sdssdc.status.i1.il13.leaf_5_closed_stat);
printf("leaf_5_open_stat=%d\n",sdssdc.status.i1.il13.leaf_5_open_stat);
printf("leaf_4_closed_stat=%d\n",sdssdc.status.i1.il13.leaf_4_closed_stat);
printf("leaf_4_open_stat=%d\n",sdssdc.status.i1.il13.leaf_4_open_stat);
printf("leaf_3_closed_stat=%d\n",sdssdc.status.i1.il13.leaf_3_closed_stat);
printf("leaf_3_open_stat=%d\n",sdssdc.status.i1.il13.leaf_3_open_stat);
printf("leaf_2_closed_stat=%d\n",sdssdc.status.i1.il13.leaf_2_closed_stat);
printf("leaf_2_open_stat=%d\n",sdssdc.status.i1.il13.leaf_2_open_stat);
printf("leaf_1_closed_stat=%d\n",sdssdc.status.i1.il13.leaf_1_closed_stat);
printf("leaf_1_open_stat=%d\n",sdssdc.status.i1.il13.leaf_1_open_stat);
printf("hgcd_4_stat=%d\n",sdssdc.status.i1.il13.hgcd_4_stat);
printf("hgcd_3_stat=%d\n",sdssdc.status.i1.il13.hgcd_3_stat);
printf("hgcd_2_stat=%d\n",sdssdc.status.i1.il13.hgcd_2_stat);
printf("hgcd_1_stat=%d\n",sdssdc.status.i1.il13.hgcd_1_stat);
printf("ne_4_stat=%d\n",sdssdc.status.i1.il13.ne_4_stat);
printf("ne_3_stat=%d\n",sdssdc.status.i1.il13.ne_3_stat);
printf("ne_2_stat=%d\n",sdssdc.status.i1.il13.ne_2_stat);
printf("ne_1_stat=%d\n",sdssdc.status.i1.il13.ne_1_stat);
printf("ff_4_stat=%d\n",sdssdc.status.i1.il13.ff_4_stat);
printf("ff_3_stat=%d\n",sdssdc.status.i1.il13.ff_3_stat);
printf("ff_2_stat=%d\n",sdssdc.status.i1.il13.ff_2_stat);
printf("ff_1_stat=%d\n",sdssdc.status.i1.il13.ff_1_stat);
printf("man_hgcd_lamp_on_cmd=%d\n",sdssdc.status.i1.il14.man_hgcd_lamp_on_cmd);
printf("man_ne_lamp_on_cmd=%d\n",sdssdc.status.i1.il14.man_ne_lamp_on_cmd);
printf("man_ff_lamp_on_cmd=%d\n",sdssdc.status.i1.il14.man_ff_lamp_on_cmd);
printf("man_ff_scrn_en_cmd=%d\n",sdssdc.status.i1.il14.man_ff_scrn_en_cmd);
printf("man_ff_scrn_opn_cmd=%d\n",sdssdc.status.i1.il14.man_ff_scrn_opn_cmd);

printf("low_lvl_light_2=%d\n",sdssdc.status.o1.ol1.low_lvl_light_2);
printf("low_lvl_light_1=%d\n",sdssdc.status.o1.ol1.low_lvl_light_1);
printf("az_stow_light=%d\n",sdssdc.status.o1.ol1.az_stow_light);
printf("stop_bypass_strobe=%d\n",sdssdc.status.o1.ol1.stop_bypass_strobe);
printf("az_stow_center_light=%d\n",sdssdc.status.o1.ol1.az_stow_center_light);
printf("inst_lift_perm=%d\n",sdssdc.status.o1.ol1.inst_lift_perm);
printf("inst_lift_dn_4=%d\n",sdssdc.status.o1.ol1.inst_lift_dn_4);
printf("inst_lift_dn_3=%d\n",sdssdc.status.o1.ol1.inst_lift_dn_3);
printf("inst_lift_dn_2=%d\n",sdssdc.status.o1.ol1.inst_lift_dn_2);
printf("inst_lift_dn_1=%d\n",sdssdc.status.o1.ol1.inst_lift_dn_1);
printf("inst_lift_up_1=%d\n",sdssdc.status.o1.ol1.inst_lift_up_1);
printf("inst_lift_up_1=%d\n",sdssdc.status.o1.ol1.inst_lift_up_2);
printf("inst_lift_up_3=%d\n",sdssdc.status.o1.ol1.inst_lift_up_3);
printf("inst_lift_up_3=%d\n",sdssdc.status.o1.ol1.inst_lift_up_4);
printf("inst_lift_high_psi=%d\n",sdssdc.status.o1.ol1.inst_lift_high_psi);
printf("sad_latch2_cls_led=%d\n",sdssdc.status.o1.ol4.sad_latch2_cls_led);
printf("sad_latch2_opn_led=%d\n",sdssdc.status.o1.ol4.sad_latch2_opn_led);
printf("sad_latch1_cls_led=%d\n",sdssdc.status.o1.ol4.sad_latch1_cls_led);
printf("sad_latch1_opn_led=%d\n",sdssdc.status.o1.ol4.sad_latch1_opn_led);
printf("sad_man_req=%d\n",sdssdc.status.o1.ol4.sad_man_req);
printf("sad_latch_perm=%d\n",sdssdc.status.o1.ol4.sad_latch_perm);
printf("sad_unlatch_perm=%d\n",sdssdc.status.o1.ol4.sad_unlatch_perm);
printf("sec_man_req=%d\n",sdssdc.status.o1.ol4.sec_man_req);
printf("sec_latch_perm=%d\n",sdssdc.status.o1.ol4.sec_latch_perm);
printf("sec_unlatch_perm=%d\n",sdssdc.status.o1.ol4.sec_unlatch_perm);
printf("inst_man_req=%d\n",sdssdc.status.o1.ol4.inst_man_req);
printf("inst_latch_perm=%d\n",sdssdc.status.o1.ol4.inst_latch_perm);
printf("inst_unlatch_perm=%d\n",sdssdc.status.o1.ol4.inst_unlatch_perm);
printf("ilcb_pres_led=%d\n",sdssdc.status.o1.ol4.ilcb_pres_led);
printf("prot_bolt2_cls_led=%d\n",sdssdc.status.o1.ol5.prot_bolt2_cls_led);
printf("prot_bolt2_opn_led=%d\n",sdssdc.status.o1.ol5.prot_bolt2_opn_led);
printf("prot_bolt1_cls_led=%d\n",sdssdc.status.o1.ol5.prot_bolt1_cls_led);
printf("prot_bolt1_opn_led=%d\n",sdssdc.status.o1.ol5.prot_bolt1_opn_led);
printf("sec_latch3_cls_led=%d\n",sdssdc.status.o1.ol5.sec_latch3_cls_led);
printf("sec_latch3_opn_led=%d\n",sdssdc.status.o1.ol5.sec_latch3_opn_led);
printf("sec_latch2_cls_led=%d\n",sdssdc.status.o1.ol5.sec_latch2_cls_led);
printf("sec_latch2_opn_led=%d\n",sdssdc.status.o1.ol5.sec_latch2_opn_led);
printf("sec_latch1_cls_led=%d\n",sdssdc.status.o1.ol5.sec_latch1_cls_led);
printf("sec_latch1_opn_led=%d\n",sdssdc.status.o1.ol5.sec_latch1_opn_led);
printf("inst_latch3_cls_led=%d\n",sdssdc.status.o1.ol5.inst_latch3_cls_led);
printf("inst_latch3_opn_led=%d\n",sdssdc.status.o1.ol5.inst_latch3_opn_led);
printf("inst_latch2_cls_led=%d\n",sdssdc.status.o1.ol5.inst_latch2_cls_led);
printf("inst_latch2_opn_led=%d\n",sdssdc.status.o1.ol5.inst_latch2_opn_led);
printf("inst_latch1_cls_led=%d\n",sdssdc.status.o1.ol5.inst_latch1_cls_led);
printf("inst_latch1_opn_led=%d\n",sdssdc.status.o1.ol5.inst_latch1_opn_led);

printf("audio_warning_2=%d\n",sdssdc.status.o1.ol6.audio_warning_2);
printf("purge_valve_permit=%d\n",sdssdc.status.o1.ol6.purge_valve_permit);

printf("slit_latch2_opn_perm=%d\n",sdssdc.status.o1.ol9.slit_latch2_opn_perm);
printf("slit_dr2_opn_perm=%d\n",sdssdc.status.o1.ol9.slit_dr2_opn_perm);
printf("slit_dr2_cls_perm=%d\n",sdssdc.status.o1.ol9.slit_dr2_cls_perm);
printf("slit_latch1_opn_perm=%d\n",sdssdc.status.o1.ol9.slit_latch1_opn_perm);
printf("slit_dr1_opn_perm=%d\n",sdssdc.status.o1.ol9.slit_dr1_opn_perm);
printf("slit_dr1_cls_perm=%d\n",sdssdc.status.o1.ol9.slit_dr1_cls_perm);
printf("flex_analog_config=%d\n",sdssdc.status.o1.ol12.flex_analog_config);
printf("audio_warning=%d\n",sdssdc.status.o1.ol14.audio_warning);
printf("hgcd_lamps_on_pmt=%d\n",sdssdc.status.o1.ol14.hgcd_lamps_on_pmt);
printf("ne_lamps_on_pmt=%d\n",sdssdc.status.o1.ol14.ne_lamps_on_pmt);
printf("ff_lamps_on_pmt=%d\n",sdssdc.status.o1.ol14.ff_lamps_on_pmt);
printf("ff_screen_open_pmt=%d\n",sdssdc.status.o1.ol14.ff_screen_open_pmt);

printf("dcm_status=%d\n",sdssdc.status.i2.il0.dcm_status);
printf("low_lvl_lighting_req=%d\n",sdssdc.status.i2.il0.low_lvl_lighting_req);
printf("wind_alt_perm=%d\n",sdssdc.status.i2.il0.wind_alt_perm);
printf("wind_az_perm=%d\n",sdssdc.status.i2.il0.wind_az_perm);
printf("wind_alt1_fault=%d\n",sdssdc.status.i2.il0.wind_alt1_fault);
printf("wind_az3_fault=%d\n",sdssdc.status.i2.il0.wind_az3_fault);
printf("wind_az2_fault=%d\n",sdssdc.status.i2.il0.wind_az2_fault);
printf("wind_az1_fault=%d\n",sdssdc.status.i2.il0.wind_az1_fault);
printf("az_lvdt_error=%d\n",sdssdc.status.i2.az_lvdt_error);
printf("alt_lvdt_error=%d\n",sdssdc.status.i2.alt_lvdt_error);
printf("az_primary_drv=%d\n",sdssdc.status.i2.az_primary_drv);
printf("az_feed_forward_drv=%d\n",sdssdc.status.i2.az_feed_forward_drv);
printf("alt_primary_drv=%d\n",sdssdc.status.i2.alt_primary_drv);

printf("inst_chg_pos_light=%d\n",sdssdc.status.o2.ol0.inst_chg_pos_light);
printf("inst_chg_pos_light=%d\n",sdssdc.status.o2.ol0.stow_pos_light);
printf("wind_mtr_dn_perm=%d\n",sdssdc.status.o2.ol0.wind_mtr_dn_perm);
printf("wind_mtr_up_perm=%d\n",sdssdc.status.o2.ol0.wind_mtr_up_perm);
printf("wind_mtr_ccw_perm=%d\n",sdssdc.status.o2.ol0.wind_mtr_ccw_perm);
printf("wind_mtr_cw_perm=%d\n",sdssdc.status.o2.ol0.wind_mtr_cw_perm);

printf("az_1_voltage=%d\n",sdssdc.status.i3.az_1_voltage);
printf("az_1_current=%d\n",sdssdc.status.i3.az_1_current);
printf("az_2_voltage=%d\n",sdssdc.status.i3.az_2_voltage);
printf("az_2_current=%d\n",sdssdc.status.i3.az_2_current);
printf("alt_1_voltage=%d\n",sdssdc.status.i3.alt_1_voltage);
printf("alt_1_current=%d\n",sdssdc.status.i3.alt_1_current);
printf("alt_2_voltage=%d\n",sdssdc.status.i3.alt_2_voltage);
printf("alt_2_current=%d\n",sdssdc.status.i3.alt_2_current);

printf("alt_position=%d\n",sdssdc.status.i4.alt_position);
printf("rot_1_voltage=%d\n",sdssdc.status.i4.rot_1_voltage);
printf("rot_1_current=%d\n",sdssdc.status.i4.rot_1_current);
printf("umbilical_dist=%d\n",sdssdc.status.i4.umbilical_dist);
printf("inst_lift_force=%d\n",sdssdc.status.i4.inst_lift_force);
printf("inst_lift_dist=%d\n",sdssdc.status.i4.inst_lift_dist);

printf("counterweight_1_pos=%d\n",sdssdc.status.i5.counterweight_1_pos);
printf("counterweight_2_pos=%d\n",sdssdc.status.i5.counterweight_2_pos);
printf("counterweight_3_pos=%d\n",sdssdc.status.i5.counterweight_3_pos);
printf("counterweight_4_pos=%d\n",sdssdc.status.i5.counterweight_4_pos);

printf("s_wind_stop=%d\n",sdssdc.status.i6.il0.s_wind_stop);
printf("w_lower_stop=%d\n",sdssdc.status.i6.il0.w_lower_stop);
printf("e_lower_stop=%d\n",sdssdc.status.i6.il0.e_lower_stop);
printf("s_lower_stop=%d\n",sdssdc.status.i6.il0.s_lower_stop);
printf("n_lower_stop=%d\n",sdssdc.status.i6.il0.n_lower_stop);
printf("w_rail_stop=%d\n",sdssdc.status.i6.il0.w_rail_stop);
printf("s_rail_stop=%d\n",sdssdc.status.i6.il0.s_rail_stop);
printf("n_rail_stop=%d\n",sdssdc.status.i6.il0.n_rail_stop);
printf("n_fork_stop=%d\n",sdssdc.status.i6.il0.n_fork_stop);
printf("n_wind_stop=%d\n",sdssdc.status.i6.il0.n_wind_stop);
printf("fiber_signal_loss=%d\n",sdssdc.status.i6.il0.fiber_signal_loss);
printf("cr_stop=%d\n",sdssdc.status.i6.il0.cr_stop);
printf("tcc_stop=%d\n",sdssdc.status.i6.il0.tcc_stop);

printf("az_stow_3b=%d\n",sdssdc.status.i6.il0.az_stow_3b);
printf("wind_az_plc_perm_in=%d\n",sdssdc.status.i6.il0.wind_az_plc_perm_in);
printf("az_plc_perm_in=%d\n",sdssdc.status.i6.il0.az_plc_perm_in);
printf("wind_az_mtr_perm_in=%d\n",sdssdc.status.i6.il0.wind_az_mtr_perm_in);
printf("az_mtr2_perm_in=%d\n",sdssdc.status.i6.il0.az_mtr2_perm_in);
printf("az_mtr1_perm_in=%d\n",sdssdc.status.i6.il0.az_mtr1_perm_in);
printf("az_mtr_ccw_perm_in=%d\n",sdssdc.status.i6.il0.az_mtr_ccw_perm_in);
printf("az_mtr_cw_perm_in=%d\n",sdssdc.status.i6.il0.az_mtr_cw_perm_in);
printf("az_stow_3a=%d\n",sdssdc.status.i6.il0.az_stow_3a);
printf("wind_alt_plc_perm_in=%d\n",sdssdc.status.i6.il0.wind_alt_plc_perm_in);
printf("alt_plc_perm_in=%d\n",sdssdc.status.i6.il0.alt_plc_perm_in);
printf("wind_alt_mtr_perm_in=%d\n",sdssdc.status.i6.il0.wind_alt_mtr_perm_in);
printf("alt_mtr2_perm_in=%d\n",sdssdc.status.i6.il0.alt_mtr2_perm_in);
printf("alt_mtr1_perm_in=%d\n",sdssdc.status.i6.il0.alt_mtr1_perm_in);
printf("alt_mtr_dn_perm_in=%d\n",sdssdc.status.i6.il0.alt_mtr_dn_perm_in);
printf("alt_mtr_up_perm_in=%d\n",sdssdc.status.i6.il0.alt_mtr_up_perm_in);

printf("az_stow_1b=%d\n",sdssdc.status.i7.il0.az_stow_1b);
printf("az_stow_1a=%d\n",sdssdc.status.i7.il0.az_stow_1a);
printf("alt_grt_18d6_limit_1=%d\n",sdssdc.status.i7.il0.alt_grt_18d6_limit_1);
printf("az_109_131_limit_1=%d\n",sdssdc.status.i7.il0.az_109_131_limit_1);
printf("bldg_on_alt=%d\n",sdssdc.status.i7.il0.bldg_on_alt);
printf("alt_les_90d5_limit=%d\n",sdssdc.status.i7.il0.alt_les_90d5_limit);
printf("alt_locking_pin_out=%d\n",sdssdc.status.i7.il0.alt_locking_pin_out);
printf("alt_grt_0d3_limit=%d\n",sdssdc.status.i7.il0.alt_grt_0d3_limit);
printf("alt_les_2d5_limit=%d\n",sdssdc.status.i7.il0.alt_les_2d5_limit);
printf("hatch_cls=%d\n",sdssdc.status.i7.il0.hatch_cls);
printf("rot_plc_perm_in=%d\n",sdssdc.status.i7.il0.rot_plc_perm_in);
printf("bldg_perm_in=%d\n",sdssdc.status.i7.il0.bldg_perm_in);
printf("rot_mtr_perm_in=%d\n",sdssdc.status.i7.il0.rot_mtr_perm_in);
printf("rot_mtr_ccw_perm_in=%d\n",sdssdc.status.i7.il0.rot_mtr_ccw_perm_in);
printf("rot_mtr_cw_perm_in=%d\n",sdssdc.status.i7.il0.rot_mtr_cw_perm_in);
printf("az_pos_410b_ccw=%d\n",sdssdc.status.i7.il0.az_pos_410b_ccw);
printf("az_neg_170b_cw=%d\n",sdssdc.status.i7.il0.az_neg_170b_cw);
printf("az_pos_410a_ccw=%d\n",sdssdc.status.i7.il0.az_pos_410a_ccw);
printf("az_neg_170a_cw=%d\n",sdssdc.status.i7.il0.az_neg_170a_cw);
printf("az_dir_ccw=%d\n",sdssdc.status.i7.il0.az_dir_ccw);
printf("az_dir_cw=%d\n",sdssdc.status.i7.il0.az_dir_cw);
printf("alt_velocity_limit=%d\n",sdssdc.status.i7.il0.alt_velocity_limit);
printf("alt_slip=%d\n",sdssdc.status.i7.il0.alt_slip);
printf("alt_grt_18d6_limit_2=%d\n",sdssdc.status.i7.il0.alt_grt_18d6_limit_2);
printf("deg_15_stop_ext=%d\n",sdssdc.status.i7.il0.deg_15_stop_ext);
printf("az_stow_2b=%d\n",sdssdc.status.i7.il0.az_stow_2b);
printf("az_stow_2a=%d\n",sdssdc.status.i7.il0.az_stow_2a);
printf("bldg_clear_alt=%d\n",sdssdc.status.i7.il0.bldg_clear_alt);
printf("alt_grt_83_limit_1=%d\n",sdssdc.status.i7.il0.alt_grt_83_limit_1);

printf("rot_velocity_limit=%d\n",sdssdc.status.i8.il0.rot_velocity_limit);
printf("rot_slip=%d\n",sdssdc.status.i8.il0.rot_slip);
printf("rot_pos_380b_ccw=%d\n",sdssdc.status.i8.il0.rot_pos_380b_ccw);
printf("rot_neg_200b_cw=%d\n",sdssdc.status.i8.il0.rot_neg_200b_cw);
printf("rot_pos_380a_ccw=%d\n",sdssdc.status.i8.il0.rot_pos_380a_ccw);
printf("rot_neg_200a_cw=%d\n",sdssdc.status.i8.il0.rot_neg_200a_cw);
printf("rot_dir_ccw=%d\n",sdssdc.status.i8.il0.rot_dir_ccw);
printf("rot_dir_cw=%d\n",sdssdc.status.i8.il0.rot_dir_cw);
printf("az_velocity_limit=%d\n",sdssdc.status.i8.il0.az_velocity_limit);
printf("az_slip=%d\n",sdssdc.status.i8.il0.az_slip);
printf("bldg_clear_az=%d\n",sdssdc.status.i8.il0.bldg_clear_az);
printf("alt_grt_83_limit_2=%d\n",sdssdc.status.i8.il0.alt_grt_83_limit_2);
printf("az_109_131_limit_2=%d\n",sdssdc.status.i8.il0.az_109_131_limit_2);
printf("bldg_on_az=%d\n",sdssdc.status.i8.il0.bldg_on_az);
printf("alt_grt_18d6_limit_3=%d\n",sdssdc.status.i8.il0.alt_grt_18d6_limit_3);
printf("t_bar_latch_stat=%d\n",sdssdc.status.i8.il0.t_bar_latch_stat);
printf("deg_15_stop_ret=%d\n",sdssdc.status.i8.il0.deg_15_stop_ret);
printf("e_stop_byp_sw=%d\n",sdssdc.status.i8.il0.e_stop_byp_sw);
printf("umbilical_strain_sw=%d\n",sdssdc.status.i8.il0.umbilical_strain_sw);
printf("rot_mtr_rdy=%d\n",sdssdc.status.i8.il0.rot_mtr_rdy);
printf("alt_mtr2_rdy=%d\n",sdssdc.status.i8.il0.alt_mtr2_rdy);
printf("alt_mtr1_rdy=%d\n",sdssdc.status.i8.il0.alt_mtr1_rdy);
printf("az_mtr2_rdy=%d\n",sdssdc.status.i8.il0.az_mtr2_rdy);
printf("az_mtr1_rdy=%d\n",sdssdc.status.i8.il0.az_mtr1_rdy);
printf("az_pos_400_ccw=%d\n",sdssdc.status.i8.il0.az_pos_400_ccw);
printf("az_neg_165_cw=%d\n",sdssdc.status.i8.il0.az_neg_165_cw);
printf("az_110_130_limit=%d\n",sdssdc.status.i8.il0.az_110_130_limit);
printf("az_stow_cntr_sw=%d\n",sdssdc.status.i8.il0.az_stow_cntr_sw);

printf("alt_brake_dis_stat=%d\n",sdssdc.status.i9.il0.alt_brake_dis_stat);
printf("alt_brake_en_stat=%d\n",sdssdc.status.i9.il0.alt_brake_en_stat);
printf("az_brake_dis_stat =%d\n",sdssdc.status.i9.il0.az_brake_dis_stat );
printf("az_brake_en_stat=%d\n",sdssdc.status.i9.il0.az_brake_en_stat);
printf("clamp_dis_stat=%d\n",sdssdc.status.i9.il0.clamp_dis_stat);
printf("clamp_en_stat=%d\n",sdssdc.status.i9.il0.clamp_en_stat);
printf("t_bar_unlatch_stat=%d\n",sdssdc.status.i9.il0.t_bar_unlatch_stat);
printf("s2_c7_bypass_sw=%d\n",sdssdc.status.i9.il0.s2_c7_bypass_sw);
printf("s2_c6_bypass_sw=%d\n",sdssdc.status.i9.il0.s2_c6_bypass_sw);
printf("s2_c5_bypass_sw=%d\n",sdssdc.status.i9.il0.s2_c5_bypass_sw);
printf("s2_c4_bypass_sw=%d\n",sdssdc.status.i9.il0.s2_c4_bypass_sw);
printf("s2_c3_bypass_sw=%d\n",sdssdc.status.i9.il0.s2_c3_bypass_sw);
printf("s2_c2_bypass_sw=%d\n",sdssdc.status.i9.il0.s2_c2_bypass_sw);
printf("s2_c1_bypass_sw=%d\n",sdssdc.status.i9.il0.s2_c1_bypass_sw);
printf("s2_c0_bypass_sw=%d\n",sdssdc.status.i9.il0.s2_c0_bypass_sw);
printf("s1_c7_bypass_sw=%d\n",sdssdc.status.i9.il0.s1_c7_bypass_sw);
printf("s1_c6_bypass_sw=%d\n",sdssdc.status.i9.il0.s1_c6_bypass_sw);
printf("s1_c5_bypass_sw=%d\n",sdssdc.status.i9.il0.s1_c5_bypass_sw);
printf("s1_c4_bypass_sw=%d\n",sdssdc.status.i9.il0.s1_c4_bypass_sw);
printf("s1_c3_bypass_sw=%d\n",sdssdc.status.i9.il0.s1_c3_bypass_sw);
printf("s1_c2_bypass_sw=%d\n",sdssdc.status.i9.il0.s1_c2_bypass_sw);
printf("s1_c1_bypass_sw=%d\n",sdssdc.status.i9.il0.s1_c1_bypass_sw);
printf("s1_c0_bypass_sw=%d\n",sdssdc.status.i9.il0.s1_c0_bypass_sw);

printf("s_ll_led=%d\n",sdssdc.status.o11.ol0.s_ll_led);
printf("n_ll_led=%d\n",sdssdc.status.o11.ol0.n_ll_led);
printf("w_rail_led=%d\n",sdssdc.status.o11.ol0.w_rail_led);
printf("s_rail_led=%d\n",sdssdc.status.o11.ol0.s_rail_led);
printf("n_rail_led=%d\n",sdssdc.status.o11.ol0.n_rail_led);
printf("rot_plc_perm=%d\n",sdssdc.status.o11.ol0.rot_plc_perm);
printf("rot_mtr_ccw_perm=%d\n",sdssdc.status.o11.ol0.rot_mtr_ccw_perm);
printf("rot_mtr_cw_perm=%d\n",sdssdc.status.o11.ol0.rot_mtr_cw_perm);
printf("wind_az_plc_perm=%d\n",sdssdc.status.o11.ol0.wind_az_plc_perm);
printf("az_plc_perm=%d\n",sdssdc.status.o11.ol0.az_plc_perm);
printf("az_mtr_ccw_perm=%d\n",sdssdc.status.o11.ol0.az_mtr_ccw_perm);
printf("az_mtr_cw_perm=%d\n",sdssdc.status.o11.ol0.az_mtr_cw_perm);
printf("wind_alt_plc_perm=%d\n",sdssdc.status.o11.ol0.wind_alt_plc_perm);
printf("alt_plc_perm=%d\n",sdssdc.status.o11.ol0.alt_plc_perm);
printf("alt_mtr_dn_perm=%d\n",sdssdc.status.o11.ol0.alt_mtr_dn_perm);
printf("alt_mtr_up_perm=%d\n",sdssdc.status.o11.ol0.alt_mtr_up_perm);
printf("clamp_en=%d\n",sdssdc.status.o11.ol0.clamp_en);
printf("clamp_dis=%d\n",sdssdc.status.o11.ol0.clamp_dis);
printf("t_bar_unlatch_perm=%d\n",sdssdc.status.o11.ol0.t_bar_unlatch_perm);
printf("t_bar_latch_perm=%d\n",sdssdc.status.o11.ol0.t_bar_latch_perm);
printf("lift_pump_on=%d\n",sdssdc.status.o11.ol0.lift_pump_on);
printf("deg_15_stop_ret_perm=%d\n",sdssdc.status.o11.ol0.deg_15_stop_ret_perm);
printf("deg_15_stop_ext_perm=%d\n",sdssdc.status.o11.ol0.deg_15_stop_ext_perm);
printf("lift_solenoid_en=%d\n",sdssdc.status.o11.ol0.lift_solenoid_en);
printf("s_wind_led=%d\n",sdssdc.status.o11.ol0.s_wind_led);
printf("n_fork_led=%d\n",sdssdc.status.o11.ol0.n_fork_led);
printf("n_wind_led=%d\n",sdssdc.status.o11.ol0.n_wind_led);
printf("w_ll_led=%d\n",sdssdc.status.o11.ol0.w_ll_led);
printf("e_ll_led=%d\n",sdssdc.status.o11.ol0.e_ll_led);

printf("umbilical_up_dn=%d\n",sdssdc.status.o12.ol0.umbilical_up_dn);
printf("umbilical_on_off=%d\n",sdssdc.status.o12.ol0.umbilical_on_off);
printf("alt_brake_en=%d\n",sdssdc.status.o12.ol0.alt_brake_en);
printf("alt_brake_dis=%d\n",sdssdc.status.o12.ol0.alt_brake_dis);
printf("az_brake_en=%d\n",sdssdc.status.o12.ol0.az_brake_en);
printf("az_brake_dis=%d\n",sdssdc.status.o12.ol0.az_brake_dis);

}
