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
#include "assert.h"
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
#include "dscTrace.h"

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
struct TM_M68K *tmaxis[NAXIS] = {
   (struct TM_M68K *)&sdssdc.axis[AZIMUTH],
   (struct TM_M68K *)&sdssdc.axis[ALTITUDE],
   (struct TM_M68K *)&sdssdc.axis[INSTRUMENT]
};
struct AXIS_STAT axis_stat[NAXIS] = {0x0, 0x0, 0x0};
struct AXIS_STAT persistent_axis_stat[NAXIS]={0x0, 0x0, 0x0};
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
void
mei_data_collection(unsigned long freq)
{
	int i;
	int rotate;
	
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
void
slc500_data_collection(unsigned long freq)
{
   char status[168*2 + 1];
   char status_b10[12 + 1];
   int stat;

   assert(sizeof(status) == sizeof(struct AB_SLC500) + 1);
   assert(sizeof(status_b10) == sizeof(struct B10) + 1);
   status[sizeof(status) - 1] = status_b10[sizeof(status_b10) - 1] = '\a';
   
   semSLCDC = semBCreate(SEM_Q_FIFO, SEM_EMPTY);
   slc_freq=freq;
   
   for(;;) {
      if(semTake(semSLCDC, WAIT_FOREVER) == ERROR) {
	 TRACE(0, "couldn't take semSLCDC semahore.", 0, 0);
	 break;
      }

      if(semTake(semSLC,60) != ERROR) {
	 stat  = slc_read_blok(1,9,BIT_FILE, 0, (uint *)&status[0], 64);
	 stat |= slc_read_blok(1,9,BIT_FILE, 64, (uint *)&status[128], 64);
	 stat |= slc_read_blok(1,9,BIT_FILE, 128, (uint *)&status[256], 38);
	 
	 stat |= slc_read_blok(1,10,BIT_FILE, 0, (uint *)status_b10,
			       sizeof(struct B10)/2);
	 
	 semGive (semSLC);
	 
	 assert(status[sizeof(status) - 1] == '\a');
	 assert(status_b10[sizeof(status_b10) - 1] == '\a');
	 
	 if(stat == 0) {
	    taskLock();
	    
	    swab(status, (char *)(&sdssdc.status.i1),
		   sizeof(struct AB_SLC500) -
		   ((char *)&sdssdc.status.i1 - (char *)&sdssdc.status));
	    swab(status_b10, (char *)(&sdssdc.b10), sizeof(struct B10));
	    taskUnlock();
	 }
      }
      
      cw_data_collection();
      il_data_collection(); 
   
      if(check_stop_in()) {
	 axis_stat[0].stop_ok=0;
      } else {
	 axis_stat[0].stop_ok=1;
      }
      axis_stat[2].stop_ok = axis_stat[1].stop_ok = axis_stat[0].stop_ok;
      
      if(az_amp_ok()) {
	 axis_stat[0].amp_ok=1;
      } else {
	 axis_stat[0].amp_ok=0;
      }
      
      if(alt_amp_ok()) {
	 axis_stat[1].amp_ok=1;
      } else {
	 axis_stat[1].amp_ok=0;
      }
      
      if(rot_amp_ok()) {
	 axis_stat[2].amp_ok=1;
      } else {
	 axis_stat[2].amp_ok=0;
      }
      
      if(semTake(semMEI,5) != ERROR) {
	 if((sdssdc.axis_state[0]=axis_state(0))<=2) {
	    axis_stat[0].closed_loop=1;
	 } else {
	    axis_stat[0].closed_loop=0;
	 }
	 
	 if((sdssdc.axis_state[1]=axis_state(2))<=2) {
	    axis_stat[1].closed_loop=1;
	 } else {
	    axis_stat[1].closed_loop=0;
	 }
	 
	 if ((sdssdc.axis_state[2]=axis_state(4))<=2) {
	    axis_stat[2].closed_loop=1;
	 } else {
	    axis_stat[2].closed_loop=0;
	 }

	 semGive(semMEI);
      }
      
      if(BCAST_Enable) {
	 ipsdss_send((char *)&sdssdc, sizeof(struct SDSS_FRAME));
      }
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
int
dc_interrupt(void)
{
   unsigned char val;
   int ikey;
   
   ikey = intLock();

   DIO316_Read_Port(cw_DIO316,DC_INTERRUPT,&val);
   DIO316_Write_Port(cw_DIO316,DC_INTERRUPT,val|DC_INTPULSE);
   DIO316_Write_Port(cw_DIO316,DC_INTERRUPT,val);

   intUnlock(ikey);

   return 0;
}
/*=========================================================================
**=========================================================================
**
**	Restore positions after a boot from shared memory
**
** GLOBALS REFERENCED:
**	sdssdc
**
**=========================================================================
*/
void
restore_pos(void)
{
   struct SDSS_FRAME *save;
   struct TM_M68K *restore;
   int i;
   
   if(SM_COPY) {
      save = (struct SDSS_FRAME *)(SHARE_MEMORY + 2);
      for(i = 0; i < NAXIS; i++) { 
	 restore = (struct TM_M68K *)&save->axis[i];
	 
	 tm_set_pos(2*i, restore->actual_position);
	 tm_set_pos(2*i + 1,restore->actual_position2);
      }
   }
}
