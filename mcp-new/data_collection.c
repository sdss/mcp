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
#include "axis.h"			/* needed, also checks SHARE_MEMORY's
					   definition is consistent */
#include "cw.h"
#include "instruments.h"
#include "dscTrace.h"
#include "cmd.h"
#include "mcpUtils.h"

/*========================================================================
**========================================================================
**
** LOCAL MACROS, DEFINITIONS, ETC.
**
**========================================================================
*/
/*------------------------------------------------------------------------
**
** LOCAL DEFINITIONS. This one is repeated in axix_cmds.c
*/
#define SHARE_MEMORY	0x02800000
 
/*-------------------------------------------------------------------------
**
** GLOBAL VARIABLES
*/

SEM_ID semMEIDC=NULL;
SEM_ID semMEIUPD=NULL;
SEM_ID semSLCDC=NULL;
SEM_ID semSDSSDC = NULL;

int rawtick=0;
/* Enables for axis 0, 2, 4: Azimuth(0), Altitude(2), Rotator(4) */
int MEIDC_Enable[]={TRUE,TRUE,TRUE};
unsigned long DC_freq=0;
unsigned long mei_freq=1;
unsigned long slc_freq=100;
int BCAST_Enable=TRUE;

float sdss_time_dc;

struct SDSS_FRAME sdssdc={SDSS_FRAME_VERSION,DATA_TYPE};
struct TM_M68K *tmaxis[NAXIS] = {
   (struct TM_M68K *)&sdssdc.axis[AZIMUTH],
   (struct TM_M68K *)&sdssdc.axis[ALTITUDE],
   (struct TM_M68K *)&sdssdc.axis[INSTRUMENT]
};
/*
 * There are two AXIS_STATs for each axis.  axis_stat[][0] is sticky,
 * and is only cleared by an INIT or STATUS command.  axis_stat[][1]
 * is the latest value, and is ORd into axis_stat[][0] after being
 * updated.
 *
 * The axis_stat[][1] is kept around so that we can set axis_stat[][0]
 * to it when we would usually clear it (i.e. INIT and STATUS don't
 * actually clear all the bits, but set it to the state that it would
 * have after we next ran data collection).
 */
struct AXIS_STAT axis_stat[NAXIS][2] = {
   {0x0, 0x0},
   {0x0, 0x0},
   {0x0, 0x0},
};

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
   int MEIDC_Rotate = 0;		/* which axis to collect next */
   int rotate;				/* used to choose next MEIDC_Rotate */

   if(semMEIDC == NULL) {
      semMEIDC = semBCreate(SEM_Q_FIFO,SEM_EMPTY);
   }
   if(semMEIUPD == NULL) {
      semMEIUPD = semMCreate(SEM_Q_PRIORITY|SEM_INVERSION_SAFE);
   }
   if(semSDSSDC == NULL) {
      semSDSSDC = semMCreate(SEM_Q_PRIORITY|SEM_INVERSION_SAFE);
   }
   
   mei_freq = freq;

   for(;;) {
      if(semTake(semMEIDC, WAIT_FOREVER) == ERROR) {
	 TRACE(0, "failed to take semMEIDC: %s", strerror(errno), 0);
	 continue;
      }
      
      if(semTake(semMEI, NO_WAIT) == ERROR) {
	 continue;
      }
      
      if(semTake(semSDSSDC, 60) == ERROR) {
	 TRACE(2, "mei_data_collection failed to take semSDSSDC: %s",
							   strerror(errno), 0);
	 semGive(semMEI);
	 continue;
      }
      
      sdss_time_dc = (float)sdss_get_time();
      time(&sdssdc.ctime);
      sdssdc.sdsstime = (long)(sdss_time_dc*1000);
      
      i = MEIDC_Rotate;
      if(!MEIDC_Enable[i]) {
	 semGive(semMEI);
	 semGive(semSDSSDC);
      } else {
	 pcdsp_transfer_block(dspPtr,TRUE,FALSE,
			      DATA_STRUCT(dspPtr, i*2, DS_PREV_ENCODER),
			      DS_SIZE+4, (short *)tmaxis[i]);
	 if(dsp_error != DSP_OK) {
	    TRACE(0, "Failed to read MEI data for %s: %d",
		  axis_name(i), dsp_error);
	    tmaxis[i]->status = dsp_error;
	    tmaxis[i]->errcnt++;
	 } else {
	    meistatcnt++;
	 }
	 
	 semGive(semMEI);
	 
	 swapwords((short *)(&tmaxis[i]->actual_position), 1);
	 swapwords((short *)(&tmaxis[i]->position), 1);
	 swapwords((short *)(&tmaxis[i]->time), 1);
	 swapwords((short *)(&tmaxis[i]->velocity), 1);
	 swapwords((short *)(&tmaxis[i]->acceleration), 1);
	 swapwords((short *)(&tmaxis[i]->actual_position2), 1);
	 
	 tmaxis[i]->position =
	   convert_mei_to_mcp(i, tmaxis[i]->position);
	 tmaxis[i]->actual_position =
	   convert_mei_to_mcp(i, tmaxis[i]->actual_position);
	 tmaxis[i]->actual_position2 =
	   convert_mei_to_mcp(i, tmaxis[i]->actual_position2);

	 semGive(semSDSSDC);
      }
      
      for(i = 1; i < 4; i++) {	/* find next enabled data collection */
	 rotate = (MEIDC_Rotate + i)%3;
	 if(MEIDC_Enable[rotate]) {
	    MEIDC_Rotate = rotate;
	    break;
	 }
      }
#if 0
      tm_data_collection();		/* values are not used */
#endif
   }
}

/*=========================================================================
**
**	Collect data from the SLC504 AB via DH+.  Also, collection of 
**	counter-weight values from hardware, instrument lift hardware and
**	ctime.  Status of the MEI controllers is produced for the TCC.  If
**	the broadcast is enabled, the datagram is sent.  All this is 
**	done at the base rate of the SLC data collection.
*/
void
slc500_data_collection(unsigned long freq)
{
   int i;
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

      if(semTake(semSDSSDC, 60) == ERROR) {
	 TRACE(2, "slc500_data_collection failed to take semSDSSDC: %s",
							   strerror(errno), 0);
	 continue;
      }
      
      if(semTake(semSLC,60) == ERROR) {
	 TRACE(2, "slc500_data_collection failed to take semSLC: %s",
							   strerror(errno), 0);
      } else {
	 stat  = slc_read_blok(1,9,BIT_FILE, 0, (uint *)&status[0], 64);
	 stat |= slc_read_blok(1,9,BIT_FILE, 64, (uint *)&status[128], 64);
	 stat |= slc_read_blok(1,9,BIT_FILE, 128, (uint *)&status[256], 38);
	 
	 stat |= slc_read_blok(1,10,BIT_FILE, 0, (uint *)status_b10,
			       sizeof(struct B10)/2);
	 semGive (semSLC);
	 
	 assert(status[sizeof(status) - 1] == '\a');
	 assert(status_b10[sizeof(status_b10) - 1] == '\a');
	 
	 if(stat == 0) {
/*
 * byteswap status/status_b10 and copy them into sdssdc at the same time
 */
	    swab(status, (char *)(&sdssdc.status.i1),
		 sizeof(struct AB_SLC500) -
		 ((char *)&sdssdc.status.i1 - (char *)&sdssdc.status));
	    swab(status_b10, (char *)(&sdssdc.b10), sizeof(struct B10));
	 }
      }

      cw_data_collection();
      il_data_collection();
	 
      while(semTake(semMEIUPD, WAIT_FOREVER) == ERROR) {
	 TRACE(0, "couldn't take semMEIUPD semaphore.", 0, 0);
	 taskSuspend(NULL);
      }
      
      axis_stat[AZIMUTH][1].stop_in =
	axis_stat[ALTITUDE][1].stop_in =
	  axis_stat[INSTRUMENT][1].stop_in = (check_stop_in() ? 1 : 0);

      {
	 const int cmdPortTask = getSemTaskId(semCmdPort);

	 axis_stat[AZIMUTH][1].semCmdPort_taken =
	   axis_stat[ALTITUDE][1].semCmdPort_taken =
	     axis_stat[INSTRUMENT][1].semCmdPort_taken =
	       (cmdPortTask == ERROR ||
		cmdPortTask == 0 ||
		cmdPortTask == taskNameToId("TCC")) ? 0 : 1;
      }
      
      axis_stat[AZIMUTH][1].amp_bad = az_amp_ok()  ? 0 : 1;
      axis_stat[ALTITUDE][1].amp_bad = alt_amp_ok() ? 0 : 1;
      axis_stat[INSTRUMENT][1].amp_bad = rot_amp_ok() ? 0 : 1;
/*
 * Set sticky versions of bump switches; these stay on until explicitly
 * cleared with clear_sticky_bumps() (called by e.g. init_cmd())
 */
      if(sdssdc.status.i1.il0.az_bump_ccw) {
	 if(!axis_stat[AZIMUTH][0].bump_up_ccw_sticky) {
	    TRACE(0, "Windscreen touched in azimuth: CCW", 0, 0);
	 }
	 axis_stat[AZIMUTH][1].bump_up_ccw_sticky = 1;
      } else {
	 axis_stat[AZIMUTH][1].bump_up_ccw_sticky = 0;
      }
      
      if(sdssdc.status.i1.il0.az_bump_cw)  {
	 if(!axis_stat[AZIMUTH][0].bump_dn_cw_sticky) {
	    TRACE(0, "Windscreen touched in azimuth: CW", 0, 0);
	 }
	 axis_stat[AZIMUTH][1].bump_dn_cw_sticky = 1;
      } else {
	 axis_stat[AZIMUTH][1].bump_dn_cw_sticky = 0;
      }
      
      if(sdssdc.status.i1.il6.alt_bump_up) {
	 if(!axis_stat[ALTITUDE][0].bump_up_ccw_sticky) {
	    TRACE(0, "Windscreen touched in altitude: UP", 0, 0);
	 }
	 axis_stat[ALTITUDE][1].bump_up_ccw_sticky = 1;
      } else {
	 axis_stat[ALTITUDE][1].bump_up_ccw_sticky = 0;
      }
      if(sdssdc.status.i1.il6.alt_bump_dn) {
	 if(!axis_stat[ALTITUDE][0].bump_dn_cw_sticky) {
	    TRACE(0, "Windscreen touched in altitude: DOWN", 0, 0);
	 }
	 axis_stat[ALTITUDE][1].bump_dn_cw_sticky = 1;
      } else {
	 axis_stat[ALTITUDE][1].bump_dn_cw_sticky = 0;
      }
/*
 * axis status, and are we in closed loop?
 */
      if(semTake(semMEI, 5) != ERROR) {
	 for(i = 0; i < NAXIS; i++) {
	    sdssdc.axis_state[i] = axis_state(2*i);
	    axis_stat[i][1].out_closed_loop =
	      (sdssdc.axis_state[i] <= NEW_FRAME) ? 0 : 1;
	 }
	 
	 semGive(semMEI);
      }
/*
 * Set old versions of some bits; these can be removed when we rebuild
 * everything that depends on data_collection.h XXX
 */
      for(i = 0; i < NAXIS; i++) {
	 axis_stat[i][1].stop_ok = !axis_stat[i][1].stop_in;
	 axis_stat[i][1].amp_ok = !axis_stat[i][1].amp_bad;
	 axis_stat[i][1].closed_loop = !axis_stat[i][1].out_closed_loop;
      }
/*
 * Copy the just-set values in axis_stat[][1] to the sticky values
 * in axis_stat[][0].  We have to be a little careful with the
 * ones in STATUS_MASK as they are _set_ if OK; we simply copy them.
 */
      for(i = 0; i < NAXIS; i++) {
	 *(long *)&axis_stat[i][0] &= ~STATUS_MASK;
	 *(long *)&axis_stat[i][0] |= *(long *)&axis_stat[i][1];
      }

      semGive(semMEIUPD);

      if(BCAST_Enable) {
	 ipsdss_send((char *)&sdssdc, sizeof(struct SDSS_FRAME));
      }
      
      semGive(semSDSSDC);
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

void
DataCollectionTrigger(void)
{
   DC_freq++;
   rawtick=tickGet();
/*
 * Giving semMEIDC and/or semSLCDC causes their data collection routines
 * to wake up and gather data
 */
   if(semMEIDC != NULL && !(DC_freq%mei_freq)) {
      semGive(semMEIDC);
   }
   
   if(semSLCDC != NULL && !(DC_freq%slc_freq)) {
      semGive(semSLCDC);
   }
/*
 * Copy data from sdssdc to shared memory
 */
   if(SM_COPY) {
      if(semTake(semSDSSDC, WAIT_FOREVER) == ERROR) {
	 TRACE(0, "failed to take semSDSSDC: %s", strerror(errno), 0);
      } else {
	 unsigned short CRC;		/* CRC of sdssdc, starting offset
					   bytes into struct */
	 const int offset = offsetof(struct SDSS_FRAME, ctime);

	 if(sdssdc.status.i1.il6.sec_mir_force_limits) {
	    printf("RHL XXX sec_mir_force_limits is true\n");
	 }

	 sdssdc.CRC = phCrcCalc(0, (char *)&sdssdc + offset,
				sizeof(sdssdc) - offset) & 0xFFFF;
	 
	 *(short *)SHARE_MEMORY = TRUE;
	 memcpy((char *)(SHARE_MEMORY + 2), (char *)&sdssdc, sizeof(sdssdc));
	 *(short *)SHARE_MEMORY = FALSE;

	 semGive(semSDSSDC);

	 dc_interrupt();
/*
 * check that CRC
 */
	 CRC = phCrcCalc(0, (char *)(SHARE_MEMORY + 2) + offset,
			 sizeof(sdssdc) - offset) & 0xFFFF;

	 if(sdssdc.CRC != CRC) {
	    printf("RHL: CRC has changed: 0x%x v. 0x%x",
		   sdssdc.CRC, CRC);
	 }
      }
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
