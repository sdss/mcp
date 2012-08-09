#include "copyright.h"
/**************************************************************************
** ABSTRACT:
**	Collects data from MEI, AB, CW, and time.  Distributes to shared
**	memory and broadcasts at 1 Hz.
***************************************************************************/
#define DATA_COLLECTION_C 1		/* allows plcVersion to be defined */
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
#include "frame.h"
#include "axis.h"			/* needed, also checks SHARE_MEMORY's
					   definition is consistent */
#include "serial.h"
#include "cw.h"
#include "instruments.h"
#include "dscTrace.h"
#include "cmd.h"
#include "mcpUtils.h"
#include "mcpFiducials.h"
#include "mcpSpectro.h"
#include "as2.h"

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
SEM_ID semStatusCmd = NULL;

int rawtick=0;
/* Enables for axis 0, 2, 4: Azimuth(0), Altitude(2), Rotator(4) */
int MEIDC_Enable[]={TRUE,TRUE,TRUE};
/*
 * DC_freq is incremented every time that DataCollectionTrigger() is called;
 * that's set in mcp.login
 *
 * The other `frequencies' are really _inverse_ frequencies; they set the
 * number of DC_freq ticks between calls to various data collection tasks
 */
unsigned long DC_freq=0;
unsigned long mei_freq=1;
unsigned long slc_freq=100;
unsigned long check_encoder_freq = 20*60;
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
   {{0x0, 0x0}},
   {{0x0, 0x0}},
   {{0x0, 0x0}},
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
   int uid = 0, cid = 0;   
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
   if(semStatusCmd == NULL) {
      semStatusCmd = semMCreate(SEM_Q_PRIORITY|SEM_INVERSION_SAFE);
   }
   
   mei_freq = freq;

   for(;;) {
      if(semTake(semMEIDC, WAIT_FOREVER) == ERROR) {
	 NTRACE_1(0, uid, cid, "failed to take semMEIDC: %s", strerror(errno));
	 continue;
      }
      
      if(semTake(semMEI, NO_WAIT) == ERROR) {
	 continue;
      }
      
      if(semTake(semSDSSDC, 60) == ERROR) {
	 NTRACE_1(2, uid, cid, "mei_data_collection failed to take semSDSSDC: %s", strerror(errno));
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
/*
 * Read desired information out of the MEI's DSP. This is done is a
 * slightly sleazy way...
 * 
 * The object tmaxis[] is a struct that looks like:
 * 
 * struct TM {
 *    short prev_encoder;
 *    short current_vel;
 *    short actual_position_hi;
 *    unsigned short actual_position_lo;
 * ...
 *    short unknown9;
 *    short prev_encoder2;
 *    short current_vel2;
 *    short actual_position2_hi;
 *    unsigned short actual_position2_lo;
 *    short status;
 *    short errcnt;
 * };
 * 
 * The DSP's memory is laid out (in terms of a fictional struct) as:
 *
 * struct DSP_TM {
 *    short prev_encoder;
 *    short current_vel;
 *    short actual_position_hi;
 *    unsigned short actual_position_lo;
 *    ...
 *    short unknown9;
 * } axis[6]
 *
 * where sizeof(struct DSP_TM) == DS_SIZE*sizeof(short).
 *
 * I.e. struct TM consists of one struct DSP_TM, followed by the first
 * 4 short words of a second struct DSP_TM, followed by
 *    short status;
 *    short errcnt;
 *
 * OK?  So the following pcdsp_transfer_block() reads the DSP_TM for
 * axis 2*i, and then the first 4 shorts of the DSP_TM for axis 2*i + 1.
 *
 * Looking at the definition of DATA_STRUCT, it is clear that there's
 * no extraneous padding, and that this is therefore OK if not clean.
 *
 * There is one other issue; the pcdsp_transfer_block actually transfers
 * blocks in units of TRANSFER_BUFF_SIZE shorts; but as this is 64 there's
 * no problem with part of the data being read at one time, and part at
 * another.
 */
	 pcdsp_transfer_block(dspPtr,TRUE,FALSE,
			      DATA_STRUCT(dspPtr, i*2, DS_PREV_ENCODER),
			      DS_SIZE+4, (unsigned short *)tmaxis[i]);
	 if(dsp_error != DSP_OK) {
	    NTRACE_2(0, uid, cid, "Failed to read MEI data for %s: %s",
		     axis_name(i), _error_msg(dsp_error));
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
	   convert_mei_to_mcp(2*i, tmaxis[i]->position);
	 tmaxis[i]->actual_position =
	   convert_mei_to_mcp(2*i, tmaxis[i]->actual_position);
	 tmaxis[i]->actual_position2 =
	   convert_mei_to_mcp(2*i + 1, tmaxis[i]->actual_position2);

	 semGive(semSDSSDC);
/*
 * Check if the encoders agree?
 */
#if 0
	 if(DC_freq%check_encoder_freq < NAXIS) { /* check all axes */
	    if(fiducial[i].scale_ratio_12 > 0 &&
				       fiducial[i].min_encoder_mismatch >= 0) {
	       const long canon = fiducial[i].canonical_position;
	       long predict_2 = canon +
				(tmaxis[i]->actual_position2 - canon)/fiducial[i].scale_ratio_12;
	       long diff = predict_2 - tmaxis[i]->actual_position;
	       
	       if(abs(diff) > fiducial[i].min_encoder_mismatch) {
		  NTRACE_2(3, uid, cid, "Encoders for axis %s differ by %d",
			   axis_name(i), diff);
	       }
	    }
	 }
#endif
/*
 * If all the axes are idle, make the TCC give up the semaphore if
 * if currently holds it.  If grabbed_semCmdPort is true don't allow
 * this -- otherwise a task (well, only the TCC) may lose the semaphore
 * before being able to execute a command.
 */
	 if(getSemTaskId(semCmdPort) == tcc_taskId &&
						  tcc_may_release_semCmdPort &&
	    axis_queue[AZIMUTH].active == NULL &&
	    axis_queue[ALTITUDE].active == NULL &&
	    axis_queue[INSTRUMENT].active == NULL) {

	    (void)give_semCmdPort(1);
	    tcc_may_release_semCmdPort = 0;
	    NTRACE(3, uid, cid, "All axes idle: TCC gave up semaphore");
	    sendStatusMsg_B(uid, cid, INFORMATION_CODE, 0, "haveSemaphore", have_semaphore(uid));
	    sendStatusMsg_S(uid, cid, INFORMATION_CODE, 1, "semaphoreOwner", semCmdPortOwner);
	 }
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
**	done at the base rate of the SLC data collection -- i.e. freq
**      is interpreted relative to the rate at which serverDCStart() is
**      triggered (see mcp.login)
*/
void
slc500_data_collection(unsigned long freq)
{
   int uid = 0, cid = 0;   
   int counter;				/* counter for how often we've been round this loop */
   int i;
   static int plc_version_id = -1;	/* plc version in data_collection.h */
   static int plc_version_mismatch = 0;	/* version_id != plc_version_id? */
   char status[192*2 + 1];		/* == sizeof(struct AB_SLC500) + 1 */
   int stat;

   {
      const int tmp = sizeof(struct AB_SLC500) + 1;
      if(sizeof(status) != tmp) {
	 NTRACE_2(0, uid, cid, "slc500_data_collection: buffer has wrong size (%d v %d)",
		  sizeof(status), tmp);
	 taskSuspend(0);
      }
   }
   status[sizeof(status) - 1] = '\a';
   
   semSLCDC = semBCreate(SEM_Q_FIFO, SEM_EMPTY);
   slc_freq=freq;
   
   for(counter = 0;; ++counter) {
      if(semTake(semSLCDC, WAIT_FOREVER) == ERROR) {
	 NTRACE(0, uid, cid, "couldn't take semSLCDC semahore.");
	 break;
      }

      if(semTake(semSDSSDC, 60) == ERROR) {
	 NTRACE_1(2, uid, cid, "slc500_data_collection failed to take semSDSSDC: %s", strerror(errno));
      }
      
      if(semTake(semSLC,60) == ERROR) {
	 NTRACE_1(2, uid, cid, "slc500_data_collection failed to take semSLC: %s", strerror(errno));
      } else {
	 const int nbyte = (int)sizeof(struct AB_SLC500) -
	   ((char *)&sdssdc.status.i1 - (char *)&sdssdc.status);
	 int base = 0;
	 const int n0 = 64;
	 const int n1 = 64;
	 const int n2 = 62;
	 
	 stat  = slc_read_blok(1, 9, BIT_FILE, base,
			       (uint *)&status[base*sizeof(short)], n0);
	 base += n0;
	 stat |= slc_read_blok(1, 9, BIT_FILE, base,
			       (uint *)&status[base*sizeof(short)], n1);
	 base += n1;
	 stat |= slc_read_blok(1, 9, BIT_FILE, base,
			       (uint *)&status[base*sizeof(short)], n2);
	 base += n2;
	 semGive (semSLC);

	 if(nbyte != base*sizeof(short)) {
	    NTRACE_2(0, uid, cid,
		     "slc500_data_collection: nbyte is wrong (%d v %d)", base*(int)sizeof(short), nbyte);
	    taskSuspend(0);
	 }
	 if(status[sizeof(status) - 1] != '\a') {
	    NTRACE(0, uid, cid, "slc500_data_collection: overwrote buffer marker");
	    taskSuspend(0);
	 }
	 
	 if(stat == 0) {
/*
 * byteswap status and copy it into sdssdc
 */
	    swab(status, (char *)(&sdssdc.status.i1), nbyte);
	 }
      }

      cw_data_collection();
      il_data_collection();
	 
      while(semTake(semMEIUPD, WAIT_FOREVER) == ERROR) {
	 NTRACE(0, uid, cid, "couldn't take semMEIUPD semaphore.");
	 taskSuspend(0);
      }
      
      axis_stat[AZIMUTH][1].stop_in =
	axis_stat[ALTITUDE][1].stop_in =
	  axis_stat[INSTRUMENT][1].stop_in = (check_stop_in(0) ? 1 : 0);

      {
	 const int cmdPortTask = getSemTaskId(semCmdPort);

	 axis_stat[AZIMUTH][1].semCmdPort_taken =
	   axis_stat[ALTITUDE][1].semCmdPort_taken =
	     axis_stat[INSTRUMENT][1].semCmdPort_taken =
	       (cmdPortTask == ERROR ||
		cmdPortTask == 0 ||
		cmdPortTask == tcc_taskId) ? 0 : 1;
      }
      
      axis_stat[AZIMUTH][1].amp_bad = az_amp_ok(0)  ? 0 : 1;
      axis_stat[ALTITUDE][1].amp_bad = alt_amp_ok(0) ? 0 : 1;
      axis_stat[INSTRUMENT][1].amp_bad = rot_amp_ok(0) ? 0 : 1;
/*
 * Set sticky versions of bump switches; these stay on until explicitly
 * cleared with clear_sticky_bumps() (called by e.g. init_cmd())
 */
      if(sdssdc.status.i1.il0.az_bump_ccw) {
	 if(!axis_stat[AZIMUTH][0].bump_up_ccw_sticky) {
	    char val[4];
	    sprintf(val, "%d%d", axis_stat[AZIMUTH][0].bump_dn_cw_sticky, axis_stat[AZIMUTH][0].bump_up_ccw_sticky);
	    sendStatusMsg_S(uid, cid, INFORMATION_CODE, 1, "azWindscreenTouched", val);
	 }
	 axis_stat[AZIMUTH][1].bump_up_ccw_sticky = 1;
      } else {
	 axis_stat[AZIMUTH][1].bump_up_ccw_sticky = 0;
      }
      
      if(sdssdc.status.i1.il0.az_bump_cw)  {
	 if(!axis_stat[AZIMUTH][0].bump_dn_cw_sticky) {
	    char val[4];
	    sprintf(val, "%d%d", axis_stat[AZIMUTH][0].bump_dn_cw_sticky, axis_stat[AZIMUTH][0].bump_up_ccw_sticky);
	    sendStatusMsg_S(uid, cid, INFORMATION_CODE, 1, "azWindscreenTouched", val);
	 }
	 axis_stat[AZIMUTH][1].bump_dn_cw_sticky = 1;
      } else {
	 axis_stat[AZIMUTH][1].bump_dn_cw_sticky = 0;
      }
      
      if(sdssdc.status.i1.il10.alt_bump_up) {
	 if(!axis_stat[ALTITUDE][0].bump_up_ccw_sticky) {
	    char val[4];
	    sprintf(val, "%d%d", axis_stat[ALTITUDE][0].bump_dn_cw_sticky, axis_stat[ALTITUDE][0].bump_up_ccw_sticky);
	    sendStatusMsg_S(uid, cid, INFORMATION_CODE, 1, "altWindscreenTouched", val);
	 }
	 axis_stat[ALTITUDE][1].bump_up_ccw_sticky = 1;
      } else {
	 axis_stat[ALTITUDE][1].bump_up_ccw_sticky = 0;
      }
      if(sdssdc.status.i1.il10.alt_bump_dn) {
	 if(!axis_stat[ALTITUDE][0].bump_dn_cw_sticky) {
	    char val[4];
	    sprintf(val, "%d%d", axis_stat[ALTITUDE][0].bump_dn_cw_sticky, axis_stat[ALTITUDE][0].bump_up_ccw_sticky);
	    sendStatusMsg_S(uid, cid, INFORMATION_CODE, 1, "altWindscreenTouched", val);
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

      broadcast_ipsdss(uid, cid);	/* N.b. sets the ab->i10.il0 bits for ipsdss_send;
					   the broadcast_ipsdss call *must* come first */

      if(BCAST_Enable) {
	 ipsdss_send((char *)&sdssdc, sizeof(struct SDSS_FRAME));
      }
      
      semGive(semSDSSDC);
/*
 * Set the axis_status arrays for the use of axis_status_cmd(),
 * and system_status for the use of system_status_cmd()
 */
      if(semTake(semStatusCmd, 60) == ERROR) {
	 NTRACE_1(2, uid, cid, "slc500_data_collection failed to take semStatusCmd: %s", strerror(errno));
      } else {
	 set_status(NOINST, system_status_buff, STATUS_BUFF_SIZE);
	 set_status(AZIMUTH, axis_status_buff[AZIMUTH], STATUS_BUFF_SIZE);
	 set_status(ALTITUDE, axis_status_buff[ALTITUDE], STATUS_BUFF_SIZE);
	 set_status(INSTRUMENT, axis_status_buff[INSTRUMENT],STATUS_BUFF_SIZE);

	 semGive(semStatusCmd);
      }
/*
 * Did someone push a button on the manual latch control box?
 */
      if(sdssdc.status.i1.il4.open_slit_doors) {
	 sp1_cmd(0, 0, NULL); slitdoor_open_cmd(0, 0, NULL);
	 sp2_cmd(0, 0, NULL); slitdoor_open_cmd(0, 0, NULL);
      } else if(sdssdc.status.i1.il4.close_slit_doors) {
	 sp1_cmd(0, 0, NULL); slitdoor_close_cmd(0, 0, NULL);
	 sp2_cmd(0, 0, NULL); slitdoor_close_cmd(0, 0, NULL);
      }
      
      if(sdssdc.status.i1.il4.slit_latch_lth_cmd) {
	 sp1_cmd(0, 0, NULL); slithead_latch_close_cmd(0, 0, NULL);
	 sp2_cmd(0, 0, NULL); slithead_latch_close_cmd(0, 0, NULL);
      } else if(sdssdc.status.i1.il4.slit_latch_unlth_cmd) {
	 sp1_cmd(0, 0, NULL); slithead_latch_open_cmd(0, 0, NULL);
	 sp2_cmd(0, 0, NULL); slithead_latch_open_cmd(0, 0, NULL);
      }
/*
 * check that the correct version of the PLC's installed
 */
      if(plc_version_id < 0) {
	 sscanf(plcVersion, "Version %d", &plc_version_id);
      }

      if(sdssdc.status.b3.w1.version_id == plc_version_id) {
	 plc_version_mismatch = 0;
      } else {
	 if(sdssdc.status.b3.w1.version_id > 32767) {
	    ;				/* a devel version; don't complain */
	 } else {
	    if(plc_version_mismatch%100 == 0) {
	       char buff[100];
	       NTRACE_2(0, uid, cid, "Saw PLC version %d; expected \"%s\"",
			sdssdc.status.b3.w1.version_id, plcVersion);

	       sprintf(buff,"%d, %s", sdssdc.status.b3.w1.version_id, plcVersion);
	       sendStatusMsg_A(uid, cid, INFORMATION_CODE, 1, "plcVersions", buff);
	    }
	    
	    plc_version_mismatch++;
	 }
      }
      /*
       * Broadcast any keywords that have changed
       */
      {
	 int include_cw = (counter%10 == 0);
	 do_info_cmd(0, 0, include_cw);
      }
   }
}


/*****************************************************************************/
/*
 * Update the sdssdc.status.i6 bits
 */
void
update_sdssdc_status_i6(void)
{
   int uid = 0, cid = 0;
   unsigned short ctrl[2];
   int err;
   const int offset = (char *)&sdssdc.status.i6 - (char *)&sdssdc.status.i1;
/*
 * set err in assertion to suppress compiler warning
 */
   assert(err = (sizeof(sdssdc.status.i6) == sizeof(ctrl)));

   if(semTake(semSLC,60) == ERROR) {
      NTRACE_2(0, uid, cid, "Unable to take semaphore: %s (%d)", strerror(errno), errno);
      return;
   }

   err = slc_read_blok(1, 9, BIT_FILE, offset/2, &ctrl[0], sizeof(ctrl)/2);
   if(err) {
      NTRACE_1(0, uid, cid, "az_amp_ok: error reading slc: 0x%04x", err);
   }
   semGive(semSLC);
   
   if(!err) {
      swab((char *)&ctrl[0], (char *)&sdssdc.status.i6,
						     sizeof(sdssdc.status.i6));
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
   int uid = 0, cid = 0;   
   static unsigned short share_memory_id = 1; /* ID for shared memory */

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
	 NTRACE_1(0, uid, cid, "failed to take semSDSSDC: %s", strerror(errno));
      } else {
	 const int offset = offsetof(struct SDSS_FRAME, ctime);

	 sdssdc.CRC = phCrcCalc(0, (char *)&sdssdc + offset,
				(int)sizeof(sdssdc) - offset) & 0xFFFF;
	 taskLock();
	 
	 *(short *)SHARE_MEMORY = 0;
	 memcpy((char *)(SHARE_MEMORY + 2), (char *)&sdssdc, sizeof(sdssdc));
	 *(short *)SHARE_MEMORY = share_memory_id++;

	 taskUnlock();
	 
	 if(share_memory_id == 0) {
	    share_memory_id++;		/* don't want it to be 0 */
	 }

	 semGive(semSDSSDC);

	 dc_interrupt();
#if 0
/*
 * check that CRC
 */
	 {
	    unsigned short CRC;		/* CRC of sdssdc, starting offset
					   bytes into struct */
	    CRC = phCrcCalc(0, (char *)(SHARE_MEMORY + 2) + offset,
			    (signed)sizeof(sdssdc) - offset) & 0xFFFF;
	    
	    if(sdssdc.CRC != CRC) {
	       NTRACE_2(0, uid, cid, "data_collection CRC has changed: 0x%x v. 0x%x", sdssdc.CRC, CRC);
	    }
	 }
#endif
      }
   }
}

/*****************************************************************************/
/*
 * ??? Provides output pulse for TPM ???
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
