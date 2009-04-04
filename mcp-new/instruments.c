#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <semLib.h>
/* #include <taskLib.h> */
#include "instruments.h"
#include "abdh.h"
#include "data_collection.h"
#include "axis.h"
#include "tm.h"
#include "dscTrace.h"
#include "cw.h"
#include "cmd.h"
#include "as2.h"

/*****************************************************************************/
/*
 * Return the ID number of the current instrument
 */
/*
 * The camera ID switches aren't installed as of Jan 2001
 *
 * Use the state of the primary latches/spec corrector to guess what's
 * installed?
 */
#define GUESS_INSTRUMENT 1
   
#define CAMERA_ID 14

int
instrument_id(void)
{
   int uid = 0, cid = 0;
   int inst_id1, inst_id2, inst_id3;	/* values of the inst ID switches */
   static int notify = -1;		/* should I notify user of bad ID? */
   int notify_rate = 60;		/* how often should I notify? */
#if GUESS_INSTRUMENT
   int pri_latch_opn;			/* the primary latches are open */
   int spec_lens;			/* the spec corrector in installed */
#endif
   
   if(semTake(semSDSSDC, 100) == ERROR) {
      return(-1);			/* unknown */
   }
   
   inst_id1 = (((sdssdc.status.i1.il8.inst_id1_1 ? 0 : 1) << 3) + 
	       ((sdssdc.status.i1.il8.inst_id1_2 ? 0 : 1) << 2) +
	       ((sdssdc.status.i1.il8.inst_id1_3 ? 0 : 1) << 1) +
	       ((sdssdc.status.i1.il8.inst_id1_4 ? 0 : 1) << 0));
   inst_id2 = (((sdssdc.status.i1.il8.inst_id2_1 ? 0 : 1) << 3) + 
	       ((sdssdc.status.i1.il8.inst_id2_2 ? 0 : 1) << 2) +
	       ((sdssdc.status.i1.il8.inst_id2_3 ? 0 : 1) << 1) +
	       ((sdssdc.status.i1.il8.inst_id2_4 ? 0 : 1) << 0));
   inst_id3 = (((sdssdc.status.i1.il8.inst_id3_1 ? 0 : 1) << 3) + 
	       ((sdssdc.status.i1.il8.inst_id3_2 ? 0 : 1) << 2) +
	       ((sdssdc.status.i1.il8.inst_id3_3 ? 0 : 1) << 1) +
	       ((sdssdc.status.i1.il8.inst_id3_4 ? 0 : 1) << 0));
#if GUESS_INSTRUMENT
   pri_latch_opn = (sdssdc.status.i1.il8.pri_latch1_opn &&
		    sdssdc.status.i1.il8.pri_latch2_opn &&
		    sdssdc.status.i1.il8.pri_latch3_opn);
   spec_lens = (!sdssdc.status.i1.il8.spec_lens1 ||
		 sdssdc.status.i1.il8.spec_lens2);
#endif

   semGive(semSDSSDC);

   if((inst_id1 == inst_id2 && inst_id1 == inst_id3) /* consistent */ || pri_latch_opn /* not latched */) {
      if (notify >= 0) {		/* we were inconsistent */
	 char buff[20];
	 sprintf(buff,"%d, %d, %d", inst_id1, inst_id2, inst_id3);
	 sendStatusMsg_A(0, 0, INFORMATION_CODE, 0, "instrumentNumValues", buff);
      }
      notify = -1;
   } else {
      notify = (notify + 1)%notify_rate;

      if(notify == 0) {
	 char buff[20];
	 sprintf(buff,"%d, %d, %d", inst_id1, inst_id2, inst_id3);
	 NTRACE_1(2, uid, cid, "Inconsistent instrument ID switches: %s", buff);
	 sendStatusMsg_A(0, 0, INFORMATION_CODE, 0, "instrumentNumValues", buff);
	 
	 return(-1);
      }
   }
#if GUESS_INSTRUMENT
   if(inst_id1 == 0 && !pri_latch_opn && !spec_lens) {
      return(CAMERA_ID);
   }
#endif
   
   return(inst_id1);
}

/*****************************************************************************/
/*
 * Is the imager saddle on the telescope? Note that switch 1 is deliberately
 * wired backwards
 */
int
saddle_is_mounted(void)
{
   int saddle_is_on;			/* is the saddle mounted? */
   int sad_mount1, sad_mount2;
  
   if(semTake(semSDSSDC, 100) == ERROR) {
      return(-1);			/* unknown */
   }
   
   sad_mount1 = (sdssdc.status.i1.il9.sad_mount1 == 0) ? 1 : 0; 
   sad_mount2 = (sdssdc.status.i1.il9.sad_mount2 == 0) ? 0 : 1;

   if(sad_mount1 == sad_mount2) {
      saddle_is_on = sad_mount1;
   } else {
      saddle_is_on = -1;		/* inconsistent */
   }
   
   semGive(semSDSSDC);

   return(saddle_is_on);
}

/*****************************************************************************/
/*
 * Report the status of instruments.
 */
void
broadcast_inst_status(int uid, unsigned long cid)
{
   int inst_id = instrument_id();
   sendStatusMsg_B(uid, cid, INFORMATION_CODE, 0, "saddleIsMounted", saddle_is_mounted());

   sendStatusMsg_B(uid, cid, INFORMATION_CODE, 0, "instrumentNumConsistent", (inst_id >= 0));
   sendStatusMsg_I(uid, cid, INFORMATION_CODE, 0, "instrumentNum", inst_id);
}

int
get_inststatus(char *status_ans,
	       int size)			/* dimen of status_ans */
{
   int len;

   sprintf(status_ans, "Inst: %d %d\n",
	   saddle_is_mounted(), instrument_id());

   len = strlen(status_ans);
   assert(len < size);
   
   return(len);
}
