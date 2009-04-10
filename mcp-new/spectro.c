#include <vxWorks.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <semLib.h>
#include <sysLib.h>
#include <taskLib.h>
#include <usrLib.h>
#include "abdh.h"
#include "idsp.h"
#include "pcdsp.h"
#include "gendefs.h"
#include "ad12f1lb.h"
#include "ip480.h"
#include "mv162IndPackInit.h"
#include "frame.h"
#include "data_collection.h"
#include "tm.h"
#include "axis.h"
#include "cmd.h"
#include "dscTrace.h"
#include "mcpMsgQ.h"
#include "mcpSpectro.h"
#include "mcpUtils.h"

char *ffsclose_cmd(int uid, unsigned long cid, char *cmd);
char *ffstatus_cmd(int uid, unsigned long cid, char *cmd);

#define ENABLE_CLAMP 0	     /* no longer needed, as per Dan Long, 2009-04-02 */

MSG_Q_ID msgFFS = NULL;			/* control flat field screens */
MSG_Q_ID msgLamps = NULL;		/* control lamps */
MSG_Q_ID msgSpecDoor = NULL;		/* control spectrograph doors */

/*****************************************************************************/
/*
 * Control the alignment clamp.
 */
#if ENABLE_CLAMP
MSG_Q_ID msgAlignClamp = NULL;		/* control alignment clamp */

void
tAlgnClmp(void)
{
   int engage;				/* are we trying to engage the clamp?*/
   int err;
   unsigned short ctrl[2];
   MCP_MSG msg;				/* message to pass around */
   int ret;				/* return code */   
   B10_L0 tm_ctrl;   

   for(;;) {
      ret = msgQReceive(msgAlignClamp, (char *)&msg, sizeof(msg),
			WAIT_FOREVER);
      assert(ret != ERROR);

      OTRACE(8, "read msg on msgAlignClamp", 0, 0);
/*
 * What sort of message?
 *   alignClamp_type        A request from the outside world to move clamp
 *   alignClampCheck_type   A request from us to check that the clamp moved
 */
      switch (msg.type) {
       case alignClamp_type:
	 engage = (msg.u.alignClamp.op == ENGAGE) ? 1 : 0;
	 timerSend(alignClampCheck_type, tmr_e_abort_ns,
		   0, alignClampCheck_type, msgAlignClamp);
	 break;
       case alignClampCheck_type:
	 if(sdssdc.status.i9.il0.clamp_en_stat == 1) { /* success */
	    continue;
	 } else {			/* Failure; turn off and disengage */
	    OTRACE(0, "Alignment clamp did NOT engage..."
		  "turning off and disengaging", 0, 0);
	    engage = 0;
	 }
	 break;
       default:
	 OTRACE(0, "Impossible message type on msgAlignClamp: %d", msg.type, 0);
	 continue;	 
      }
/*
 * Time to do the work
 */
      if(semTake(semSLC,60) == ERROR) {
	 OTRACE(0, "Unable to take semaphore: %s (%d)", strerror(errno), errno);
	 continue;
      }
      
      err = slc_read_blok(1,10,BIT_FILE,0,&ctrl[0],sizeof(tm_ctrl)/2);
      if(err) {
	 semGive(semSLC);
	 OTRACE(0, "tAlgnClmp: error reading slc: 0x%04x", err, 0);
	 continue;
      }
      
      swab((char *)&ctrl[0], (char *)&tm_ctrl, sizeof(tm_ctrl));

      if(engage) {
	 tm_ctrl.mcp_clamp_engage_cmd = 1;
	 tm_ctrl.mcp_clamp_disen_cmd = 0;
      } else {
	 tm_ctrl.mcp_clamp_engage_cmd = 0;
	 tm_ctrl.mcp_clamp_disen_cmd = 1;
      }
      
      swab((char *)&tm_ctrl, (char *)&ctrl[0], sizeof(tm_ctrl));
      err = slc_write_blok(1, 10, BIT_FILE, 0, &ctrl[0], sizeof(tm_ctrl)/2);
      semGive(semSLC);
   
      if(err) {
	 OTRACE(0, "tAlgnClmp: error writing slc: 0x%04x", err, 0);
	 continue;
      }
      
      if(engage) {			/* wait, then see if we succeeded */
	 int wait = 15;			/* how many seconds to wait */
	 OTRACE(1, "Waiting %ds for alignment clamp to engage", wait, 0);
	 
	 if(timerSend(alignClampCheck_type, tmr_e_add,
		      wait*60, alignClampCheck_type, msgAlignClamp) == ERROR) {
	    OTRACE(0, "Failed to send message to timer task: %s (%d)",
		  strerror(errno), errno);
	 }
      }
   }
}

static void
alignment_clamp_set(int uid, unsigned long cid, int engage)
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */
   
   msg.type = alignClamp_type;
   msg.u.alignClamp.op = engage ? ENGAGE : DISENGAGE;

   ret = msgQSend(msgAlignClamp, (char *)&msg, sizeof(msg),
		  NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);

   /*
    * This is a lie.  I should delay until after I've had a chance to see if the
    * clamp moved, but as we don't use it I shan't bother.  The code would be
    * very similar to the lamps
    */
   {
      char buff[10];
      sprintf(buff, "%d, %d", sdssdc.status.i9.il0.clamp_en_stat, sdssdc.status.i9.il0.clamp_dis_stat);
      sendStatusMsg_A(uid, cid, INFORMATION_CODE, 0, "alignmentClamp", buff);
   }
}

/*****************************************************************************/
/*
 * Command the alignment clamp
 */
char *
clampon_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   alignment_clamp_set(uid, cid, 1);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 0, "command", "clampOn");
   
   return "";
}

char *
clampoff_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   alignment_clamp_set(uid, cid, 0);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 0, "command", "clampOff");
   
   return "";
}
#endif

/*****************************************************************************/
/*
 * Control the spectrographs' slithead doors
 */
void
tSpecDoor(void)
{
   int uid;
   unsigned long cid;
   int err;
   unsigned short ctrl[2];
   MCP_MSG msg;				/* message to pass around */
   int ret;				/* return code */
   B10_L0 tm_ctrl;
   int spec;				/* which spectrograph? */

   for(;;) {
      ret = msgQReceive(msgSpecDoor, (char *)&msg, sizeof(msg), WAIT_FOREVER);
      assert(ret != ERROR);

      NTRACE(8, msg.uid, msg.cid, "read msg in tSpecDoor");
      assert(msg.type == specDoor_type);
      spec = msg.u.specDoor.spec;
      uid = msg.uid;
      cid = msg.cid;
      
      if(spec != SPECTROGRAPH1 && spec != SPECTROGRAPH2) {
	 OTRACE(0, "tSpecDoor illegal choice of spectrograph %d", spec, 0);
	 sendStatusMsg_I(uid, cid, INFORMATION_CODE, 1, "badSpectrograph", spec);
	 sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "specdoorOperation");
	 continue;
      }

      if(semTake(semSLC,60) == ERROR) {
	 NTRACE_2(0, msg.uid, msg.cid, "tSpecDoor: SP%d unable to take semSLM semaphore: %s",
		   spec, strerror(errno));
	 sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "specdoorOperation");
	 continue;
      }

      err = slc_read_blok(1, 10, BIT_FILE, 0, &ctrl[0], sizeof(tm_ctrl)/2);
      if(err) {
	 semGive(semSLC);
	 NTRACE_2(0, msg.uid, msg.cid, "tSpecDoor: SP%d error reading slc: 0x%04x", spec + 1, err);
	 sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "specdoorOperation");
	 continue;
      }
      swab ((char *)&ctrl[0], (char *)&tm_ctrl, sizeof(tm_ctrl));

      switch (msg.u.specDoor.op) {
       case OPEN:
	 if(msg.u.specDoor.spec == SPECTROGRAPH1) {
	    tm_ctrl.mcp_slit_dr1_opn_cmd = 1;
	    tm_ctrl.mcp_slit_dr1_cls_cmd = 0;
	 } else {
	    tm_ctrl.mcp_slit_dr2_opn_cmd = 1;
	    tm_ctrl.mcp_slit_dr2_cls_cmd = 0;
	 }
	 break;
       case CLOSE:
	 if(msg.u.specDoor.spec == SPECTROGRAPH1) {
	    tm_ctrl.mcp_slit_dr1_opn_cmd = 0;
	    tm_ctrl.mcp_slit_dr1_cls_cmd = 1;
	 } else {
	    tm_ctrl.mcp_slit_dr2_opn_cmd = 0;
	    tm_ctrl.mcp_slit_dr2_cls_cmd = 1;
	 }
	 break;
       case CLEAR:
	 if(msg.u.specDoor.spec == SPECTROGRAPH1) {
	    tm_ctrl.mcp_slit_dr1_opn_cmd = 0;
	    tm_ctrl.mcp_slit_dr1_cls_cmd = 0;
	 } else {
	    tm_ctrl.mcp_slit_dr2_opn_cmd = 0;
	    tm_ctrl.mcp_slit_dr2_cls_cmd = 0;
	 }
	 break;
       default:
	 sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "specdoorOperation");
	 NTRACE_2(0, msg.uid, msg.cid, "tSpecDoor: SP%d illegal op %d", spec+1, msg.u.specDoor.op);
	 break;
      }
      
      swab((char *)&tm_ctrl, (char *)&ctrl[0], sizeof(tm_ctrl));
      err = slc_write_blok(1, 10, BIT_FILE, 0, &ctrl[0], sizeof(tm_ctrl)/2);

      semGive(semSLC);
      if(err) {
	 sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "specdoorOperation");
	 NTRACE_2(0, msg.uid, msg.cid, "tSpecDoor: SP%d error writing slc: 0x%04x", spec + 1, err);
	 continue;
      }

      sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "specdoorOperation");
   }
}

/*
 * Commands to send messages to the tSpecDoor task
 */
int
mcp_specdoor_clear(int uid,
		   unsigned long cid,
		   int spec)
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   if(spec != SPECTROGRAPH1 && spec != SPECTROGRAPH2) {
      sendStatusMsg_I(uid, cid, INFORMATION_CODE, 1, "badSpectrograph", spec);
      return(-1);
   }
   
   msg.type = specDoor_type;
   msg.u.specDoor.spec = spec;
   msg.u.specDoor.op = CLEAR;
   msg.uid = uid;
   msg.cid = cid;

   ret = msgQSend(msgSpecDoor, (char *)&msg, sizeof(msg),
		  NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);

   return(0);
}

int
mcp_specdoor_open(int uid,
		  unsigned long cid,
		  int spec)
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   if(spec != SPECTROGRAPH1 && spec != SPECTROGRAPH2) {
      sendStatusMsg_I(uid, cid, INFORMATION_CODE, 1, "badSpectrograph", spec);
      return(-1);
   }
   
   msg.type = specDoor_type;
   msg.u.specDoor.spec = spec;
   msg.u.specDoor.op = OPEN;
   msg.uid = uid;
   msg.cid = cid;

   ret = msgQSend(msgSpecDoor, (char *)&msg, sizeof(msg),
		  NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);

   return(0);
}

int
mcp_specdoor_close(int uid,
		   unsigned long cid,
		   int spec)
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   if(spec != SPECTROGRAPH1 && spec != SPECTROGRAPH2) {
      sendStatusMsg_I(uid, cid, INFORMATION_CODE, 1, "badSpectrograph", spec);
      return(-1);
   }
   
   msg.type = specDoor_type;
   msg.u.specDoor.spec = spec;
   msg.u.specDoor.op = CLOSE;
   msg.uid = uid;
   msg.cid = cid;

   ret = msgQSend(msgSpecDoor, (char *)&msg, sizeof(msg),
		  NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);

   return(0);
}

/*=========================================================================
**
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
tm_slithead(short val) 
{
   int err;
   unsigned short ctrl[2];
   B10_L0 tm_ctrl;   
             
   if(semTake (semSLC,60) == ERROR) {
      printf("tm_slithead: unable to take semaphore: %s", strerror(errno));
      NTRACE_1(0, 0, 0, "Unable to take semaphore: %d", errno);
      return(-1);
   }

   err = slc_read_blok(1, 10, BIT_FILE, 0, &ctrl[0], sizeof(tm_ctrl)/2);
   if(err) {
      semGive (semSLC);
      printf ("R Err=%04x\r\n",err);
      return err;
   }
   swab ((char *)&ctrl[0], (char *)&tm_ctrl, sizeof(tm_ctrl));

   switch (val) {
    case 3:
      tm_ctrl.mcp_slit_latch2_cmd = 1;
      break;
    case 2:
      tm_ctrl.mcp_slit_latch2_cmd = 0;
      break;
    case 1:
      tm_ctrl.mcp_slit_latch1_cmd = 1;
      break;
    case 0:
      tm_ctrl.mcp_slit_latch1_cmd = 0;
      break;
   }
   
   swab ((char *)&tm_ctrl, (char *)&ctrl[0], sizeof(tm_ctrl));
   err = slc_write_blok(1, 10, BIT_FILE, 0, &ctrl[0], sizeof(tm_ctrl)/2);
   semGive (semSLC);
   if(err) {
      NTRACE_1(0, 0, 0, "tm_slithead: error writing slc: 0x%04x", err);
      return err;
   }

   return 0;
}

void
tm_slithead_latch(int spec)
{
   tm_slithead(2*spec + 1);
}

void
tm_slithead_unlatch(int spec)
{
   tm_slithead(2*spec);
}

int
tm_slit_status()
{
   int err;
   unsigned short ctrl[2];
   B10_L0 tm_ctrl;

   if(semTake(semSLC,60) == ERROR) {
      printf("tm_slit_status: unable to take semaphore: %s", strerror(errno));
      NTRACE_1(0, 0, 0, "Unable to take semaphore: %d", errno);
      return(-1);
   }

   err = slc_read_blok(1, 10, BIT_FILE, 0, &ctrl[0], sizeof(tm_ctrl)/2);
   semGive (semSLC);
   if (err) {
      printf ("R Err=%04x\r\n",err);
      return err;
   }
   swab((char *)&ctrl[0],(char *)&tm_ctrl, sizeof(tm_ctrl));
   
  printf (" read ctrl = 0x%04lx\r\n", *(unsigned long *)&ctrl);
  printf ("\r\n mcp_slit_dr1_opn_cmd=%d, mcp_slit_dr1_cls_cmd=%d",
     tm_ctrl.mcp_slit_dr1_opn_cmd,tm_ctrl.mcp_slit_dr1_cls_cmd);
  printf ("\r\n mcp_slit_dr2_opn_cmd=%d, mcp_slit_dr2_cls_cmd=%d",
     tm_ctrl.mcp_slit_dr2_opn_cmd,tm_ctrl.mcp_slit_dr2_cls_cmd);
  printf ("\r\n mcp_slit_latch1_cmd=%d, mcp_slit_latch2_cmd=%d",
     tm_ctrl.mcp_slit_latch1_cmd,tm_ctrl.mcp_slit_latch2_cmd);

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
mcp_slithead_latch_open(int spec)
{
   if(spec != SPECTROGRAPH1 && spec != SPECTROGRAPH2) {
      sendStatusMsg_I(0, 0, INFORMATION_CODE, 1, "badSpectrograph", spec);
      return(-1);
   }
   
   tm_slithead_unlatch(spec);

   return(0);
}

int
mcp_slithead_latch_close(int spec)
{
   if(spec != SPECTROGRAPH1 && spec != SPECTROGRAPH2) {
      sendStatusMsg_I(0, 0, INFORMATION_CODE, 1, "badSpectrograph", spec);
      return(-1);
   }
   
   tm_slithead_latch(spec);

   return(0);
}

/*****************************************************************************/
/*
 * Open or close the flat field screens
 *
 * Set the FFS control bits. If val is < 0, it isn't set
 */
static int which_ffs = 0x3;		/* which petals should I move? */

static int
set_mcp_ffs_bits(int val,		/* value of mcp_ff_scrn_opn_cmd */
		 int enab)		/* value of mcp_ff_screen_enable */
{
   int uid = 0, cid = 0;
   struct B10 b10;
   unsigned short ctrl[sizeof(b10)/2];
   int err;
   int val1, val2;			/* operations for screen[12] */
             
   NTRACE_2(3, 0, 0, "Setting FFS: %d %d", val, enab);
/*
 * What do they want us to do?
 */
   switch (val) {
    case FFS_SAME:
    case FFS_OPEN:
    case FFS_CLOSE:
      val1 = val2 = val;
      break;
    case FFS_TOGGLE:
      val1 = ffs_open_status(1, 1) ? FFS_CLOSE : FFS_OPEN;
      val2 = ffs_open_status(2, 1) ? FFS_CLOSE : FFS_OPEN;
      break;
   }
/*
 * If we're currently enabled and the requested motion is in the opposite
 * direction, disable the FFS for a moment before complying with the request.
 * (Why? French doesn't want us to blow relays)
 */
   if(enab && val >= 0 &&
      (((which_ffs & 0x1) && sdssdc.status.b10.w0.mcp_ff_screen_enable &&
	sdssdc.status.b10.w0.mcp_ff_scrn_opn_cmd != val1) ||
       ((which_ffs & 0x2) && sdssdc.status.b10.w1.mcp_ff_screen2_enabl &&
	sdssdc.status.b10.w1.mcp_ff_scrn2_opn_cmd != val2))) {
      int ntick = sysClkRateGet();		/* 1 second */

      NTRACE_2(3, uid, cid, "Disabling FFS for %d ticks: %d", ntick, val);
      set_mcp_ffs_bits(val, 0);

      taskDelay(ntick);
   }
/*
 * Do what is asked of us
 */
   if(semTake(semSLC,60) == ERROR) {
      NTRACE_2(0, uid, cid, "Unable to take semaphore: %s (%d)", strerror(errno), errno);
      return(-1);
   }

   err = slc_read_blok(1, 10, BIT_FILE, 0, ctrl, sizeof(b10)/2);
   if(err) {
      semGive (semSLC);
      NTRACE_1(0, uid, cid, "set_mcp_ffs_bits: error reading slc: 0x%04x", err);
      return err;
   }
   swab((char *)ctrl, (char *)&b10, sizeof(b10));
      
   if(which_ffs & 0x1) {
      if(val1 >= 0) {
	 b10.w0.mcp_ff_scrn_opn_cmd = val1;
      }
      b10.w0.mcp_ff_screen_enable = enab;
   }

   if(which_ffs & 0x2) {
      if(val2 >= 0) {
	 b10.w1.mcp_ff_scrn2_opn_cmd = val2;
      }
      b10.w1.mcp_ff_screen2_enabl = enab;
   }

   swab ((char *)&b10, (char *)ctrl, sizeof(b10));
   err = slc_write_blok(1, 10, BIT_FILE, 0, ctrl, sizeof(b10)/2);
      
   if(err) { 
      semGive (semSLC);
      NTRACE_1(0, uid, cid, "set_mcp_ffs_bits: error writing slc: 0x%04x", err);
      return err;
   }

   semGive (semSLC);

   return 0;
}

/*
 * Enable the flat field screen
 */
int
ffs_enable(int val)
{
   return(set_mcp_ffs_bits(FFS_SAME, 1));
}

/*
 * Return status of flat field screen.  Only 6 of the 8 petals need have moved,
 * or 3 of the 4 if only half are being commanded
 */
int
ffs_open_status(int which,
		int silent)
{
   int uid = 0, cid = 0;
   int nopen;				/* number of open petals */
   int nopen1 =				/* first 4 petals */
     (sdssdc.status.i1.il13.leaf_1_open_stat ? 1 : 0) + 
       (sdssdc.status.i1.il13.leaf_2_open_stat ? 1 : 0) +
	 (sdssdc.status.i1.il13.leaf_3_open_stat ? 1 : 0) +
	   (sdssdc.status.i1.il13.leaf_4_open_stat ? 1 : 0);
   int nopen2 =				/* last 4 petals */
     (sdssdc.status.i1.il13.leaf_5_open_stat ? 1 : 0) +
       (sdssdc.status.i1.il13.leaf_6_open_stat ? 1 : 0) +
	 (sdssdc.status.i1.il13.leaf_7_open_stat ? 1 : 0) +
	   (sdssdc.status.i1.il13.leaf_8_open_stat ? 1 : 0);

   if(which == 0x3) {			/* all petals */
      nopen = nopen1 + nopen2;
      if(nopen >= 6) {
	 if(!silent && nopen != 8) {
	    NTRACE_1(0, uid, cid, "Only %d flat field screen petals are open", nopen);
	 }
	 return(TRUE);
      } else {
	 if(!silent && nopen != 0) {
	    NTRACE_1(0, uid, cid, "%d flat field screen petals are still open", nopen);
	 }
	 return(FALSE);
      }
   } else {				/* just first/last 4 */
      nopen = (which & 0x1) ? nopen1 : nopen2;
      if(nopen >= 3) {
	 if(!silent && nopen != 4) {
	    NTRACE_1(0, uid, cid, "Only %d flat field screen petals are open", nopen);
	 }
	 return(TRUE);
      } else {
	 if(!silent && nopen != 0) {
	    NTRACE_1(0, uid, cid, "%d flat field screen petals are still open", nopen);
	 }
	 return(FALSE);
      }
   }
}

int
ffs_close_status(int which,
		int silent)
{   
   int uid = 0, cid = 0;
   int nclosed;
   int nclosed1 =
     (sdssdc.status.i1.il13.leaf_1_closed_stat ? 1 : 0) + 
       (sdssdc.status.i1.il13.leaf_2_closed_stat ? 1 : 0) +
	 (sdssdc.status.i1.il13.leaf_3_closed_stat ? 1 : 0) +
	   (sdssdc.status.i1.il13.leaf_4_closed_stat ? 1 : 0);
   int nclosed2 =
     (sdssdc.status.i1.il13.leaf_5_closed_stat ? 1 : 0) +
       (sdssdc.status.i1.il13.leaf_6_closed_stat ? 1 : 0) +
	 (sdssdc.status.i1.il13.leaf_7_closed_stat ? 1 : 0) +
	   (sdssdc.status.i1.il13.leaf_8_closed_stat ? 1 : 0);
   
   if(which == 0x3) {			/* all petals */
      nclosed = nclosed1 + nclosed2;
      if(nclosed >= 6) {
	 if(!silent && nclosed != 8) {
	    NTRACE_1(0, uid, cid, "Only %d flat field screen petals are closed", nclosed);
	 }
	 return(TRUE);
      } else {
	 if(!silent && nclosed != 0) {
	    NTRACE_1(0, uid, cid, "%d flat field screen petals are still closed", nclosed);
	 }
	 return(FALSE);
      }
   } else {
      nclosed = (which & 0x1) ? nclosed1 : nclosed2;
      if(nclosed >= 3) {
	 if(!silent && nclosed != 4) {
	    NTRACE_1(0, uid, cid, "Only %d flat field screen petals are closed", nclosed);
	 }
	 return(TRUE);
      } else {
	 if(!silent && nclosed != 0) {
	    NTRACE_1(0, uid, cid, "%d flat field screen petals are still closed", nclosed);
	 }
	 return(FALSE);
      }
   }
}

/*****************************************************************************/
/*
 * Here's the spawned task that actually does the work
 */
void
tFFS(void)
{
   int uid = 0;				/* User ID */
   unsigned long cid = 0;		/* Command ID */
   int FFS_vals[3];			/* do we want to open/close
					   screen[12] */
   MCP_MSG msg;				/* message to pass around */
   int msg_type;			/* type of check message to send */
   int ret;				/* return code */
   int wait = 15;			/* FF screen timeout (seconds) */

   for(;;) {
      ret = msgQReceive(msgFFS, (char *)&msg, sizeof(msg), WAIT_FOREVER);
      assert(ret != ERROR);

      uid = msg.uid;
      cid = msg.cid;
      
      NTRACE(8, uid, cid, "read msg on msgFFS");
/*
 * What sort of message?
 *   FFS_type            A request from the outside world to move screens
 *   FFSCheckMoved_type  A request from us to check that the screens moved
 */
      if(msg.type == FFS_type) {
	 (void)timerSendArg(FFSCheckMoved_type, tmr_e_abort_ns,
			    0, msg.uid, msg.cid, 0);
      } else if(msg.type == FFSCheckMoved_type) {
	 int i;
	 int move_ok = 1;		/* did move complete OK? */

	 get_uid_cid_from_tmr_msg(&msg, &uid, &cid);

	 for(i = 1; i <= 2; i++) {
	    if(FFS_vals[i] == FFS_OPEN) {
	       if(!ffs_open_status(i, 0)) {
		  move_ok = 0;
		  sendStatusMsg_B(uid, cid, INFORMATION_CODE, 1, "ffsMoved", 0);
		  sendStatusMsg_B(uid, cid, INFORMATION_CODE, 1, "ffsOpenFailed", 1);

		  NTRACE_1(0, uid, cid, "FFS %d did NOT all open; closing", i);
		  ffsclose_cmd(INTERNAL_UID, nextInternalCid(), NULL);
	       }
	    } else {
	       if(!ffs_close_status(i, 0)) {
		  move_ok = 0;
		  sendStatusMsg_B(uid, cid, INFORMATION_CODE, 1, "ffsMoved", 0);
		  sendStatusMsg_B(uid, cid, INFORMATION_CODE, 1, "ffsCloseFailed", 1);
		  NTRACE_1(0, uid, cid, "FFS %d did NOT all close", i);
	       }
	    }
	 }

	 if(move_ok) {
	    OTRACE(1, "Flat field screen moved OK", 0, 0);
	 }

	 broadcast_ffs_lamp_status(uid, cid, 1, 0);
	 sendStatusMsg_S(uid, cid, (move_ok ? FINISHED_CODE : FATAL_CODE), 1, "command", "ffs_move");

	 continue;
      } else {
	 NTRACE_1(0, msg.uid, msg.cid, "Impossible message type on msgFFS: %d", msg.type);
	 continue;
      }
/*
 * Setup to check if the move succeeded
 */
      msg_type = FFSCheckMoved_type;
      if(msg.u.FFS.op == FFS_OPEN) {
	 FFS_vals[1] = (which_ffs & 0x1) ? FFS_OPEN : FFS_SAME;
	 FFS_vals[2] = (which_ffs & 0x2) ? FFS_OPEN : FFS_SAME;

	 if(ffs_open_status(which_ffs, 1)) { /* already open */
	    sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "ffs_open");
	    continue;
	 }
      } else if(msg.u.FFS.op == FFS_TOGGLE) {
	 if(which_ffs & 0x1) {
	    FFS_vals[1] = ffs_open_status(1, 1) ? FFS_CLOSE : FFS_OPEN;
	 } else {
	    FFS_vals[1] = FFS_SAME;
	 }
	 if(which_ffs & 0x2) {
	    FFS_vals[2] = ffs_open_status(2, 1) ? FFS_CLOSE : FFS_OPEN;
	 } else {
	    FFS_vals[2] = FFS_SAME;
	 }
      } else {
	 FFS_vals[1] = (which_ffs & 0x1) ? FFS_CLOSE : FFS_SAME;
	 FFS_vals[2] = (which_ffs & 0x2) ? FFS_CLOSE : FFS_SAME;

	 if(ffs_close_status(which_ffs, 1)) { /* already closed */
	    sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "ffs_close");
	    continue;
	 }
      }
/*
 * Time to do the work
 */
      if(set_mcp_ffs_bits(msg.u.FFS.op, 1) != 0) {
	 continue;
      }
      
      sendStatusMsg_B(msg.uid, msg.cid, INFORMATION_CODE, 1, "ffsCommanded", 1);
      OTRACE(1, "Waiting %ds for flat field screen to move", wait, 0);
/*
 * And schedule the check
 */
      if(timerSendArg(msg_type, tmr_e_add, wait*60, msg.uid, msg.cid, msgFFS) == ERROR) {
	 NTRACE_2(0, msg.uid, msg.cid, "Failed to send message to timer task: %s (%d)",
		   strerror(errno), errno);
      }
   }
}

/*****************************************************************************/
/*
 * A task to control the lamps
 */
void
tLamps(void)
{
   int err;
   unsigned short ctrl[2];
   int b10_l0;				/* set bit in B10_L0? */
   MCP_MSG msg;				/* message to pass around */
   int ret;				/* return code */
   B10_L0 tm_ctrl0;
   B10_L1 tm_ctrl1;

   for(;;) {
      ret = msgQReceive(msgLamps, (char *)&msg, sizeof(msg), WAIT_FOREVER);
      assert(ret != ERROR);

      NTRACE(8, msg.uid, msg.cid, "read msg on msgLamps");

      if (msg.type == lampsCheck_type) {
	 int uid = 0; unsigned long cid = 0;
	 get_uid_cid_from_tmr_msg(&msg, &uid, &cid);

	 broadcast_ffs_lamp_status(uid, cid, 0, 1);
	 sendStatusMsg_N(uid, cid, FINISHED_CODE, 1, "controlLamps");
	      
	 continue;
      }

      assert(msg.type == lamps_type);

      b10_l0 = -1;
      switch (msg.u.lamps.type) {
       case FF_LAMP:
	 b10_l0 = 1;
	 break;
       case NE_LAMP:
	 b10_l0 = 1;
	 break;
       case HGCD_LAMP:
	 b10_l0 = 1;
	 break;
       case UV_LAMP:
	 b10_l0 = 0;
	 break;
       case WHT_LAMP:
	 b10_l0 = 0;
	 break;
       default:
	 NTRACE_1(0, msg.uid, msg.cid, "Impossible lamp type: %d", msg.type);
	 break;
      }

      if(b10_l0 != 0 && b10_l0 != 1) {	/* an illegal value */
	 break;
      }

      if(semTake(semSLC,60) == ERROR) {
	 printf("Unable to take semaphore: %s", strerror(errno));
	 NTRACE_1(0, msg.uid, msg.cid, "Unable to take semaphore: %d", errno);
      }

      if(b10_l0) {
	 err = slc_read_blok(1, 10, BIT_FILE, 0, &ctrl[0], sizeof(tm_ctrl0)/2);
      } else {
	 err = slc_read_blok(1, 10, BIT_FILE, 2, &ctrl[0], sizeof(tm_ctrl1)/2);
      }
      
      if(err) {
	 semGive(semSLC);
	 printf("R Err=%04x\r\n",err);
      }

      swab((char *)&ctrl[0], (char *)&tm_ctrl0, sizeof(tm_ctrl0));
      
      switch (msg.u.lamps.type) {
       case FF_LAMP:
	 tm_ctrl0.mcp_ff_lamp_on_cmd = msg.u.lamps.on_off;
	 break;
       case NE_LAMP:
	 tm_ctrl0.mcp_ne_lamp_on_cmd = msg.u.lamps.on_off;
	 break;
       case HGCD_LAMP:
	 tm_ctrl0.mcp_hgcd_lamp_on_cmd = msg.u.lamps.on_off;
	 break;
       case UV_LAMP:
	 tm_ctrl1.mcp_im_ff_uv_req = msg.u.lamps.on_off;
	 break;
       case WHT_LAMP:
	 tm_ctrl1.mcp_im_ff_wht_req = msg.u.lamps.on_off;
	 break;
       default:
	 NTRACE_1(0, msg.uid, msg.cid, "Impossible lamp type: %d", msg.type);
	 break;
      }
      
      if(b10_l0) {
	 swab ((char *)&tm_ctrl0, (char *)&ctrl[0], sizeof(tm_ctrl0));
	 err = slc_write_blok(1, 10, BIT_FILE, 0, &ctrl[0],sizeof(tm_ctrl0)/2);
      } else {
	 swab ((char *)&tm_ctrl1, (char *)&ctrl[0], sizeof(tm_ctrl1));
	 err = slc_write_blok(1, 10, BIT_FILE, 2, &ctrl[0],sizeof(tm_ctrl1)/2);
      }
      semGive (semSLC);
      
      if(err) {
	 printf("W Err=%04x\r\n",err);
      }
      /*
       * Wait a few seconds and see if the lights did what they were told
       */
      {
	 int wait = 5;			/* how many seconds to wait */

	 sendStatusMsg_B(msg.uid, msg.cid, INFORMATION_CODE, 1, "lampsCommanded", 1);
	 
	 if(timerSendArg(lampsCheck_type, tmr_e_add, wait*60,
			 msg.uid, msg.cid, msgLamps) == ERROR) {
	    NTRACE_2(0, msg.uid, msg.cid, "Failed to send message to timer task: %s (%d)",
		     strerror(errno), errno);
	 }
      }
   }
}

/*****************************************************************************/
/*
 * Report the status of the spectrograph doors/slits
 */
int
get_slitstatus(char *slitstatus_ans,
	       int size)			/* dimen of slitstatus_ans */
{
  int len;

  sprintf(slitstatus_ans,"SP1: %d %d %d %d  SP2: %d %d %d %d\n",
	  sdssdc.status.i1.il9.slit_head_door1_opn,
	  sdssdc.status.i1.il9.slit_head_door1_cls,
	  sdssdc.status.i1.il9.slit_head_latch1_ext,
	  sdssdc.status.i1.il9.slit_head_1_in_place,
	  sdssdc.status.i1.il9.slit_head_door2_opn,
	  sdssdc.status.i1.il9.slit_head_door2_cls,
	  sdssdc.status.i1.il9.slit_head_latch2_ext,
	  sdssdc.status.i1.il9.slit_head_2_in_place);
  
  len = strlen(slitstatus_ans);
  assert(len < size);

  return(len);
}

/*****************************************************************************/
/*
 * Report the status of the flatfield screen and lamps
 */
int
get_ffstatus(char *ffstatus_ans,
	     int size)			/* dimen of ffstatus_ans */
{
  int len;

  sprintf (&ffstatus_ans[strlen(&ffstatus_ans[0])],
	"FFS  %d%d %d%d %d%d %d%d  %d%d %d%d %d%d %d%d  %d %d\n",
	   sdssdc.status.i1.il13.leaf_1_open_stat,
	   sdssdc.status.i1.il13.leaf_1_closed_stat,
	   sdssdc.status.i1.il13.leaf_2_open_stat,
	   sdssdc.status.i1.il13.leaf_2_closed_stat,
	   sdssdc.status.i1.il13.leaf_3_open_stat,
	   sdssdc.status.i1.il13.leaf_3_closed_stat,
	   sdssdc.status.i1.il13.leaf_4_open_stat,
	   sdssdc.status.i1.il13.leaf_4_closed_stat,
	   sdssdc.status.i1.il13.leaf_5_open_stat,
	   sdssdc.status.i1.il13.leaf_5_closed_stat,
	   sdssdc.status.i1.il13.leaf_6_open_stat,
	   sdssdc.status.i1.il13.leaf_6_closed_stat,
	   sdssdc.status.i1.il13.leaf_7_open_stat,
	   sdssdc.status.i1.il13.leaf_7_closed_stat,
	   sdssdc.status.i1.il13.leaf_8_open_stat,
	   sdssdc.status.i1.il13.leaf_8_closed_stat,
	   sdssdc.status.o1.ol14.ff_screen_open_pmt,
	   which_ffs);
  sprintf(&ffstatus_ans[strlen(ffstatus_ans)],"FF   %d %d %d %d  %d\n",
	  sdssdc.status.i1.il13.ff_1_stat,
	  sdssdc.status.i1.il13.ff_2_stat,
	  sdssdc.status.i1.il13.ff_3_stat,
	  sdssdc.status.i1.il13.ff_4_stat,
	  sdssdc.status.o1.ol14.ff_lamps_on_pmt);
  sprintf(&ffstatus_ans[strlen(ffstatus_ans)],"Ne   %d %d %d %d  %d\n",
	  sdssdc.status.i1.il13.ne_1_stat,
	  sdssdc.status.i1.il13.ne_2_stat,
	  sdssdc.status.i1.il13.ne_3_stat,
	  sdssdc.status.i1.il13.ne_4_stat,
	  sdssdc.status.o1.ol14.ne_lamps_on_pmt);
  sprintf(&ffstatus_ans[strlen(ffstatus_ans)],"HgCd %d %d %d %d  %d\n",
	  sdssdc.status.i1.il13.hgcd_1_stat,
	  sdssdc.status.i1.il13.hgcd_2_stat,
	  sdssdc.status.i1.il13.hgcd_3_stat,
	  sdssdc.status.i1.il13.hgcd_4_stat,
	  sdssdc.status.o1.ol14.hgcd_lamps_on_pmt);
  sprintf(&ffstatus_ans[strlen(ffstatus_ans)],"UV %d %d %d %d  %d\n",
	  0, 0, 0, 0,
	  sdssdc.status.o1.ol14.im_ff_uv_on_pmt);
  sprintf(&ffstatus_ans[strlen(ffstatus_ans)],"WHT %d %d %d %d  %d\n",
	  0, 0, 0, 0,
	  sdssdc.status.o1.ol14.im_ff_wht_on_pmt);

  len = strlen(ffstatus_ans);
  if (len >= size) {
     printf("RHL len = %d size = %d\n", len, size);
  }
  assert(len < size);
  
  return(len);	
}

/*****************************************************************************/
/*
 * Flatfield screen
 */
/*
 * A routine to broadcast the status of the screens and lamps
 */
void
broadcast_ffs_lamp_status(int uid, unsigned long cid, int petals, int lamps)
{
   char buff[100];

   if (petals) {
      sprintf(buff, "%d%d, %d%d, %d%d, %d%d, %d%d, %d%d, %d%d, %d%d",
	      sdssdc.status.i1.il13.leaf_1_open_stat,
	      sdssdc.status.i1.il13.leaf_1_closed_stat,
	      sdssdc.status.i1.il13.leaf_2_open_stat,
	      sdssdc.status.i1.il13.leaf_2_closed_stat,
	      sdssdc.status.i1.il13.leaf_3_open_stat,
	      sdssdc.status.i1.il13.leaf_3_closed_stat,
	      sdssdc.status.i1.il13.leaf_4_open_stat,
	      sdssdc.status.i1.il13.leaf_4_closed_stat,
	      sdssdc.status.i1.il13.leaf_5_open_stat,
	      sdssdc.status.i1.il13.leaf_5_closed_stat,
	      sdssdc.status.i1.il13.leaf_6_open_stat,
	      sdssdc.status.i1.il13.leaf_6_closed_stat,
	      sdssdc.status.i1.il13.leaf_7_open_stat,
	      sdssdc.status.i1.il13.leaf_7_closed_stat,
	      sdssdc.status.i1.il13.leaf_8_open_stat,
	      sdssdc.status.i1.il13.leaf_8_closed_stat);
      sendStatusMsg_A(uid, cid, INFORMATION_CODE, 1, "ffsStatus", buff);
      sendStatusMsg_B(uid, cid, INFORMATION_CODE, 1, "ffsCommandedOn", sdssdc.status.o1.ol14.ff_screen_open_pmt);
      sprintf(buff, "%d%d", !!(which_ffs & 0x1), !!(which_ffs & 0x2));
      sendStatusMsg_A(uid, cid, INFORMATION_CODE, 1, "ffsSelected", buff);
   }

   if (lamps) {
      sprintf(buff, "%d, %d, %d, %d",
	      sdssdc.status.i1.il13.ff_1_stat,
	      sdssdc.status.i1.il13.ff_2_stat,
	      sdssdc.status.i1.il13.ff_3_stat,
	      sdssdc.status.i1.il13.ff_4_stat);
      sendStatusMsg_A(uid, cid, INFORMATION_CODE, 1, "ffLamp", buff);
      sendStatusMsg_B(uid, cid, INFORMATION_CODE, 1, "ffLampCommandedOn", sdssdc.status.o1.ol14.ff_lamps_on_pmt);

      sprintf(buff, "%d, %d, %d, %d",
	      sdssdc.status.i1.il13.ne_1_stat,
	      sdssdc.status.i1.il13.ne_2_stat,
	      sdssdc.status.i1.il13.ne_3_stat,
	      sdssdc.status.i1.il13.ne_4_stat);
      sendStatusMsg_A(uid, cid, INFORMATION_CODE, 1, "NeLamp", buff);
      sendStatusMsg_B(uid, cid, INFORMATION_CODE, 1, "NeLampCommandedOn", sdssdc.status.o1.ol14.ne_lamps_on_pmt);

      sprintf(buff, "%d, %d, %d, %d",
	      sdssdc.status.i1.il13.hgcd_1_stat,
	      sdssdc.status.i1.il13.hgcd_2_stat,
	      sdssdc.status.i1.il13.hgcd_3_stat,
	      sdssdc.status.i1.il13.hgcd_4_stat);
      sendStatusMsg_A(uid, cid, INFORMATION_CODE, 1, "HgCdLamp", buff);
      sendStatusMsg_B(uid, cid, INFORMATION_CODE, 1, "HgCdLampCommandedOn", sdssdc.status.o1.ol14.hgcd_lamps_on_pmt);

      sendStatusMsg_B(uid, cid, INFORMATION_CODE, 1, "UVLampCommandedOn", sdssdc.status.o1.ol14.im_ff_uv_on_pmt);

      sendStatusMsg_B(uid, cid, INFORMATION_CODE, 1, "whtLampCommandedOn", sdssdc.status.o1.ol14.im_ff_wht_on_pmt);
   }
}

char *
ffstatus_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   broadcast_ffs_lamp_status(uid, cid, 1, 1);
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "ff_status");

   if (ublock->protocol == NEW_PROTOCOL) {
      return "";
   }
   /*
    * Now send command reply
    */
   ublock->buff[0] = '\0';
   (void)get_ffstatus(ublock->buff, UBLOCK_SIZE);
   
   return(ublock->buff);
}

char *
ffsopen_cmd(int uid, unsigned long cid, char *cmd)			/* If == 1, toggle petals */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */
   int toggle = atoi(cmd);

   msg.type = FFS_type;
   msg.u.FFS.op = toggle ? FFS_TOGGLE : FFS_OPEN;
   msg.uid = uid;
   msg.cid = cid;

   ret = msgQSend(msgFFS, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
ffsclose_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = FFS_type;
   msg.u.FFS.op = FFS_CLOSE;
   msg.uid = uid;
   msg.cid = cid;

   ret = msgQSend(msgFFS, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
ffsselect_cmd(int uid, unsigned long cid, char *cmd)
{
   int which = atoi(cmd);

   if(which == 0) {			/* may be hex */
      if(cmd[0] == '0' && (cmd[1] == 'x' || cmd[1] == 'X')) {
	 which = atoi(&cmd[2]);
      }
   }
   
   if(which < 0 || which > 3) {
      NTRACE_1(0, uid, cid, "Invalid FFS.SELECT argument: %s", cmd);
      return("Invalid FFS.SELECT argument");
   }

   which_ffs = which;

   broadcast_ffs_lamp_status(uid, cid, 1, 0);
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "ffs_select");

   return("");
}

/*****************************************************************************/
/*
 * Turn the flatfield/calibration lamps on/off
 */
char *
fflon_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = lamps_type;
   msg.u.lamps.type = FF_LAMP;
   msg.u.lamps.on_off = ON;
   msg.uid = uid;
   msg.cid = cid;

   ret = msgQSend(msgLamps, (char *)&msg, sizeof(msg), NO_WAIT,MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
ffloff_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = lamps_type;
   msg.u.lamps.type = FF_LAMP;
   msg.u.lamps.on_off = OFF;
   msg.uid = uid;
   msg.cid = cid;

   ret = msgQSend(msgLamps, (char *)&msg, sizeof(msg), NO_WAIT,MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
neon_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = lamps_type;
   msg.u.lamps.type = NE_LAMP;
   msg.u.lamps.on_off = ON;
   msg.uid = uid;
   msg.cid = cid;

   ret = msgQSend(msgLamps, (char *)&msg, sizeof(msg), NO_WAIT,MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
neoff_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = lamps_type;
   msg.u.lamps.type = NE_LAMP;
   msg.u.lamps.on_off = OFF;
   msg.uid = uid;
   msg.cid = cid;

   ret = msgQSend(msgLamps, (char *)&msg, sizeof(msg), NO_WAIT,MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
hgcdon_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = lamps_type;
   msg.u.lamps.type = HGCD_LAMP;
   msg.u.lamps.on_off = ON;
   msg.uid = uid;
   msg.cid = cid;

   ret = msgQSend(msgLamps, (char *)&msg, sizeof(msg), NO_WAIT,MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
hgcdoff_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = lamps_type;
   msg.u.lamps.type = HGCD_LAMP;
   msg.u.lamps.on_off = OFF;
   msg.uid = uid;
   msg.cid = cid;

   ret = msgQSend(msgLamps, (char *)&msg, sizeof(msg), NO_WAIT,MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
uvon_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = lamps_type;
   msg.u.lamps.type = UV_LAMP;
   msg.u.lamps.on_off = ON;
   msg.uid = uid;
   msg.cid = cid;

   ret = msgQSend(msgLamps, (char *)&msg, sizeof(msg), NO_WAIT,MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
uvoff_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = lamps_type;
   msg.u.lamps.type = UV_LAMP;
   msg.u.lamps.on_off = OFF;
   msg.uid = uid;
   msg.cid = cid;

   ret = msgQSend(msgLamps, (char *)&msg, sizeof(msg), NO_WAIT,MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
whton_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = lamps_type;
   msg.u.lamps.type = WHT_LAMP;
   msg.u.lamps.on_off = ON;
   msg.uid = uid;
   msg.cid = cid;

   ret = msgQSend(msgLamps, (char *)&msg, sizeof(msg), NO_WAIT,MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
whtoff_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = lamps_type;
   msg.u.lamps.type = WHT_LAMP;
   msg.u.lamps.on_off = OFF;
   msg.uid = uid;
   msg.cid = cid;

   ret = msgQSend(msgLamps, (char *)&msg, sizeof(msg), NO_WAIT,MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

/*****************************************************************************/
/*
 * Commands for the selected spectrograph.
 *
 * These could all be static, except that I want them to show up in cmdList
 */
char *
sp1_cmd(int uid, unsigned long cid, char *cmd)
{
   ublock->spectrograph_select = SPECTROGRAPH1;
   sendStatusMsg_S(uid, cid, INFORMATION_CODE, 0, "command", "sp1"); /* not a "real" command that completes */
   return("");
}

char *
sp2_cmd(int uid, unsigned long cid, char *cmd)
{
  ublock->spectrograph_select = SPECTROGRAPH2;
  sendStatusMsg_S(uid, cid, INFORMATION_CODE, 0, "command", "sp2"); /* not a "real" command that completes */
  return("");
}

/*
 * Status of slithead doors and slit latches
 */
void
broadcast_slit_status(int uid, unsigned long cid)
{
   char buff[100];

   sprintf(buff, "%d%d, %d, %d",
	   sdssdc.status.i1.il9.slit_head_door1_opn,
	   sdssdc.status.i1.il9.slit_head_door1_cls,
	   sdssdc.status.i1.il9.slit_head_latch1_ext,
	   sdssdc.status.i1.il9.slit_head_1_in_place);
   sendStatusMsg_A(uid, cid, INFORMATION_CODE, 1, "sp1Slithead", buff);
   
   sprintf(buff, "%d%d, %d, %d",
	  sdssdc.status.i1.il9.slit_head_door2_opn,
	  sdssdc.status.i1.il9.slit_head_door2_cls,
	  sdssdc.status.i1.il9.slit_head_latch2_ext,
	  sdssdc.status.i1.il9.slit_head_2_in_place);
   sendStatusMsg_A(uid, cid, INFORMATION_CODE, 1, "sp2Slithead", buff);
}


char *
slitstatus_cmd(int uid, unsigned long cid, char *cmd)		/* NOTUSED */
{
   (void)get_slitstatus(ublock->buff, UBLOCK_SIZE);

   broadcast_slit_status(uid, cid);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "slitstatus");
   
   return(ublock->buff);
}

/*
 * Neither close nor open the slit door...allow it to be moved by hand
 */
char *
slitdoor_clear_cmd(int uid, unsigned long cid, char *cmd)		/* NOTUSED */
{
   if(mcp_specdoor_clear(uid, cid, ublock->spectrograph_select) < 0) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "slitdoor_clear");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "slitdoor_clear");
   return "";
}

/*
 * open the door
 */
char *
slitdoor_open_cmd(int uid, unsigned long cid, char *cmd)		/* NOTUSED */
{
   if(mcp_specdoor_open(uid, cid, ublock->spectrograph_select) < 0) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "slitdoor_open");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   broadcast_slit_status(uid, cid);
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "slitdoor_open");
   
   return "";
}

/*
 * close the door
 */
char *
slitdoor_close_cmd(int uid, unsigned long cid, char *cmd)		/* NOTUSED */
{
   if(mcp_specdoor_close(uid, cid, ublock->spectrograph_select) < 0) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "slitdoor_close");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   broadcast_slit_status(uid, cid);
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "slitdoor_close");

   return "";
}

/*
 * latch the slithead
 */
char *
slithead_latch_close_cmd(int uid, unsigned long cid, char *cmd)		/* NOTUSED */
{
   if(mcp_slithead_latch_close(ublock->spectrograph_select) < 0) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "slitdoor_close");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   return "";
}

/*
 * unlatch the slithead
 */
char *
slithead_latch_open_cmd(int uid, unsigned long cid, char *cmd)		/* NOTUSED */
{
   if(mcp_slithead_latch_open(ublock->spectrograph_select) < 0) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "slitdoor_close");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   return "";
}

/*****************************************************************************/
/*
 * Initialise the spectroscopic system:
 *  the FF screen, lamps, the spectrographs themselves
 *
 * Define commands
 */
void
spectroInit(void)
{
   int ret;				/* return code */
/*
 * Create the message queue to control the lamps, and spawn the task
 * that actually turns lamps on/off
 */
   if(msgFFS == NULL) {
#if ENABLE_CLAMP
      msgAlignClamp = msgQCreate(40, sizeof(MCP_MSG), MSG_Q_FIFO);
      assert(msgAlignClamp != NULL);
      ret = taskSpawn("tAlgnClmp",90,0,5000,(FUNCPTR)tAlgnClmp,
		      0,0,0,0,0,0,0,0,0,0);
      assert(ret != ERROR);
   }
#endif
      msgFFS = msgQCreate(40, sizeof(MCP_MSG), MSG_Q_FIFO);
      assert(msgFFS != NULL);
      ret = taskSpawn("tFFS",90,0,5000,(FUNCPTR)tFFS,
		      0,0,0,0,0,0,0,0,0,0);
      assert(ret != ERROR);

      msgLamps = msgQCreate(40, sizeof(MCP_MSG), MSG_Q_FIFO);
      assert(msgLamps != NULL);
      ret = taskSpawn("tLamps",90,0,5000,(FUNCPTR)tLamps,
		      0,0,0,0,0,0,0,0,0,0);
      assert(ret != ERROR);

      msgSpecDoor = msgQCreate(40, sizeof(MCP_MSG), MSG_Q_FIFO);
      assert(msgSpecDoor != NULL);
      ret = taskSpawn("tSpecDoor",90,0,5000,
		      (FUNCPTR)tSpecDoor,
		      0,0,0,0,0,0,0,0,0,0);
      assert(ret != ERROR);
   }
/*
 * define spectro commands to the command interpreter
 */
#if ENABLE_CLAMP
   define_cmd("CLAMP_OFF",           clampoff_cmd, 	       0, 1, 0, 1, "");
   define_cmd("CLAMP_ON",            clampon_cmd, 	       0, 1, 0, 1, "");
#endif
   define_cmd("FF_OFF",              ffloff_cmd,               0, 0, 0, 1, "");
   define_cmd("FF_ON",               fflon_cmd,                0, 0, 0, 1, "");
   define_cmd("FF_STATUS",           ffstatus_cmd,             0, 0, 0, 1, "");
   define_cmd("FFL_OFF",             ffloff_cmd,               0, 0, 0, 1, "");
   define_cmd("FFL_ON",              fflon_cmd,                0, 0, 0, 1, "");
   define_cmd("FFS_CLOSE",           ffsclose_cmd,             0, 0, 0, 1, "Close flat field screens");
   define_cmd("FFS_OPEN",            ffsopen_cmd,             -1, 0, 0, 1,
	      "Open flat field screens;\n"
	      "with an argument 1 open closed ones and close open ones");
   define_cmd("FFS_SELECT",          ffsselect_cmd,            1, 0, 0, 1,
	      "Select petal set 1 (0x1), petal set 2 (0x2) or all (0x3)");
   define_cmd("HGCD_OFF",            hgcdoff_cmd,              0, 0, 0, 1, "");
   define_cmd("HGCD_ON",             hgcdon_cmd,               0, 0, 0, 1, "");
   define_cmd("NE_OFF",              neoff_cmd,                0, 0, 0, 1, "");
   define_cmd("NE_ON",               neon_cmd,                 0, 0, 0, 1, "");
   define_cmd("UV_OFF",              uvoff_cmd,                0, 0, 0, 1, "");
   define_cmd("UV_ON",               uvon_cmd,                 0, 0, 0, 1, "");
   define_cmd("WHT_OFF",             whtoff_cmd,               0, 0, 0, 1, "");
   define_cmd("WHT_ON",              whton_cmd,                0, 0, 0, 1, "");
   define_cmd("SLIT_STATUS",         slitstatus_cmd,           0, 0, 0, 1, "");
   define_cmd("SLITDOOR_CLEAR",      slitdoor_clear_cmd,       0, 0, 0, 1, "");
   define_cmd("SLITDOOR_CLOSE",      slitdoor_close_cmd,       0, 0, 0, 1, "");
   define_cmd("SLITDOOR_OPEN",       slitdoor_open_cmd,        0, 0, 0, 1, "");
   define_cmd("SLITHEADLATCH_CLOSE", slithead_latch_close_cmd, 0, 0, 0, 1, "");
   define_cmd("SLITHEADLATCH_EXT",   slithead_latch_open_cmd,  0, 0, 0, 1, "");
   define_cmd("SLITHEADLATCH_OPEN",  slithead_latch_open_cmd,  0, 0, 0, 1, "");
   define_cmd("SLITHEADLATCH_RET",   slithead_latch_close_cmd, 0, 0, 0, 1, "");
   define_cmd("SP1", sp1_cmd,                                  0, 0, 0, 0, "");
   define_cmd("SP2", sp2_cmd,                                  0, 0, 0, 0, "");
}
