#include <vxWorks.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <semLib.h>
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

char *ffsclose_cmd(char *cmd);

MSG_Q_ID msgAlignClamp = NULL;		/* control alignment clamp */
MSG_Q_ID msgFFS = NULL;			/* control flat field screens */
MSG_Q_ID msgLamps = NULL;		/* control lamps */
MSG_Q_ID msgSpecDoor = NULL;		/* control spectrograph doors */

/*****************************************************************************/
/*
 * Control the alignment clamp.
 */
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

      TRACE(8, "read msg on msgAlignClamp", 0, 0);
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
	    TRACE(0, "Alignment clamp did NOT engage..."
		  "turning off and disengaging", 0, 0);
	    engage = 0;
	 }
	 break;
       default:
	 TRACE(0, "Impossible message type: %d", msg.type, 0);
	 continue;	 
      }
/*
 * Time to do the work
 */
      if(semTake(semSLC,60) == ERROR) {
	 TRACE(0, "Unable to take semaphore: %s (%d)", strerror(errno), errno);
	 continue;
      }
      
      err = slc_read_blok(1,10,BIT_FILE,0,&ctrl[0],sizeof(tm_ctrl)/2);
      if(err) {
	 semGive(semSLC);
	 TRACE(0, "tAlgnClmp: error reading slc: 0x%04x", err, 0);
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
	 TRACE(0, "tAlgnClmp: error writing slc: 0x%04x", err, 0);
	 continue;
      }
      
      if(engage) {			/* wait, then see if we succeeded */
	 int wait = 15;			/* how many seconds to wait */
	 TRACE(1, "Waiting %ds for alignment clamp to engage", wait, 0);
	 
	 if(timerSend(alignClampCheck_type, tmr_e_add,
		      wait*60, alignClampCheck_type, msgAlignClamp) == ERROR) {
	    TRACE(0, "Failed to send message to timer task: %s (%d)",
		  strerror(errno), errno);
	 }
      }
   }
}

static void
alignment_clamp_set(int engage)
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */
   
   msg.type = alignClamp_type;
   msg.u.alignClamp.op = engage ? ENGAGE : DISENGAGE;

   ret = msgQSend(msgAlignClamp, (char *)&msg, sizeof(msg),
		  NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);
}

/*****************************************************************************/
/*
 * Control the spectrographs' slithead doors
 */
void
tSpecDoor(void)
{
   int err;
   unsigned short ctrl[2];
   MCP_MSG msg;				/* message to pass around */
   int ret;				/* return code */
   B10_L0 tm_ctrl;
   int spec;				/* which spectrograph? */

   for(;;) {
      ret = msgQReceive(msgSpecDoor, (char *)&msg, sizeof(msg), WAIT_FOREVER);
      assert(ret != ERROR);

      TRACE(8, "read msg in tSpecDoor", 0, 0);
      assert(msg.type == specDoor_type);
      spec = msg.u.specDoor.spec;
      
      if(spec != SPECTROGRAPH1 && spec != SPECTROGRAPH2) {
	 TRACE(0, "tSpecDoor illegal choice of spectrograph %d", spec, 0);
	 continue;
      }

      if(semTake(semSLC,60) == ERROR) {
	 TRACE(0, "tSpecDoor: SP%d unable to take semSLM semaphore: %s",
	       spec, strerror(errno));
	 continue;
      }

      err = slc_read_blok(1, 10, BIT_FILE, 0, &ctrl[0], sizeof(tm_ctrl)/2);
      if(err) {
	 semGive(semSLC);
	 TRACE(0, "tSpecDoor: SP%d error reading slc: 0x%04x", spec + 1, err);
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
	 TRACE(0, "tSpecDoor: SP%d illegal op %d", spec+1, msg.u.specDoor.op);
	 break;
      }
      
      swab((char *)&tm_ctrl, (char *)&ctrl[0], sizeof(tm_ctrl));
      err = slc_write_blok(1, 10, BIT_FILE, 0, &ctrl[0], sizeof(tm_ctrl)/2);

      semGive(semSLC);
      if(err) {
	 TRACE(0, "tSpecDoor: SP%d error writing slc: 0x%04x", spec + 1, err);
	 continue;
      }
   }
}

/*
 * Commands to send messages to the tSpecDoor task
 */
int
mcp_specdoor_clear(int spec)
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   if(spec != SPECTROGRAPH1 && spec != SPECTROGRAPH2) {
      return(-1);
   }
   
   msg.type = specDoor_type;
   msg.u.specDoor.spec = spec;
   msg.u.specDoor.op = CLEAR;

   ret = msgQSend(msgSpecDoor, (char *)&msg, sizeof(msg),
		  NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);

   return(0);
}

int
mcp_specdoor_open(int spec)
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   if(spec != SPECTROGRAPH1 && spec != SPECTROGRAPH2) {
      return(-1);
   }
   
   msg.type = specDoor_type;
   msg.u.specDoor.spec = spec;
   msg.u.specDoor.op = OPEN;

   ret = msgQSend(msgSpecDoor, (char *)&msg, sizeof(msg),
		  NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);

   return(0);
}

int
mcp_specdoor_close(int spec)
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   if(spec != SPECTROGRAPH1 && spec != SPECTROGRAPH2) {
      return(-1);
   }
   
   msg.type = specDoor_type;
   msg.u.specDoor.spec = spec;
   msg.u.specDoor.op = CLOSE;

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
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
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
      TRACE(0, "tm_slithead: error writing slc: 0x%04x", err, 0);
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
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
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
      return(-1);
   }
   
   tm_slithead_unlatch(spec);

   return(0);
}

int
mcp_slithead_latch_close(int spec)
{
   if(spec != SPECTROGRAPH1 && spec != SPECTROGRAPH2) {
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
   struct B10 b10;
   unsigned short ctrl[sizeof(b10)/2];
   int err;
   int val1, val2;			/* operations for screen[12] */
             
   TRACE(3, "Setting FFS: %d %d", val, enab); /* XXX */
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
      int ntick = 60;			/* 1 second */

      TRACE(3, "Disabling FFS for %d ticks: %d", ntick, val);
      set_mcp_ffs_bits(val, 0);

      taskDelay(ntick);
   }
/*
 * Do what is asked of us
 */
   if(semTake(semSLC,60) == ERROR) {
      TRACE(0, "Unable to take semaphore: %s (%d)", strerror(errno), errno);
      return(-1);
   }

   err = slc_read_blok(1, 10, BIT_FILE, 0, ctrl, sizeof(b10)/2);
   if(err) {
      semGive (semSLC);
      TRACE(0, "set_mcp_ffs_bits: error reading slc: 0x%04x", err, 0);
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
      TRACE(0, "set_mcp_ffs_bits: error writing slc: 0x%04x", err, 0);
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
	    TRACE(0, "Only %d flat field screen petals are open", nopen, 0);
	 }
	 return(TRUE);
      } else {
	 if(!silent && nopen != 0) {
	    TRACE(0, "%d flat field screen petals are still open", nopen, 0);
	 }
	 return(FALSE);
      }
   } else {				/* just first/last 4 */
      nopen = (which & 0x1) ? nopen1 : nopen2;
      if(nopen >= 3) {
	 if(!silent && nopen != 4) {
	    TRACE(0, "Only %d flat field screen petals are open", nopen, 0);
	 }
	 return(TRUE);
      } else {
	 if(!silent && nopen != 0) {
	    TRACE(0, "%d flat field screen petals are still open", nopen, 0);
	 }
	 return(FALSE);
      }
   }
}

int
ffs_close_status(int which,
		int silent)
{   
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
	    TRACE(0, "Only %d flat field screen petals are closed",nclosed,0);
	 }
	 return(TRUE);
      } else {
	 if(!silent && nclosed != 0) {
	    TRACE(0, "%d flat field screen petals are still closed",nclosed,0);
	 }
	 return(FALSE);
      }
   } else {
      nclosed = (which & 0x1) ? nclosed1 : nclosed2;
      if(nclosed >= 3) {
	 if(!silent && nclosed != 4) {
	    TRACE(0, "Only %d flat field screen petals are closed",nclosed,0);
	 }
	 return(TRUE);
      } else {
	 if(!silent && nclosed != 0) {
	    TRACE(0, "%d flat field screen petals are still closed",nclosed,0);
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
   int FFS_vals[3];			/* do we want to open/close
					   screen[12] */
   MCP_MSG msg;				/* message to pass around */
   int msg_type;			/* type of check message to send */
   int ret;				/* return code */
   int wait = 15;			/* FF screen timeout (seconds) */

   for(;;) {
      ret = msgQReceive(msgFFS, (char *)&msg, sizeof(msg), WAIT_FOREVER);
      assert(ret != ERROR);

      TRACE(8, "read msg on msgFFS", 0, 0);
/*
 * What sort of message?
 *   FFS_type            A request from the outside world to move screens
 *   FFSCheckMoved_type  A request from us to check that the screens moved
 */
      if(msg.type == FFS_type) {
	 (void)timerSend(FFSCheckMoved_type, tmr_e_abort_ns,
			 0, FFSCheckMoved_type, 0);
      } else if(msg.type == FFSCheckMoved_type) {
	 int i;
	 int move_ok = 1;		/* did move complete OK? */

	 for(i = 1; i <= 2; i++) {
	    if(FFS_vals[i] == FFS_OPEN) {
	       if(!ffs_open_status(i, 0)) {
		  move_ok = 0;
		  TRACE(0, "FFS %d did NOT all open; closing", i, 0);
		  ffsclose_cmd(NULL);
	       }
	    } else {
	       if(!ffs_close_status(i, 0)) {
		  move_ok = 0;
		  TRACE(0, "FFS %d did NOT all close", i, 0);
	       }
	    }
	 }

	 if(move_ok) {
	    TRACE(1, "Flat field screen moved OK", 0, 0);
	 }

	 continue;
      } else {
	 TRACE(0, "Impossible message type: %d", msg.type, 0);
	 continue;
      }
/*
 * Setup to check if the move succeeded
 */
      msg_type = FFSCheckMoved_type;
      if(msg.u.FFS.op == FFS_OPEN) {
	 if(ffs_open_status(which_ffs, 1)) { /* already open */
	    continue;
	 }
	 FFS_vals[1] = (which_ffs & 0x1) ? FFS_OPEN : FFS_SAME;
	 FFS_vals[2] = (which_ffs & 0x2) ? FFS_OPEN : FFS_SAME;
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
	 if(ffs_close_status(which_ffs, 1)) { /* already closed */
	    continue;
	 }
	 FFS_vals[1] = (which_ffs & 0x1) ? FFS_CLOSE : FFS_SAME;
	 FFS_vals[2] = (which_ffs & 0x2) ? FFS_CLOSE : FFS_SAME;
      }
/*
 * Time to do the work
 */
      if(set_mcp_ffs_bits(msg.u.FFS.op, 1) != 0) {
	 continue;
      }
      
      TRACE(1, "Waiting %ds for flat field screen to move", wait, 0);
/*
 * And schedule the check
 */
      if(timerSend(msg_type, tmr_e_add, wait*60, msg_type, msgFFS) == ERROR) {
	 TRACE(0, "Failed to send message to timer task: %s (%d)",
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
   MCP_MSG msg;				/* message to pass around */
   int ret;				/* return code */
   B10_L0 tm_ctrl;   

   for(;;) {
      ret = msgQReceive(msgLamps, (char *)&msg, sizeof(msg), WAIT_FOREVER);
      assert(ret != ERROR);

      TRACE(8, "read msg on msgLamps", 0, 0);
      assert(msg.type == lamps_type);
      
      if(semTake(semSLC,60) == ERROR) {
	 printf("Unable to take semaphore: %s", strerror(errno));
	 TRACE(0, "Unable to take semaphore: %d", errno, 0);
      }

      err = slc_read_blok(1, 10, BIT_FILE, 0, &ctrl[0], sizeof(tm_ctrl)/2);
      if(err) {
	 semGive(semSLC);
	 printf("R Err=%04x\r\n",err);
      }

      swab((char *)&ctrl[0], (char *)&tm_ctrl, sizeof(tm_ctrl));
      
      switch (msg.u.lamps.type) {
       case FF_LAMP:
	 tm_ctrl.mcp_ff_lamp_on_cmd = msg.u.lamps.on_off;
	 break;
       case NE_LAMP:
	 tm_ctrl.mcp_ne_lamp_on_cmd = msg.u.lamps.on_off;
	 break;
       case HGCD_LAMP:
	 tm_ctrl.mcp_hgcd_lamp_on_cmd = msg.u.lamps.on_off;
	 break;
       default:
	 TRACE(0, "Impossible lamp type: %d", msg.type, 0);
	 break;
      }
      
      swab ((char *)&tm_ctrl, (char *)&ctrl[0], sizeof(tm_ctrl));
      err = slc_write_blok(1, 10, BIT_FILE, 0, &ctrl[0], sizeof(tm_ctrl)/2);
      semGive (semSLC);
      
      if(err) {
	 printf("W Err=%04x\r\n",err);
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

  len = strlen(ffstatus_ans);
  assert(len < size);
  
  return(len);	
}

/*****************************************************************************/
/*
 * Command the alignment clamp
 */
char *
clampon_cmd(char *cmd)			/* NOTUSED */
{
   alignment_clamp_set(1);

   return "";
}

char *
clampoff_cmd(char *cmd)			/* NOTUSED */
{
   alignment_clamp_set(0);

   return "";
}

/*****************************************************************************/
/*
 * Flatfield screen
 */
char *
ffstatus_cmd(char *cmd)			/* NOTUSED */
{
   (void)get_ffstatus(ublock->buff, UBLOCK_SIZE);
   
   return(ublock->buff);
}

char *
ffsopen_cmd(char *cmd)			/* If == 1, toggle petals */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */
   int toggle = atoi(cmd);

   msg.type = FFS_type;
   msg.u.FFS.op = toggle ? FFS_TOGGLE : FFS_OPEN;

   ret = msgQSend(msgFFS, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
ffsclose_cmd(char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = FFS_type;
   msg.u.FFS.op = FFS_CLOSE;

   ret = msgQSend(msgFFS, (char *)&msg, sizeof(msg), NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
ffsselect_cmd(char *cmd)
{
   int which = atoi(cmd);

   if(which == 0) {			/* may be hex */
      if(cmd[0] == '0' && (cmd[1] == 'x' || cmd[1] == 'X')) {
	 which = atoi(&cmd[2]);
      }
   }
   
   if(which < 0 || which > 3) {
      TRACE(0, "Invalid FFS.SELECT argument: %s", cmd, 0);
      return("Invalid FFS.SELECT argument");
   }

   which_ffs = which;

   return("");
}

/*****************************************************************************/
/*
 * Turn the flatfield/calibration lamps on/off
 */
char *
fflon_cmd(char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = lamps_type;
   msg.u.lamps.type = FF_LAMP;
   msg.u.lamps.on_off = ON;

   ret = msgQSend(msgLamps, (char *)&msg, sizeof(msg), NO_WAIT,MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
ffloff_cmd(char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = lamps_type;
   msg.u.lamps.type = FF_LAMP;
   msg.u.lamps.on_off = OFF;

   ret = msgQSend(msgLamps, (char *)&msg, sizeof(msg), NO_WAIT,MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
neon_cmd(char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = lamps_type;
   msg.u.lamps.type = NE_LAMP;
   msg.u.lamps.on_off = ON;

   ret = msgQSend(msgLamps, (char *)&msg, sizeof(msg), NO_WAIT,MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
neoff_cmd(char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = lamps_type;
   msg.u.lamps.type = NE_LAMP;
   msg.u.lamps.on_off = OFF;

   ret = msgQSend(msgLamps, (char *)&msg, sizeof(msg), NO_WAIT,MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
hgcdon_cmd(char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = lamps_type;
   msg.u.lamps.type = HGCD_LAMP;
   msg.u.lamps.on_off = ON;

   ret = msgQSend(msgLamps, (char *)&msg, sizeof(msg), NO_WAIT,MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

char *
hgcdoff_cmd(char *cmd)			/* NOTUSED */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   msg.type = lamps_type;
   msg.u.lamps.type = HGCD_LAMP;
   msg.u.lamps.on_off = OFF;

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
sp1_cmd(char *cmd)
{
   ublock->spectrograph_select = SPECTROGRAPH1;
  
   return("");
}

char *
sp2_cmd(char *cmd)
{
  ublock->spectrograph_select = SPECTROGRAPH2;

  return("");
}

/*
 * Status of slithead doors and slit latches
 */
char *
slitstatus_cmd(char *cmd)		/* NOTUSED */
{
   (void)get_slitstatus(ublock->buff, UBLOCK_SIZE);

   return(ublock->buff);
}

/*
 * Neither close nor open the slit door...allow it to be moved by hand
 */
char *
slitdoor_clear_cmd(char *cmd)		/* NOTUSED */
{
   if(mcp_specdoor_clear(ublock->spectrograph_select) < 0) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   return "";
}

/*
 * open the door
 */
char *
slitdoor_open_cmd(char *cmd)		/* NOTUSED */
{
   if(mcp_specdoor_open(ublock->spectrograph_select) < 0) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   return "";
}

/*
 * close the door
 */
char *
slitdoor_close_cmd(char *cmd)		/* NOTUSED */
{
   if(mcp_specdoor_close(ublock->spectrograph_select) < 0) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   return "";
}

/*
 * latch the slithead
 */
char *
slithead_latch_close_cmd(char *cmd)		/* NOTUSED */
{
   if(mcp_slithead_latch_close(ublock->spectrograph_select) < 0) {
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
   
   return "";
}

/*
 * unlatch the slithead
 */
char *
slithead_latch_open_cmd(char *cmd)		/* NOTUSED */
{
   if(mcp_slithead_latch_open(ublock->spectrograph_select) < 0) {
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
   if(msgAlignClamp == NULL) {
      msgAlignClamp = msgQCreate(40, sizeof(MCP_MSG), MSG_Q_FIFO);
      assert(msgAlignClamp != NULL);
      ret = taskSpawn("tAlgnClmp",90,0,5000,(FUNCPTR)tAlgnClmp,
		      0,0,0,0,0,0,0,0,0,0);
      assert(ret != ERROR);

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
   define_cmd("CLAMP.OFF",           clampoff_cmd, 	       0, 1, 0, 1, "");
   define_cmd("CLAMP.ON",            clampon_cmd, 	       0, 1, 0, 1, "");
   define_cmd("FF.OFF",              ffloff_cmd,               0, 0, 0, 1, "");
   define_cmd("FF.ON",               fflon_cmd,                0, 0, 0, 1, "");
   define_cmd("FF.STATUS",           ffstatus_cmd,             0, 0, 0, 1, "");
   define_cmd("FFL.OFF",             ffloff_cmd,               0, 0, 0, 1, "");
   define_cmd("FFL.ON",              fflon_cmd,                0, 0, 0, 1, "");
   define_cmd("FFS.CLOSE",           ffsclose_cmd,             0, 0, 0, 1, "");
   define_cmd("FFS.OPEN",            ffsopen_cmd,             -1, 0, 0, 1,
	      "Open flat field screens;\n"
	      "with an argument 1 open closed ones and close open ones");
   define_cmd("FFS.SELECT",          ffsselect_cmd,            1, 0, 0, 1,
	      "Select petal set 1 (0x1), petal set 2 (0x2) or all (0x3)");
   define_cmd("HGCD.OFF",            hgcdoff_cmd,              0, 0, 0, 1, "");
   define_cmd("HGCD.ON",             hgcdon_cmd,               0, 0, 0, 1, "");
   define_cmd("NE.OFF",              neoff_cmd,                0, 0, 0, 1, "");
   define_cmd("NE.ON",               neon_cmd,                 0, 0, 0, 1, "");
   define_cmd("SLIT.STATUS",         slitstatus_cmd,           0, 0, 0, 1, "");
   define_cmd("SLITDOOR.CLEAR",      slitdoor_clear_cmd,       0, 0, 0, 1, "");
   define_cmd("SLITDOOR.CLOSE",      slitdoor_close_cmd,       0, 0, 0, 1, "");
   define_cmd("SLITDOOR.OPEN",       slitdoor_open_cmd,        0, 0, 0, 1, "");
   define_cmd("SLITHEADLATCH.CLOSE", slithead_latch_close_cmd, 0, 0, 0, 1, "");
   define_cmd("SLITHEADLATCH.EXT",   slithead_latch_open_cmd,  0, 0, 0, 1, "");
   define_cmd("SLITHEADLATCH.OPEN",  slithead_latch_open_cmd,  0, 0, 0, 1, "");
   define_cmd("SLITHEADLATCH.RET",   slithead_latch_close_cmd, 0, 0, 0, 1, "");
   define_cmd("SP1", sp1_cmd,                                  0, 0, 0, 0, "");
   define_cmd("SP2", sp2_cmd,                                  0, 0, 0, 0, "");
}
