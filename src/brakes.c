#include <vxWorks.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <semLib.h>
#include <taskLib.h>
#include "dscTrace.h"
#include "data_collection.h"
#include "frame.h"
#include "axis.h"
#include "pcdsp.h"
#include "abdh.h"
#include "mcpMsgQ.h"
#include "tm.h"
#include "cmd.h"
#include "as2.h"
/*
 * The brakes message queue
 */
MSG_Q_ID msgBrakes;

/*****************************************************************************/
/*
 * A task to set the brakes
 *
 * Why spawn a task? Because we may have to wait for a semaphore
 */
void
tBrakes(void)
{
   int uid = 0, cid = 0;
   int axis;				/* the axis to (un)set the brake for */
   unsigned short ctrl[2];		/* short to read/write; SLC byte ordr*/
   MCP_MSG msg;				/* message to pass around */
   int ret;				/* return code */
   int set_brake;			/* true if the brake should go on */
   B10_L0 tm_ctrl;			/* the bits we want */

   for(;;) {
      ret = msgQReceive(msgBrakes, (char *)&msg, sizeof(msg),
			WAIT_FOREVER);
      assert(ret != ERROR);

      uid = msg.uid;
      cid = msg.cid;

      switch (msg.type) {
       case brakesSet_type:
	 set_brake = 1;
	 axis = msg.u.brakes.axis;
	 break;
       case brakesUnset_type:
	 set_brake = 0;
	 axis = msg.u.brakes.axis;
	 break;
       default:
	 NTRACE_1(0, uid, cid, "Impossible message type on msgBrakes: %d", msg.type);
	 continue;
      }
/*
 * Is this an axis with a brake?
 */
      if(axis == AZIMUTH || axis == ALTITUDE) {
	 ;				/* OK */
      } else if(axis == INSTRUMENT) {
	 continue;
      } else {
	 NTRACE_1(0, uid, cid, "illegal axis %d", axis);
	 continue;
      }
/*
 * set the bits that control the brakes
 */
      /* OTRACE(10, "Taking semSLC semaphore", 0, 0); */
      if(semTake(semSLC,60) == ERROR) {
	 NTRACE_2(0, uid, cid, "failed to take semaphore to %s %s brake",
	       (set_brake ? "set" : "unset"), axis_name(axis));
	 NTRACE_2(1, uid, cid, "    %s %d", strerror(errno), errno);
	 continue;
      }

      /* OTRACE(10, "Reading blok", 0, 0); */
      ret = slc_read_blok(1,10,BIT_FILE,0,&ctrl[0],sizeof(tm_ctrl)/2);
      if(ret) {
	 NTRACE_2(0, uid, cid, "%s: error reading slc: 0x%04x", axis_name(axis), ret);
	 semGive(semSLC);
	 continue;
      }
      swab((char *)&ctrl[0], (char *)&tm_ctrl, sizeof(tm_ctrl));

      if(axis == ALTITUDE) {
	 tm_ctrl.mcp_alt_brk_en_cmd =  set_brake ? 1 : 0;
	 tm_ctrl.mcp_alt_brk_dis_cmd = set_brake ? 0 : 1;
      } else if(axis == AZIMUTH) {
	 tm_ctrl.mcp_az_brk_en_cmd =  set_brake ? 1 : 0;
	 tm_ctrl.mcp_az_brk_dis_cmd = set_brake ? 0 : 1;
      } else {
	 NTRACE_1(0, uid, cid, "Impossible instrument %d", axis);
	 abort();
      }

      swab((char *)&tm_ctrl, (char *)&ctrl[0], sizeof(tm_ctrl));
      ret = slc_write_blok(1, 10, BIT_FILE, 0, &ctrl[0], sizeof(tm_ctrl)/2);
      semGive(semSLC);
      if(ret) {
	 NTRACE_2(0, uid, cid, "%s: error writing slc: 0x%04x", axis_name(axis), ret);
	 continue;
      }
/*
 * done with setting the bits; now deal with the MEI
 */
      /* OTRACE(10, "Taking semMEI semaphore", 0, 0); */
      if(semTake(semMEI, 60) == ERROR) {
	 NTRACE_2(0, uid, cid, "failed to take semMEI: %s (%d)", strerror(errno), errno);
	 continue;
      }

      if(set_brake) {
	 NTRACE_1(3, uid, cid, "Taking axis %s out of closed loop", axis_name(axis));
	 sem_controller_idle(2*axis);
	 reset_integrator(2*axis);
      } else {
	 NTRACE_1(3, uid, cid, "Putting axis %s into closed loop", axis_name(axis));
	 sem_controller_run(2*axis);

#if SWITCH_PID_COEFFS
	 if(axis == INSTRUMENT) {
	    while(coeffs_state_cts(2*axis, 0) == TRUE) {
	       ;
	    }
	 }
#endif

	 NTRACE_1(3, uid, cid, "Stopping axis %s", axis_name(axis));
 	 v_move(2*axis, (double)0, (double)5000);
      }

      semGive(semMEI);
   }
}

/*****************************************************************************/
/*
 * Put on a brake
 */
int
mcp_set_brake(int uid, unsigned long cid,
	      int axis)
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      NTRACE_1(0, uid, cid, "mcp_set_brake: illegal axis %d", axis);

      return(-1);
   }

   NTRACE_1(3, uid, cid, "Setting brake for axis %s", axis_name(axis));

   msg.type = brakesSet_type;;
   msg.u.brakes.axis = axis;

   ret = msgQSend(msgBrakes, (char *)&msg, sizeof(msg),
		  NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);

   return(0);
}

/*
 * Clear a brake
 */
int
mcp_unset_brake(int uid, unsigned long cid,
		int axis)		/* axis to set */
{
   MCP_MSG msg;				/* message to send */
   int ret;				/* return code */

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      NTRACE_1(0, uid, cid, "mcp_unset_brake: illegal axis %d", axis);

      return(-1);
   }

   NTRACE_1(3, uid, cid, "Clearing brake for axis %s", axis_name(axis));

   msg.type = brakesUnset_type;;
   msg.u.brakes.axis = axis;

   ret = msgQSend(msgBrakes, (char *)&msg, sizeof(msg),
		  NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);

   return(0);
}

/*****************************************************************************/
/*
 * The actual commands to turn the brakes on/off
 */
char *brakeon_cmd(int uid, unsigned long cid, char *cmd)
{
   if (mcp_set_brake(uid, cid, ublock->axis_select) < 0) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "brake_on");
   } else {
      sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "brake_on");
   }

  return "";
}

char *brakeoff_cmd(int uid, unsigned long cid, char *cmd)
{
   if (mcp_unset_brake(uid, cid, ublock->axis_select) < 0) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "brake_off");
   } else {
      sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "brake_off");
   }

   return "";
}

/*****************************************************************************/
/*
 * Setup the brakes commands
 */
int
tBrakesInit(void)
{
/*
 * Create the message queue
 */
   if(msgBrakes == NULL) {
      msgBrakes = msgQCreate(40, sizeof(MCP_MSG), MSG_Q_FIFO);
      assert(msgBrakes != NULL);
   }
/*
 * Spawn the task that controls the brakes
 */
   taskSpawn("tBrakes", 90, 0, 4000,
	     (FUNCPTR)tBrakes,
	     0,0,0,0,0,0,0,0,0,0);
/*
 * Declare commands
 */
   define_cmd("BRAKE_OFF",     brakeoff_cmd, 	  0, 1, 0, 1, "");
   define_cmd("BRAKE_ON",      brakeon_cmd, 	  0, 1, 0, 1, "");

   return 0;
}
