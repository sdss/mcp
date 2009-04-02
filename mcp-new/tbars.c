#include <vxWorks.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <semLib.h>
#include <taskLib.h>
#include "dscTrace.h"
#include "axis.h"
#include "abdh.h"
#include "data_collection.h"
#include "mcpMsgQ.h"
#include "cmd.h"

/*
 * The T-bars message queue
 */
MSG_Q_ID msgTbars;

/*****************************************************************************/
/*
 * A task to set the T-bars
 *
 * Why spawn a task? Because we have to wait and see if they moved, and
 * if they didn't reset them to their initial state
 */
int
tBars(void)
{
   int err;
   struct B10 b10;			/* AB struct to set */
   unsigned short ctrl[sizeof(b10)/2];
   int latch_status;			/* latched status from ABs */
   int latch_tbars;			/* should we latch the tbars? */
   MCP_MSG msg;				/* message to pass around */
   int ret;				/* return code */   
   int unlatch_status;			/* unlatched status from ABs */
   int wait = 60;			/* how many seconds to wait */

   for(;;) {
      ret = msgQReceive(msgTbars, (char *)&msg, sizeof(msg),
			WAIT_FOREVER);
      assert(ret != ERROR);

      TRACE(8, "read msg on msgTbars", 0, 0);

      switch (msg.type) {
       case TbarsLatch_type:
       case TbarsUnlatch_type:
	 (void)timerSend(TbarsLatchCheck_type, tmr_e_abort_ns,
			 0, TbarsLatchCheck_type, 0); /* abort any pending
							 confimations */
	 (void)timerSend(TbarsUnlatchCheck_type, tmr_e_abort_ns,
			 0, TbarsUnlatchCheck_type, 0);	/* abort any pending
							   confimations */
	 
	 latch_tbars = (msg.type == TbarsLatch_type) ? 1 : 0;
	 break;
       case TbarsLatchCheck_type:
       case TbarsUnlatchCheck_type:
	 unlatch_status = sdssdc.status.i9.il0.t_bar_tel_stat;
	 latch_status = sdssdc.status.i8.il0.t_bar_xport_stat;

	 if(msg.type == TbarsLatchCheck_type) {
	    latch_tbars = 1;
	 } else {
	    latch_tbars = 0;
	 }
	 
	 if(latch_status && unlatch_status) {
	    TRACE(0, "Imager T-bars are both latched and unlatched", 0, 0);
	 } else if(!latch_status && !unlatch_status) {
	    TRACE(0, "Imager T-bars are neither latched nor unlatched", 0, 0);
	 } else {
	    if(latch_tbars == latch_status) {
	       TRACE(1, "tbar latches moved", 0, 0);
	    } else {
	       TRACE(0, "Imager T-bars failed to go to %s state",
		     (latch_tbars ? "latched" : "unlatched"), 0);
	    }
	 }
	 
	 latch_tbars = -1;		/* set both to 0 */
	 break;
       default:
	 TRACE(0, "Impossible message type on msgTbars: %d", msg.type, 0);
	 continue;
      }

      if(semTake(semSLC,60) == ERROR) {
	 TRACE(0, "mcp_set_tbars: failed to get semSLC: %s (%d)",
	       strerror(errno), errno);
	 continue;
      }
      
      err = slc_read_blok(1, 10, BIT_FILE, 0, ctrl, sizeof(b10)/2);
      if(err) {
	 semGive(semSLC);
	 TRACE(0, "tBars: error reading slc: 0x%04x", err, 0);
	 continue;
      }
      swab((char *)ctrl, (char *)&b10, sizeof(b10));
      
      if(latch_tbars < 0) {
	 b10.w1.mcp_t_bar_xport = b10.w1.mcp_t_bar_tel = 0;
      } else {
	 b10.w1.mcp_t_bar_xport = latch_tbars;
	 b10.w1.mcp_t_bar_tel = !latch_tbars;
      }
      
      swab ((char *)&b10, (char *)ctrl, sizeof(b10));
      err = slc_write_blok(1, 10, BIT_FILE, 0, ctrl, sizeof(b10)/2);
      semGive (semSLC);
      
      if(err) {
	 TRACE(0, "tBars: error writing slc: 0x%04x", err, 0);
	 continue;
      }
/*
 * Prepare to confirm that the tbars actually (un)latched
 */
      if(latch_tbars < 0) {		/* this _is_ the check */
	 continue;
      }
      
      TRACE(1, "Waiting %ds for tbar latches to move", wait, 0);

      msg.type = latch_tbars ? TbarsLatchCheck_type : TbarsUnlatchCheck_type;
      if(timerSend(msg.type, tmr_e_add,
		   wait*60, msg.type, msgTbars) == ERROR) {
	 TRACE(0, "Failed to send message to timer task: %s (%d)",
	       strerror(errno), errno);
      }
   }
}

char *
tbar_latch_cmd(int uid, unsigned long cid, char *cmd)
{
   MCP_MSG msg;				/* message to send */
   int on_off;				/* true to latch T-bars */
   int ret;				/* return code */

   if(sscanf(cmd, "%d", &on_off) != 1) {
      return("ERR: malformed command argument");
   }

   if(on_off) {
      msg.type = TbarsLatch_type;
   } else {
      msg.type = TbarsUnlatch_type;
   }

   ret = msgQSend(msgTbars, (char *)&msg, sizeof(msg),
		  NO_WAIT, MSG_PRI_NORMAL);
   assert(ret == OK);
   
   return("");
}

/*****************************************************************************/
/*
 * Initialise the imager T-bars task
 */
int
tBarsInit(void)
{
/*
 * Create the message queue
 */
   if(msgTbars == NULL) {
      msgTbars = msgQCreate(40, sizeof(MCP_MSG), MSG_Q_FIFO);
      assert(msgTbars != NULL);
   }
/*
 * Spawn the task that moves the T-bars
 */
   taskSpawn("tBars", 90, 0, 2000,
	     (FUNCPTR)tBars,
	     0,0,0,0,0,0,0,0,0,0);
/*
 * Declare command
 */
   define_cmd("TBAR.LATCH",    tbar_latch_cmd,    1, 1, 0, 1,
	      "Move the tbar latches; latch if arg is true, else unlatch");
   
   return 0;
}
