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

MSG_Q_ID msgLamps = NULL;		/* control lamps */

/*=========================================================================
**
**      Turn on/off the instrument change clamp.
**
** GLOBALS REFERENCED:
**	sdssdc
**	semSLC
**
**=========================================================================
*/
int clamp_cnt;

int
tm_clamp(short val) 
{
   int err;
   unsigned short ctrl[2];
   struct B10_0 tm_ctrl;   
   struct B10_1 tm_ctrl1;   
   int cnt;
             
   if(semTake(semSLC,60) == ERROR) {
      printf("Unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,0,&ctrl[0],2);
   if(err) {
      semGive (semSLC);
      printf ("R Err=%04x\r\n",err);
      return err;
   }
   swab ((char *)&ctrl[0],(char *)&tm_ctrl,2);
   swab ((char *)&ctrl[1],(char *)&tm_ctrl1,2);

   if(val == 1) {
      tm_ctrl.mcp_clamp_engage_cmd = 1;
      tm_ctrl1.mcp_clamp_disen_cmd = 0;
   } else {
      tm_ctrl.mcp_clamp_engage_cmd = 0;
      tm_ctrl1.mcp_clamp_disen_cmd = 1;
   }
   
   swab((char *)&tm_ctrl, (char *)&ctrl[0],2);
   swab((char *)&tm_ctrl1,(char *)&ctrl[1],2);
   err = slc_write_blok(1,10,BIT_FILE,0,&ctrl[0],2);
   semGive (semSLC);
   
   if(err) {
      printf ("W Err=%04x\r\n",err);
      return err;
   }
   
   if(val == 0) {
      return 0;
   }

   cnt=60*15;				/* wait 15s */
   while(sdssdc.status.i9.il0.clamp_en_stat == 0 && cnt > 0) {
      taskDelay(1);
      cnt--;
   }
   clamp_cnt = cnt;

   if (sdssdc.status.i9.il0.clamp_en_stat == 1) { /* success */
      return -1;
   }
/*
 * Failure; turn off clamp and disengage
 */
   printf ("\r\n Clamp did NOT engage...turning off and disengaging ");
   if(semTake(semSLC,60) == ERROR) {
      printf("Unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,0,&ctrl[0],2);
   if(err) {
      semGive (semSLC);
      printf ("R2 Err=%04x\r\n",err);
      return err;
   }
   swab ((char *)&ctrl[0],(char *)&tm_ctrl,2);
   swab ((char *)&ctrl[1],(char *)&tm_ctrl1,2);

   tm_ctrl.mcp_clamp_engage_cmd = 0;
   tm_ctrl1.mcp_clamp_disen_cmd = 1;
   
   swab((char *)&tm_ctrl,(char *)&ctrl[0],2);
   swab((char *)&tm_ctrl1,(char *)&ctrl[1],2);
   
   err = slc_write_blok(1,10,BIT_FILE,0,&ctrl[0],2);
   semGive(semSLC);
   if(err) {
      printf ("W Err=%04x\r\n",err);
      return err;
   }

   return 0;
}
void tm_clamp_on()
{
    tm_clamp (1);
}
void tm_clamp_off()
{
    tm_clamp (0);
}
void tm_sp_clamp_on()
{
  if (taskIdFigure("tmClamp")==ERROR)
    taskSpawn("tmClamp",90,0,2000,(FUNCPTR)tm_clamp,1,0,0,0,0,0,0,0,0,0);
}
void tm_sp_clamp_off()
{
  if (taskIdFigure("tmClamp")==ERROR)
    taskSpawn("tmClamp",90,0,2000,(FUNCPTR)tm_clamp,0,0,0,0,0,0,0,0,0,0);
}
int tm_clamp_status()
{
   int err;
   unsigned short ctrl[2],sctrl[2];
   
   if(semTake(semSLC,60) == ERROR) {
      printf("Unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,0,&ctrl[0],2);
   semGive (semSLC);
   if(err) {
      printf ("R Err=%04x\r\n",err);
      return err;
   }
   
   swab ((char *)&ctrl[0],(char *)&sctrl[0],2);
   swab ((char *)&ctrl[1],(char *)&sctrl[1],2);
   
   printf (" read ctrl = 0x%4x 0x%4x\r\n",sctrl[0],sctrl[1]);
   
   return 0;
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_slit
**	    tm_slit_clear
**	    tm_slit_open
**	    tm_slit_close
**	    tm_sp_slit_open
**	    tm_sp_slit_close
**
** DESCRIPTION:
**      Open/close/clear the slit door.  Clear neither closes nor opens the
**	door, the actuator is inactive.  There are two spectographs so the
**	door must be specified.
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
tm_slit(short val) 
{
   int err;
   unsigned short ctrl[1];
   struct B10_1 tm_ctrl1;   
             
   if(semTake(semSLC,60) == ERROR) {
      printf("tm_slit: unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   if(err) {
      semGive (semSLC);
      printf ("R Err=%04x\r\n",err);
      return err;
   }
   swab ((char *)&ctrl[0],(char *)&tm_ctrl1,2);

   switch (val) {
    case 5:
      tm_ctrl1.mcp_slit_dr2_opn_cmd = 0;
      tm_ctrl1.mcp_slit_dr2_cls_cmd = 0;
      break;
    case 4:
      tm_ctrl1.mcp_slit_dr2_opn_cmd = 1;
      tm_ctrl1.mcp_slit_dr2_cls_cmd = 0;
      break;
    case 3:
      tm_ctrl1.mcp_slit_dr2_opn_cmd = 0;
      tm_ctrl1.mcp_slit_dr2_cls_cmd = 1;
      break;
    case 2:
      tm_ctrl1.mcp_slit_dr1_opn_cmd = 0;
      tm_ctrl1.mcp_slit_dr1_cls_cmd = 0;
      break;
    case 1:
      tm_ctrl1.mcp_slit_dr1_opn_cmd = 1;
      tm_ctrl1.mcp_slit_dr1_cls_cmd = 0;
      break;
    case 0:
      tm_ctrl1.mcp_slit_dr1_opn_cmd = 0;
      tm_ctrl1.mcp_slit_dr1_cls_cmd = 1;
      break;
   }

   swab((char *)&tm_ctrl1,(char *)&ctrl[0],2);

   err = slc_write_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   semGive (semSLC);
   if(err) {
      printf("W Err=%04x\r\n",err);
      return err;
   }

   return 0;
}
void tm_slit_clear(int door)
{
    tm_slit (2+(door*3));
}
void tm_slit_open(int door)
{
    tm_slit (1+(door*3));
}
void tm_slit_close(int door)
{
    tm_slit (0+(door*3));
}

void
tm_sp_slit_clear(int door)
{
   if(taskIdFigure("tmSlit") == ERROR) {
      taskSpawn("tmSlit",90,0,2000,
		(FUNCPTR)tm_slit_clear, door,
		0,0,0,0,0,0,0,0,0);
   }
}

void
tm_sp_slit_open(int door)
{
   if(taskIdFigure("tmSlit") == ERROR) {
      taskSpawn("tmSlit",90,0,2000,
		(FUNCPTR)tm_slit_open, door,
		0,0,0,0,0,0,0,0,0);
   }
}

void
tm_sp_slit_close(int door)
{
   if(taskIdFigure("tmSlit") == ERROR) {
      taskSpawn("tmSlit",90,0,2000,
		(FUNCPTR)tm_slit_close, door,
		0,0,0,0,0,0,0,0,0);
   }
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_cart
**	    tm_cart_latch
**	    tm_cart_unlatch
**	    tm_sp_cart_latch
**	    tm_sp_cart_unlatch
*
** DESCRIPTION:
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
tm_cart(short val) 
{
   int err;
   unsigned short ctrl[1];
   struct B10_1 tm_ctrl1;   
             
   if(semTake (semSLC,60) == ERROR) {
      printf("tm_cart: unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   if(err) {
      semGive (semSLC);
      printf ("R Err=%04x\r\n",err);
      return err;
   }
   swab ((char *)&ctrl[0],(char *)&tm_ctrl1,2);

   switch (val) {
    case 3:
      tm_ctrl1.mcp_slit_latch2_cmd = 1;
      break;
    case 2:
      tm_ctrl1.mcp_slit_latch2_cmd = 0;
      break;
    case 1:
      tm_ctrl1.mcp_slit_latch1_cmd = 1;
      break;
    case 0:
      tm_ctrl1.mcp_slit_latch1_cmd = 0;
      break;
   }
   
   swab ((char *)&tm_ctrl1,(char *)&ctrl[0],2);
   err = slc_write_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   semGive (semSLC);
   if(err) {
      printf ("W Err=%04x\r\n",err);
      return err;
   }

   return 0;
}
void tm_cart_latch(int door)
{
    tm_cart (1+(door*2));
}
void tm_cart_unlatch(int door)
{
    tm_cart (0+(door*2));
}
void tm_sp_cart_latch(int door)
{
  if (taskIdFigure("tmCart")==ERROR)
    taskSpawn("tmCart",90,0,1000,(FUNCPTR)tm_cart_latch,door,0,0,0,0,0,0,0,0,0);
}
void tm_sp_cart_unlatch(int door)
{
  if (taskIdFigure("tmCart")==ERROR)
    taskSpawn("tmCart",90,0,1000,(FUNCPTR)tm_cart_unlatch,door,0,0,0,0,0,0,0,0,0);
}

int
tm_slit_status()
{
   int err;
   unsigned short ctrl[1];
   struct B10_1 tm_ctrl1;   

   if(semTake(semSLC,60) == ERROR) {
      printf("tm_slit_status: unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   semGive (semSLC);
   if (err) {
      printf ("R Err=%04x\r\n",err);
      return err;
   }
   swab ((char *)&ctrl[0],(char *)&tm_ctrl1,2);
   
  printf (" read ctrl = 0x%04x\r\n",(unsigned int)ctrl);
  printf ("\r\n mcp_slit_dr1_opn_cmd=%d, mcp_slit_dr1_cls_cmd=%d",
     tm_ctrl1.mcp_slit_dr1_opn_cmd,tm_ctrl1.mcp_slit_dr1_cls_cmd);
  printf ("\r\n mcp_slit_dr2_opn_cmd=%d, mcp_slit_dr2_cls_cmd=%d",
     tm_ctrl1.mcp_slit_dr2_opn_cmd,tm_ctrl1.mcp_slit_dr2_cls_cmd);
  printf ("\r\n mcp_slit_latch1_cmd=%d, mcp_slit_latch2_cmd=%d",
     tm_ctrl1.mcp_slit_latch1_cmd,tm_ctrl1.mcp_slit_latch2_cmd);

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
mcp_slit_clear(int spec)
{
   if(spec != SPECTOGRAPH1 && spec != SPECTOGRAPH2) {
      return(-1);
   }
   
   tm_sp_slit_clear(spec);

   return(0);
}

int
mcp_slit_open(int spec)
{
   if(spec != SPECTOGRAPH1 && spec != SPECTOGRAPH2) {
      return(-1);
   }
   
   tm_sp_slit_open(spec);

   return(0);
}

int
mcp_slit_close(int spec)
{
   if(spec != SPECTOGRAPH1 && spec != SPECTOGRAPH2) {
      return(-1);
   }
   
   tm_sp_slit_close(spec);

   return(0);
}

int
mcp_slithead_latch_open(int spec)
{
   if(spec != SPECTOGRAPH1 && spec != SPECTOGRAPH2) {
      return(-1);
   }
   
   tm_cart_unlatch(spec);

   return(0);
}

int
mcp_slithead_latch_close(int spec)
{
   if(spec != SPECTOGRAPH1 && spec != SPECTOGRAPH2) {
      return(-1);
   }
   
   tm_cart_latch(spec);

   return(0);
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: tm_ffs
**	    tm_ffs_open
**	    tm_ffs_close
**          tm_ffs_enable
**	    tm_sp_ffs_move
**	    tm_ffs_open_status
**	    tm_ffs_close_status
**
** DESCRIPTION:
**      Open/close the flat field screen
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
/*
 * Set the FFS control bits. If val is < 0, it isn't set
 */
static int
set_mcp_ffs_bits(int val,		/* value of mcp_ff_scrn_opn_cmd */
		 int enab)		/* value of mcp_ff_screen_enable */
{
   unsigned short ctrl[1];
   struct B10_1 tm_ctrl1;   
   int err;
             
   if(semTake(semSLC,60) == ERROR) {
      printf("Unable to take semaphore: %s", strerror(errno));
      TRACE(0, "Unable to take semaphore: %d", errno, 0);
      return(-1);
   }

   err = slc_read_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   if(err) {
      semGive (semSLC);
      printf("R Err=%04x\r\n",err);
      return err;
   }
   swab((char *)&ctrl[0], (char *)&tm_ctrl1, 2);

   if(val >= 0) {
      tm_ctrl1.mcp_ff_scrn_opn_cmd = val;
   }
   tm_ctrl1.mcp_ff_screen_enable = enab;

   swab ((char *)&tm_ctrl1,(char *)&ctrl[0],2);
   err = slc_write_blok(1,10,BIT_FILE,1,&ctrl[0],1);
   semGive (semSLC);

   if(err) {
      printf ("W Err=%04x\r\n",err);
      return err;
   }

   return 0;
}

/*
 * Open or close the FF screen
 */
int
tm_ffs(short val)			/* FFS_CLOSE or FFS_OPEN */
{
   int cnt;
   int err;
   int failed = 0;			/* did operation fail? */
   int wait_time = 30;			/* FF screen timeout (seconds) */

   if(set_mcp_ffs_bits(val, 1) != 0) {
      return(err);
   }

   cnt=60*wait_time;
   printf("Waiting up to %ds for flat field to move\n", wait_time);
   if(val == 1) {
      while(!tm_ffs_open_status() && cnt > 0) {
	 taskDelay(1);
	 cnt--;
      }
      if(!tm_ffs_open_status()) {	/* did not work */
	 failed = 1;
	 TRACE(0, "FFS did NOT all open", 0, 0);
	 printf("\r\n FFS did NOT all open ");
      }
   } else {
      while(!tm_ffs_close_status() && cnt > 0) {
	 taskDelay(1);
	 cnt--;
      }
      if(!tm_ffs_close_status()) {	/* did not work */
	 failed = 1;
	 TRACE(0, "FFS did NOT all close", 0, 0);
	 printf("\r\n FFS did NOT all close ");
      }
   }
   
   if(failed) {
      return(set_mcp_ffs_bits(!val, 1)); /* don't leave command pending */
   } else {
      return 0;
   }
}

/*
 * Enable the flat field screen
 */
int
tm_ffs_enable(int val)
{
   return(set_mcp_ffs_bits(-1, 1));
}

void
tm_ffs_open(void)
{
   tm_ffs(FFS_OPEN);
}

void
tm_ffs_close(void)
{
   tm_ffs(FFS_CLOSE);
}

void
tm_sp_ffs_move(int open_close)		/* FFS_CLOSE or FFS_OPEN */
{
   if(taskIdFigure("tmFFS") == ERROR) {
      taskSpawn("tmFFS", 90, 0, 2000, (FUNCPTR)tm_ffs, open_close,
		0,0,0,0,0,0,0,0,0);
   } else {
      TRACE(0, "Task tmFFS is already active; not moving FF screen (%d)",
	    open_close, 0);
   }
}

int
tm_ffs_open_status(void)
{
  if ((sdssdc.status.i1.il13.leaf_1_open_stat)&&
	(sdssdc.status.i1.il13.leaf_2_open_stat)&&
	(sdssdc.status.i1.il13.leaf_3_open_stat)&&
	(sdssdc.status.i1.il13.leaf_4_open_stat)&&
	(sdssdc.status.i1.il13.leaf_5_open_stat)&&
	(sdssdc.status.i1.il13.leaf_6_open_stat)&&
	(sdssdc.status.i1.il13.leaf_7_open_stat)&&
	(sdssdc.status.i1.il13.leaf_8_open_stat))
    return TRUE;
  else 
    return FALSE;
}

int
tm_ffs_close_status(void)
{
  if ((sdssdc.status.i1.il13.leaf_1_closed_stat)&&
 	(sdssdc.status.i1.il13.leaf_2_closed_stat)&&
	(sdssdc.status.i1.il13.leaf_3_closed_stat)&&
	(sdssdc.status.i1.il13.leaf_4_closed_stat)&&
	(sdssdc.status.i1.il13.leaf_5_closed_stat)&&
	(sdssdc.status.i1.il13.leaf_6_closed_stat)&&
	(sdssdc.status.i1.il13.leaf_7_closed_stat)&&
	(sdssdc.status.i1.il13.leaf_8_closed_stat)) 
    return TRUE;
  else 
    return FALSE;
}

/*****************************************************************************/
/*
 * A task to control the lamps
 */
void
tLamps(void)
{
   int err;
   unsigned short ctrl[1];
   MCP_MSG msg;				/* message to pass around */
   int ret;				/* return code */
   struct B10_1 tm_ctrl1;   

   for(;;) {
      ret = msgQReceive(msgLamps, (char *)&msg, sizeof(msg), WAIT_FOREVER);
      assert(ret != ERROR);

      TRACE(8, "read msg on msgLamps", 0, 0);
      assert(msg.type == lamp_type);
      
      if(semTake(semSLC,60) == ERROR) {
	 printf("Unable to take semaphore: %s", strerror(errno));
	 TRACE(0, "Unable to take semaphore: %d", errno, 0);
      }

      err = slc_read_blok(1,10,BIT_FILE,1,&ctrl[0],1);
      if(err) {
	 semGive(semSLC);
	 printf("R Err=%04x\r\n",err);
      }

      swab((char *)&ctrl[0],(char *)&tm_ctrl1,2);
      
      switch (msg.u.lamps.type) {
       case FF_LAMP:
	 tm_ctrl1.mcp_ff_lamp_on_cmd = msg.u.lamps.on_off;
	 break;
       case NE_LAMP:
	 tm_ctrl1.mcp_ne_lamp_on_cmd = msg.u.lamps.on_off;
	 break;
       case HGCD_LAMP:
	 tm_ctrl1.mcp_hgcd_lamp_on_cmd = msg.u.lamps.on_off;
	 break;
       default:
	 TRACE(0, "Impossible lamp type: %d", msg.type, 0);
	 break;
      }
      
      swab ((char *)&tm_ctrl1,(char *)&ctrl[0],2);
      err = slc_write_blok(1,10,BIT_FILE,1,&ctrl[0],1);
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
	"FFS  %d%d %d%d %d%d %d%d  %d%d %d%d %d%d %d%d  %d\n",
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
	   sdssdc.status.o1.ol14.ff_screen_open_pmt);
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
   tm_sp_clamp_on();
   return "";
}

char *
clampoff_cmd(char *cmd)			/* NOTUSED */
{
   tm_sp_clamp_off();
   return "";
}

/*****************************************************************************/
/*
 * Flatfield screen and the various lamps
 */
char *
ffstatus_cmd(char *cmd)			/* NOTUSED */
{
   static char ffstatus_ans[250];
   
   (void)get_ffstatus(ffstatus_ans, 250);
   
   return(ffstatus_ans);
}

char *
ffsopen_cmd(char *cmd)			/* NOTUSED */
{
   tm_sp_ffs_move(FFS_OPEN);
   return "";
}

char *
ffsclose_cmd(char *cmd)			/* NOTUSED */
{
   tm_sp_ffs_move(FFS_CLOSE);
   return "";
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

   msg.type = lamp_type;
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

   msg.type = lamp_type;
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

   msg.type = lamp_type;
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

   msg.type = lamp_type;
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

   msg.type = lamp_type;
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

   msg.type = lamp_type;
   msg.u.lamps.type = HGCD_LAMP;
   msg.u.lamps.on_off = OFF;

   ret = msgQSend(msgLamps, (char *)&msg, sizeof(msg), NO_WAIT,MSG_PRI_NORMAL);
   assert(ret == OK);

   return "";
}

/*****************************************************************************/
/*
 * Which spectrograph are we commanding?
 */
static int spectograph_select = -1;	/* -1 == ERROR  */

/*****************************************************************************/
/*
 * Commands for the selected spectrograph.
 *
 * These could all be static, except that I want them to show up in cmdList
 */
char *
sp1_cmd(char *cmd)
{
  spectograph_select = SPECTOGRAPH1;
  
  return("");
}

char *
sp2_cmd(char *cmd)
{
  spectograph_select = SPECTOGRAPH2;

  return("");
}

/*
 * Status of slithead doors and slit latches
 */
char *
slitstatus_cmd(char *cmd)		/* NOTUSED */
{
   static char slitstatus_ans[50];

   (void)get_slitstatus(slitstatus_ans, 50);

   return(slitstatus_ans);
}

/*
 * Neither close nor open the slit door...allow it to be moved by hand
 */
char *
slitdoor_clear_cmd(char *cmd)		/* NOTUSED */
{
   if(mcp_slit_clear(spectograph_select) < 0) {
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
   if(mcp_slit_open(spectograph_select) < 0) {
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
   if(mcp_slit_close(spectograph_select) < 0) {
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
   if(mcp_slithead_latch_close(spectograph_select) < 0) {
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
   if(mcp_slithead_latch_open(spectograph_select) < 0) {
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
   if(msgLamps == NULL) {
      msgLamps = msgQCreate(40, sizeof(MCP_MSG), MSG_Q_FIFO);
      assert(msgLamps != NULL);

      ret = taskSpawn("tLamps",90,0,2000,(FUNCPTR)tLamps,0,0,0,0,0,0,0,0,0,0);
      assert(ret != ERROR);
   }
/*
 * define spectro commands to the command interpreter
 */
   define_cmd("FF.OFF",              ffloff_cmd,               0, 0, 1);
   define_cmd("FF.ON",               fflon_cmd,                0, 0, 1);
   define_cmd("FF.STATUS",           ffstatus_cmd,             0, 0, 1);
   define_cmd("FFL.OFF",             ffloff_cmd,               0, 0, 1);
   define_cmd("FFL.ON",              fflon_cmd,                0, 0, 1);
   define_cmd("FFS.CLOSE",           ffsclose_cmd,             0, 0, 1);
   define_cmd("FFS.OPEN",            ffsopen_cmd,              0, 0, 1);
   define_cmd("HGCD.OFF",            hgcdoff_cmd,              0, 0, 1);
   define_cmd("HGCD.ON",             hgcdon_cmd,               0, 0, 1);
   define_cmd("NE.OFF",              neoff_cmd,                0, 0, 1);
   define_cmd("NE.ON",               neon_cmd,                 0, 0, 1);
   define_cmd("SLIT.STATUS",         slitstatus_cmd,           0, 0, 1);
   define_cmd("SLITDOOR.CLEAR",      slitdoor_clear_cmd,       0, 1, 1);
   define_cmd("SLITDOOR.CLOSE",      slitdoor_close_cmd,       0, 1, 1);
   define_cmd("SLITDOOR.OPEN",       slitdoor_open_cmd,        0, 1, 1);
   define_cmd("SLITHEADLATCH.CLOSE", slithead_latch_close_cmd, 0, 1, 1);
   define_cmd("SLITHEADLATCH.EXT",   slithead_latch_open_cmd,  0, 1, 1);
   define_cmd("SLITHEADLATCH.OPEN",  slithead_latch_open_cmd,  0, 1, 1);
   define_cmd("SLITHEADLATCH.RET",   slithead_latch_close_cmd, 0, 1, 1);
   define_cmd("SP1", sp1_cmd,                                  0, 0, 0);
   define_cmd("SP2", sp2_cmd,                                  0, 0, 0);
}
