#if !defined(MCP_MSG_Q_H)
#define MCP_MSG_Q_H 1

#include <msgQLib.h>
#include "timerTask.h"

typedef struct {
   enum {
      alignClamp_type = 1,		/* move alignment clamp */
      alignClampCheck_type,		/* check if alignment clamp moved */

      FFS_type,				/* move Flat Field Screen */
      FFSCheckOpen_type,		/* check that FFS opened */
      FFSCheckClosed_type,		/* check that FFS closed */

      lamps_type,			/* turn lamps on/off */

      latchCrossed_type,		/* we crossed a fiducial */
      ms_on_az_type,			/* process an MS.ON command for AZ */
      ms_off_az_type,			/* process an MS.OFF */
      ms_on_alt_type,			/* process an MS.ON command for ALT */
      ms_off_alt_type,			/* process an MS.OFF */
      ms_on_inst_type,			/* process an MS.ON command for INST */
      ms_off_inst_type,			/* process an MS.OFF */

      latchReenable_type,		/* reenable fiducial latches */

      moveCW_type,			/* counterweights motion */
      moveCWAbort_type,			/* abort counterweights */

      TbarsLatch_type,			/* latch imager t-bars */
      TbarsUnlatch_type,		/* unlatch imager t-bars */
      TbarsLatchCheck_type,		/* check that imager t-bars latched */
      TbarsUnlatchCheck_type,		/* check that imager t-bars unlatched*/

      brakesSet_type,			/* set the brakes */
      brakesUnset_type,			/* clear the brakes */

      specDoor_type			/* control spectrograph doors */
   } type;
   
   union {
      struct s_tmr_msg tmr;		/* a message type from tTimerTask */

      struct {
	 enum {ENGAGE, DISENGAGE} op;
      } alignClamp;

      struct {
	 enum {FFS_OPEN = 1, FFS_CLOSE = 0} op;
      } FFS;

      struct {
	 enum { FF_LAMP, HGCD_LAMP, NE_LAMP } type; /* type off lamp */
	 enum { ON = 1, OFF = 0 } on_off; /* should I turn it on or off? */
      } lamps;

      struct {
	 unsigned int time;		/* when we saw the fiducial */
	 unsigned char dio316int_bit;	/* bits set by interrupt */
      } latchCrossed;

      struct {
	 int timeout;
	 unsigned char dio316int_bit;	/* bits set by interrupt */
      } latchReenable;

      struct {
	 int inst;			/* instrument to balance for */
	 int cw;			/* CW to move, or ALL_CW */
	 int cwpos;			/* desired position, or 0 to balance */
      } moveCW;

      struct {
	 int abort;			/* always true */
      } moveCWAbort;

      struct {
	 int dummy;			/* NOTUSED */
      } Tbars;

      struct {
	 int spec;			/* which spectrograph? */
	 enum { OPEN, CLOSE, CLEAR } op; /* desired operation */
      } specDoor;

      struct {
	 int axis;			/* which axis */
      } brakes;
   } u;
} MCP_MSG;

/*
 * tAlgnClmp task
 */
extern MSG_Q_ID msgAlignClamp;
/*
 * tFFS task
 */
extern MSG_Q_ID msgFFS;
/*
 * moveCW task
 */
extern MSG_Q_ID msgMoveCW;
extern MSG_Q_ID msgMoveCWAbort;
/*
 * tm_ClrInt/tLatch tasks
 */
extern MSG_Q_ID msgLatched;
extern MSG_Q_ID msgLatchReenable;
/*
 * tLamps task
 */
extern MSG_Q_ID msgLamps;
/*
 * tSpecDoor task
 */
extern MSG_Q_ID msgSpecDoor;
/*
 * tBars task
 */
extern MSG_Q_ID msgTbars;

#endif
