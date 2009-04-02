#if !defined(MCP_MSG_Q_H)
#define MCP_MSG_Q_H 1

#include <msgQLib.h>
#include "timerTask.h"

typedef struct {
   enum {
      alignClamp_type = 1,		/* move alignment clamp */
      alignClampCheck_type,		/* check if alignment clamp moved */

      FFS_type,				/* move Flat Field Screen */
      FFSCheckMoved_type,		/* check that FFS moved correctly */

      lamps_type,			/* turn lamps on/off */
      lampsCheck_type,			/* check that lamps obeyed us */

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

      specDoor_type,			/* control spectrograph doors */

      cmdLog_type,			/* there's a command to log */
      cmdFlush_type			/* flush command log file */
   } type;
   
   union {
      struct s_tmr_msg tmr;		/* a message type from tTimerTask */

      struct {
	 enum {ENGAGE, DISENGAGE} op;
      } alignClamp;

      struct {
	 enum {FFS_SAME = -1, FFS_OPEN = 1, FFS_CLOSE = 0, FFS_TOGGLE = 2} op;
      } FFS;

      struct {
	 enum {
	    FF_LAMP, HGCD_LAMP, NE_LAMP, UV_LAMP, WHT_LAMP
	 } type;			/* type off lamp */
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

      struct {
	 char cmd[1];			/* The command to log.
					   N.b. message queue must be declared
					   with sizeof(MCP_MSG) + UBLOCK_SIZE */
      } cmdLog;

      struct {
	 int dummy;
      } cmdFlush;
   } u;

    unsigned long cid;			/* The command ID */
    int uid;				/* The user ID */
} MCP_MSG;

/*
 * A message reporting status to be broadcast
 */
typedef enum {
    ERROR_CODE = '!',
    FATAL_CODE = 'f',
    FINISHED_CODE = ':',
    INFORMATION_CODE = 'i',
    QUEUED_CODE = '>',
    WARNING_CODE = 'w'
} MSG_CODE;				/* Status of command */ 

typedef struct {
    unsigned long cid;			/* The command ID */
    int uid;				/* The user ID */
    signed char code;			/* Status of command */ 
    char key[23];			/* The desired keyword */

    enum {
        array,                          /* the value is a comma separated set of values, passed as a string */
        boolean,                        /* the value is a boolean (the value's still in u.ival) */
        file_descriptor,                /* the value is a file descriptor to write to */
	integer,                        /* there is a keyword with an integer value */
	none,                           /* there is no keyword */
        novalue,                        /* there is a keyword, but it has no value */
        string                          /* there is a keyword with a string value */
    } type;

    union {
	int ival;
        char sval[80];
    } u;
} MCP_STATUS_MSG;

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
/*
 * tCmdLog task
 */
extern MSG_Q_ID msgCmdLog;
/*
 * tStatus task
 */
extern MSG_Q_ID msgStatus;

#endif
