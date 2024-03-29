/**************************************************************************
**
**	Action routines for commands from the TCC which involve setting up
**	queues for each axis to execute motion.
**	tm_TCC is spawned for each axis to execute motion
*/
#include <assert.h>
#include <string.h>
#include <ctype.h>
#include "vxWorks.h"
#include "wdLib.h"
#include "semLib.h"
#include "sigLib.h"
#include "stdio.h"
#include "tickLib.h"
#include "inetLib.h"
#include "taskLib.h"
#include "rebootLib.h"
#include "sysLib.h"
#include "ioLib.h"
#include "errno.h"
#include "usrLib.h"
#include "in.h"
#include "timers.h"
#include "time.h"
#include "frame.h"
#include "iv.h"
#include "intLib.h"
#include "ms.h"
#include "idsp.h"
#include "pcdsp.h"
#include "gendefs.h"
#include "mv162IndPackInit.h"
#include "logLib.h"
#include "data_collection.h"
#include "io.h"
#include "math.h"
#include "tm.h"
#include "axis.h"
#include "cw.h"
#include "serial.h"
#include "mcpUtils.h"
#include "dscTrace.h"
#include "mcpMsgQ.h"
#include "abdh.h"
#include "mcpTimers.h"
#include "mcpFiducials.h"
#include "cmd.h"
#include "instruments.h"
#include "ipcast.h"
#include "as2.h"

/*========================================================================
**========================================================================
**
** LOCAL MACROS, DEFINITIONS, ETC.
**
**========================================================================
*/
#define	DSP_IO_BASE		(0x300)			/* in A16/D16 space. */

/*-------------------------------------------------------------------------
**
** GLOBAL VARIABLES
*/
SEM_ID semMEI = NULL;
SEM_ID semSLC = NULL;

int MEI_interrupt = FALSE;
int sdss_was_init = FALSE;
struct FRAME_QUEUE axis_queue[3] = {
   {0, NULL, NULL, NULL},
   {0, NULL, NULL, NULL},
   {0, NULL, NULL, NULL}
};
/*
 * Status buffers for AXIS.STATUS/SYSTEM.STATUS
 */
char axis_status_buff[NAXIS][STATUS_BUFF_SIZE]; /* usually set  */
char system_status_buff[STATUS_BUFF_SIZE];	/*           by set_status() */

/*
 * Maximum allowed velocity/acceleration and arrays to keep account of
 * the maximum |values| actually requested
 */
double max_velocity[NAXIS] = {2.4, 1.8, 2.4};
double max_acceleration[NAXIS] = {0.24, 0.12, 0.275};
double max_velocity_requested[NAXIS] = {0, 0, 0};
double max_acceleration_requested[NAXIS]={0, 0, 0};

double sec_per_tick[NAXIS];
double ticks_per_degree[NAXIS];

/*****************************************************************************/
/*
 * return an axis's name
 */
const char *
axis_name(int axis)
{
   switch (axis) {
    case AZIMUTH:    return("azimuth");
    case ALTITUDE:   return("altitude");
    case INSTRUMENT: return("rotator");
    default:
      return("(unknown axis)");
   }
}

/*
 * return an axis's abbrev
 */
const char *
axis_abbrev(int axis)
{
   switch (axis) {
    case AZIMUTH:    return("az");
    case ALTITUDE:   return("alt");
    case INSTRUMENT: return("rot");
    default:
      return("??");
   }
}

float
axis_ticks_deg(int axis)
{
   switch (axis) {
    case AZIMUTH:
    case ALTITUDE:
    case INSTRUMENT:
      return(ticks_per_degree[axis]);
    default:
      fprintf(stderr,"axis_ticks_deg: unknown axis %d\n", axis);
      return(1);
   }
}

/*****************************************************************************/
/*
 * The encoders are assumed to be wrong by axis_encoder_error[] which
 * is indexed by 2*axis + ((encoder == 1) ? 0 : 1)
 *
 * Two routines to get/set this error:
 */
/* static */ int axis_encoder_error[2*NAXIS] = { 0, 0, 0, 0, 0, 0 };

int
get_axis_encoder_error(int axis,	/* the axis in question */
		       int encoder)	/* which encoder? 1 or 2 */
{
   assert(axis == AZIMUTH || axis == ALTITUDE || axis == INSTRUMENT);
   assert(encoder == 1 || encoder == 2);

   return(axis_encoder_error[2*axis + (encoder - 1)]);
}

void
set_axis_encoder_error(int axis,	/* the axis in question */
		       int encoder,	/* the encoder in question */
		       long error,	/* value of errors */
		       int write_log)	/* write a log entry? */
{
   assert(axis == AZIMUTH || axis == ALTITUDE || axis == INSTRUMENT);
   assert(encoder == 1 || encoder == 2);

   if(write_log) {
      write_fiducial_log("SET_FIDUCIAL_ERROR", axis, 0, 0, 0, 0, 0,
			 0.0, 0.0, error, encoder);
   }

   axis_encoder_error[2*axis + (encoder - 1)] += error;
}

/*
 * Seven routines to intermediate between the MCP and the MEIs.  These
 * routines are responsible for correcting the measured encoder readings
 * by the known errors in those readings, as determined by crossing the
 * fiducials
 *
 * ALL COMMUNICATION OF POSITIONS TO/FROM THE MEIs MUST USE THESE ROUTINES!
 */
double
convert_mei_to_mcp(int mei_axis,
		   double pos)
{
   return(pos + axis_encoder_error[mei_axis]);
}

int
get_position_corr(int mei_axis,		/* 2*(desired axis) */
		  double *position)	/* position to get */
{
   int uid = 0, cid = 0;
   int ret = get_position(mei_axis, position); /* read encoder */

   if(ret != DSP_OK) {
      NTRACE_2(0, uid, cid, "get_position_corr: %s %s", axis_name(mei_axis/2), _error_msg(ret));
      return(ret);
   }

   *position += axis_encoder_error[mei_axis]; /* undo correction */

   return(ret);
}

int
get_latched_position_corr(int mei_axis,	/* 2*(desired axis) */
			  double *position) /* position of latch */
{
   int uid = 0, cid = 0;
   int ret = get_latched_position(mei_axis, position);

   if(ret != DSP_OK) {
      NTRACE_2(0, uid, cid, "%s get_latched_position failed: %s",
	       axis_name(mei_axis/2), _error_msg(dsp_error));
   } else {
      *position += axis_encoder_error[mei_axis];
   }

   return(ret);
}

int
set_position_corr(int mei_axis,		/* 2*(desired axis) */
		  double position)	/* position to set */
{
   int uid = 0, cid = 0;
   int ret;

   position -= axis_encoder_error[mei_axis]; /* apply correction */
   ret = set_position(mei_axis, position); /* set position */
   if(ret != DSP_OK) {
      NTRACE_2(0, uid, cid, "%s set_position failed: %s",
	       axis_name(mei_axis/2), _error_msg(dsp_error));
   }

   return(ret);
}

int
start_move_corr(int mei_axis,
		double pos,
		double vel,
		double acc)
{
   int uid = 0, cid = 0;
   int ret;

   pos -= axis_encoder_error[mei_axis]; /* apply correction */
   ret = start_move(mei_axis, pos, vel, acc); /* do move */
/*
 * The MEIs seem to sometimes be confused as to whether they are all
 * ready in place; if they say they are do a tiny offset and try again
 */
   if(ret == DSP_NO_DISTANCE) {		/* offset a little and try again */
      NTRACE(2, uid, cid, "start_move failed; offsetting and trying again");
      r_move(10, pos, vel, acc);
      ret = start_move(mei_axis, pos, vel, acc); /* do move */
   }

   if(ret != DSP_OK) {
      NTRACE_2(0, uid, cid, "%s start_move failed: %s",
	       axis_name(mei_axis/2), _error_msg(dsp_error));
   }

   return(ret);
}

int
frame_m_xvajt_corr(PFRAME frame,	/* frame for MEI */
		   char *cmd_str,	/* what we want the DSP to do */
		   int mei_axis,	/* 2*axis */
		   double x,		/* desired position */
		   double v,		/* desired velocity */
		   double a,		/* desired acceleration */
		   double j,		/* desired jerk */
		   double t,		/* at time t */
		   int new_frame)	/* new frame? */
{
   int uid = 0, cid = 0;
   int ret;

   x -= axis_encoder_error[mei_axis];	/* apply correction */

   ret = frame_m(frame, cmd_str, mei_axis, x, v, a, j, t,
		 FUPD_ACCEL|FUPD_VELOCITY|FUPD_POSITION|FUPD_JERK|FTRG_TIME,
		 new_frame);
   if(ret != DSP_OK) {
      NTRACE_1(0, uid, cid, "frame_m: %s", _error_msg(ret));
   }

   return(ret);
}

int
dsp_set_last_command_corr(PDSP pdsp,
			  int16 mei_axis, /* 2*axis */
			  double final)
{
   final -= axis_encoder_error[mei_axis]; /* apply correction */

   return(dsp_set_last_command(pdsp, mei_axis, final));
}

/*=========================================================================
**=========================================================================
**
**      ID -> MEI firmware
**
** RETURN VALUES:
**      "axis date firmware"
**=========================================================================
*/
char *
id_cmd(int uid, unsigned long cid, char *cmd)
{
   const int axis = ublock->axis_select;

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badAxis");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "id");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

   sprintf(ublock->buff,
	   "%d %s %s\r\n%s\r\nDSP Firmware: V%f R%d S%d, Option=%d, Axes=%d",
	   axis, axis_name(axis),__DATE__, getCvsTagname(),
	   dsp_version()/1600., dsp_version()&0xF, (dsp_option()>>12)&0x7,
	   (dsp_option() & 0xFFF), dsp_axes());

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "id");

   return(ublock->buff);
}

/*=========================================================================
**
**      INIT -> Init the axis
**
** RETURN VALUES:
**      NULL string or "ERR:..."
*/
char *
init_cmd(int uid, unsigned long cid, char *cmd)
{
   const int axis = ublock->axis_select;
   int i;
   int state;

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badAxis");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "init");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }
/*
 * send MS.OFF to stop updating of axis position from fiducials
 */
   if(set_ms_off(uid, cid, axis, 0) < 0) {
      NTRACE(0, uid, cid, "init failed to set MS.OFF");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "init");

      return "ERR: failed to set MS.OFF";
   }
/*
 * I (Charlie) don't really agree with this, JEG directed...
 * I think the axis should remain in closed loop if in close loop
 */
   state = tm_axis_state(2*axis);

   if(state > NEW_FRAME) {		/* not NOEVENT, running, or NEW_FRAME*/
      NTRACE_2(1, uid, cid, "INIT axis %s: not running: %s",
	       axis_name(axis), axis_state_str(2*axis));
      tm_sem_controller_idle(2*axis);
   }

   tm_reset_integrator(2*axis);
   enable_pvt(axis);
/*
 * OK, the axis status is updated
 */
   switch(axis) {
    case AZIMUTH:
      amp_reset(2*axis);		/* two amplifiers, */
      amp_reset(2*axis + 1);		/* param is index to bit field */
      if(sdssdc.status.i9.il0.az_brake_en_stat) {
	 mcp_unset_brake(uid, cid, axis);
      }
      break;
    case ALTITUDE:
      amp_reset(2*axis);		/* two amplifiers */
      amp_reset(2*axis + 1);		/* param is index to bit field */
      if(sdssdc.status.i9.il0.alt_brake_en_stat) {
	 mcp_unset_brake(uid, cid, axis);
      }
      break;
    case INSTRUMENT:
      amp_reset(2*axis);		/* one amplifier */
      break;				/* param is index to bit field */
   }
/*
 * reinitialize the queue of pvts to none
 */
   axis_queue[axis].active = NULL;

   tcc_may_release_semCmdPort = 0;	/* don't allow the TCC
					   to drop the semCmdPort */
/*
 * zero the velocity
 */
   if(semTake(semMEI,60) == ERROR) {
      NTRACE_1(0, uid, cid, "Failed to take semMEI to zero velocity for axis %s",
	       axis_name(axis));
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "init");
   } else {
      taskLock();
      set_stop(2*axis);
      while(!motion_done(2*axis)) ;
      clear_status(2*axis);
      taskUnlock();

      semGive(semMEI);

      if(dsp_error != DSP_OK) {
	 NTRACE_2(0, uid, cid, "failed to stop %s : %d", axis_name(axis), dsp_error);
	 sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "init");
      }
   }
/*
 * tm_axis_state retries as well...so this is really redundant, but then the
 * brakes don't come off really quickly so this will assure closed loop for
 * at least sometime before continuing
 */
   for(i = 0; i < 4; i++) {
      tm_sem_controller_run(2*axis);

      if(tm_axis_state(2*axis) <= NEW_FRAME) { /* axis is OK */
	 break;
      }

      taskDelay(20);
   }
/*
 * If there's a known positional error on an axis take this opportunity to
 * zero it by setting the encoder's position
 */
   if(fiducial[axis].max_correction != 0) {
      if(semTake(semLatch, 60) == ERROR) {
	 NTRACE(0, uid, cid, "ERR: init cannot take semLatch");
	 sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "init");
      } else {
	 long correction[3];		/* how much to correct encoder posns */

	 correction[0] = 0;		/* unused */
	 correction[1] = get_axis_encoder_error(axis, 1);
	 correction[2] = get_axis_encoder_error(axis, 2);

	 if(abs(correction[1]) > 0) {
	    NTRACE_2(3, uid, cid,"Adjusting %s's encoder 1 by %ld",
		     axis_name(axis), correction[1]);
	 }
	 if(abs(correction[2]) > 0) {
	    NTRACE_2(3, uid, cid,"Adjusting %s's encoder 2 by %ld",
		     axis_name(axis), correction[2]);
	 }

	 if(tm_adjust_position(axis, correction) < 0) {
	    NTRACE_1(0, uid, cid ,"Failed to adjust encoder for axis %s", axis_name(axis));
	 }
      }

      semGive(semLatch);
   }
/*
 * Update the axis scales from the fiducial tables
 */
   switch (axis) {
    case AZIMUTH:
      set_axis_scale(axis, fiducial[axis].scale[AZ_ENCODER]);
      break;
    case INSTRUMENT:
      set_axis_scale(axis, fiducial[axis].scale[ROT_ENCODER]);
      break;
   }

/*
 * Set the axis coefficients for slewing.
 */
   select_pid_block(uid, cid, axis, PID_COEFFS_SLEWING);

/*
 * Clear the status of the bump switches
 */
   clear_sticky_bumps(axis, 0);
   clear_sticky_bumps(axis, 1);
/*
 * Clear the axis status; well, don't actually _clear_ it, set it to
 * the last non-sticky value. That way the TCC will see the current
 * set of bad bits.
 *
 * This isn't quite good enough (see PR 2553) as some of those bits may
 * have been reporting problems such as not having the semCmdPort semaphore
 * which is taken by INIT itself.
 *
 * A compromise is to explicitly clear bits that are known to be now OK
 */
   while(semTake(semMEIUPD, WAIT_FOREVER) == ERROR) {
      NTRACE_2(0, uid, cid, "init_cmd: failed to get semMEIUPD: %d %s",
	       errno, strerror(errno));
      taskSuspend(0);
   }

   axis_stat[axis][0] = axis_stat[axis][1];
/*
 * Clear some bits after checking their current status
 */
   taskDelay(60);

   if(axis_stat[axis][0].semCmdPort_taken) {
      if(have_semaphore(uid)) {
	 axis_stat[axis][0].semCmdPort_taken = 0;
      }
   }

   if(axis_stat[axis][0].amp_bad) {
      int amp_ok;
      switch (axis) {
       case AZIMUTH:    amp_ok = az_amp_ok(1);  break;
       case ALTITUDE:   amp_ok = alt_amp_ok(1); break;
       case INSTRUMENT: amp_ok = rot_amp_ok(1); break;
      }
      axis_stat[axis][0].amp_bad = amp_ok ? 0 : 1;
   }

   if(semTake(semMEI, 5) != ERROR) {
      sdssdc.axis_state[axis] = axis_state(2*axis);
      axis_stat[axis][0].out_closed_loop =
				(sdssdc.axis_state[axis] <= NEW_FRAME) ? 0 : 1;

      semGive(semMEI);
   }

   if(axis_stat[axis][0].stop_in) {
      axis_stat[axis][0].stop_in = check_stop_in(0) ? 1 : 0;
   }

   semGive(semMEIUPD);
/*
 * flush the MCP command logfile
 */
   log_mcp_command(0, NULL);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "init");

   return "";
}

/*=========================================================================
**
**      MC.MAX.ACC -> Display the maximum permitted acceleration.
**
** RETURN VALUES:
**      "acc" or "ERR:..."
*/
char *
mc_maxacc_cmd(int uid, unsigned long cid, char *cmd)
{
   const int axis = ublock->axis_select;

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badAxis");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "mc_maxacc");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

   sprintf(ublock->buff ,"%12f", max_acceleration[axis]);

   {
      char key[20];
      sprintf(key, "%sMaxAccLimit", axis_abbrev(axis));
      sendStatusMsg_F(uid, cid, INFORMATION_CODE, 1, key, max_acceleration[axis]);
   }

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "mc_maxacc");
   return ublock->buff;
}

/*=========================================================================
**
**      MC.MAX.VEL -> Display the maximum permitted velocity.
*/
char *
mc_maxvel_cmd(int uid, unsigned long cid, char *cmd)		/* NOTUSED */
{
   const int axis = ublock->axis_select;

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badAxis");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "mc_maxacc");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

   sprintf(ublock->buff ,"%12f", max_velocity[axis]);

   {
      char key[20];
      sprintf(key, "%sMaxVelLimit", axis_abbrev(axis));
      sendStatusMsg_F(uid, cid, INFORMATION_CODE, 1, key, max_velocity[axis]);
   }
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "mc_maxvel");
   return ublock->buff;
}

/*****************************************************************************/
/*
 * Select an instrument
 */
char *
rot_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   ublock->axis_select = INSTRUMENT;
   sendStatusMsg_S(uid, cid, INFORMATION_CODE, 0, "command", "rot"); /* not a "real" command that completes */
   return "";
}

char *
az_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   ublock->axis_select = AZIMUTH;
   sendStatusMsg_S(uid, cid, INFORMATION_CODE, 0, "command", "az"); /* not a "real" command that completes */
   return "";
}

char *
alt_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   ublock->axis_select = ALTITUDE;
   sendStatusMsg_S(uid, cid, INFORMATION_CODE, 0, "command", "alt"); /* not a "real" command that completes */
   return "";
}

/*=========================================================================
**=========================================================================
**
** ROUTINE: stats_cmd
**
** DESCRIPTION:
**	STATS - Tracking error statistics based on fiducial crossing.
**	Unimplemented until fiducials are better understood.  The information
**	is available via print_fiducials(axis, show_all).
**
** RETURN VALUES:
**	NULL string or "ERR:..."
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
char *
stats_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 0, "command", "stats");
   return "";
}

/*=========================================================================
**
**	STATUS - Returns status of the axis.
**
** RETURN VALUES:
**	return "pos vel time status index" or "ERR:..."
*/
char *
status_cmd(int uid, unsigned long cid, char *cmd)
{
   const int axis = ublock->axis_select;
   double pos, vel;			/* position and velocity */
   int ret;				/* a return code */
   double sdss_time;			/* time from sdss_get_time() */

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badAxis");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "status");
      return("ERR: ILLEGAL DEVICE SELECTION");
   }

   sdss_time = sdss_get_time();
   if(sdss_time < 0) {
      sdss_time += 1.0;
   }

   if(semTake(semMEI,60) == ERROR) {
      NTRACE_2(0, uid, cid, "status_cmd: failed to get semMEI: %d %s",
	       errno, strerror(errno));

      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "status");

      sprintf(ublock->buff, "ERR: semMEI : %s", strerror(errno));

      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "status");
      return(ublock->buff);
   }

   get_position_corr(2*axis, &pos);
   if((ret = get_velocity(2*axis, &vel)) != DSP_OK) {
      NTRACE_2(0, uid, cid, "STATUS get_velocity: %s %s", axis_name(axis), _error_msg(ret));
   }

   semGive(semMEI);

   pos /= axis_ticks_deg(axis);
   vel /= axis_ticks_deg(axis);

   NTRACE(8, uid, cid, "taking semMEIUPD");
   if (semTake(semMEIUPD, 60) == ERROR) {
      NTRACE_1(5, uid, cid, "status_cmd: semMEIUPD : %d", errno);
      sprintf(ublock->buff, "ERR: semMEIUPD : %s", strerror(errno));

      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "status");

      return(ublock->buff);
   }

   sprintf(ublock->buff, "%f %f %f %ld %f", pos, vel, sdss_time,
	   (*(long *)&axis_stat[axis][0] & ~STATUS_MASK), 0.0);

   axis_stat[axis][0] = axis_stat[axis][1];
   *(long *)&axis_stat[axis][1] &= STATUS_MASK;

   semGive(semMEIUPD);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 0, "command", "status");

   return(ublock->buff);
}

/*=========================================================================
**=========================================================================
**
**	STATUS.LONG - Returns long status of the axis.
**
** RETURN VALUES:
**	return "date time
**		pos position
**		vel velocity
**		bits status
**		deg last index" or "ERR:..."
**	currently returning only "bits status"
**=========================================================================
*/
char *
status_long_cmd(int uid, unsigned long cid, char *cmd)		/* NOTUSED */
{
   int i;
   long status = *(long *)&axis_stat[ublock->axis_select][0];

   printf (" STATUS.LONG command fired\r\n");
   for(i = 0;i < 32; i++) {
      ublock->buff[i]= (status & (1 << (31 - i))) ? '1' : '0';
   }
   ublock->buff[i] = '\0';

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 0, "command", "status_long");

   return(ublock->buff);
}

/*****************************************************************************/
/*
 * Is an e-stop in?
 */
int
check_stop_in(int update)
{
   if(update) {
#if ALLOW_AMP_UPDATE
      update_sdssdc_status_i6();
#endif
   }

   if(sdssdc.status.i6.il0.w_lower_stop &&
      sdssdc.status.i6.il0.e_lower_stop &&
      sdssdc.status.i6.il0.s_lower_stop &&
      sdssdc.status.i6.il0.n_lower_stop &&
      sdssdc.status.i6.il0.w_rail_stop &&
      sdssdc.status.i6.il0.s_rail_stop &&
      sdssdc.status.i6.il0.n_rail_stop &&
      sdssdc.status.i6.il0.n_fork_stop &&
      sdssdc.status.i6.il0.nw_fork_stop &&
      sdssdc.status.i6.il0.n_wind_stop &&
      sdssdc.status.i6.il0.cr_stop &&
      sdssdc.status.i6.il0.s_wind_stop) {
      return FALSE;
   } else {
      return TRUE;
   }
}

/*****************************************************************************/
/*
 * convert a clinometer reading to degrees
 */
float
convert_clinometer(float val)
{
/*
 * 8857 is pinned at 0 degrees; -9504 is zenith 90 degrees 22-Aug-98
 */
   const float altclino_sf = 0.0048683116163; /* scale factor */
   const int altclino_off = 8937;	/* offset */

   return((altclino_off - val)*altclino_sf);
}

/*
 * Return the clinometer reading
 */
float
read_clinometer(void)
{
   return(convert_clinometer(sdssdc.status.i4.alt_position));
}

/*****************************************************************************/
/*
 * An axis-status command that can be used by IOP to get enough information
 * to update the MCP Menu
 */
char *
axis_status_cmd(int uid, unsigned long cid, char *cmd)
{
   const int axis = ublock->axis_select;

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badAxis");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 0, "command", "axis_status");
      return("ERR: ILLEGAL DEVICE SELECTION");
   }

   if(semTake(semStatusCmd, 2) == ERROR) {
/* #if 0
      OTRACE(3, "axis_status_cmd failed to take semStatusCmd: %s",
	     strerror(errno), 0);
#endif */
      sendStatusMsg_S(uid, cid, ERROR_CODE, 0, "command", "axis_status");
      return("ERR: Cannot take semStatusCmd");
   }

   strncpy(ublock->buff, axis_status_buff[axis], UBLOCK_SIZE);

   semGive(semStatusCmd);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 0, "command", "axis_status");

   return(ublock->buff);
}

/*****************************************************************************/
/*
 * Return the status of misc things
 */
static int
get_miscstatus(char *status,
	       int size)			/* dimen of status[] */
{
  int len;
  char *fidver;
  char errorBuf[3*FIDVERLEN+4];

  /* Decide whether we have sensible fiducial versions. */
  if (!strcmp(fiducialVersion[0], fiducialVersion[1]) &&
      !strcmp(fiducialVersion[0], fiducialVersion[2])) {
    fidver = fiducialVersion[0];
  } else {
    sprintf(errorBuf, "%s|%s|%s",
	    fiducialVersion[0], fiducialVersion[1], fiducialVersion[2]);
    fidver = errorBuf;
  }

  sprintf(status,"Misc: %d %d  %d %d %s\n",
	  sdssdc.status.i9.il0.clamp_en_stat, /* alignment clamp */
	  sdssdc.status.i9.il0.clamp_dis_stat,
	  iacked,
	  sdssdc.status.b3.w1.version_id, /* PLC version, per the PLC */
	  fidver);

  len = strlen(status);
  assert(len < size);

  return(len);
}

/*****************************************************************************/
void
do_info_cmd(int uid, unsigned long cid, int include_cw) {
   broadcast_ffs_lamp_status(uid, cid, 1, 1);
   broadcast_inst_status(uid, cid);
   broadcast_fiducial_status(uid, cid);
   if (include_cw) {			/* special cased as their values are analogue and fluctuate */
      broadcast_cw_status(uid, cid);
   }
   broadcast_slit_status(uid, cid);
   broadcast_ipsdss(uid, cid);
}

char *
info_cmd(int uid, unsigned long cid, char *cmd)
{
   if(semTake(semStatusCmd, 2) == ERROR) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "info");
      return("ERR: Cannot take semStatusCmd");
   }

   clearKeywordCache();

   do_info_cmd(uid, cid, 1);

   semGive(semStatusCmd);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "info");

   return "";
}
/*****************************************************************************/
/*
 * An status command that can be used by IOP to get enough information
 * to update the MCP Menu. Returns everything except the axis status and
 * the state of the semCmdPort
 */
char *
system_status_cmd(int uid, unsigned long cid, char *cmd)
{
   if(semTake(semStatusCmd, 2) == ERROR) {
/* #if 0
       OTRACE(3, "system_status_cmd failed to take semStatusCmd: %s",
	     strerror(errno), 0);
#endif */
      sendStatusMsg_S(uid, cid, ERROR_CODE, 0, "command", "system_status");
      return("ERR: Cannot take semStatusCmd");
   }

   strncpy(ublock->buff, system_status_buff, UBLOCK_SIZE);

   semGive(semStatusCmd);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 0, "command", "system_status");

   return(ublock->buff);
}

/*****************************************************************************/
/*
 * Set the string to return in response to an {AXIS,SYSTEM}.STATUS command
 */
void
set_status(int axis,			/* axis, or NOINST for system status */
	   char *buff,			/* buffer to set */
	   int size)			/* size of buff[] */
{
   int uid = 0, cid = 0;
   int brake_is_on = -1;		/* is the brake on for current axis? */
   long fid_mark = 0;			/* position of fiducial mark */

   if(axis != NOINST && axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badAxis");
      strncpy(buff, "ERR: ILLEGAL DEVICE SELECTION", size);
      return;
   }

   buff[size - 1] = '\a';

   if(axis == NOINST) {
      int i = 0;
      i += get_cwstatus(&buff[i], size - i);
      i += get_ffstatus(&buff[i], size - i);
      i += get_slitstatus(&buff[i], size - i);
      i += get_miscstatus(&buff[i], size - i);
      i += get_inststatus(&buff[i], size - i);
   } else {
      switch (axis) {
       case AZIMUTH:
	 brake_is_on = sdssdc.status.i9.il0.az_brake_en_stat;
	 fid_mark = az_fiducial[fiducialidx[axis]].mark[1];
	 break;
       case ALTITUDE:
	 brake_is_on = sdssdc.status.i9.il0.alt_brake_en_stat;
	 fid_mark = alt_fiducial[fiducialidx[axis]].mark[1];
	 break;
       case INSTRUMENT:
	 brake_is_on = -1;			/* there is no brake */
	 fid_mark = rot_fiducial[fiducialidx[axis]].mark[1] - ROT_FID_BIAS;
	 break;
       default:
	 fprintf(stderr,"axis_status_cmd: impossible instrument %d\n",
		 axis);
	 abort();
	 break;				/* NOTREACHED */
      }

      sprintf(buff,
	      "%f %d %d  %ld %ld %d %ld  %d %d %ld  %d %d 0x%lx  %.4f",
	      ticks_per_degree[axis], monitor_on[axis], axis_state(2*axis),
	      tmaxis[axis]->actual_position, tmaxis[axis]->position,
	      tmaxis[axis]->voltage, tmaxis[axis]->velocity,
	      fiducialidx[axis], 0, fid_mark,
	      check_stop_in(0), brake_is_on, *(long *)&axis_stat[axis][0],
	      read_clinometer());
   }

   /* OTRACE(8, "set_status: checking buff[end]: %d", buff[size - 1], 0); */
   if(buff[size - 1] != '\a') {		/* check for overflow */
      NTRACE(0, uid, cid, "set_status: overwrote buffer marker");
      taskSuspend(0);
   }
}

/*=========================================================================
**
**	AB.STATUS off len - return up to 20 words of allen-bradley status
**      starting at a specified position.
**
** RETURN VALUES:
**	return "0xnnnn 0xnnnn ...."
*/
char *
abstatus_cmd(int uid, unsigned long cid, char *cmd)
{
   int i,idx;
   short *dt;
   int off,len;

   {
      volatile int tmp = 7*20;		/* avoid gcc warning */
      assert(UBLOCK_SIZE >= tmp);
   }

   if(sscanf(cmd,"%d %d",&off,&len) != 2) {
      NTRACE_1(0, uid, cid, "Need offset and len for AB.STATUS: %s", cmd);
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "abstatus");
      return("ERR: missing arguments");
   }

   if(len > 20) {
      NTRACE_1(0, uid, cid, "AB.STATUS: bad len %d", len);
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "abstatus");
      return("ERR: bad length");
   }

   dt = (short *)&sdssdc.status;
   dt += off;

   for(i = idx = 0;i < len; i++, dt++) {
      idx += sprintf(&ublock->buff[idx],"%s0x%04x", (idx==0 ? "" : " "), *dt);
   }

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "abstatus");

   return(ublock->buff);
}

/*=========================================================================
**=========================================================================
**
** ROUTINE:
**	    set_rot_state (int state)
**
**	    set_rot_coeffs
**	    print_rot_coeffs
**	    set_rot_uplimit
**	    set_rot_dnlimit
**	    coeffs_state_cts	modify the state based on velocity in raw counts
**
** DESCRIPTION:
**	Modify the instrument rotators coeffs as a function of velocity.
**	Supporting routines for the specification of the state machine which
**	provides hysterisis to prevent undue switching.  This was used and
**	tested, but is deprecated by reducing the sampling frequency of the
**	rotator and thus increasing the gain of the system.  I want to maintain
**	these functions.
**
**      N.b. These routines are only compiled/called if SWITCH_PID_COEFFS
**      is true, with the exception of set_rot_state()
**
** RETURN VALUES:
**	number of MEI frames
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
#if !SWITCH_PID_COEFFS

void
set_rot_state(int state)
{
   assert(state == -1);			/* called from mcp.login */
}

#else

int axis_coeffs_state[]={-1,-1,-1,-1,-1,-1};
int *rotcoeffs=&axis_coeffs_state[4];
struct SW_COEFFS rot_coeffs[]={
        {{320,48,600,0,0,32767,0,15000,-6,1000},7000/ROT_TICKS_DEG,-1.0,              7000,-1},
        {{320,44,600,0,0,32767,0,15000,-6,1200},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{320,40,600,0,0,32767,0,15000,-6,1500},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{320,36,600,0,0,32767,0,15000,-6,1700},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{320,32,600,0,0,32767,0,15000,-6,2000},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{320,28,600,0,0,32767,0,15000,-6,2200},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{320,24,600,0,0,32767,0,15000,-6,2500},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{320,20,600,0,0,32767,0,15000,-6,2700},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{320,16,600,0,0,32767,0,15000,-6,3000},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{320,12,600,0,0,32767,0,15000,-6,3200},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{300,10,600,0,0,32767,0,15000,-6,3500},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{260, 8,600,0,0,32767,0,15000,-6,3600},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{220, 6,600,0,0,32767,0,15000,-6,3700},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{180, 4,600,0,0,32767,0,15000,-6,3800},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{140, 3,600,0,0,32767,0,15000,-6,3900},7000/ROT_TICKS_DEG,5000/ROT_TICKS_DEG,7000,5000},
        {{100, 2,300,0,0,32767,0,15000,-6,4000},-1.0              ,5000/ROT_TICKS_DEG,   -1,5000}
};

void
set_rot_coeffs (int state, int index, short val)
{
   rot_coeffs[state].coeffs[index] = val;
}

void
print_rot_coeffs(void)
{
   int i;

   printf ("State %d Active\n",axis_coeffs_state[4]);
   for (i=0;i<sizeof(rot_coeffs)/sizeof(struct SW_COEFFS);i++) {
      printf ("rot_coeffs state %d: uplimit=%d, dnlimit=%d\n",
	      i,rot_coeffs[i].uplimit_cts,rot_coeffs[i].dnlimit_cts);
      printf("P=%d,I=%d,D=%d,AFF=%d,VFF=%d,ILIM=%d,OFF=%d,OLIM=%d,S=%d,FFF=%d\n",
	     rot_coeffs[i].coeffs[0],      /* P */
	     rot_coeffs[i].coeffs[1],      /* I */
	     rot_coeffs[i].coeffs[2],      /* D */
	     rot_coeffs[i].coeffs[3],      /* AFF */
	     rot_coeffs[i].coeffs[4],      /* VFF */
	     rot_coeffs[i].coeffs[5],      /* ILIM */
	     rot_coeffs[i].coeffs[6],      /* OFF */
	     rot_coeffs[i].coeffs[7],      /* OLIM */
	     rot_coeffs[i].coeffs[8],      /* S */
	     rot_coeffs[i].coeffs[9]);     /* FFF */
   }
}

void
set_rot_uplimit (int state, int val)
{
   rot_coeffs[state].uplimit_cts=val;
   rot_coeffs[state].uplimit_deg=val/ROT_TICKS_DEG;
}

void
set_rot_dnlimit (int state, int val)
{
   rot_coeffs[state].dnlimit_cts=val;
   rot_coeffs[state].dnlimit_deg=val/ROT_TICKS_DEG;
}

void
set_rot_state(int state)
{
   axis_coeffs_state[4] = state;
}

int
coeffs_state_cts(int mei_axis,
		 int cts)
{
   int state;
   struct SW_COEFFS *coeff;

   state = axis_coeffs_state[mei_axis];
   if(state == -1) return FALSE;

   taskDelay(1);
   coeff = &rot_coeffs[state];
#if 0
   printf ("\r\nAXIS %d: state=%d coeff=%p",mei_axis,state,coeff);
#endif

   if(coeff->uplimit_cts > 0 && abs(cts) > coeff->uplimit_cts) {
#if 0
      printf ("UP cts=%d, uplimit=%d",cts,coeff->uplimit_cts);
#endif
      state++; coeff++;
   } else if(abs(cts) < coeff->dnlimit_cts) {
#if 0
      printf("DN cts=%d, dnlimit=%d", cts, coeff->dnlimit_cts);
#endif
      if(state == 0) return FALSE;
      state--; coeff--;
   } else {
      return FALSE;
   }

   dsp_set_filter(mei_axis, (P_INT)&coeff->coeffs[0]);
   axis_coeffs_state[mei_axis] = state;
   return TRUE;
}
#endif

/*=========================================================================
**=========================================================================
**
** ROUTINE: ip_shutdown
**
** DESCRIPTION:
**	Shutdown all ip hardware with a reset to the ip slots for the
**	MVME162 and the SYSTRAN carrier.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void
ip_shutdown(int type)
{
    char *ip;
/* reset the systran carrier */
    ip=(char *)0xffff4501;
    printf("IP shutdown: reset SYSTRAN carrier ip @%p\r\n",ip);
    *ip=0x1F;
    taskDelay(20);
    *ip=0x00;
/* reset MVME162 board */
    ip=(char *)0xfffbc01f;
    printf("IP shutdown: reset MVME162 ip @%p\r\n",ip);
    *ip=0x1;
    taskDelay(20);
    *ip=0x00;
    taskDelay(30);
}
/*=========================================================================
**=========================================================================
**
** ROUTINE: amp_reset
**
** DESCRIPTION:
**	Resets the amps; 2 Az, 2 Alt, and 1 Rot.  Pulses the corresponding
**	output bit.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**	cw_DIO316
**
**=========================================================================
*/
void
amp_reset(int mei_axis)
{
   /* TBD: jkp: don't have a uid/cid now; could get it passed up from mcp_amp_reset */
   NTRACE_2(0, 0, 0, "Resetting amp for axis %s: %d", axis_name(mei_axis/2), mei_axis);

   DIO316_Write_Port(cw_DIO316, AMP_RESET, 1<<mei_axis);
   taskDelay (2);
   DIO316_Write_Port(cw_DIO316, AMP_RESET, 0);
}

#if 0
/*=========================================================================
**=========================================================================
**
** ROUTINE: save_firmware
**	    restore_firmware
**
** DESCRIPTION:
**	Save/restore of MEI firmware...primarily for upgrades which was
**	not part of the VME package.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void
save_firmware(void)
{
  upload_firmware_file ("vx_mei_firmware.bin");
}

void
restore_firmware(void)
{
  download_firmware_file ("vx_mei_firmware.bin");
}
#endif

/*=========================================================================
**=========================================================================
**
** ROUTINE: print_max
**
** DESCRIPTION:
**	Diagnostic routines to monitor the max vel and accel.
**
** RETURN VALUES:
**	void
**
** CALLS TO:
**
** GLOBALS REFERENCED:
**
**=========================================================================
*/
void
print_max(void)
{
   int i;

   for(i = 0; i < NAXIS; i++) {
      printf("%-10s: MAX ACC limit %f deg/sec^2 maximum requested %f\n",
	     axis_name(i), max_acceleration[i], max_acceleration_requested[i]);
   }
   for(i = 0; i < NAXIS; i++) {
      printf("%-10s: MAX VEL limit %f deg/sec maximum requested %f\n",
	     axis_name(i), max_velocity[i], max_velocity_requested[i]);
   }
}

/*****************************************************************************/
/*
 * Functions called by Menu() to do various things; also accessible
 * via the serial or tcp interfaces
 */
/*
 * Set whether an axis is monitored
 */
int
mcp_set_monitor(int axis,		/* which axis? */
		int on_off)		/* set monitor on/off/toggle */
{
   int uid = 0, cid = 0;

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      NTRACE_1(0, uid, cid, "mcp_set_monitor: illegal axis %d", axis);

      return(-1);
   }

   if(on_off == -1) {
      on_off = !monitor_on[axis];
   }

   monitor_on[axis] = on_off ? 1 : 0;

   return(0);
}

/*
 * Set the position of an axis
 */
int
mcp_set_pos(int axis,			/* the axis to set */
	    double pos)			/* the position to go to */
{
   int uid = 0, cid = 0;

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      NTRACE_1(0, uid, cid, "mcp_set_pos: illegal axis %d", axis);

      return(-1);
   }

   tm_set_position(2*axis, pos);
   tm_set_position(2*axis + 1, pos);

   return 0;
}

/*
 * move to a position with specified velocity and acceleration
 */
int
mcp_move_va(int axis,			/* the axis to move */
	    long pos,			/* desired position */
	    long vel,			/* desired velocity */
	    long acc)			/* desired acceleration */
{
   int uid = 0, cid = 0;
   int ret;				/* return code from MEI */
   double sf;				/* scale factor for axis */

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      NTRACE_1(0, uid, cid, "mcp_move_va: illegal axis %d", axis);

      return(-1);
   }
   sf = ticks_per_degree[axis];

   if(semTake(semMEI,60) == ERROR) {
      NTRACE_2(0, uid, cid, "mcp_move_va: failed to take semMEI: %s (%d)",
	    strerror(errno), errno);

      return(-1);
   }

   sem_controller_run(2*axis);

#if 0 /*debugging*/
   NTRACE_2(1,uid,cid,"mcp_move_va: %s pos=%ld",axis_name(axis),pos);
   NTRACE_2(1,uid,cid,"mcp_move_va: %s vel=%ld",axis_name(axis),vel);
   NTRACE_2(1,uid,cid,"mcp_move_va: %s acc=%ld",axis_name(axis),acc);
#endif

#if SWITCH_PID_COEFFS
   if(axis == INSTRUMENT) {
      while(coeffs_state_cts(2*axis, vel) == TRUE) {
	 continue;
      }
   }
#endif
/*
 * check that acceleration/velocity are within limits
 */
   if(fabs(vel) > max_velocity[axis]*sf) {
      printf("AXIS %d: MAX VEL %f exceeded; limit %f\n",
	     axis, vel/sf, max_velocity[axis]);
      NTRACE_2(2, uid, cid, "Max vel. for %s exceeded: %ld", axis_name(axis), vel);

      if(fabs(vel) > fabs(max_velocity_requested[axis])) {
	 max_velocity_requested[axis] = vel;
      }

      vel = (vel > 0) ? sf*max_velocity[axis] : -sf*max_velocity[axis];
   }
   if(fabs(acc) > sf*max_acceleration[axis]) {
      printf("AXIS %d: MAX ACC %f exceeded; limit %f\n",
	     axis, acc/sf, max_acceleration[axis]);
      NTRACE_2(2, uid, cid, "Max accl. for %s exceeded: %ld", axis_name(axis), acc);

      if(fabs(acc) > sf*fabs(max_acceleration_requested[axis])) {
	 max_acceleration_requested[axis] = acc/sf;
      }

      acc = (acc > 0) ? sf*max_acceleration[axis] : -sf*max_acceleration[axis];
   }
/*
 * actually do the move
 */
   if((ret = start_move_corr(2*axis, pos, vel, acc)) != DSP_OK) {
      char *err = _error_msg(ret);
      int murmur_level = (ret == DSP_NO_DISTANCE) ? 3 : 0;
      if(err == NULL) { err = "unknown DSP error"; }
      NTRACE_2(murmur_level, uid, cid, "start_move failed for %s : %s",axis_name(axis),err);
      if(ret == DSP_NO_DISTANCE) {
	 double mei_pos;
	 get_position_corr(2*axis, &mei_pos);
	 NTRACE_2(3, uid, cid, "positions: MCP: %ld MEI: %ld", (long)pos, (long)mei_pos);
      }
   }
   semGive(semMEI);

   return(0);
}

/*
 * set a velocity
 */
int
mcp_set_vel(int axis,			/* the axis to set */
	    double vel)			/* the position to go to */
{
   int uid = 0, cid = 0;
   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      NTRACE_1(0, uid, cid, "mcp_set_vel: illegal axis %d", axis);

      return(-1);
   }

   if(axis == AZIMUTH && sdssdc.status.i9.il0.az_brake_en_stat) {
      NTRACE(0, uid, cid, "mcp_set_vel: AZ Brake is Engaged");
      return(-1);
   } else if(axis == ALTITUDE && sdssdc.status.i9.il0.alt_brake_en_stat) {
      NTRACE(0, uid, cid, "mcp_set_vel: ALT Brake is Engaged");
      return(-1);
   }

   if(semTake(semMEI, 60) == ERROR) {
      NTRACE_2(0, uid, cid, "mcp_set_val: failed to take semMEI: %s (%d)",
	       strerror(errno), errno);
      return(-1);
   }

#if SWITCH_PID_COEFFS
   if(axis == INSTRUMENT) {
      while(coeffs_state_cts(2*axis, vel) == TRUE) {
	 ;
      }
   }
#endif
   set_velocity (2*axis, vel);
   semGive (semMEI);

   return 0;
}

/*****************************************************************************/
/*
 * Hold an axis
 */
int
mcp_hold(int axis)			/* desired axis */
{
   int uid = 0, cid = 0;
   double vel;				/* velocity of axis */

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      NTRACE_1(0, uid, cid, "mcp_halt: illegal axis %d", axis);

      return(-1);
   }

   NTRACE_1(3, uid, cid, "Holding axis %s", axis_name(axis));

   if(semTake(semMEI,60) == ERROR) {
      NTRACE_2(0, uid, cid, "mcp_hold: %s: couldn't take semMEI: %d", axis_name(axis), errno);
      return(-1);
   }

   if(get_velocity(2*axis, &vel) != DSP_OK) {
      NTRACE_2(0, uid, cid, "Holding axis %s: failed to read velocity: %d",
	       axis_name(axis), errno);
      vel = 0;
   }

#if 1
   set_stop(2*axis);
   {
      int i, ntry = 120*60;		/* two minutes */

      for(i = 0; i < ntry; i++) {
	 if(motion_done(2*axis)) {
	    break;
	 }
	 taskDelay(1);
      }
      if(i == ntry) {
	 NTRACE_2(0, uid, cid, "Failed to get motion_done() for %s: in_seq = %d",
		  axis_name(axis), in_sequence(2*axis));
	 NTRACE_2(0, uid, cid, "           in_mot = %d, frames = %d",
		  in_motion(2*axis), frames_left(2*axis));
      }
   }
   clear_status(2*axis);

   if(dsp_error != DSP_OK) {
      NTRACE_2(0, uid, cid, "failed to stop %s : %d", axis_name(axis), dsp_error);
   }

   NTRACE_1(5, uid, cid, "Holding axis %s: back into closed loop", axis_name(axis));
   sem_controller_run(2*axis);		/* back into closed loop */
#else
   NTRACE_1(5, uid, cid, "Holding axis %s: back into closed loop", axis_name(axis));
   sem_controller_run(2*axis);		/* back into closed loop */

   while(fabs(vel) > 5000) {
      vel -= (vel > 0) ? 5000 : -5000;
      NTRACE_1(5, uid, cid, "Velocity = %g", vel);
      set_velocity(2*axis, vel);

#if SWITCH_PID_COEFFS
      if(axis == INSTRUMENT) {
	 while(coeffs_state_cts(2*axis, vel) == TRUE) {
	    ;
	 }
      }
#endif

      taskDelay (15);
   }

   vel = 0;
   set_velocity(2*axis, vel);

   {
      double pos;
      get_position_corr(2*axis, &pos);
      set_position_corr(2*axis, pos);
   }
#if SWITCH_PID_COEFFS
   while(coeffs_state_cts(2*axis, vel) == TRUE) {
      ;
   }
#endif
   NTRACE_1(5, uid, cid, "Velocity = %g", vel);
#endif

   semGive (semMEI);

   return(0);
}

/*****************************************************************************/
/*
 * Reset amplifiers for an axis
 */
int
mcp_amp_reset(int axis)
{
   int uid = 0, cid = 0;
   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      NTRACE_1(0, uid, cid, "mcp_amp_reset: illegal axis %d", axis);

      return(-1);
   }

   /* Per Dan Long, this is the mcpMenu command to use for setting slewing coeffs. */
   select_pid_block(uid, cid, axis, PID_COEFFS_SLEWING);

   if(axis == INSTRUMENT) {
      amp_reset(2*axis);
   } else {
      amp_reset(2*axis);
      amp_reset(2*axis + 1);
   }

   return(0);
}

/*****************************************************************************/
/*
 * Stop an axis
 */
int
mcp_stop_axis(int axis)
{
   int uid = 0, cid = 0;

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      NTRACE_1(0, uid, cid, "mcp_stop_axis: illegal axis %d", axis);

      return(-1);
   }

   NTRACE_1(3, uid, cid, "Stopping axis %s", axis_name(axis));

   if(semTake(semMEI,60) == ERROR) {
      NTRACE_2(0, uid, cid, "Stopping axis %s: couldn't take semMEI: %d",
	       axis_name(axis), errno);
      return(-1);
   }

   sem_controller_idle(2*axis);
   reset_integrator(2*axis);

   semGive (semMEI);

   return(0);
}

/*****************************************************************************/
/*
 * Wrappers for MCP commands.
 *
 * These aren't declared static so that cmdList can find them
 */
char *
drift_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   const int axis = ublock->axis_select;
   double arcdeg, veldeg, t;		/* as returned by mcp_drift */

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badAxis");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "drift");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

   if(mcp_drift(uid, cid, axis, &arcdeg, &veldeg, &t) < 0) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "drift");
      return("ERR: DRIFT");
   }

   sprintf(ublock->buff, "%f %f %f", arcdeg, veldeg, t);
   NTRACE_2(3, uid, cid, "DRIFT %s: %s", axis_name(axis), ublock->buff);
#if 0
   printf("at end DRIFT %s: %s\n", axis_name(axis), ublock->buff);
#endif

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "drift");

   return(ublock->buff);
}

char *
move_cmd(int uid, unsigned long cid, char *cmd)
{
   const int axis = ublock->axis_select;
   double params[3];
   int cnt;

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badAxis");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "move");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

   cnt = sscanf(cmd,"%lf %lf %lf", &params[0], &params[1], &params[2]);
   if(mcp_move(uid, cid, axis, params, cnt) < 0) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "move");
      return("ERR: MOVE");
   }

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "move");

   return("");
}

char *
plus_move_cmd(int uid, unsigned long cid, char *cmd)
{
   const int axis = ublock->axis_select;
   double params[3];
   int cnt;

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badAxis");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "plus_move");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

   cnt = sscanf(cmd,"%lf %lf %lf", &params[0], &params[1], &params[2]);
   if(mcp_plus_move(axis, params, cnt) < 0) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "plus_move");
      return("ERR: +MOVE");
   }

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "plus_move");

   return("");
}

/*****************************************************************************/

char *
amp_reset_cmd(int uid, unsigned long cid, char *cmd)		/* NOTUSED */
{
   mcp_amp_reset(ublock->axis_select);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "amp_reset");

   return("");
}

char *
hold_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */

{
   mcp_hold(ublock->axis_select);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "hold");

   return("");
}

char *
stop_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   mcp_stop_axis(ublock->axis_select);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "stop");

   return("");
}

char *
set_monitor_cmd(int uid, unsigned long cid, char *cmd)			/* NOTUSED */
{
   int on_off;

   if(sscanf(cmd, "%d", &on_off) != 1) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "set_monitor");
      return("ERR: malformed command argument");
   }

   mcp_set_monitor(ublock->axis_select, on_off);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "set_monitor");

   return("");
}

char *
set_pos_cmd(int uid, unsigned long cid, char *cmd)
{
   double pos;

   if(sscanf(cmd, "%lf", &pos) != 1) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "set_pos");
      return("ERR: malformed command argument");
   }

   mcp_set_pos(ublock->axis_select, pos);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "set_pos");

   return("");
}

char *
goto_pos_va_cmd(int uid, unsigned long cid, char *cmd)
{
   double pos, vel, acc;

   if(sscanf(cmd, "%lf %lf %lf", &pos, &vel, &acc) != 3) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "goto_pos_va");
      return("ERR: malformed command argument");
   }

#if 0 /* debugging */
   NTRACE_1(1,uid,cid,"goto_pos_va_cmd: pos=%f",pos);
   NTRACE_1(1,uid,cid,"goto_pos_va_cmd: vel=%f",vel);
   NTRACE_1(1,uid,cid,"goto_pos_va_cmd: acc=%f",acc);
#endif

   mcp_move_va(ublock->axis_select, pos, vel, acc);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "goto_pos_va");

   return("");
}

char *
set_vel_cmd(int uid, unsigned long cid, char *cmd)
{
   double pos;

   if(sscanf(cmd, "%lf", &pos) != 1) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "get_val");
      return("ERR: malformed command argument");
   }

   mcp_set_vel(ublock->axis_select, pos);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "get_val");

   return("");
}

char *
set_scale_cmd(int uid, unsigned long cid, char *cmd)
{
   double ticksize;

   if(sscanf(cmd, "%lf", &ticksize) != 1) {
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "set_scale");
      return("ERR: malformed command argument");
   }

   set_axis_scale(ublock->axis_select, ticksize);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "set_scale");

   return("");
}

char *
bump_clear_cmd(int uid, unsigned long cid, char *cmd)		/* NOTUSED */
{
   clear_sticky_bumps(ublock->axis_select, 0);
   clear_sticky_bumps(ublock->axis_select, 1);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "bump_clear");

   return("");
}

/*****************************************************************************/
/*
 * Restore positions from shared memory after a boot
 */
void
restore_pos(void)
{
   int uid = 0, cid = 0;
   struct SDSS_FRAME *save;
   struct TM_M68K *restore;
   int i;

   save = (struct SDSS_FRAME *)(SHARE_MEMORY + 2);
   for(i = 0; i < NAXIS; i++) {
      restore = (struct TM_M68K *)&save->axis[i];

      fprintf(stderr, "Restoring encoder position for %s: %ld\n",
	      axis_name(i), restore->actual_position);
      NTRACE_2(3, uid, cid, "Restoring encoder position for %s: %ld",
	       axis_name(i), restore->actual_position);

      tm_set_position(2*i, restore->actual_position);
      tm_set_position(2*i + 1,restore->actual_position2);
   }
}

/*****************************************************************************/
/*
 * Set the axis scales for an axis
 */
void
set_axis_scale(int axis,		/* the axis in question */
	       double ticksize)		/* size of an encoder tick in asec,
					   use default if <= 0 */
{
   int uid = 0, cid = 0;

   if(semTake(semMEI, WAIT_FOREVER) == ERROR) {
      NTRACE_2(0, uid, cid, "Cannot take semMEI to set scale for %s: %s",
	       axis_name(axis), strerror(errno));
   }

   if(ticksize <= 0) {
      switch (axis) {
       case AZIMUTH:
	 ticksize = AZ_TICK0;
	 break;
       case ALTITUDE:
	 ticksize = ALT_TICK0;
	 break;
       case INSTRUMENT:
	 ticksize = ROT_TICK0;
	 break;
      }
   }

   sec_per_tick[axis] = ticksize;
   ticks_per_degree[axis] = 3600/sec_per_tick[axis];

   semGive(semMEI);
}

/*****************************************************************************/
/*
 * Provide an MCP command to set an MEI axis control coeff (e.g. PID)
 */
char *
set_filter_coeff_cmd(int uid, unsigned long cid, char *cmd)
{
   const int axis = ublock->axis_select;
   int ind = -1;			/* index for name */
   char name[40];			/* name of desired coefficient */
   int val = 0;				/* new value for coefficient */

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badAxis");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "set_filter_coeff");
      return("ERR: ILLEGAL DEVICE SELECTION");
   }

   if(sscanf(cmd, "%s %d", name, &val) != 2) {
      NTRACE_1(0, uid, cid, "Need name and val for SET.FILTER.COEFF: %s", cmd);
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "set_filter_coeff");
      return("ERR: missing arguments");
   }

   if(strcmp(name, "P") == 0) {
      ind = DF_P;
   } else if(strcmp(name, "I") == 0) {
      ind = DF_I;
   } else if(strcmp(name, "D") == 0) {
      ind = DF_D;
   } else if(strcmp(name, "ACCEL_FF") == 0) {
      ind = DF_ACCEL_FF;
   } else if(strcmp(name, "VEL_FF") == 0) {
      ind = DF_VEL_FF;
   } else if(strcmp(name, "I_LIMIT") == 0) {
      ind = DF_I_LIMIT;
   } else if(strcmp(name, "OFFSET") == 0) {
      ind = DF_OFFSET;
   } else if(strcmp(name, "DAC_LIMIT") == 0) {
      ind = DF_DAC_LIMIT;
   } else if(strcmp(name, "SHIFT") == 0) {
      ind = DF_SHIFT;
   } else if(strcmp(name, "FRICT_FF") == 0) {
      ind = DF_FRICT_FF;
   } else {
      NTRACE_1(0, uid, cid, "Unknown coefficient name %s", name);
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "set_filter_coeff");
      return("ERR: unknown coefficient name");
   }

   tm_set_filter_coeff(2*axis, ind, val);

   NTRACE_2(3, uid, cid, "Set %s coefficient for axis %s",
	    name, axis_name(axis));
   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "set_filter_coeff");

   return("");
}

/*
 * Return filter coefficients
 */
char *
get_filter_coeffs_cmd(int uid, unsigned long cid, char *cmd)
{
   const int axis = ublock->axis_select;
   short coeff[COEFFICIENTS];
   int i, idx;
   const char *names[COEFFICIENTS] = {
      "P", "I", "D", "ACCEL_FF", "VEL_FF", "I_LIMIT",
      "OFFSET", "DAC_LIMIT", "SHIFT", "FRICT_FF"
   };

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "badAxis");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "get_filter_coeffs");
      return("ERR: ILLEGAL DEVICE SELECTION");
   }

   semTake(semMEI,WAIT_FOREVER);
   get_filter(2*axis, (P_INT)coeff);
   semGive(semMEI);

   {
      char key[20];
      sprintf(key, "%sPidCoeffs", axis_abbrev(axis));

      idx = 0;
      for(i = 0;i < COEFFICIENTS; i++) {
	 idx += sprintf(&ublock->buff[idx],(i == 0 ? "%d" : ", %d"), coeff[i]);
      }

      sendStatusMsg_A(uid, cid, INFORMATION_CODE, 1, key, ublock->buff);
   }

   idx = 0;
   idx += sprintf(&ublock->buff[idx],"axis=%s", axis_name(axis));
   for(i = 0;i < COEFFICIENTS; i++) {
      idx += sprintf(&ublock->buff[idx]," %s=%d", names[i], coeff[i]);
   }

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "get_filter_coeffs");

   return(ublock->buff);
}

/*****************************************************************************/
/*
 * Here are the axis PID coefficients, arranged into NPID_BLOCK each of which
 * has the values for AZIMUTH, ALTITUDE, and INSTRUMENT in that order
 */
typedef short PID_COEFFS[COEFFICIENTS];

/* We maintain two sets of coefficients: one (0) for tracking, and one (1) for slewing.
   The code run by the INIT and MS.ON commands selects the slewing block, and MS.OFF selects
   the tracking block
*/
#define NPID_BLOCK 2
PID_COEFFS  pid_coeffs[NPID_BLOCK][3] = {
   {					/* Block 0 */
      {					/*   AZIMUTH */
	 135,				/*     DF_P */
	 4,				/*     DF_I */
	 550,				/*     DF_D */
	 0,				/*     DF_ACCEL_FF */
	 0,				/*     DF_VEL_FF */
	 32767,				/*     DF_I_LIMIT */
	 0,				/*     DF_OFFSET */
	 18000,				/*     DF_DAC_LIMIT */
	 -4,				/*     1/16; DF_SHIFT */
	 0				/*     DF_FRICT_FF */
      },
      {					/*   ALTITUDE */
	 120,				/*     DF_P */
	 4,				/*     DF_I */
	 600,				/*     DF_D */
	 0,				/*     DF_ACCEL_FF */
	 0,				/*     DF_VEL_FF */
	 32767,				/*     DF_I_LIMITv */
	 0,				/*     DF_OFFSET */
	 10000,				/*     DF_DAC_LIMIT */
	 -4,				/*     1/16; DF_SHIFT */
	 0				/*     DF_FRICT_FF */
      },
      {					/*   INSTRUMENT */
	 50,				/*     DF_P */
	 5,				/*     DF_I */
	 160,				/*     DF_D */
	 0,				/*     DF_ACCEL_FF */
	 0,				/*     DF_VEL_FF */
	 32767,				/*     DF_I_LIMIT */
	 0,				/*     DF_OFFSET */
	 12000,				/*     DF_DAC_LIMIT */
	 -5,				/*     1/32; DF_SHIFT */
	 0				/*     DF_FRICT_FF */
      },
   },
   {					/* Block 1 */
      {					/*   AZIMUTH */
	 135,				/*     DF_P */
	 4,				/*     DF_I */
	 550,				/*     DF_D */
	 0,				/*     DF_ACCEL_FF */
	 0,				/*     DF_VEL_FF */
	 32767,				/*     DF_I_LIMIT */
	 0,				/*     DF_OFFSET */
	 18000,				/*     DF_DAC_LIMIT */
	 -4,				/*     1/16; DF_SHIFT */
	 0				/*     DF_FRICT_FF */
      },
      {					/*   ALTITUDE */
	 120,				/*     DF_P */
	 4,				/*     DF_I */
	 600,				/*     DF_D */
	 0,				/*     DF_ACCEL_FF */
	 0,				/*     DF_VEL_FF */
	 32767,				/*     DF_I_LIMITv */
	 0,				/*     DF_OFFSET */
	 10000,				/*     DF_DAC_LIMIT */
	 -4,				/*     1/16; DF_SHIFT */
	 0				/*     DF_FRICT_FF */
      },
      {					/*   INSTRUMENT */
	 50,				/*     DF_P */
	 4,				/*     DF_I */
	 160,				/*     DF_D */
	 0,				/*     DF_ACCEL_FF */
	 0,				/*     DF_VEL_FF */
	 32767,				/*     DF_I_LIMIT */
	 0,				/*     DF_OFFSET */
	 12000,				/*     DF_DAC_LIMIT */
	 -5,				/*     1/32; DF_SHIFT */
	 0				/*     DF_FRICT_FF */
      }
   }
};

void
select_pid_block(int uid,		/* user id */
		 unsigned long cid,	/* command id */
		 int axis,		/* desired axis */
		 int block)		/* desired block of coefficients */
{
   assert(axis == AZIMUTH || axis == ALTITUDE || axis == INSTRUMENT);
   assert(block >= 0 && block < NPID_BLOCK);

/*
 * Set filter coefficients for the axes.  Note that it is essential that we
 * provide values for all of the coefficients.
 */
   while(semTake(semMEI, WAIT_FOREVER) == ERROR) {
      NTRACE_1(0, uid, cid, "Failed to take semMEI: %s", strerror(errno));
      taskSuspend(0);
   }

   set_filter(2*axis, (P_INT)pid_coeffs[block][axis]);

   semGive(semMEI);
   NTRACE_2(1, uid, cid, "set PID coeffs. axis=%d, block=%d", axis, block);
}

char *
select_pid_block_cmd(int uid, unsigned long cid, char *cmd)
{
   int block = -1;
   const int axis = ublock->axis_select;

   if(axis != AZIMUTH && axis != ALTITUDE && axis != INSTRUMENT) {
      sendStatusMsg_N(uid, cid, INFORMATION_CODE, 1, "select_pid_block");
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "select_pid_block");
      return "ERR: ILLEGAL DEVICE SELECTION";
   }

   if (sscanf(cmd,"%d", &block) != 1 || block < 0 || block >= NPID_BLOCK) {
      sendStatusMsg_I(uid, cid, INFORMATION_CODE, 1, "badBlock", block);
      sendStatusMsg_S(uid, cid, ERROR_CODE, 1, "command", "switch_pid_block");
      return "ERR: ILLEGAL PID BLOCK SELECTION";
   }

   select_pid_block(uid, cid, axis, block);

   sendStatusMsg_I(uid, cid, INFORMATION_CODE, 1, "pid_coeffs_block", block);

   sendStatusMsg_S(uid, cid, FINISHED_CODE, 1, "command", "select_pid_block");

   return "";
}

/*****************************************************************************/
/*
 * General initialization of MEI board...called from startup script.
 *
 *
 * Initialize queue data structures.
 *
 * Setup semaphores
 *
 * Declare commands to the command interpreter
 */
double az_tick, alt_tick, rot_tick;	/* size of an encoder tick in asec */
double az_ticks_deg, alt_ticks_deg, rot_ticks_deg; /* ticks/degree */

int
axisMotionInit(void)
{
   int uid = 0, cid = 0;
   short action;
   char buffer[MAX_ERROR_LEN] ;
   int err;
   int i;
   double limit;
   int mei_axis;			/* counter for all 2*NAXIS "axes" */
   double rate;
/*
 * Create semaphores
 */
   if(semMEI == NULL) {
      semMEI = semMCreate(SEM_Q_PRIORITY|SEM_INVERSION_SAFE);
      assert(semMEI != 0);
   }
   if(semSLC == NULL) {
      semSLC = semMCreate(SEM_Q_PRIORITY|SEM_INVERSION_SAFE);
      assert(semSLC != NULL);
   }
/*
 * We don't really need to take semMEI here as we're the only process
 * running, but it's clearer if we do
 */
   while(semTake(semMEI, WAIT_FOREVER) == ERROR) {
      NTRACE_1(0, uid, cid, "Cannot take semMEI in axisMotionInit: %s", strerror(errno));
      taskSuspend(0);
   }

   for(i = 0; i < NAXIS; i++) {
      axis_queue[i].top = (struct FRAME *)malloc(sizeof(struct FRAME));
      if(axis_queue[i].top == NULL) {
	 NTRACE(0, uid, cid, "axisMotionInit: no memory for queue");
	 return(ERROR);
      }

      axis_queue[i].end = axis_queue[i].top;
      axis_queue[i].active = NULL;
      axis_queue[i].cnt = 1;
      axis_queue[i].top->nxt = NULL;
      axis_queue[i].top->position = 0;
      axis_queue[i].top->velocity = 0;
      axis_queue[i].top->end_time = 0;
   }
   sdss_was_init = TRUE;

   err = dsp_init(DSP_IO_BASE);
   if(err) {
      error_msg(err, buffer) ;	/* convert an error code to a human message */
      NTRACE_2(0, uid, cid, "dsp_init failed--%s (%d)", buffer, err);
#if 0
      return(ERROR);
#endif
   } else {
      printf("dsp_init Passed!\n");
   }

   err = dsp_reset();
   if(err) {
      error_msg(err, buffer) ;	/* convert an error code to a human message */
      NTRACE_2(0, uid, cid, "dsp_reset failed--%s (%d)", buffer, err);
#if 0
      return(ERROR);
#endif
   } else {
      printf("dsp_reset Passed!\n");
   }

   set_sample_rate(160);
   NTRACE_1(3, uid, cid, "Sample Rate=%d", dsp_sample_rate());

   for(mei_axis = 0; mei_axis < dsp_axes(); mei_axis++) {
      NTRACE_1(3, uid, cid, "Initialising AXIS %d", mei_axis);

      get_stop_rate(mei_axis, &rate);
      NTRACE_1(3, uid, cid, "old stop rate = %f", rate);
      set_stop_rate(mei_axis, (double)SDSS_STOP_RATE);
      get_stop_rate(mei_axis, &rate);
      NTRACE_1(3, uid, cid, "set stop rate = %f", rate);

      get_e_stop_rate(mei_axis, &rate);
      NTRACE_1(3, uid, cid, "old e_stop rate=%f", rate);
      set_e_stop_rate(mei_axis, (double)SDSS_E_STOP_RATE);
      get_e_stop_rate(mei_axis, &rate);
      NTRACE_1(3, uid, cid, "set e_stop rate=%f", rate);

      get_error_limit(mei_axis, &limit, &action);
      NTRACE_2(3, uid, cid, "old error limit=%ld, action=%d", (long)limit, action);
#if 0
      set_error_limit(mei_axis, 24000, NO_EVENT);
#else
      set_error_limit(mei_axis, 24000, ABORT_EVENT);
#endif
      get_error_limit(mei_axis, &limit, &action);
      NTRACE_2(3, uid, cid, "set error limit=%ld, action=%d", (long)limit, action);

      set_integration(mei_axis, IM_ALWAYS);
   }

   restore_pos();			/* restore axis positions */

   semGive(semMEI);

   VME2_pre_scaler(0xE0);  /* 256-freq, defaults to 33 MHz, but sys is 32MHz */
   init_io(2,IO_INPUT);

   {
      int block = PID_COEFFS_SLEWING;
      select_pid_block(uid, cid, AZIMUTH, block);
      select_pid_block(uid, cid, ALTITUDE, block);
      select_pid_block(uid, cid, INSTRUMENT, block);
   }
/*
 * Check that we can cast axis_stat[] to long and do bit manipulations
 */
   if(sizeof(struct AXIS_STAT) != sizeof(long)) {
      NTRACE_2(0, uid, cid, "sizeof(struct AXIS_STAT) != sizeof(long) (%d != %d)",
	       sizeof(struct AXIS_STAT), sizeof(long));
      abort();
   }
/*
 * Set the default axis scales.  These may be updated later from
 * the fiducial tables or (eventually) dynamically.
 */
   for(i = 0; i < NAXIS; i++) {
      set_axis_scale(i, 0);
   }
/*
 * Spawn the tasks that run the axes
 */
   taskSpawn("tmAz",47,VX_FP_TASK,20000,
	     (FUNCPTR)tm_TCC, AZIMUTH,
	     0,0,0,0,0,0,0,0,0);
   taskSpawn("tmAlt",47,VX_FP_TASK,20000,
	     (FUNCPTR)tm_TCC, ALTITUDE,
	     0,0,0,0,0,0,0,0,0);
   taskSpawn("tmRot",47,VX_FP_TASK,20000,
	     (FUNCPTR)tm_TCC, INSTRUMENT,
	     0,0,0,0,0,0,0,0,0);
/*
 * Declare commands
 */
   define_cmd("+MOVE",         plus_move_cmd, 	 -1, 1, 0, 1, "");
   define_cmd("AB_STATUS",     abstatus_cmd, 	  2, 0, 0, 1, "");
   define_cmd("AMP_RESET",     amp_reset_cmd, 	  0, 1, 0, 1, "");
   define_cmd("AXIS_STATUS",   axis_status_cmd,   0, 0, 0, 0, "");
   define_cmd("BUMP_CLEAR",    bump_clear_cmd,    0, 1, 0, 1, "");
   define_cmd("DRIFT",         drift_cmd, 	  0, 1, 1, 1, "");
   define_cmd("GOTO_POS_VA",   goto_pos_va_cmd,   3, 1, 1, 1,
	      "Go to a POSITION with specified VELOCITY and ACCELERATION (all in telescope ticks)");
   define_cmd("HALT",          hold_cmd, 	  0, 1, 0, 1, "");
   define_cmd("HOLD",          hold_cmd, 	  0, 1, 0, 1, "");
   define_cmd("ID",            id_cmd, 		  0, 0, 0, 1, "");
   define_cmd("INIT",          init_cmd, 	  0, 0, 1, 1, "");
   define_cmd("INFO", info_cmd, 0, 0, 0, 0,
	      "Broadcast everything we know about the non-axis related parts of the system");
   define_cmd("ROT",           rot_cmd, 	  0, 0, 0, 0,
	      "All succeeding commands apply to the rotator");
   define_cmd("IR",            rot_cmd, 	  0, 0, 0, 0,
	      "All succeeding commands apply to the rotator");
   define_cmd("MC_MAXACC",     mc_maxacc_cmd,     0, 0, 0, 1,
	      "Return current axis's maximum permitted acceleration");
   define_cmd("MC_MAXVEL",     mc_maxvel_cmd,     0, 0, 0, 1,
	      "Return current axis's maximum permitted velocity");
   define_cmd("MOVE",          move_cmd, 	 -1, 1, 1, 1,
              "Go to a POSITION (degrees) with an optional VELOCITY (degrees/sec) and TIME (seconds since start of UTC day)");
   define_cmd("SET_MONITOR",   set_monitor_cmd,   1, 1, 0, 1, "");
   define_cmd("SET_POS_VA",    goto_pos_va_cmd,   3, 1, 1, 1,
	      "Go to a POSITION with specified VELOCITY and ACCELERATION (all in telescope ticks)");
   define_cmd("SET_POSITION",  set_pos_cmd, 	  1, 1, 0, 1, "");
   define_cmd("SET_VELOCITY",  set_vel_cmd, 	  1, 1, 0, 1, "");
   define_cmd("STATS",         stats_cmd, 	  0, 0, 0, 1, "");
   define_cmd("STATUS",        status_cmd, 	  0, 0, 0, 1, "");
   define_cmd("STATUS_LONG",   status_long_cmd,   0, 0, 0, 1, "");
   define_cmd("STOP",          stop_cmd, 	  0, 1, 0, 1, "");
   define_cmd("SYSTEM_STATUS", system_status_cmd, 0, 0, 0, 0, "");
   define_cmd("SET_SCALE",     set_scale_cmd,     1, 1, 0, 1,
	      "Set the scale (arcsec/tick) for the current axis");
   define_cmd("AZ",            az_cmd,          0, 0, 0, 0,
	      "All succeeding commands apply to azimuth");
   define_cmd("TEL1",          az_cmd,          0, 0, 0, 0,
	      "All succeeding commands apply to azimuth");
   define_cmd("ALT",           alt_cmd,          0, 0, 0, 0,
	      "All succeeding commands apply to altitude");
   define_cmd("TEL2",          alt_cmd,          0, 0, 0, 0,
	      "All succeeding commands apply to altitude");
   define_cmd("SET_FILTER_COEFF", set_filter_coeff_cmd, 2, 1, 0, 1,
	      "Set a filter (e.g. PID) coefficient for the current axis");
   define_cmd("GET_FILTER_COEFFS", get_filter_coeffs_cmd, 0, 0, 0, 0,
	      "Get the filter (PID etc.) coefficients for the current axis");
   define_cmd("GET_FILTER_COEFF", get_filter_coeffs_cmd, 0, 0, 0, 0,
	      "Alias for GET.FILTER.COEFFS");

   define_cmd("SELECT_PID_BLOCK", select_pid_block_cmd,     1, 1, 0, 1,
	      "Select which set of PID coefficients should be active");

   return 0;
}
